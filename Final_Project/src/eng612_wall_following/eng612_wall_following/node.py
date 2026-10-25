"""ROS 2 interface, monotonic scan watchdog, and experiment CSV logging."""
import csv
from dataclasses import fields
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from .core import Config, WallController, observe
from .telemetry import planar_pose, number


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        defaults = Config()
        for field in fields(Config):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        for name, default in [('scan_topic', '/scan'), ('cmd_vel_topic', '/cmd_vel'),
                              ('scan_timeout', 0.75), ('enabled', False),
                              ('stamped_cmd_vel', False), ('log_path', ''),
                              ('odom_topic', '/odom'), ('odom_timeout', 1.0),
                              ('status_period', 1.0)]:
            self.declare_parameter(name, default)
        self.cfg = Config(**{f.name: self.get_parameter(f.name).value for f in fields(Config)})
        self.controller = WallController(self.cfg)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.timeout = float(self.get_parameter('scan_timeout').value)
        if not 0.1 <= self.timeout <= 2.0:
            raise ValueError('scan_timeout must be between 0.1 and 2.0 s')
        self.stamped = bool(self.get_parameter('stamped_cmd_vel').value)
        self.publisher = self.create_publisher(
            TwistStamped if self.stamped else Twist,
            self.get_parameter('cmd_vel_topic').value, 10)
        self.subscription = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan,
            qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom,
            qos_profile_sensor_data)
        self.service = self.create_service(SetBool, '~/enable', self.on_enable)
        self.last_receive = None
        self.last_observation = None
        self.pose = None
        self.odom_received = None
        self.odom_stamp = None
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        period = float(self.get_parameter('status_period').value)
        if not (math.isfinite(period) and 0.1 <= period <= 10.0 and
                math.isfinite(self.odom_timeout) and 0.1 <= self.odom_timeout <= 5.0):
            raise ValueError('status_period must be 0.1-10 s; odom_timeout 0.1-5 s')
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.previous_stamp = None
        self.state = None
        self.started = self.get_clock().now().nanoseconds * 1e-9
        self.file = None
        self.writer = None
        path = self.get_parameter('log_path').value
        if path:
            target = Path(path).expanduser()
            target.parent.mkdir(parents=True, exist_ok=True)
            # Never silently overwrite an experiment.
            self.file = target.open('x', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['time_s', 'mode', 'state', 'distance_m', 'predicted_m',
                                  'target_m', 'error_m', 'front_m', 'nearest_m',
                                  'linear_mps', 'angular_radps', 'slowdown_error_m',
                                  'wall_error_m', 'x_m', 'y_m', 'yaw_deg', 'odom_frame',
                                  'turn_remaining_deg'])
        # A steady timer still runs when Gazebo /clock is paused.
        self.watchdog = self.create_timer(0.05, self.check_watchdog,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status_timer = self.create_timer(period, self.report_status,
                                             clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'{self.cfg.mode} PID; {self.cfg.wall_side} wall; '
                               f'enabled={self.enabled}; call /wall_follower/enable to start')

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = TwistStamped() if self.stamped else Twist()
        twist = msg.twist if self.stamped else msg
        if self.stamped:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
        twist.linear.x, twist.angular.z = float(linear), float(angular)
        self.commanded_linear, self.commanded_angular = float(linear), float(angular)
        self.publisher.publish(msg)

    def set_state(self, state):
        self.state = state

    def on_odom(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = planar_pose(message)
        if pose is None or stamp <= 0 or not -0.25 <= now - stamp <= self.odom_timeout:
            self.pose = None
            self.odom_received = None
            return
        self.pose = pose
        self.odom_stamp = stamp
        self.odom_received = time.monotonic()

    def current_pose(self):
        if self.pose is None or self.odom_received is None:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - self.odom_stamp
        if (time.monotonic() - self.odom_received > self.odom_timeout or
                not -0.25 <= age <= self.odom_timeout):
            return None
        return self.pose

    def report_status(self):
        pose = self.current_pose()
        state = self.state or 'WAIT_SCAN'
        # Never present the two-wall corner geometry or stale readings as
        # a reliable perpendicular tracking error.
        obs = self.last_observation
        usable = (state in ('FOLLOW', 'DISABLED') and obs is not None and
                  self.last_receive is not None and
                  time.monotonic() - self.last_receive <= self.timeout)
        distance = obs.distance if usable else math.nan
        error = distance - self.cfg.target_distance
        x, y, yaw, frame = pose if pose else (math.nan, math.nan, math.nan, 'unavailable')
        turn_remaining = self.turn_remaining(pose)
        fresh = (obs is not None and self.last_receive is not None and
                 time.monotonic() - self.last_receive <= self.timeout)
        self.get_logger().info(
            f'{state} | frame={frame} x={number(x, signed=True)} '
            f'y={number(y, signed=True)} yaw={number(math.degrees(yaw), "deg", True)} | '
            f'wall={self.cfg.wall_side} distance={number(distance)} '
            f'target={number(self.cfg.target_distance)} error={number(error, signed=True)} | '
            f'front={number(obs.front if fresh else math.nan)} '
            f'turn_remaining={number(turn_remaining, "deg")} | '
            f'v_cmd={number(self.commanded_linear, "m/s")} '
            f'w_cmd={number(self.commanded_angular, "rad/s", True)}')

    def on_enable(self, request, response):
        requested = bool(request.data)
        if requested and not self.enabled and self.controller.fault:
            self.controller.reset()
        else:
            self.controller.pause()
        self.enabled = requested
        self.previous_stamp = None
        self.publish_velocity()
        self.set_state('WAIT_SCAN' if self.enabled else 'DISABLED')
        response.success = True
        response.message = 'Enabled; waiting for a valid scan' if self.enabled else 'Stopped'
        return response

    def on_scan(self, scan):
        self.last_receive = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        # Check source timestamps as well as receipt time: delayed queued scans
        # must not keep the robot moving. Synchronise PC/robot clocks in the lab.
        age = now - stamp
        if stamp <= 0 or age > self.timeout or age < -0.25:
            self.last_observation = None
            self.controller.pause()
            self.previous_stamp = None
            self.publish_velocity()
            self.set_state('BAD_SCAN_TIMESTAMP')
            self.log(now, 'BAD_SCAN_TIMESTAMP')
            return
        self.last_observation = observe(scan, self.cfg)
        if not self.enabled:
            self.publish_velocity()
            self.previous_stamp = None
            self.set_state('DISABLED')
            self.log(now, 'DISABLED')
            return
        if self.previous_stamp is None:
            dt = 0.1
        else:
            dt = stamp - self.previous_stamp
        self.previous_stamp = stamp
        pose = self.current_pose()
        command = self.controller.step(scan, dt,
                                       yaw=pose[2] if pose else None,
                                       position=(pose[0], pose[1]) if pose else None)
        self.publish_velocity(command.linear, command.angular)
        self.set_state(command.state)
        self.log(now, command.state, command)

    def check_watchdog(self):
        if self.last_receive is None or time.monotonic() - self.last_receive > self.timeout:
            self.controller.pause()
            self.previous_stamp = None
            self.publish_velocity()
            self.set_state('WAIT_SCAN' if self.last_receive is None else 'STALE_SCAN')
            self.log(self.get_clock().now().nanoseconds * 1e-9, self.state)

    def turn_remaining(self, pose):
        if pose is None or not self.controller.turning or self.controller.turn_target_yaw is None:
            return math.nan
        delta = self.controller.turn_target_yaw - pose[2]
        return abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))

    def log(self, now, state, cmd=None):
        if not self.writer:
            return
        obs = cmd.observation if cmd else None
        pose = self.current_pose()
        x, y, yaw, frame = pose if pose else (math.nan, math.nan, math.nan, '')
        wall_error = (obs.distance - self.cfg.target_distance
                      if obs and state == 'FOLLOW' else math.nan)
        self.writer.writerow([now - self.started, self.cfg.mode, state,
                              obs.distance if obs else math.nan,
                              obs.predicted if obs else math.nan,
                              self.cfg.target_distance, cmd.error if cmd else math.nan,
                              obs.front if obs else math.nan, obs.nearest if obs else math.nan,
                              cmd.linear if cmd else 0.0, cmd.angular if cmd else 0.0,
                              cmd.slowdown_error if cmd else math.nan,
                              wall_error, x, y, math.degrees(yaw), frame,
                              self.turn_remaining(pose)])
        self.file.flush()

    def close(self):
        if rclpy.ok():
            self.publish_velocity()
        if self.file:
            self.file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WallFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
