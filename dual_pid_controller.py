
import math
import time
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class PID:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def calculate(self, error, dt):
        derivative = 0.0

        if dt > 0.0:
            self.integral += error * dt

            self.integral = max(
                -0.3, min(0.3, self.integral)
            )

            if self.previous_error is not None:
                derivative = (
                    error - self.previous_error
                ) / dt

        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        return max(
            self.output_min,
            min(self.output_max, output)
        )


class DualPIDWallFollower(Node):

    def __init__(self):
        super().__init__('dual_pid_wall_follower')

        # ------------------------------------
        # TARGET WALL DISTANCE
        # ------------------------------------
        self.desired_distance = 0.30

        # ------------------------------------
        # PID 1 - ANGULAR VELOCITY
        # UNCHANGED: Test B gains
        # ------------------------------------
        self.angular_pid = PID(
            kp=1.2,
            ki=0.0,
            kd=0.04,
            output_min=-0.25,
            output_max=0.25
        )

        # ------------------------------------
        # PID 2 - LINEAR VELOCITY
        # TEST 2: UPDATED GAINS
        # ------------------------------------
        self.linear_pid = PID(
            kp=0.10,
            ki=0.0,
            kd=0.005,
            output_min=0.0,
            output_max=0.025
        )

        # ------------------------------------
        # FORWARD SPEED LIMITS
        # ------------------------------------
        self.max_linear_speed = 0.05
        self.min_linear_speed = 0.025

        # Slowdown parameters (unchanged)
        self.slowdown_distance = 0.80
        self.speed_error_tolerance = 0.02

        # ------------------------------------
        # CORNER SETTINGS - UNCHANGED
        # ------------------------------------
        self.corner_distance = 0.40
        self.front_clear_distance = 0.75
        self.turn_speed = 0.22

        self.next_wall_min = 0.20
        self.next_wall_max = 0.65
        self.alignment_tolerance = 0.04

        self.minimum_turn_time = 1.0
        self.maximum_turn_time = 15.0
        self.required_clear_count = 4

        # ------------------------------------
        # CONTROLLER STATE
        # ------------------------------------
        self.state = 'FOLLOW_WALL'
        self.corner_start_time = None
        self.clear_count = 0
        self.corner_count = 0

        self.last_control_time = None
        self.last_log_time = 0.0

        # ------------------------------------
        # LIDAR VARIABLES
        # ------------------------------------
        self.left_distance = None
        self.front_distance = None
        self.left_front = None
        self.left_rear = None
        self.last_scan_time = None

        # ------------------------------------
        # ROS 2 PUBLISHER AND SUBSCRIBER
        # ------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            'DUAL PID - TEST 2 STARTED'
        )

        self.get_logger().info(
            'Angular PID: Kp=1.2 Ki=0.0 Kd=0.04'
        )

        self.get_logger().info(
            'Linear PID: Kp=0.10 Ki=0.0 Kd=0.005'
        )

        self.get_logger().info(
            'Target=0.30m | Speed=0.025-0.050m/s'
        )

    # ------------------------------------
    # LIDAR SECTOR MEASUREMENT
    # ------------------------------------
    def get_distance(
        self, msg, angle_deg,
        width_deg=3, use_min=False
    ):

        if msg.angle_increment <= 0:
            return None

        values = []

        span = msg.angle_max - msg.angle_min

        full_circle = (
            span >= 2 * math.pi
            - 2 * msg.angle_increment
        )

        for degree in range(
            angle_deg - width_deg,
            angle_deg + width_deg + 1
        ):

            angle = math.radians(degree)

            if full_circle:
                angle = (
                    (angle - msg.angle_min)
                    % (2 * math.pi)
                ) + msg.angle_min

            index = round(
                (angle - msg.angle_min)
                / msg.angle_increment
            )

            if 0 <= index < len(msg.ranges):

                distance = msg.ranges[index]

                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance
                    <= msg.range_max
                ):
                    values.append(distance)

        if not values:
            return None

        if use_min:
            return min(values)

        return statistics.median(values)

    # ------------------------------------
    # LIDAR CALLBACK
    # ------------------------------------
    def scan_callback(self, msg):

        self.left_distance = self.get_distance(
            msg, 90, 5
        )

        self.front_distance = self.get_distance(
            msg, 0, 12, use_min=True
        )

        self.left_front = self.get_distance(
            msg, 80, 3
        )

        self.left_rear = self.get_distance(
            msg, 100, 3
        )

        self.last_scan_time = time.monotonic()

    # ------------------------------------
    # RESET BOTH PID CONTROLLERS
    # ------------------------------------
    def reset_pids(self):

        self.angular_pid.reset()
        self.linear_pid.reset()

        self.last_control_time = None

    # ------------------------------------
    # START CORNER TURN
    # ------------------------------------
    def start_corner(self, now):

        self.state = 'TURN_RIGHT'
        self.corner_start_time = now
        self.clear_count = 0

        self.reset_pids()
        self.cmd_pub.publish(Twist())

        self.get_logger().info(
            'CORNER DETECTED - TURN RIGHT'
        )

    # ------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------
    def control_loop(self):

        now = time.monotonic()
        command = Twist()

        # Stop if LiDAR data is unavailable
        if (
            self.last_scan_time is None
            or now - self.last_scan_time > 1.0
            or self.left_distance is None
            or self.front_distance is None
        ):
            self.cmd_pub.publish(command)
            return

        left = self.left_distance
        front = self.front_distance

        # Emergency safety stop
        if left < 0.15 or front < 0.15:

            if self.state != 'STOPPED':
                self.get_logger().error(
                    'SAFETY STOP: Obstacle too close'
                )

            self.state = 'STOPPED'
            self.cmd_pub.publish(Twist())
            return

        # ====================================
        # STATE 1 - FOLLOW LEFT WALL
        # ====================================
        if self.state == 'FOLLOW_WALL':

            # Same corner detection
            if front <= self.corner_distance:
                self.start_corner(now)
                return

            if self.last_control_time is None:
                dt = 0.0
            else:
                dt = now - self.last_control_time

            self.last_control_time = now

            # --------------------------------
            # WALL DISTANCE ERROR
            # --------------------------------
            error = self.desired_distance - left

            # --------------------------------
            # PID 1: ANGULAR CONTROL
            # --------------------------------
            angular_output = (
                self.angular_pid.calculate(
                    error, dt
                )
            )

            angular_velocity = -angular_output

            # --------------------------------
            # PID 2: LINEAR CONTROL
            # --------------------------------
            distance_error = abs(error)

            front_risk = max(
                0.0,
                self.slowdown_distance - front
            )

            speed_error = max(
                0.0,
                distance_error
                + front_risk
                - self.speed_error_tolerance
            )

            speed_reduction = (
                self.linear_pid.calculate(
                    speed_error, dt
                )
            )

            linear_velocity = (
                self.max_linear_speed
                - speed_reduction
            )

            linear_velocity = max(
                self.min_linear_speed,
                min(
                    self.max_linear_speed,
                    linear_velocity
                )
            )

            # Publish dual PID outputs
            command.linear.x = linear_velocity
            command.angular.z = angular_velocity

            self.cmd_pub.publish(command)

            # --------------------------------
            # LOG PERFORMANCE
            # --------------------------------
            if now - self.last_log_time >= 1.0:

                self.get_logger().info(
                    f'FOLLOW | '
                    f'Left: {left:.3f} | '
                    f'Front: {front:.3f} | '
                    f'Error: {error:+.3f} | '
                    f'Linear: {linear_velocity:.3f} | '
                    f'Angular: {angular_velocity:+.3f}'
                )

                self.last_log_time = now

        # ====================================
        # STATE 2 - TURN RIGHT
        # ====================================
        elif self.state == 'TURN_RIGHT':

            elapsed = (
                now - self.corner_start_time
            )

            # Stop if turning takes too long
            if elapsed >= self.maximum_turn_time:

                self.state = 'STOPPED'
                self.cmd_pub.publish(Twist())

                self.get_logger().error(
                    'TURN TIMEOUT - ROBOT STOPPED'
                )
                return

            front_clear = (
                front >= self.front_clear_distance
            )

            wall_detected = (
                self.next_wall_min
                <= left
                <= self.next_wall_max
            )

            alignment_error = None
            aligned = False

            if (
                self.left_front is not None
                and self.left_rear is not None
            ):

                alignment_error = abs(
                    self.left_front - self.left_rear
                )

                aligned = (
                    alignment_error
                    <= self.alignment_tolerance
                )

            ready = (
                elapsed >= self.minimum_turn_time
                and front_clear
                and wall_detected
                and aligned
            )

            if ready:
                self.clear_count += 1
            else:
                self.clear_count = 0

            # Complete the corner turn
            if (
                self.clear_count
                >= self.required_clear_count
            ):

                self.cmd_pub.publish(Twist())

                self.corner_count += 1
                self.state = 'FOLLOW_WALL'

                self.reset_pids()

                self.get_logger().info(
                    f'CORNER {self.corner_count} COMPLETED'
                )

                self.get_logger().info(
                    f'Next Left Wall: {left:.3f} m'
                )

                return

            # LiDAR-based right turn
            command.linear.x = 0.0
            command.angular.z = -self.turn_speed

            self.cmd_pub.publish(command)

            if now - self.last_log_time >= 1.0:

                alignment_text = (
                    f'{alignment_error:.3f}'
                    if alignment_error is not None
                    else 'N/A'
                )

                self.get_logger().info(
                    f'TURN RIGHT | '
                    f'Front: {front:.3f} | '
                    f'Left: {left:.3f} | '
                    f'Alignment: {alignment_text} | '
                    f'Time: {elapsed:.1f}s'
                )

                self.last_log_time = now

        # ====================================
        # STATE 3 - STOPPED
        # ====================================
        else:
            self.cmd_pub.publish(Twist())

    # ------------------------------------
    # STOP ROBOT
    # ------------------------------------
    def stop_robot(self):

        if rclpy.ok():
            self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)

    node = DualPIDWallFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

