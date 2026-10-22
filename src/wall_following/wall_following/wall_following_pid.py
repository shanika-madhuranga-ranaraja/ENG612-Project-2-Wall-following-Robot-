
import math
import time
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowingPID(Node):

    def __init__(self):
        super().__init__('wall_following_pid')

        # -----------------------------------
        # WALL FOLLOWING PARAMETERS
        # -----------------------------------
        self.desired_distance = 0.30

        # TEST B: Single PID controller gains
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.04

        # Constant linear speed during FOLLOW
        self.linear_speed = 0.05
        self.max_angular_speed = 0.25

        # -----------------------------------
        # CORNER SETTINGS (UNCHANGED)
        # -----------------------------------
        self.corner_distance = 0.40
        self.front_clear_distance = 0.75
        self.turn_speed = 0.22

        # Next wall detection
        self.next_wall_min = 0.20
        self.next_wall_max = 0.65

        # Wall alignment
        self.alignment_tolerance = 0.04

        # Turning conditions
        self.minimum_turn_time = 1.0
        self.maximum_turn_time = 15.0
        self.required_clear_count = 4
        self.corner_cooldown = 2.0

        # -----------------------------------
        # PID VARIABLES
        # -----------------------------------
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        # -----------------------------------
        # SENSOR VARIABLES
        # -----------------------------------
        self.left_distance = None
        self.front_distance = None
        self.left_front = None
        self.left_rear = None
        self.last_scan_time = None

        # -----------------------------------
        # CONTROLLER STATES
        # -----------------------------------
        self.state = 'FOLLOW_WALL'
        self.corner_start_time = None
        self.last_corner_end = None

        self.clear_count = 0
        self.corner_count = 0
        self.last_log_time = 0.0

        # ROS publisher
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # ROS subscriber
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Control loop at 10 Hz
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            'TEST B: LEFT WALL FOLLOWING STARTED'
        )
        self.get_logger().info(
            'Single PID: Kp=1.2, Ki=0.0, Kd=0.04'
        )
        self.get_logger().info(
            'Target: 0.30 m | Linear: 0.05 m/s'
        )

    # -----------------------------------
    # READ LIDAR SECTOR
    # -----------------------------------
    def get_distance(
        self, msg, angle_deg,
        width_deg=3, use_min=False
    ):

        readings = []

        if msg.angle_increment <= 0:
            return None

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
                    readings.append(distance)

        if not readings:
            return None

        if use_min:
            return min(readings)

        return statistics.median(readings)

    # -----------------------------------
    # LIDAR CALLBACK
    # -----------------------------------
    def scan_callback(self, msg):

        # Left wall: +90 degrees
        self.left_distance = self.get_distance(
            msg, 90, 5
        )

        # Front obstacle: 0 degrees
        self.front_distance = self.get_distance(
            msg, 0, 12, use_min=True
        )

        # Measurements for wall alignment
        self.left_front = self.get_distance(
            msg, 80, 3
        )

        self.left_rear = self.get_distance(
            msg, 100, 3
        )

        self.last_scan_time = time.monotonic()

    # -----------------------------------
    # RESET PID
    # -----------------------------------
    def reset_pid(self):

        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    # -----------------------------------
    # START CORNER TURN
    # -----------------------------------
    def start_corner(self, now):

        self.state = 'TURN_RIGHT'
        self.corner_start_time = now
        self.clear_count = 0

        self.reset_pid()
        self.publisher.publish(Twist())

        self.get_logger().info(
            'CORNER DETECTED - TURN RIGHT'
        )

    # -----------------------------------
    # MAIN CONTROL LOOP
    # -----------------------------------
    def control_loop(self):

        now = time.monotonic()
        command = Twist()

        # Stop if LiDAR readings are unavailable
        if (
            self.last_scan_time is None
            or now - self.last_scan_time > 1.0
            or self.left_distance is None
            or self.front_distance is None
        ):
            self.publisher.publish(command)
            return

        left = self.left_distance
        front = self.front_distance

        # -----------------------------------
        # STATE 1: FOLLOW WALL
        # -----------------------------------
        if self.state == 'FOLLOW_WALL':

            # Corner detection (unchanged)
            if front <= self.corner_distance:
                self.start_corner(now)
                return

            dt = 0.0

            if self.previous_time is not None:
                dt = now - self.previous_time

            self.previous_time = now

            # Calculate wall-distance error
            error = self.desired_distance - left

            derivative = 0.0

            if dt > 0:

                self.integral += error * dt

                self.integral = max(
                    -0.3,
                    min(0.3, self.integral)
                )

                if self.previous_error is not None:
                    derivative = (
                        error - self.previous_error
                    ) / dt

            self.previous_error = error

            # -----------------------------------
            # SINGLE PID ANGULAR CONTROLLER
            # -----------------------------------
            pid_output = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
            )

            # Left wall: positive error -> turn right
            angular = -pid_output

            angular = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular)
            )

            # Constant linear velocity
            command.linear.x = self.linear_speed
            command.angular.z = angular

            self.publisher.publish(command)

            if now - self.last_log_time >= 1.0:

                self.get_logger().info(
                    f'FOLLOW | '
                    f'Left: {left:.3f} m | '
                    f'Front: {front:.3f} m | '
                    f'Error: {error:+.3f} m | '
                    f'Linear: {command.linear.x:.3f} | '
                    f'Angular: {angular:+.3f}'
                )

                self.last_log_time = now

        # -----------------------------------
        # STATE 2: TURN RIGHT
        # -----------------------------------
        elif self.state == 'TURN_RIGHT':

            elapsed = now - self.corner_start_time

            # Safety timeout
            if elapsed >= self.maximum_turn_time:

                self.state = 'STOPPED'
                self.publisher.publish(Twist())

                self.get_logger().warning(
                    'TURN TIMEOUT - ROBOT STOPPED'
                )
                return

            # Check front clearance
            front_clear = (
                front >= self.front_clear_distance
            )

            # Check whether next left wall exists
            wall_detected = (
                self.next_wall_min
                <= left
                <= self.next_wall_max
            )

            # Wall alignment measurements
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

            # Same corner completion conditions
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

            if self.clear_count >= self.required_clear_count:

                self.publisher.publish(Twist())

                self.corner_count += 1
                self.last_corner_end = now
                self.state = 'FOLLOW_WALL'

                self.reset_pid()

                self.get_logger().info(
                    f'CORNER {self.corner_count} COMPLETED'
                )

                self.get_logger().info(
                    f'Next Left Wall: {left:.3f} m'
                )

                return

            # Rotate right without fixed 90-degree turn
            command.linear.x = 0.0
            command.angular.z = -self.turn_speed

            self.publisher.publish(command)

            if now - self.last_log_time >= 1.0:

                align_text = (
                    f'{alignment_error:.3f}'
                    if alignment_error is not None
                    else 'N/A'
                )

                self.get_logger().info(
                    f'TURN RIGHT | '
                    f'Front: {front:.3f} m | '
                    f'Left: {left:.3f} m | '
                    f'Alignment: {align_text} | '
                    f'Time: {elapsed:.1f} s'
                )

                self.last_log_time = now

        # -----------------------------------
        # STATE 3: SAFETY STOP
        # -----------------------------------
        else:
            self.publisher.publish(Twist())

    # -----------------------------------
    # STOP ROBOT
    # -----------------------------------
    def stop_robot(self):

        if rclpy.ok():
            self.publisher.publish(Twist())


def main(args=None):

    rclpy.init(args=args)
    node = WallFollowingPID()

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

