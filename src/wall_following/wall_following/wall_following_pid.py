
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

        # Desired LEFT-wall distance
        self.desired_distance = 0.30

        # Single PID controller
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.02

        # Constant forward speed during wall following
        self.linear_speed = 0.05
        self.max_angular_speed = 0.25

        # Corner detection and turning
        self.corner_distance = 0.40
        self.front_clear_distance = 0.75
        self.turn_speed = 0.25

        # Detect next wall after turning
        self.next_wall_min = 0.20
        self.next_wall_max = 0.55

        # Turning safety
        self.minimum_turn_time = 1.0
        self.maximum_turn_time = 15.0
        self.required_clear_count = 5

        # PID variables
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        # LiDAR measurements
        self.left_distance = None
        self.front_distance = None
        self.last_scan_time = None

        # Controller states
        self.state = 'FOLLOW_WALL'
        self.corner_start_time = None
        self.clear_count = 0
        self.corner_count = 0

        self.last_log_time = 0.0

        # ROS publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ROS subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Control loop: 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'LEFT WALL FOLLOWING STARTED'
        )
        self.get_logger().info(
            'Single PID Controller'
        )
        self.get_logger().info(
            'Target: 0.30 m | Linear speed: 0.05 m/s'
        )

    # ----------------------------------
    # READ LIDAR SECTOR
    # ----------------------------------
    def get_distance(
        self, msg, angle_deg,
        width_deg=5, use_min=False
    ):

        readings = []

        if msg.angle_increment <= 0:
            return None

        scan_span = msg.angle_max - msg.angle_min

        full_circle = (
            scan_span >= 2 * math.pi
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
                    % (2.0 * math.pi)
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

    # ----------------------------------
    # LIDAR CALLBACK
    # ----------------------------------
    def scan_callback(self, msg):

        # LEFT (+90 degrees)
        self.left_distance = self.get_distance(
            msg, 90, 5
        )

        # FRONT (0 degrees)
        self.front_distance = self.get_distance(
            msg, 0, 12, use_min=True
        )

        self.last_scan_time = time.monotonic()

    # ----------------------------------
    # RESET PID
    # ----------------------------------
    def reset_pid(self):

        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    # ----------------------------------
    # START CORNER
    # ----------------------------------
    def start_corner(self, now):

        self.state = 'TURN_RIGHT'
        self.corner_start_time = now
        self.clear_count = 0

        self.reset_pid()

        self.publisher.publish(Twist())

        self.get_logger().info(
            'CORNER DETECTED - TURN RIGHT'
        )

    # ----------------------------------
    # MAIN CONTROL LOOP
    # ----------------------------------
    def control_loop(self):

        now = time.monotonic()
        command = Twist()

        # Stop if scan is unavailable or outdated
        if (
            self.last_scan_time is None
            or now - self.last_scan_time > 1.0
            or self.front_distance is None
            or self.left_distance is None
        ):
            self.publisher.publish(command)
            return

        front = self.front_distance
        left = self.left_distance

        # ----------------------------------
        # STATE 1: FOLLOW WALL
        # ----------------------------------
        if self.state == 'FOLLOW_WALL':

            if front <= self.corner_distance:
                self.start_corner(now)
                return

            if self.previous_time is None:
                dt = 0.0
            else:
                dt = now - self.previous_time

            self.previous_time = now

            # Distance error
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

            # Single PID for angular velocity
            pid_output = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
            )

            angular = -pid_output

            angular = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular)
            )

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

        # ----------------------------------
        # STATE 2: TURN RIGHT
        # ----------------------------------
        elif self.state == 'TURN_RIGHT':

            elapsed = now - self.corner_start_time

            # Timeout safety
            if elapsed >= self.maximum_turn_time:

                self.state = 'STOPPED'
                self.publisher.publish(Twist())

                self.get_logger().warning(
                    'TURN TIMEOUT - ROBOT STOPPED'
                )
                return

            # Front must be clear
            front_clear = (
                front >= self.front_clear_distance
            )

            # Left wall must be detected
            left_wall_detected = (
                self.next_wall_min
                <= left
                <= self.next_wall_max
            )

            ready = (
                elapsed >= self.minimum_turn_time
                and front_clear
                and left_wall_detected
            )

            if ready:
                self.clear_count += 1
            else:
                self.clear_count = 0

            if self.clear_count >= self.required_clear_count:

                self.publisher.publish(Twist())

                self.corner_count += 1
                self.state = 'FOLLOW_WALL'

                self.reset_pid()

                self.get_logger().info(
                    f'CORNER {self.corner_count} COMPLETED'
                )

                self.get_logger().info(
                    f'Next Left Wall: {left:.3f} m'
                )
                return

            # Rotate right without fixed angle
            command.linear.x = 0.0
            command.angular.z = -self.turn_speed

            self.publisher.publish(command)

            if now - self.last_log_time >= 1.0:

                self.get_logger().info(
                    f'TURN RIGHT | '
                    f'Front: {front:.3f} m | '
                    f'Left: {left:.3f} m | '
                    f'Time: {elapsed:.1f} s'
                )

                self.last_log_time = now

        # ----------------------------------
        # STATE 3: STOPPED
        # ----------------------------------
        else:
            self.publisher.publish(Twist())

    # ----------------------------------
    # STOP ROBOT
    # ----------------------------------
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

