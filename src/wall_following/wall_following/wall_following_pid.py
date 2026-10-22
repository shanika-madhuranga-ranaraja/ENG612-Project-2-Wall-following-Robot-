
import math
import time
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WallFollowingPID(Node):

    def __init__(self):
        super().__init__('wall_following_pid')

        # -----------------------------------
        # WALL FOLLOWING SETTINGS
        # -----------------------------------
        self.desired_distance = 0.30

        # Initial PID gains
        self.kp = 0.9
        self.ki = 0.0
        self.kd = 0.015

        # Slow initial test speed
        self.linear_speed = 0.05
        self.max_angular_speed = 0.35

        # Corner detection distance
        self.front_limit = 0.40

        # 90-degree clockwise turn
        self.target_turn = math.pi / 2
        self.turn_speed = 0.35
        self.turn_tolerance = math.radians(3)

        # PID variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

        # Robot state
        self.state = 'FOLLOW_WALL'

        # Odometry variables
        self.current_yaw = None
        self.previous_yaw = None
        self.turned_angle = 0.0

        # Timing and safety
        self.last_scan_time = None
        self.last_log_time = 0.0

        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # LiDAR subscriber
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Odometry subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Control timer
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'LEFT Wall Following PID Started'
        )
        self.get_logger().info(
            'Target distance: 0.30 m'
        )
        self.get_logger().info(
            'Corner turning: 90 degrees RIGHT'
        )

    # -----------------------------------
    # ANGLE NORMALIZATION
    # -----------------------------------
    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # -----------------------------------
    # ODOMETRY CALLBACK
    # -----------------------------------
    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        siny = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        yaw = math.atan2(siny, cosy)

        # Accumulate actual rotation during a turn
        if (
            self.state == 'TURNING'
            and self.previous_yaw is not None
        ):
            delta = self.normalize_angle(
                yaw - self.previous_yaw
            )

            self.turned_angle += delta

        self.current_yaw = yaw
        self.previous_yaw = yaw

    # -----------------------------------
    # READ LIDAR SECTOR
    # -----------------------------------
    def get_sector_distance(
        self, msg, angle_deg, width_deg,
        use_min=False
    ):

        values = []

        start = angle_deg - width_deg
        end = angle_deg + width_deg

        for degree in range(
            int(start),
            int(end) + 1
        ):

            angle = math.radians(degree)

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

    # -----------------------------------
    # LIDAR CALLBACK
    # -----------------------------------
    def scan_callback(self, msg):

        self.left_distance = self.get_sector_distance(
            msg, 90, 5
        )

        self.front_distance = self.get_sector_distance(
            msg, 0, 15, use_min=True
        )

        self.last_scan_time = time.monotonic()

    # -----------------------------------
    # START CORNER TURN
    # -----------------------------------
    def start_turn(self):

        self.state = 'TURNING'

        self.turned_angle = 0.0

        self.previous_yaw = self.current_yaw

        self.integral = 0.0
        self.previous_time = None

        # Stop before rotating
        self.publisher.publish(Twist())

        self.get_logger().info(
            'CORNER DETECTED - Turning RIGHT 90 degrees'
        )

    # -----------------------------------
    # MAIN CONTROL LOOP
    # -----------------------------------
    def control_loop(self):

        command = Twist()
        now = time.monotonic()

        # Stop if sensor information is missing
        if (
            self.last_scan_time is None
            or now - self.last_scan_time > 1.0
            or self.current_yaw is None
        ):
            self.publisher.publish(command)
            return

        # -----------------------------------
        # STATE 1: FOLLOW WALL
        # -----------------------------------
        if self.state == 'FOLLOW_WALL':

            front = self.front_distance
            left = self.left_distance

            # Invalid readings: stop safely
            if front is None or left is None:
                self.publisher.publish(command)
                return

            # Detect corner
            if front < self.front_limit:
                self.start_turn()
                return

            # Time difference
            if self.previous_time is None:
                dt = 0.0
            else:
                dt = now - self.previous_time

            self.previous_time = now

            # Distance error
            error = self.desired_distance - left

            # Integral and derivative
            derivative = 0.0

            if dt > 0:
                self.integral += error * dt

                self.integral = max(
                    -0.3,
                    min(0.3, self.integral)
                )

                derivative = (
                    error - self.previous_error
                ) / dt

            self.previous_error = error

            # PID output
            pid = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
            )

            # LEFT wall following:
            # Too close -> turn RIGHT
            # Too far -> turn LEFT
            angular = -pid

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
                    f'Angular: {angular:+.3f}'
                )

                self.last_log_time = now

        # -----------------------------------
        # STATE 2: TURN RIGHT 90 DEGREES
        # -----------------------------------
        elif self.state == 'TURNING':

            rotation = abs(self.turned_angle)

            if rotation < (
                self.target_turn - self.turn_tolerance
            ):

                # Rotate clockwise in place
                command.linear.x = 0.0
                command.angular.z = -self.turn_speed

                self.publisher.publish(command)

            else:

                # Stop rotating
                self.publisher.publish(Twist())

                self.get_logger().info(
                    '90 DEGREE TURN COMPLETED'
                )

                self.state = 'FOLLOW_WALL'

                # Reset PID for next wall
                self.integral = 0.0
                self.previous_error = 0.0
                self.previous_time = None

                self.get_logger().info(
                    'Following NEXT LEFT wall'
                )

    # -----------------------------------
    # STOP ROBOT
    # -----------------------------------
    def stop_robot(self):
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


