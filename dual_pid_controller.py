
import math
import time
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PID:
    def __init__(self, kp, ki, kd, minimum, maximum):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.minimum = minimum
        self.maximum = maximum
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def calculate(self, error, dt):
        derivative = 0.0

        if dt > 0.0:
            self.integral += error * dt
            self.integral = max(-0.3, min(0.3, self.integral))

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
            self.minimum,
            min(self.maximum, output)
        )


class DualPIDWallFollower(Node):

    def __init__(self):
        super().__init__('dual_pid_wall_follower')

        # ==================================
        # TWO PID CONTROLLERS - UNCHANGED
        # ==================================
        self.desired_distance = 0.30

        self.angular_pid = PID(
            1.2, 0.0, 0.04, -0.25, 0.25
        )

        self.linear_pid = PID(
            0.10, 0.0, 0.005, 0.0, 0.025
        )

        self.max_linear_speed = 0.05
        self.min_linear_speed = 0.025

        self.slowdown_distance = 0.80
        self.speed_error_tolerance = 0.02

        # ==================================
        # CORNER NAVIGATION
        # ==================================
        self.corner_distance = 0.32

        # Normal LiDAR rearming threshold
        self.rearm_front_distance = 0.65

        # Required clearance after a turn
        self.front_clear_distance = 0.52

        # Odometry-guided right turn
        self.turn_finish_min = 87.0
        self.turn_max_degrees = 94.0
        self.max_turn_time = 18.0

        self.turn_speed = 0.20
        self.final_turn_speed = 0.10
        self.slow_turn_after = 75.0

        self.required_clear_count = 2

        # Prevent immediate repeated turns
        # during the first two seconds after
        # completing a corner.
        self.corner_cooldown = 2.0

        # Safety
        self.emergency_distance = 0.15

        # ==================================
        # STATE
        # ==================================
        self.state = 'FOLLOW_WALL'

        self.corner_count = 0
        self.corner_armed = True

        self.last_corner_time = None
        self.turn_start_time = None
        self.turn_angle = 0.0
        self.last_turn_yaw = None
        self.clear_count = 0

        self.last_control_time = None
        self.last_log_time = 0.0

        # ==================================
        # SENSOR DATA
        # ==================================
        self.left_distance = None
        self.front_distance = None
        self.left_front = None
        self.left_rear = None

        self.last_scan_time = None
        self.last_odom_time = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # ==================================
        # ROS COMMUNICATION
        # ==================================
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'DUAL PID - LIDAR REARMING TEST STARTED'
        )
        self.get_logger().info(
            'Angular PID: Kp=1.2 Ki=0.0 Kd=0.04'
        )
        self.get_logger().info(
            'Linear PID: Kp=0.10 Ki=0.0 Kd=0.005'
        )
        self.get_logger().info(
            'Corner=0.32m | Rearm=0.65m'
        )

    # ==================================
    # READ LIDAR DISTANCE
    # ==================================
    def get_distance(
        self, msg, angle_deg,
        width_deg=3, use_min=False
    ):
        if msg.angle_increment <= 0:
            return None

        readings = []

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
                    and msg.range_min <= distance <= msg.range_max
                ):
                    readings.append(distance)

        if not readings:
            return None

        return (
            min(readings)
            if use_min
            else statistics.median(readings)
        )

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

    # ==================================
    # READ ODOMETRY
    # ==================================
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.robot_x = position.x
        self.robot_y = position.y

        siny = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        yaw = math.atan2(siny, cosy)

        if (
            self.state == 'TURN_RIGHT'
            and self.last_turn_yaw is not None
        ):
            delta = math.atan2(
                math.sin(yaw - self.last_turn_yaw),
                math.cos(yaw - self.last_turn_yaw)
            )

            self.turn_angle -= delta

        if self.state == 'TURN_RIGHT':
            self.last_turn_yaw = yaw

        self.robot_yaw = yaw
        self.last_odom_time = time.monotonic()

    # ==================================
    # HELPERS
    # ==================================
    def reset_pids(self):
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.last_control_time = None

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def emergency_stop(self, reason):
        if self.state != 'STOPPED':
            self.get_logger().error(
                'ROBOT STOPPED: ' + reason
            )

        self.state = 'STOPPED'
        self.stop_robot()

    # ==================================
    # START CORNER
    # ==================================
    def start_corner(self, now):
        self.state = 'TURN_RIGHT'

        self.turn_start_time = now
        self.turn_angle = 0.0
        self.last_turn_yaw = self.robot_yaw
        self.clear_count = 0

        self.corner_armed = False

        self.reset_pids()
        self.stop_robot()

        self.get_logger().info(
            'CORNER DETECTED - RIGHT TURN'
        )

    # ==================================
    # FINISH CORNER
    # ==================================
    def finish_corner(self, now):
        self.stop_robot()

        self.corner_count += 1
        self.state = 'FOLLOW_WALL'

        self.last_corner_time = now
        self.corner_armed = False

        self.reset_pids()

        self.get_logger().info(
            f'CORNER {self.corner_count} COMPLETED'
        )

        self.get_logger().info(
            f'Angle: '
            f'{math.degrees(self.turn_angle):.1f} deg'
        )

        self.get_logger().info(
            f'Left: {self.left_distance:.3f} m | '
            f'Front: {self.front_distance:.3f} m'
        )

    # ==================================
    # LIDAR-BASED REARMING
    # ==================================
    def update_corner_rearm(self, now, front):
        if self.corner_armed:
            return

        if self.last_corner_time is None:
            self.corner_armed = True
            return

        elapsed = now - self.last_corner_time

        if elapsed < self.corner_cooldown:
            return

        if front >= self.rearm_front_distance:
            self.corner_armed = True

            self.get_logger().info(
                'NEXT CORNER ENABLED - FRONT CLEAR'
            )

    # ==================================
    # MAIN CONTROL LOOP
    # ==================================
    def control_loop(self):
        now = time.monotonic()

        if self.state == 'STOPPED':
            self.stop_robot()
            return

        # Check data availability
        if (
            self.last_scan_time is None
            or self.last_odom_time is None
            or now - self.last_scan_time > 1.0
            or now - self.last_odom_time > 1.0
            or self.left_distance is None
            or self.front_distance is None
            or self.robot_yaw is None
        ):
            self.stop_robot()
            return

        left = self.left_distance
        front = self.front_distance

        # Safety stop - always retained
        if (
            left < self.emergency_distance
            or front < self.emergency_distance
        ):
            self.emergency_stop(
                'Obstacle closer than 0.15 m'
            )
            return

        command = Twist()

        # ==================================
        # STATE 1 - FOLLOW WALL
        # ==================================
        if self.state == 'FOLLOW_WALL':

            self.update_corner_rearm(now, front)

            # CORNER DETECTION
            if front <= self.corner_distance:

                # Never continue forward into
                # a blocked front sector.
                self.stop_robot()

                if (
                    self.last_corner_time is not None
                    and now - self.last_corner_time
                    < self.corner_cooldown
                ):
                    self.emergency_stop(
                        'Obstacle encountered '
                        'immediately after turn'
                    )
                    return

                if not self.corner_armed:
                    # Fallback: a genuine front
                    # obstruction must be handled,
                    # even if 0.65 m was never seen.
                    #
                    # This can create extra turns.
                    self.get_logger().warning(
                        'FRONT BLOCKED BEFORE LIDAR '
                        'REARM - STARTING RECOVERY TURN'
                    )

                self.start_corner(now)
                return

            if self.last_control_time is None:
                dt = 0.0
            else:
                dt = now - self.last_control_time

            self.last_control_time = now

            # PID 1 - ANGULAR
            error = self.desired_distance - left

            angular_output = (
                self.angular_pid.calculate(error, dt)
            )

            angular_velocity = -angular_output

            # PID 2 - LINEAR
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

            command.linear.x = linear_velocity
            command.angular.z = angular_velocity

            self.cmd_pub.publish(command)

            if now - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f'FOLLOW | '
                    f'Left: {left:.3f} | '
                    f'Front: {front:.3f} | '
                    f'Error: {error:+.3f} | '
                    f'Linear: {linear_velocity:.3f} | '
                    f'Angular: {angular_velocity:+.3f} | '
                    f'Armed: {self.corner_armed}'
                )

                self.last_log_time = now

        # ==================================
        # STATE 2 - RIGHT TURN
        # ==================================
        elif self.state == 'TURN_RIGHT':

            elapsed = now - self.turn_start_time

            angle_deg = math.degrees(
                self.turn_angle
            )

            if elapsed >= self.max_turn_time:
                self.emergency_stop(
                    'Turn timeout'
                )
                return

            if angle_deg >= self.turn_max_degrees:
                self.emergency_stop(
                    'Maximum turn angle reached '
                    'without front clearance'
                )
                return

            enough_rotation = (
                angle_deg >= self.turn_finish_min
            )

            front_clear = (
                front >= self.front_clear_distance
            )

            if enough_rotation and front_clear:
                self.clear_count += 1
            else:
                self.clear_count = 0

            if (
                self.clear_count
                >= self.required_clear_count
            ):
                self.finish_corner(now)
                return

            command.linear.x = 0.0

            if angle_deg >= self.slow_turn_after:
                command.angular.z = (
                    -self.final_turn_speed
                )
            else:
                command.angular.z = (
                    -self.turn_speed
                )

            self.cmd_pub.publish(command)

            if now - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f'TURN | '
                    f'Angle: {angle_deg:.1f} deg | '
                    f'Front: {front:.3f} | '
                    f'Left: {left:.3f} | '
                    f'Time: {elapsed:.1f}s'
                )

                self.last_log_time = now


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

