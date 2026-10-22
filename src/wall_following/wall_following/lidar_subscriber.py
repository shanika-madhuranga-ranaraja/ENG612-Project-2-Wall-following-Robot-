
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')

        # Desired distance from the LEFT wall
        self.desired_distance = 0.30

        # Subscribe to TurtleBot3 LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('LiDAR Subscriber Started')
        self.get_logger().info('Following LEFT wall')
        self.get_logger().info('Desired Distance: 0.30 m')

    def scan_callback(self, msg):

        # LEFT direction = +90 degrees
        angle = math.pi / 2

        # Calculate the LiDAR array index
        index = round(
            (angle - msg.angle_min) / msg.angle_increment
        )

        if 0 <= index < len(msg.ranges):

            distance = msg.ranges[index]

            # Check whether LiDAR measurement is valid
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):

                # Calculate distance error
                error = self.desired_distance - distance

                # Display measured distance and error
                self.get_logger().info(
                    f'Left Distance: {distance:.3f} m | '
                    f'Error: {error:+.3f} m'
                )


def main(args=None):

    rclpy.init(args=args)

    node = LidarSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

