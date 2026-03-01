"""Obstacle substitution node for Follow The Gap algorithm.

Converts LiDAR scan data (sensor_msgs/LaserScan) into obstacle
representations (obstacle_msgs/ObstaclesStamped) for use with the
follow_the_gap_v0 package.
"""

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

try:
    from obstacle_msgs.msg import ObstaclesStamped, Obstacles, CircleObstacle
    USE_OBSTACLE_MSGS = True
except ImportError:
    print('Unable to find obstacle_msgs. Publisher will be disabled.', file=sys.stderr)
    USE_OBSTACLE_MSGS = False


class ObstacleSubstitutionNode(Node):
    """Converts LiDAR scan to obstacle representations."""

    def __init__(self):
        super().__init__('obstacle_substitution')

        self.declare_parameter('car_radius', 0.4)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_range', 0.1)

        self.car_radius = self.get_parameter('car_radius').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value

        qos = qos_profile_sensor_data
        qos.depth = 1

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        if USE_OBSTACLE_MSGS:
            self.obstacles_pub = self.create_publisher(
                ObstaclesStamped,
                '/obstacles',
                10
            )

        self.get_logger().info('ObstacleSubstitutionNode initialized')

    def scan_callback(self, msg: LaserScan):
        """Convert LaserScan to ObstaclesStamped."""
        if not USE_OBSTACLE_MSGS:
            return

        obstacles_msg = ObstaclesStamped()
        obstacles_msg.header = msg.header
        obstacles_msg.obstacles = Obstacles()

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if math.isnan(r) or math.isinf(r):
                continue
            if r < self.min_range or r > self.max_range:
                continue

            circle = CircleObstacle()
            circle.center.x = r * math.cos(angle)
            circle.center.y = r * math.sin(angle)
            circle.center.z = 0.0
            circle.radius = self.car_radius

            obstacles_msg.obstacles.circles.append(circle)

        self.obstacles_pub.publish(obstacles_msg)


def main(args=None):
    """Entry point for obstacle_substitution_node."""
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = None
    try:
        node = ObstacleSubstitutionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node is not None:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
