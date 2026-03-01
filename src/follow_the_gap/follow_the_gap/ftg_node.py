import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Obstacle:
    """Represents a detected obstacle with position and size information."""

    distance_to_center: float = 0.0
    angle: float = 0.0
    x: float = 0.0
    y: float = 0.0
    radius: float = 0.0
    theta_d: float = 0.0
    angle_left: float = 0.0
    angle_right: float = 0.0
    distance: float = 0.0

    @staticmethod
    def from_polar(dist: float, angle: float, radius: float) -> 'Obstacle':
        """Create obstacle from polar coordinates (distance, angle, radius)."""
        o = Obstacle()
        o.distance_to_center = dist
        o.angle = angle
        o.x = dist * math.cos(angle)
        o.y = dist * math.sin(angle)
        o.radius = radius
        if radius > dist - 0.01:
            o.radius = dist - 0.01
        o.theta_d = math.asin(o.radius / dist)
        o.angle_left = angle + o.theta_d
        o.angle_right = angle - o.theta_d
        o.distance = math.sqrt(dist * dist - o.radius * o.radius)
        return o

    @staticmethod
    def from_cartesian(x: float, y: float, radius: float) -> 'Obstacle':
        """Create obstacle from Cartesian coordinates."""
        o = Obstacle()
        o.x = x
        o.y = y
        o.radius = radius
        o.distance_to_center = math.hypot(x, y)
        o.angle = math.atan2(y, x)
        if o.radius > o.distance_to_center - 0.01:
            o.radius = o.distance_to_center - 0.01
        o.theta_d = math.asin(o.radius / o.distance_to_center)
        o.angle_left = o.angle + o.theta_d
        o.angle_right = o.angle - o.theta_d
        o.distance = math.sqrt(o.distance_to_center ** 2 - o.radius ** 2)
        return o

    def distance_between_centres(self, other: 'Obstacle') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def distance_between_edges(self, other: 'Obstacle') -> float:
        return self.distance_between_centres(other) - self.radius - other.radius


@dataclass
class Gap:
    """Represents a gap between two obstacles."""

    obstacle_left: Optional[Obstacle] = None
    obstacle_right: Optional[Obstacle] = None
    angle_left: float = 0.0
    angle_right: float = 0.0
    gap_size: float = 0.0
    gap_distance: float = 0.0

    @staticmethod
    def from_obstacles(o1: Obstacle, o2: Obstacle) -> 'Gap':
        """Create a gap from two obstacles (o1=left, o2=right)."""
        g = Gap()
        g.obstacle_left = o1
        g.obstacle_right = o2
        g.angle_left = o1.angle_right
        g.angle_right = o2.angle_left
        g.gap_size = abs(g.angle_left - g.angle_right)
        g.gap_distance = o1.distance_between_centres(o2)
        return g


def create_obstacles(scan_msg, car_radius: float = 0.4) -> List[Obstacle]:
    """Create obstacles from a LaserScan message."""
    obstacles = []
    angle = scan_msg.angle_min
    for i, r in enumerate(scan_msg.ranges):
        if math.isnan(r) or math.isinf(r):
            angle += scan_msg.angle_increment
            continue
        if r < scan_msg.range_min or r > scan_msg.range_max:
            angle += scan_msg.angle_increment
            continue
        obstacles.append(Obstacle.from_polar(r, angle, car_radius))
        angle += scan_msg.angle_increment
    # Reverse so first obstacle is leftmost (largest angle)
    obstacles.reverse()
    return obstacles


def filter_obstacles(
    obstacles: List[Obstacle], fov_angle_max: float, max_range: float
) -> List[Obstacle]:
    """Filter obstacles by FOV angle and max range."""
    return [
        o for o in obstacles
        if abs(o.angle) <= fov_angle_max and o.distance_to_center <= max_range
    ]


def find_gaps_angle(obstacles: List[Obstacle]) -> List[Gap]:
    """Find gaps between consecutive obstacles."""
    gaps = []
    for i in range(len(obstacles) - 1):
        gaps.append(Gap.from_obstacles(obstacles[i], obstacles[i + 1]))
    return gaps


def find_largest_gap(gaps: List[Gap]) -> Optional[Gap]:
    """Return the gap with the maximum gap_size."""
    if not gaps:
        return None
    return max(gaps, key=lambda g: g.gap_size)


def calculate_gap_center_angle(gap: Gap) -> float:
    """Calculate the center angle of a gap using the paper's formula.

    Variable naming follows the original C++ implementation:
    d_1 = obstacle_right.distance, d_2 = obstacle_left.distance,
    theta_1 = |angle_right|, theta_2 = |angle_left|.
    """
    if gap.obstacle_left is None or gap.obstacle_right is None:
        return 0.0

    d_1 = gap.obstacle_right.distance
    d_2 = gap.obstacle_left.distance
    theta_1 = abs(gap.angle_right)
    theta_2 = abs(gap.angle_left)

    fallback = (gap.angle_right + gap.angle_left) / 2.0

    try:
        # Case 1: gap straddles zero (left >= 0, right <= 0)
        if gap.angle_left >= 0.0 and gap.angle_right <= 0.0:
            cos_sum = math.cos(theta_1 + theta_2)
            denom_sq = d_1 * d_1 + d_2 * d_2 + 2.0 * d_1 * d_2 * cos_sum
            if denom_sq <= 0.0:
                return fallback
            denom = math.sqrt(denom_sq)
            arg = (d_1 + d_2 * cos_sum) / denom
            arg = max(-1.0, min(1.0, arg))
            theta_gap_c = math.acos(arg) - theta_1
            if math.isnan(theta_gap_c):
                return fallback
            return theta_gap_c

        # Case 2: both angles positive (gap entirely on the left)
        if gap.angle_right >= 0.0:
            l_squared = (d_1 * d_1 + d_2 * d_2
                         - 2.0 * d_1 * d_2 * math.cos(theta_2 - theta_1)) / 4.0
            h_squared = (d_1 * d_1 + d_2 * d_2 - 2.0 * l_squared) / 2.0
            if h_squared < 0.0:
                return fallback
            h = math.sqrt(h_squared)
            if h == 0.0:
                return fallback
            arg = (h_squared + d_1 * d_1 - l_squared) / (2.0 * h * d_1)
            arg = max(-1.0, min(1.0, arg))
            theta_x = math.acos(arg)
            theta_gap_c = theta_1 + theta_x
            if math.isnan(theta_gap_c):
                return fallback
            return theta_gap_c

        # Case 3: both angles negative (gap entirely on the right)
        if gap.angle_left <= 0.0:
            l_squared = (d_1 * d_1 + d_2 * d_2
                         - 2.0 * d_1 * d_2 * math.cos(theta_1 - theta_2)) / 4.0
            h_squared = (d_1 * d_1 + d_2 * d_2 - 2.0 * l_squared) / 2.0
            if h_squared < 0.0:
                return fallback
            h = math.sqrt(h_squared)
            if h == 0.0:
                return fallback
            arg = (h_squared + d_2 * d_2 - l_squared) / (2.0 * h * d_2)
            arg = max(-1.0, min(1.0, arg))
            theta_x = math.acos(arg)
            theta_gap_c = -(theta_2 + theta_x)
            if math.isnan(theta_gap_c):
                return fallback
            return theta_gap_c

    except (ValueError, ZeroDivisionError):
        pass

    return fallback


def calculate_final_heading_angle(
    goal_angle: float,
    gap_center_angle: float,
    d_min: float,
    alpha: float = 100.0,
) -> float:
    """Calculate the final heading angle combining goal and gap center."""
    if d_min <= 0.0:
        d_min = 0.01
    weight = alpha / d_min
    return (weight * gap_center_angle + goal_angle) / (weight + 1.0)


try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from ackermann_msgs.msg import AckermannDriveStamped
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False
    Node = object


class FollowTheGapNode(Node):
    """ROS2 node implementing the Follow-the-Gap algorithm."""

    def __init__(self):
        super().__init__('follow_the_gap_node')

        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('fov_angle_max', math.pi / 2.0 + math.pi / 16.0)
        self.declare_parameter('car_radius', 0.4)
        self.declare_parameter('gap_weight_coefficient', 100.0)
        self.declare_parameter('goal_angle', 0.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_steering_angle', 0.44)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/autonomous/ackermann_cmd', 10
        )

        self.get_logger().info('FollowTheGapNode started')

    def scan_callback(self, scan_msg):
        max_range = self.get_parameter('max_range').value
        fov_angle_max = self.get_parameter('fov_angle_max').value
        car_radius = self.get_parameter('car_radius').value
        alpha = self.get_parameter('gap_weight_coefficient').value
        goal_angle = self.get_parameter('goal_angle').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        steering_gain = self.get_parameter('steering_gain').value
        max_steering_angle = self.get_parameter('max_steering_angle').value

        # 1. Create obstacles from scan
        obstacles = create_obstacles(scan_msg, car_radius)
        if not obstacles:
            self.get_logger().debug('No obstacles detected')
            return

        # 2. Filter obstacles
        obstacles = filter_obstacles(obstacles, fov_angle_max, max_range)
        if not obstacles:
            self.get_logger().debug('No obstacles after filtering')
            return

        # 3. Find gaps
        gaps = find_gaps_angle(obstacles)
        if not gaps:
            self.get_logger().debug('No gaps found')
            return

        # 4. Find largest gap
        largest_gap = find_largest_gap(gaps)
        if largest_gap is None:
            self.get_logger().debug('No largest gap found')
            return

        # 5. Calculate gap center angle
        gap_center_angle = calculate_gap_center_angle(largest_gap)

        # 6. Calculate minimum distance to obstacles
        d_min = min(o.distance for o in obstacles)

        # 7. Calculate final heading angle
        final_heading = calculate_final_heading_angle(
            goal_angle, gap_center_angle, d_min, alpha
        )

        if math.isnan(final_heading):
            self.get_logger().warn('Final heading is NaN, skipping')
            return

        # 8. Convert to speed and steering
        steering_angle = final_heading * steering_gain
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))

        speed = max_speed - (max_speed - min_speed) * abs(steering_angle) / (math.pi / 4.0)
        speed = max(min_speed, min(max_speed, speed))

        # 9. Publish command
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
