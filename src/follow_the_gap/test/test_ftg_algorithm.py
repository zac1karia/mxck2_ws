import math
import pytest

from follow_the_gap.ftg_node import (
    Obstacle,
    Gap,
    find_gaps_angle,
    find_largest_gap,
    calculate_gap_center_angle,
    calculate_final_heading_angle,
)


class TestObstacle:
    """Tests for the Obstacle dataclass."""

    def test_from_polar_basic(self):
        o = Obstacle.from_polar(2.0, 0.0, 0.3)
        assert math.isclose(o.distance_to_center, 2.0)
        assert math.isclose(o.angle, 0.0)
        assert math.isclose(o.x, 2.0, rel_tol=1e-6)
        assert math.isclose(o.y, 0.0, abs_tol=1e-9)
        assert math.isclose(o.radius, 0.3)
        expected_theta_d = math.asin(0.3 / 2.0)
        assert math.isclose(o.theta_d, expected_theta_d)
        assert math.isclose(o.angle_left, expected_theta_d)
        assert math.isclose(o.angle_right, -expected_theta_d)
        expected_dist = math.sqrt(4.0 - 0.09)
        assert math.isclose(o.distance, expected_dist)

    def test_from_polar_with_angle(self):
        angle = math.pi / 4.0
        o = Obstacle.from_polar(3.0, angle, 0.4)
        assert math.isclose(o.distance_to_center, 3.0)
        assert math.isclose(o.angle, angle)
        assert math.isclose(o.x, 3.0 * math.cos(angle), rel_tol=1e-6)
        assert math.isclose(o.y, 3.0 * math.sin(angle), rel_tol=1e-6)

    def test_from_polar_clamps_radius(self):
        # radius > distance - 0.01 should be clamped
        o = Obstacle.from_polar(0.5, 0.0, 0.5)
        assert o.radius == 0.5 - 0.01

    def test_from_cartesian(self):
        o = Obstacle.from_cartesian(1.0, 1.0, 0.3)
        expected_dist = math.hypot(1.0, 1.0)
        assert math.isclose(o.distance_to_center, expected_dist)
        assert math.isclose(o.angle, math.atan2(1.0, 1.0))

    def test_distance_between_centres(self):
        o1 = Obstacle.from_polar(2.0, math.pi / 4.0, 0.3)
        o2 = Obstacle.from_polar(2.0, -math.pi / 4.0, 0.3)
        d = o1.distance_between_centres(o2)
        expected = math.hypot(o1.x - o2.x, o1.y - o2.y)
        assert math.isclose(d, expected)

    def test_distance_between_edges(self):
        o1 = Obstacle.from_polar(2.0, math.pi / 4.0, 0.3)
        o2 = Obstacle.from_polar(2.0, -math.pi / 4.0, 0.3)
        d = o1.distance_between_edges(o2)
        assert math.isclose(d, o1.distance_between_centres(o2) - 0.3 - 0.3)


class TestGap:
    """Tests for the Gap dataclass."""

    def test_from_obstacles(self):
        o1 = Obstacle.from_polar(2.0, math.pi / 6.0, 0.3)
        o2 = Obstacle.from_polar(2.0, -math.pi / 6.0, 0.3)
        g = Gap.from_obstacles(o1, o2)
        assert math.isclose(g.angle_left, o1.angle_right)
        assert math.isclose(g.angle_right, o2.angle_left)
        assert g.gap_size > 0.0
        assert g.gap_distance > 0.0

    def test_gap_size_is_positive(self):
        o1 = Obstacle.from_polar(2.0, 0.5, 0.2)
        o2 = Obstacle.from_polar(2.0, -0.5, 0.2)
        g = Gap.from_obstacles(o1, o2)
        assert g.gap_size > 0.0


class TestFindGaps:
    """Tests for find_gaps_angle."""

    def test_two_obstacles(self):
        o1 = Obstacle.from_polar(2.0, 0.5, 0.3)
        o2 = Obstacle.from_polar(2.0, -0.5, 0.3)
        gaps = find_gaps_angle([o1, o2])
        assert len(gaps) == 1

    def test_three_obstacles(self):
        o1 = Obstacle.from_polar(2.0, 0.5, 0.3)
        o2 = Obstacle.from_polar(2.0, 0.0, 0.3)
        o3 = Obstacle.from_polar(2.0, -0.5, 0.3)
        gaps = find_gaps_angle([o1, o2, o3])
        assert len(gaps) == 2

    def test_empty_obstacles(self):
        gaps = find_gaps_angle([])
        assert len(gaps) == 0

    def test_single_obstacle(self):
        o1 = Obstacle.from_polar(2.0, 0.0, 0.3)
        gaps = find_gaps_angle([o1])
        assert len(gaps) == 0


class TestFindLargestGap:
    """Tests for find_largest_gap."""

    def test_no_gaps(self):
        assert find_largest_gap([]) is None

    def test_single_gap(self):
        o1 = Obstacle.from_polar(2.0, 0.5, 0.3)
        o2 = Obstacle.from_polar(2.0, -0.5, 0.3)
        gaps = find_gaps_angle([o1, o2])
        largest = find_largest_gap(gaps)
        assert largest is not None
        assert largest is gaps[0]

    def test_multiple_gaps_returns_largest(self):
        # Create obstacles so one gap is larger
        o1 = Obstacle.from_polar(2.0, 1.0, 0.1)
        o2 = Obstacle.from_polar(2.0, 0.8, 0.1)  # small gap between o1-o2
        o3 = Obstacle.from_polar(2.0, -0.8, 0.1)  # large gap between o2-o3
        gaps = find_gaps_angle([o1, o2, o3])
        largest = find_largest_gap(gaps)
        assert largest is not None
        assert largest.gap_size == max(g.gap_size for g in gaps)


class TestCalculateGapCenterAngle:
    """Tests for calculate_gap_center_angle."""

    def test_symmetric_gap_around_zero(self):
        # Two obstacles symmetrically placed around zero
        o1 = Obstacle.from_polar(2.0, math.pi / 6.0, 0.2)
        o2 = Obstacle.from_polar(2.0, -math.pi / 6.0, 0.2)
        g = Gap.from_obstacles(o1, o2)
        center = calculate_gap_center_angle(g)
        # Symmetric gap should have center near 0
        assert abs(center) < 0.1

    def test_gap_entirely_positive(self):
        o1 = Obstacle.from_polar(2.0, 1.0, 0.1)
        o2 = Obstacle.from_polar(2.0, 0.5, 0.1)
        g = Gap.from_obstacles(o1, o2)
        center = calculate_gap_center_angle(g)
        # Center should be between the two angles
        assert center is not None

    def test_gap_entirely_negative(self):
        o1 = Obstacle.from_polar(2.0, -0.5, 0.1)
        o2 = Obstacle.from_polar(2.0, -1.0, 0.1)
        g = Gap.from_obstacles(o1, o2)
        center = calculate_gap_center_angle(g)
        assert center is not None

    def test_none_obstacles(self):
        g = Gap()
        center = calculate_gap_center_angle(g)
        assert center == 0.0


class TestCalculateFinalHeadingAngle:
    """Tests for calculate_final_heading_angle."""

    def test_goal_straight_ahead(self):
        heading = calculate_final_heading_angle(0.0, 0.0, 1.0, 100.0)
        assert math.isclose(heading, 0.0, abs_tol=1e-9)

    def test_gap_center_dominates_when_close(self):
        # When d_min is small, alpha/d_min is large, so gap_center dominates
        heading = calculate_final_heading_angle(0.0, 0.5, 0.1, 100.0)
        assert heading > 0.4  # Should be close to 0.5

    def test_goal_dominates_when_far(self):
        # When d_min is large, alpha/d_min is small, so goal dominates
        heading = calculate_final_heading_angle(0.0, 0.5, 100.0, 1.0)
        assert heading < 0.1  # Should be close to 0.0

    def test_small_d_min_clamped(self):
        # d_min <= 0 should be clamped to 0.01
        heading = calculate_final_heading_angle(0.0, 0.5, 0.0, 100.0)
        assert not math.isnan(heading)
        assert not math.isinf(heading)
