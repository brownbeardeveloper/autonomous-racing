"""Tests for follow mode.

The math in rc_control.lib.follow, closed-loop runs of follow_node's command()
against a ray-cast world, and one test over real topics.
"""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rc_control.follow_node import Follow
from rc_control.lib.follow import (
    braking_speed,
    clean_ranges,
    extend_disparities,
    fit_wall,
    free_distance_ahead,
    gap_target,
    largest_gap,
    wall_target,
)

N, ANGLE_MIN, ANGLE_MAX = 1080, -2.35, 2.35  # Hokuyo UST-10LX, 1080 rays +/- 135 degrees
ANGLES = np.linspace(ANGLE_MIN, ANGLE_MAX, N)
INC = float(ANGLES[1] - ANGLES[0])
FAR = 1000.0  # a wall this long is a straight wall for the car


def ray_cast(x, y, yaw, walls):
    """Ranges from a lidar at (x, y, yaw) to wall segments ((x1, y1), (x2, y2)); inf = no hit."""
    dx, dy = np.cos(yaw + ANGLES), np.sin(yaw + ANGLES)
    ranges = np.full(N, np.inf)
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        wx, wy = x1 - x, y1 - y
        cross = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (wx * ey - wy * ex) / cross
            u = (wx * dy - wy * dx) / cross
        hit = (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    return ranges


def corridor(left=1.0, right=-1.5):
    return [((-FAR, left), (FAR, left)), ((-FAR, right), (FAR, right))]


def test_nan_and_inf_become_max_range():
    """NaN and inf mean no hit: clear all the way to max_range, not an obstacle."""
    out = clean_ranges(np.array([1.0, float('nan'), float('inf'), -1.0]), max_range=3.0)
    assert out.tolist() == pytest.approx([1.0, 3.0, 3.0, 0.0])


def test_free_distance_ignores_side_walls_but_sees_the_lane():
    ranges = clean_ranges(ray_cast(0.0, 0.0, 0.0, corridor()), 3.0)
    assert free_distance_ahead(ranges, ANGLES, 0.25, 3.0) == pytest.approx(3.0, abs=0.02)

    box = corridor() + [((1.2, -0.1), (1.2, 0.1))]
    ranges = clean_ranges(ray_cast(0.0, 0.0, 0.0, box), 3.0)
    assert free_distance_ahead(ranges, ANGLES, 0.25, 3.0) == pytest.approx(1.2, abs=1e-3)


def test_braking_speed():
    assert braking_speed(0.4, stop_distance=0.5, max_decel=2.0, max_speed=2.0) == 0.0
    assert braking_speed(0.75, 0.5, 2.0, 2.0) == pytest.approx(1.0)  # sqrt(2 * 2 * 0.25)
    assert braking_speed(10.0, 0.5, 2.0, 2.0) == 2.0


def test_disparities_widen_the_near_side_by_half_the_car():
    ranges = np.full(N, 3.0, dtype=np.float32)
    ranges[500:580] = 1.0
    out = extend_disparities(ranges, INC, half_width=0.25)
    n = math.ceil(math.asin(0.25 / 1.0) / INC)
    assert np.all(out[500 - n:580 + n] == 1.0)
    assert out[500 - n - 1] == 3.0 and out[580 + n] == 3.0


def test_largest_gap_picks_the_widest_run():
    free = np.zeros(40, dtype=bool)
    free[0:5] = free[15:28] = free[35:40] = True
    assert largest_gap(free) == (15, 28)


def test_gap_target_aims_through_the_middle_of_a_doorway():
    ranges = np.full(N, 1.0, dtype=np.float32)
    ranges[np.abs(ANGLES - math.radians(20.0)) < math.radians(10.0)] = 3.0
    x, y = gap_target(ranges, ANGLES)
    assert math.degrees(math.atan2(y, x)) == pytest.approx(20.0, abs=0.5)


def test_gap_target_never_aims_between_two_deep_runs():
    """Two openings: aim through the wider one, not at the post between them."""
    ranges = np.full(N, 1.0, dtype=np.float32)
    ranges[np.abs(ANGLES - math.radians(-30.0)) < math.radians(5.0)] = 3.0
    ranges[np.abs(ANGLES - math.radians(30.0)) < math.radians(10.0)] = 3.0
    x, y = gap_target(ranges, ANGLES)
    assert math.degrees(math.atan2(y, x)) == pytest.approx(30.0, abs=0.5)


def test_fit_wall_recovers_distance_and_angle():
    yaw = 0.1  # car turned 0.1 rad towards the wall
    ranges = clean_ranges(ray_cast(0.0, 0.0, yaw, [((-FAR, 1.0), (FAR, 1.0))]), 3.0)
    foot, direction = fit_wall(ranges, ANGLES, 'left', 3.0)
    assert np.linalg.norm(foot) == pytest.approx(1.0, abs=1e-3)
    assert math.atan2(direction[1], direction[0]) == pytest.approx(-yaw, abs=1e-3)


def test_fit_wall_finds_nothing_in_the_open_or_on_the_other_side():
    open_ground = np.full(N, 3.0, dtype=np.float32)
    assert fit_wall(open_ground, ANGLES, 'left', 3.0) is None

    ranges = clean_ranges(ray_cast(0.0, 0.0, 0.0, [((-FAR, -1.0), (FAR, -1.0))]), 3.0)
    assert fit_wall(ranges, ANGLES, 'left', 3.0) is None
    assert fit_wall(ranges, ANGLES, 'right', 3.0) is not None


def test_a_wall_straight_ahead_is_not_one_to_follow():
    ranges = clean_ranges(ray_cast(0.0, 0.0, 0.0, [((1.0, -FAR), (1.0, FAR))]), 3.0)
    assert fit_wall(ranges, ANGLES, 'left', 3.0) is None


def test_wall_target_sits_on_the_offset_line_ahead():
    target = wall_target(np.array([0.0, 1.0]), np.array([1.0, 0.0]), distance=0.7, lookahead=1.0)
    assert target == pytest.approx([1.0, 0.3])


@pytest.fixture
def node(ros):
    n = Follow()
    yield n
    n.destroy_node()


def make_scan(ranges):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min, scan.angle_max, scan.angle_increment = ANGLE_MIN, ANGLE_MAX, INC
    scan.range_min, scan.range_max = 0.0, 30.0
    scan.ranges = [float(r) for r in ranges]
    return scan


def drive(node, walls, x, y, yaw, steps, dt=0.025):
    """Kinematic bicycle at the rear axle, steering saturated like the firmware's."""
    trace = []
    for _ in range(steps):
        lx, ly = x + node.laser_x * math.cos(yaw), y + node.laser_x * math.sin(yaw)
        steer, speed, _ = node.command(make_scan(ray_cast(lx, ly, yaw, walls)))
        steer = float(np.clip(steer, -0.36, 0.36))
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        yaw += speed / node.wheelbase * math.tan(steer) * dt
        trace.append((x, y, yaw, speed, node.behavior))
    return trace


def test_holds_the_wall_distance_in_a_corridor(node):
    """Starts 0.3 m too far from the left wall and settles on wall_distance."""
    trace = drive(node, corridor(left=1.0), x=0.0, y=0.0, yaw=0.0, steps=400)
    ys = np.array([t[1] for t in trace[-100:]])
    assert all(t[4] == 'wall' for t in trace)
    assert 1.0 - ys.mean() == pytest.approx(node.wall_distance, abs=0.05)
    assert ys.std() < 0.01, 'still weaving'


def test_obstacle_in_the_lane_hands_over_to_gap_follow(node):
    box = corridor(left=1.0) + [((1.0, -0.2), (1.0, 0.2))]
    steer, speed, _ = node.command(make_scan(ray_cast(0.0, 0.0, 0.0, box)))
    assert node.behavior == 'gap'
    assert speed < node.max_speed


def test_stops_short_of_an_obstacle_inside_stop_distance(node):
    box = corridor(left=1.0) + [((0.4, -1.5), (0.4, 1.0))]
    _, speed, _ = node.command(make_scan(ray_cast(0.0, 0.0, 0.0, box)))
    assert speed == 0.0


def test_turns_away_from_a_wall_ahead_in_the_open(node):
    """No wall to follow, a wall across the way: it must turn, never touch it."""
    wall = [((3.0, -FAR), (3.0, FAR))]
    trace = drive(node, wall, x=0.0, y=0.0, yaw=0.0, steps=400)
    nose = max(x + node.laser_x * math.cos(yaw) for x, _, yaw, _, _ in trace)
    assert nose < 3.0 - node.half_width


def one_sided_wall(wall_deg, wall=0.5, far=3.0, width_deg=40.0):
    """Open space everywhere except a wall on one side, `width_deg` wide."""
    ranges = np.full(N, far)
    ranges[np.abs(ANGLES - math.radians(wall_deg)) < math.radians(width_deg)] = wall
    return make_scan(ranges)


class Harness(Node):
    """Plays scan_filter_node and a consumer of /follow/drive."""

    def __init__(self):
        super().__init__('follow_harness')
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.drives = []
        self.create_subscription(AckermannDriveStamped, '/follow/drive', self.drives.append, 10)


@pytest.fixture
def rig(node):
    harness = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(harness)
    yield node, harness, ex
    ex.remove_node(node)
    ex.remove_node(harness)
    ex.shutdown()
    harness.destroy_node()


def drive_once(rig, scan, timeout=5.0):
    """Publish until a drive command comes back, and return it."""
    _, harness, ex = rig
    harness.drives.clear()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        harness.scan_pub.publish(scan)
        ex.spin_once(timeout_sec=0.05)
        if harness.drives:
            return harness.drives[-1]
    raise AssertionError('node did not respond within the timeout')


def test_steers_away_from_a_wall_on_the_right(rig):
    """A wall to the right (negative angle) should steer left (positive)."""
    out = drive_once(rig, one_sided_wall(wall_deg=-60.0))
    assert out.drive.steering_angle > 0.0
