"""Tests for safety_node.

They call the callbacks directly and replace the publishers with list appends,
which keeps them fast and free of DDS timing.
"""

import math

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import pytest
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from rc_safety.safety_node import Safety

N = 1080
ANGLE_MIN, ANGLE_MAX = -2.35, 2.35
ANGLES = np.linspace(ANGLE_MIN, ANGLE_MAX, N)
SPEED = 1.8  # [m/s]


@pytest.fixture
def node(ros):
    n = Safety()
    # The e-stop starts active. Clear it, so the TTC tests test only TTC.
    n.estop_callback(Bool(data=False))
    yield n
    n.destroy_node()


class Recorder:
    """Collects what the node publishes, without going through DDS."""

    def __init__(self):
        self.drives = []
        self.bools = []

    @property
    def braked(self):
        return len(self.drives) > 0

    def clear(self):
        self.drives.clear()
        self.bools.clear()


@pytest.fixture
def rec(node):
    r = Recorder()
    node.drive_pub.publish = r.drives.append
    node.bool_pub.publish = r.bools.append
    return r


def make_scan(ranges):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min, scan.angle_max = ANGLE_MIN, ANGLE_MAX
    scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / (N - 1)
    scan.range_min, scan.range_max = 0.05, 30.0
    scan.ranges = [float(x) for x in np.asarray(ranges, dtype=float).reshape(N)]
    return scan


def sector(default, lo_deg, hi_deg, value):
    """An obstacle in one angular sector, everything else clear."""
    r = np.full(N, default)
    r[(ANGLES >= math.radians(lo_deg)) & (ANGLES <= math.radians(hi_deg))] = value
    return make_scan(r)


def corridor(half_width, wall_ahead=np.inf):
    """A ray-cast hallway: two side walls and an end wall."""
    c, s = np.cos(ANGLES), np.sin(ANGLES)
    with np.errstate(divide='ignore', invalid='ignore'):
        to_side = np.where(np.abs(s) > 1e-9, half_width / np.abs(s), np.inf)
        to_end = np.where(c > 1e-9, wall_ahead / c, np.inf)
    return make_scan(np.minimum(to_side, to_end))


def feed(node, scan, speed):
    """One scan at this speed, with fresh odometry and drive_cmd."""
    node.speed = float(speed)
    node._odom_stamp = node._drive_cmd_stamp = node.get_clock().now()
    node.scan_callback(scan)


def make_drive(speed=1.0, steer=0.2):
    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steer
    return msg


def age(node, stamp_attr, seconds):
    setattr(node, stamp_attr, node.get_clock().now() - Duration(seconds=seconds))


def test_a_closing_obstacle_inside_ttc_threshold_triggers_the_brake(node, rec):
    feed(node, sector(np.inf, -30, 30, 0.5), SPEED)
    assert rec.braked


def test_driving_away_from_a_wall_never_brakes(node, rec):
    """Reversing: the swept path leads away from the wall ahead."""
    feed(node, sector(np.inf, -30, 30, 0.4), -SPEED)
    assert not rec.braked


@pytest.mark.parametrize('steering, brakes', [(0.0, True), (0.36, False)])
def test_steering_away_from_an_obstacle_does_not_brake(node, rec, steering, brakes):
    """Ahead and to the right: in the way going straight, clear on full left lock."""
    node.drive_cmd_callback(make_drive(speed=SPEED, steer=steering))
    rec.clear()
    feed(node, sector(np.inf, -25, -5, 0.8), SPEED)
    assert rec.braked == brakes


@pytest.mark.parametrize('steering, brakes', [(0.0, False), (0.36, True)])
def test_steering_into_an_obstacle_beside_the_car_brakes(node, rec, steering, brakes):
    """Ahead and to the left, outside a straight path, inside the arc of full left lock."""
    node.drive_cmd_callback(make_drive(speed=SPEED, steer=steering))
    rec.clear()
    feed(node, sector(np.inf, 40, 70, 0.6), SPEED)
    assert rec.braked == brakes


def test_safety_parameters_are_read_only(node):
    """A running car keeps its limits: no ros2 param set or Foxglove client loosens them."""
    result = node.set_parameters([Parameter('use_estop', value=False)])[0]
    assert not result.successful
    assert node.get_parameter('use_estop').value is True


def test_missing_odometry_engages_the_brake(node, rec):
    """Without a speed there is no time to collision, and unknown must mean stop."""
    node._odom_stamp = None
    node.scan_callback(make_scan(np.full(N, 10.0)))
    assert rec.braked


def test_drive_cmd_is_relayed_when_not_braking(node, rec):
    node.drive_cmd_callback(make_drive(speed=2.5, steer=0.3))
    assert rec.drives[-1].drive.speed == pytest.approx(2.5)
    assert rec.drives[-1].drive.steering_angle == pytest.approx(0.3)


def test_brake_command_is_exactly_zero(node, rec):
    """The simulator reads 0.0 as a stop and 0.001 as a slow drive into the wall."""
    feed(node, corridor(1.5, wall_ahead=0.5), SPEED)
    assert rec.drives[-1].drive.speed == 0.0
    assert rec.drives[-1].drive.steering_angle == 0.0


def test_node_starts_with_the_estop_active(ros):
    n = Safety()
    try:
        assert n._estop_is_active()
    finally:
        n.destroy_node()


def test_estop_brakes_on_the_timer_without_any_scan(node, rec):
    node.estop_callback(Bool(data=True))
    node.check_callback()
    assert rec.drives[-1].drive.speed == 0.0
    assert rec.drives[-1].drive.steering_angle == 0.0


def test_quiet_estop_topic_brakes(node, rec):
    node._scan_stamp = node.get_clock().now()
    age(node, '_estop_stamp', 1.0)
    node.check_callback()
    assert rec.braked


def test_drive_cmd_is_not_relayed_while_estopped(node, rec):
    node.estop_callback(Bool(data=True))
    node.drive_cmd_callback(make_drive(speed=1.5))
    assert not rec.braked


def test_estop_holds_the_brake_on_scans_even_when_the_car_is_stopped(node, rec):
    node.estop_callback(Bool(data=True))
    feed(node, make_scan(np.full(N, 10.0)), 0.0)
    node._brake_since -= 10.0  # well past min_hold_s
    rec.clear()
    feed(node, make_scan(np.full(N, 10.0)), 0.0)
    assert rec.drives[-1].drive.speed == 0.0
    assert node._braking


def test_clearing_the_estop_releases_and_relays_again(node, rec):
    node.estop_callback(Bool(data=True))
    feed(node, make_scan(np.full(N, 10.0)), 0.0)
    node._brake_since -= 10.0
    node.estop_callback(Bool(data=False))
    feed(node, make_scan(np.full(N, 10.0)), 0.0)
    assert not node._braking
    rec.clear()
    node.drive_cmd_callback(make_drive(speed=1.5))
    assert rec.drives[-1].drive.speed == pytest.approx(1.5)


def test_stale_scan_brakes_on_the_timer(node, rec):
    age(node, '_scan_stamp', 1.0)
    node.check_callback()
    assert rec.braked
    rec.clear()
    node.drive_cmd_callback(make_drive(speed=1.5))
    assert not rec.braked


def test_fresh_scan_and_clear_estop_do_not_brake_on_the_timer(node, rec):
    node._scan_stamp = node.get_clock().now()
    node.check_callback()
    assert not rec.braked


def test_use_estop_false_ignores_a_silent_estop(node, rec):
    node.use_estop = False
    node._estop_stamp = None
    node.drive_cmd_callback(make_drive(speed=1.5))
    assert rec.drives[-1].drive.speed == pytest.approx(1.5)
