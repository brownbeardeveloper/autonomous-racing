"""Tests for scan_filter_node: filter() on its own, then once over real topics."""

import time

import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rc_perception.scan_filter_node import ScanFilter

N = 1080  # beams of a Hokuyo UST-10LX


def make_scan(ranges, range_min=0.0, range_max=30.0):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min, scan.angle_max = -2.35, 2.35
    scan.angle_increment = (scan.angle_max - scan.angle_min) / max(len(ranges) - 1, 1)
    scan.range_min, scan.range_max = range_min, range_max
    scan.ranges = [float(r) for r in ranges]
    return scan


@pytest.fixture
def node(ros):
    n = ScanFilter()
    yield n
    n.destroy_node()


def test_nan_becomes_range_max(node):
    """NaN means no echo, e.g. a hallway longer than the lidar's reach: no hit."""
    scan = make_scan([float('nan')], range_min=0.1, range_max=10.0)
    assert node.filter(scan)[0] == pytest.approx(10.0)


def test_inf_becomes_range_max(node):
    scan = make_scan([float('inf')], range_min=0.1, range_max=10.0)
    assert node.filter(scan)[0] == pytest.approx(10.0)


def test_below_range_min_stays_very_close(node):
    scan = make_scan([0.01], range_min=0.1, range_max=10.0)
    assert node.filter(scan)[0] == pytest.approx(0.1)


def test_in_range_value_is_unchanged(node):
    scan = make_scan([3.0], range_min=0.0, range_max=10.0)
    assert node.filter(scan)[0] == pytest.approx(3.0)


def test_output_length_matches_input(node):
    assert len(node.filter(make_scan([1.0] * 200))) == 200


class Harness(Node):
    """Plays the lidar and a consumer: publishes /scan, listens on /scan_filtered."""

    def __init__(self):
        super().__init__('scan_filter_harness')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.filtered = []
        self.create_subscription(
            LaserScan, '/scan_filtered', self.filtered.append, qos_profile_sensor_data
        )


@pytest.fixture
def rig(node):
    harness = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    yield harness, executor
    executor.shutdown()
    harness.destroy_node()


def filter_once(rig, scan, timeout=5.0):
    """Publish until a filtered scan comes back, and return it."""
    harness, executor = rig
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        harness.scan_pub.publish(scan)
        executor.spin_once(timeout_sec=0.05)
        if harness.filtered:
            return harness.filtered[-1]
    raise AssertionError('node did not respond within the timeout')


def test_nan_and_inf_are_cleaned_end_to_end(rig):
    scan = make_scan([float('nan'), float('inf')] + [1.0] * (N - 2))
    out = filter_once(rig, scan)
    assert out.ranges[:3] == pytest.approx([30.0, 30.0, 1.0])
