"""Tests for drive_mux_node: resolve_mode on its own, then the node over real topics."""

import time

from ackermann_msgs.msg import AckermannDriveStamped
import pytest
from rc_msgs.msg import BehaviorMode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rc_control.drive_mux_node import DriveMux, resolve_mode

MODE_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


def test_resolve_mode_passes_through_a_valid_mode():
    mode = resolve_mode('pure_pursuit', {'follow', 'pure_pursuit'}, 'follow')
    assert mode == 'pure_pursuit'


def test_resolve_mode_falls_back_on_unknown_mode():
    assert resolve_mode('bogus', {'follow', 'pure_pursuit'}, 'follow') == 'follow'


@pytest.fixture
def node(ros):
    n = DriveMux()
    yield n
    n.destroy_node()


def make_drive(speed=1.0, steer=0.1, frame_id='laser'):
    msg = AckermannDriveStamped()
    msg.header.frame_id = frame_id
    msg.drive.speed = speed
    msg.drive.steering_angle = steer
    return msg


class Harness(Node):
    """Plays the candidate controllers and a consumer of /drive_cmd."""

    def __init__(self):
        super().__init__('drive_mux_harness')
        self.follow_pub = self.create_publisher(AckermannDriveStamped, '/follow/drive', 10)
        self.pure_pursuit_pub = self.create_publisher(
            AckermannDriveStamped, '/pure_pursuit/drive', 10
        )
        self.behavior_mode_pub = self.create_publisher(BehaviorMode, '/behavior_mode', MODE_QOS)
        self.drive_cmds = []
        self.create_subscription(AckermannDriveStamped, '/drive_cmd', self.drive_cmds.append, 10)


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


def publish_and_wait(rig, publisher, msg, timeout=5.0):
    """Publish until a /drive_cmd comes back or the timeout passes; return what arrived."""
    _, harness, ex = rig
    harness.drive_cmds.clear()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publisher.publish(msg)
        ex.spin_once(timeout_sec=0.05)
        if harness.drive_cmds:
            break
    return harness.drive_cmds


def test_default_mode_is_follow_at_startup(node):
    assert node.active_mode == 'follow'


def test_active_mode_is_relayed_to_drive_cmd(rig):
    node, harness, ex = rig
    node.active_mode = 'follow'
    out = publish_and_wait(rig, harness.follow_pub, make_drive(speed=2.5, steer=0.2))
    assert len(out) == 1
    assert out[0].drive.speed == pytest.approx(2.5)


def test_inactive_mode_is_not_relayed(rig):
    node, harness, ex = rig
    node.active_mode = 'follow'
    out = publish_and_wait(rig, harness.pure_pursuit_pub, make_drive(speed=9.0), timeout=1.0)
    assert len(out) == 0


def test_behavior_mode_message_switches_the_active_source(rig):
    node, harness, ex = rig
    node.active_mode = 'follow'

    msg = BehaviorMode()
    msg.mode = 'pure_pursuit'
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and node.active_mode != 'pure_pursuit':
        harness.behavior_mode_pub.publish(msg)
        ex.spin_once(timeout_sec=0.05)
    assert node.active_mode == 'pure_pursuit'
