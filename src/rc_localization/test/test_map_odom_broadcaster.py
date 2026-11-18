"""Tests for map_odom_broadcaster_node over real topics."""

import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from rc_localization.map_odom_broadcaster_node import MapOdomBroadcaster

STAMP = Time(seconds=100.0).to_msg()


class Harness(Node):
    """Plays rf2o and the particle filter, and listens for map -> odom."""

    def __init__(self):
        super().__init__('map_odom_harness')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Odometry, 'pf/pose/odom', 1)
        self.corrections = []
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)

    def _on_tf(self, msg: TFMessage) -> None:
        self.corrections += [t for t in msg.transforms if t.header.frame_id == 'map']

    def publish_estimate(self, x: float) -> None:
        pose = Odometry()
        pose.header.stamp = STAMP
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        self.pose_pub.publish(pose)

    def publish_odometry(self, x: float) -> None:
        tf = TransformStamped()
        tf.header.stamp = STAMP
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.transform.translation.x = x
        self.tf_broadcaster.sendTransform(tf)


@pytest.fixture
def rig(ros):
    node = MapOdomBroadcaster()
    node.transform_timeout = Duration(seconds=5.0)  # the test must not race it
    harness = Harness()
    executor = MultiThreadedExecutor(num_threads=2)  # as main() runs the node
    executor.add_node(node)
    executor.add_node(harness)
    yield harness, executor
    executor.shutdown()
    harness.destroy_node()
    node.destroy_node()


def spin_until(executor, done, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    return done()


def test_an_estimate_that_arrives_before_its_odometry_is_still_broadcast(rig):
    """The wait for TF only works if TF can arrive while the callback waits."""
    harness, executor = rig
    connected = spin_until(
        executor,
        lambda: harness.pose_pub.get_subscription_count() == 1
        and harness.count_subscribers('/tf') == 2,
    )
    assert connected

    harness.publish_estimate(x=2.0)
    spin_until(executor, lambda: False, timeout=0.3)  # the estimate is in, and waiting
    assert not harness.corrections

    harness.publish_odometry(x=0.5)
    assert spin_until(executor, lambda: harness.corrections)
    correction = harness.corrections[0]
    assert correction.child_frame_id == 'odom'
    assert correction.transform.translation.x == pytest.approx(1.5)
