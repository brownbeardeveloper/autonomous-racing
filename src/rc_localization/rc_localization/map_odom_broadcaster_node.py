#!/usr/bin/env python3
"""Publish map -> odom from the particle filter's map-frame pose estimate.

The filter estimates base_link in the map frame. Published as map -> base_link,
that would give base_link a second parent next to rf2o's odom -> base_link, and
tf2 allows one (REP 105). So this node publishes the correction instead:
map -> odom, which composed with the live odom -> base_link reproduces the
estimate.

The transform is post-dated by transform_tolerance_s, as AMCL does. map -> odom
drifts slowly, so a lookup of map -> base_link can then resolve at the newest
odometry instead of the last filter update. It also bounds how long a dead
filter's last estimate stays on TF.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from rc_localization.conversions import to_pose2d, to_quaternion
from rc_localization.lib.frame_math import map_to_odom


class MapOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('map_odom_broadcaster_node')

        self.declare_parameters('', [
            ('pose_topic', 'pf/pose/odom'),
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('transform_timeout_s', 0.05),
            ('transform_tolerance_s', 0.1),
        ])
        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.transform_timeout = Duration(seconds=p('transform_timeout_s').value)
        self.transform_tolerance = Duration(seconds=p('transform_tolerance_s').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, p('pose_topic').value, self._pose_callback, 1)

    def _pose_callback(self, msg: Odometry) -> None:
        # At the estimate's own stamp: newer odometry would fold the motion
        # since then into the correction.
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp, timeout=self.transform_timeout
            )
        except TransformException as e:
            self.get_logger().warn(
                f'no {self.odom_frame}->{self.base_frame} transform: {e}',
                throttle_duration_sec=2.0,
            )
            return

        map_to_base = to_pose2d(msg.pose.pose.position, msg.pose.pose.orientation)
        odom_to_base = to_pose2d(tf.transform.translation, tf.transform.rotation)
        correction = map_to_odom(map_to_base, odom_to_base)

        out = TransformStamped()
        out.header.stamp = (stamp + self.transform_tolerance).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x, out.transform.translation.y = correction.x, correction.y
        out.transform.rotation = to_quaternion(correction.yaw)
        self.tf_broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomBroadcaster()
    # _pose_callback waits for TF, which needs a second thread to arrive on.
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        rclpy.spin(node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
