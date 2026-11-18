#!/usr/bin/env python3
"""Monte Carlo localization: a map-frame pose from the lidar scan and odometry.

Particles are base_frame poses: the frame odometry moves, and the one the
"2D Pose Estimate" tool sets. Every ray is cast from the lidar, offset by its
mount on base_frame as read from TF.

Publishes pf/pose/odom (map frame) for map_odom_broadcaster_node, which turns
it into the map -> odom transform.

Adapted from Corey Walsh's particle_filter for the MIT RACECAR (MIT license):
the same sensor and motion models, on the same author's RangeLibc.
"""

from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from rc_localization.conversions import to_pose2d, to_quaternion
from rc_localization.lib.frame_math import compose, Pose2D
from rc_localization.lib.motion_model import MotionDispersion, odometry_delta
from rc_localization.lib.occupancy_map import MapInfo, permissible_cells
from rc_localization.lib.particle_filter import MonteCarloLocalizer
from rc_localization.lib.sensor_model import (
    build_ray_caster,
    evaluate,
    precompute_sensor_model_table,
    SensorModelParams,
)


class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        self.declare_parameters('', [
            ('scan_topic', 'scan'),
            ('odom_topic', 'odom'),
            ('angle_step', 18),             # cast every n-th beam of the scan
            ('max_particles', 4000),
            ('max_viz_particles', 60),
            ('squash_factor', 2.2),
            ('max_range', 10.0),            # [m] the lidar's
            # Sensor model: how likely a short, max-range, random and correct reading is.
            ('z_short', 0.01),
            ('z_max', 0.07),
            ('z_rand', 0.12),
            ('z_hit', 0.75),
            ('sigma_hit', 8.0),
            ('motion_dispersion_x', 0.05),
            ('motion_dispersion_y', 0.025),
            ('motion_dispersion_theta', 0.25),
            ('publish_odom', True),
            ('publish_tf', True),           # false when map_odom_broadcaster_node owns the TF
            ('viz', True),
            ('map_frame', 'map'),
            ('base_frame', 'base_link'),
        ])
        p = self.get_parameter
        self.angle_step = p('angle_step').value
        self.max_viz_particles = p('max_viz_particles').value
        self.squash_exponent = 1.0 / p('squash_factor').value
        self.max_range = p('max_range').value
        self.publish_tf = p('publish_tf').value
        self.viz = p('viz').value
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value

        map_msg = self._fetch_map()
        info = map_msg.info
        self.map_info = MapInfo(
            info.resolution, to_pose2d(info.origin.position, info.origin.orientation)
        )
        self.permissible_cells = permissible_cells(map_msg.data, info.width, info.height)

        max_range_px = int(self.max_range / info.resolution)
        self.ray_caster = build_ray_caster(map_msg, max_range_px)
        self.ray_caster.set_sensor_model(precompute_sensor_model_table(SensorModelParams(
            z_short=p('z_short').value,
            z_max=p('z_max').value,
            z_rand=p('z_rand').value,
            z_hit=p('z_hit').value,
            sigma_hit=p('sigma_hit').value,
            max_range_px=max_range_px,
        )))
        self.get_logger().info('map loaded')

        self._mount = None  # base_frame -> lidar, set once TF has it
        self._scan_frame = None
        self._angles = None  # set by the first scan, with the two buffers below
        self._queries = None
        self._ranges = None
        self._observed_ranges = None
        self._last_odom_pose = None
        self._last_stamp = None
        self._current_speed = 0.0

        dispersion = MotionDispersion(
            x=p('motion_dispersion_x').value,
            y=p('motion_dispersion_y').value,
            theta=p('motion_dispersion_theta').value,
        )
        self.localizer = MonteCarloLocalizer(
            p('max_particles').value, dispersion, self._evaluate_sensor_model
        )
        self.localizer.initialize_global(self.permissible_cells, self.map_info)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = None
        if p('publish_odom').value:
            self.odom_pub = self.create_publisher(Odometry, 'pf/pose/odom', 1)
        self.pose_pub = self.create_publisher(PoseStamped, 'pf/viz/inferred_pose', 1)
        self.particle_pub = self.create_publisher(PoseArray, 'pf/viz/particles', 1)
        self.fake_scan_pub = self.create_publisher(LaserScan, 'pf/viz/fake_scan', 1)

        self.create_subscription(
            LaserScan, p('scan_topic').value, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, p('odom_topic').value, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._initial_pose_callback, 1
        )
        self.create_subscription(PointStamped, 'clicked_point', self._clicked_point_callback, 1)

        self.get_logger().info('waiting for scan and odometry')

    def _fetch_map(self):
        client = self.create_client(GetMap, 'map_server/map')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for the map_server/map service')
        future = client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def _evaluate_sensor_model(
        self, particles: np.ndarray, observed_ranges: np.ndarray
    ) -> np.ndarray:
        return evaluate(
            self.ray_caster,
            particles,
            self._mount,
            self._angles,
            observed_ranges,
            self.squash_exponent,
            self._queries,
            self._ranges,
        )

    def _lookup_mount(self, laser_frame: str):
        """The lidar's fixed mount on base_frame, from the URDF; None until TF has it."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, laser_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'waiting for {self.base_frame}->{laser_frame}: {e}', throttle_duration_sec=2.0
            )
            return None
        return to_pose2d(tf.transform.translation, tf.transform.rotation)

    def _scan_callback(self, msg: LaserScan) -> None:
        if self._mount is None:
            self._mount = self._lookup_mount(msg.header.frame_id)
            if self._mount is None:
                return
            self._scan_frame = msg.header.frame_id
        if self._angles is None:
            self.get_logger().info('received the first scan')
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self._angles = angles[::self.angle_step].astype(np.float32)
            particles = self.localizer.particle_count
            self._queries = np.zeros((particles, 3), dtype=np.float32)
            self._ranges = np.zeros(particles * len(self._angles), dtype=np.float32)
        # NaN is urg_node2's "no echo": to the sensor model, a max-range reading.
        self._observed_ranges = np.nan_to_num(
            np.array(msg.ranges[::self.angle_step], dtype=np.float32),
            nan=self.max_range, posinf=self.max_range, neginf=0.0,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = to_pose2d(msg.pose.pose.position, msg.pose.pose.orientation)
        self._current_speed = msg.twist.twist.linear.x
        self._last_stamp = msg.header.stamp

        if self._last_odom_pose is None:
            self.get_logger().info('received the first odometry')
        elif self._observed_ranges is not None:
            delta = odometry_delta(self._last_odom_pose, pose)
            self._publish(self.localizer.update(delta, self._observed_ranges))
        self._last_odom_pose = pose

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = to_pose2d(msg.pose.pose.position, msg.pose.pose.orientation)
        self.get_logger().info(f'initializing the pose at {pose.x:.2f}, {pose.y:.2f}')
        self.localizer.initialize_pose(pose)

    def _clicked_point_callback(self, _msg: PointStamped) -> None:
        self.get_logger().info('initializing the particles globally')
        self.localizer.initialize_global(self.permissible_cells, self.map_info)

    def _publish(self, pose: Pose2D) -> None:
        stamp = self._last_stamp

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = stamp, self.map_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            tf.transform.rotation = to_quaternion(pose.yaw)
            self.tf_broadcaster.sendTransform(tf)

        if self.odom_pub is not None:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, self.map_frame
            odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
            odom.pose.pose.orientation = to_quaternion(pose.yaw)
            odom.twist.twist.linear.x = self._current_speed
            self.odom_pub.publish(odom)

        if self.viz:
            self._visualize(pose, stamp)

    def _visualize(self, pose: Pose2D, stamp) -> None:
        """The pose, a sample of the particles and the scan the pose predicts, if watched."""
        if self.pose_pub.get_subscription_count() > 0:
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp, self.map_frame
            msg.pose.position.x, msg.pose.position.y = pose.x, pose.y
            msg.pose.orientation = to_quaternion(pose.yaw)
            self.pose_pub.publish(msg)

        if self.particle_pub.get_subscription_count() > 0:
            particles = self.localizer.particles
            if particles.shape[0] > self.max_viz_particles:
                shown = np.random.choice(
                    particles.shape[0], self.max_viz_particles, p=self.localizer.weights
                )
                particles = particles[shown]
            msg = PoseArray()
            msg.header.stamp, msg.header.frame_id = stamp, self.map_frame
            msg.poses = [
                Pose(position=Point(x=float(x), y=float(y)), orientation=to_quaternion(float(yaw)))
                for x, y, yaw in particles
            ]
            self.particle_pub.publish(msg)

        if self.fake_scan_pub.get_subscription_count() > 0 and self._angles is not None:
            lidar = compose(pose, self._mount)
            queries = np.zeros((self._angles.shape[0], 3), dtype=np.float32)
            queries[:, 0], queries[:, 1] = lidar.x, lidar.y
            queries[:, 2] = self._angles + lidar.yaw
            ranges = np.zeros(self._angles.shape[0], dtype=np.float32)
            self.ray_caster.calc_range_many(queries, ranges)
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp, self._scan_frame
            msg.angle_min, msg.angle_max = float(self._angles[0]), float(self._angles[-1])
            msg.angle_increment = float(abs(self._angles[1] - self._angles[0]))
            msg.range_min, msg.range_max = 0.0, float(np.max(ranges))
            msg.ranges = ranges.tolist()
            self.fake_scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
