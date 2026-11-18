#!/usr/bin/env python3
"""Pure pursuit on a closed loop of waypoints loaded from CSV.

Three things differ from the textbook controller; pure_pursuit.md has the why:

* The pose comes from TF (map -> rear_axle), not from the odometry message:
  the raceline lives in the map frame, and odom drifts away from it.
* Commands go out on a timer, and a stale pose brakes instead of going quiet.
* The target speed is a profile built once per path: the file's speed, capped
  by max_speed and by grip, then braked backwards round the whole loop.

With no waypoints_file the node idles at zero speed. A waypoints_file that
fails to load exits at startup, instead of running with no path.
"""

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from rc_control.lib.pure_pursuit import (
    compute_steering,
    cross_track_error,
    load_waypoints,
    lookahead_point_on_path,
    nearest_point_on_path,
    quaternion_to_yaw,
    sample_at,
    speed_profile,
)

# Retunable while driving. Topics, frames, the waypoints file and the control
# rate are bound at startup, and the wheelbase is a fact about the car.
LIVE_TUNABLE = {
    'max_steering_angle', 'lookahead_distance', 'lookahead_gain', 'lookahead_min',
    'lookahead_max', 'speed_gain', 'max_speed', 'min_speed', 'max_path_error',
    'max_lateral_accel', 'max_decel', 'steering_speed_factor', 'pose_timeout', 'publish_debug',
}


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameters('', [
            ('waypoints_file', ''),
            ('odom_topic', 'odom'),
            ('drive_topic', 'pure_pursuit/drive'),

            ('use_tf', True),                # pose from TF; False trusts the odom pose
            ('path_frame', 'map'),
            ('base_frame', 'rear_axle'),     # the bicycle model is only valid at the rear axle

            ('control_rate', 50.0),          # [Hz]
            ('pose_timeout', 0.3),           # [s] no fresh pose: brake

            ('wheelbase', 0.33),             # [m]
            ('max_steering_angle', 0.36),    # [rad] the servo's mechanical limit

            ('lookahead_distance', 1.0),     # [m] at standstill
            ('lookahead_gain', 0.3),         # [m] more per m/s
            ('lookahead_min', 0.5),
            ('lookahead_max', 3.0),

            ('speed_gain', 1.0),             # scales the file's speed
            # A planning cap, not an actuator guard: the braking pass has to aim
            # at a speed the car can reach. The firmware owns the real limit.
            ('max_speed', 4.0),              # [m/s] independent of speed_gain
            ('min_speed', 0.0),              # [m/s] floor once moving; 0 disables
            ('max_lateral_accel', 6.0),      # [m/s^2] grip the profile plans for; 0 disables
            ('max_decel', 3.0),              # [m/s^2] braking the profile plans for; 0 disables
            ('steering_speed_factor', 0.4),  # share of the speed kept at full lock; 1 disables

            ('search_window', 40),           # waypoints either side of the last match
            ('max_path_error', 2.0),         # [m] farther off than this: stop, don't steer back
            ('publish_debug', True),
        ])
        for name in LIVE_TUNABLE:
            setattr(self, name, self.get_parameter(name).value)
        p = self.get_parameter
        self.use_tf = p('use_tf').value
        self.path_frame = p('path_frame').value
        self.base_frame = p('base_frame').value
        self.wheelbase = p('wheelbase').value
        self.search_window = p('search_window').value

        self.path = None
        self._profile = None
        self._load_waypoints(p('waypoints_file').value)
        self._update_profile()

        self._pose = None       # (x, y, yaw) in path_frame
        self._pose_time = None
        self._speed = 0.0
        self._seed = None       # last matched segment, where the next search starts
        self._last_idx = None
        self._laps = 0

        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(AckermannDriveStamped, p('drive_topic').value, 10)
        self.create_subscription(Odometry, p('odom_topic').value, self.odom_callback, 10)

        # Transient local: Foxglove still gets the path when it connects late.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(PathMsg, '~/path', latched)
        self.target_pub = self.create_publisher(PointStamped, '~/lookahead_point', 10)
        self.diag_pub = self.create_publisher(Float32MultiArray, '~/diagnostics', 10)
        if self.publish_debug:
            self._publish_path()

        rate = max(1.0, p('control_rate').value)
        self.create_timer(1.0 / rate, self.control_step)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        pose_source = f'TF {self.path_frame}->{self.base_frame}' if self.use_tf else 'odom'
        self.get_logger().info(
            f'pose from {pose_source}, control at {rate:.0f} Hz, wheelbase {self.wheelbase} m, '
            f'lookahead {self.lookahead_min}-{self.lookahead_max} m'
        )

    def _load_waypoints(self, path: str) -> None:
        if not path:
            self.get_logger().warn('no waypoints_file configured - staying stopped')
            return
        try:
            self.path = load_waypoints(path)
        except (OSError, ValueError) as e:
            # Set but broken is a misconfiguration: fail loudly, don't idle looking healthy.
            self.get_logger().fatal(f"failed to load waypoints_file '{path}': {e}")
            raise SystemExit(1)

        if not np.any(self.path.v > 0):
            self.get_logger().warn('the file has no speed column - using max_speed everywhere')
            self.path.v[:] = self.max_speed
        v = self.path.v
        self.get_logger().info(
            f'loaded {len(self.path)} waypoints, {self.path.length:.2f} m loop, '
            f'speed {v.min():.2f}-{v.max():.2f} m/s'
        )

    def _update_profile(self) -> None:
        if self.path is not None:
            self._profile = speed_profile(
                self.path, self.speed_gain, self.max_speed, self.max_lateral_accel, self.max_decel
            )

    def _publish_path(self) -> None:
        if self.path is None:
            return
        msg = PathMsg()
        msg.header.frame_id = self.path_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path.xy:
            pose = PoseStamped(header=msg.header)
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self._speed = abs(float(msg.twist.twist.linear.x))
        if self.use_tf:
            return  # TF is the pose source; odom only gives the speed
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = (position.x, position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self._pose_time = self.get_clock().now()

    def _update_pose_from_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.path_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'no {self.path_frame}->{self.base_frame} transform: {e}',
                throttle_duration_sec=2.0,
            )
            return
        t, r = tf.transform.translation, tf.transform.rotation
        self._pose = (t.x, t.y, quaternion_to_yaw(r.x, r.y, r.z, r.w))
        # The transform's own stamp, not now(): TF keeps answering with a dead
        # localizer's last estimate, and only its age gives that away.
        self._pose_time = Time.from_msg(tf.header.stamp)

    def control_step(self) -> None:
        if self.use_tf:
            self._update_pose_from_tf()

        if self.path is None:
            self.get_logger().warn('no path loaded - staying stopped', throttle_duration_sec=5.0)
            return self.publish(0.0, 0.0)

        if self._pose is None:
            self.get_logger().warn('waiting for a pose', throttle_duration_sec=5.0)
            return self.publish(0.0, 0.0)

        age = (self.get_clock().now() - self._pose_time).nanoseconds * 1e-9
        if age > self.pose_timeout:
            # Braking beats coasting on a stale pose, and beats going quiet and
            # hoping safety_node times us out first.
            self.get_logger().error(f'pose {age:.2f} s stale - braking', throttle_duration_sec=1.0)
            self._seed = None
            return self.publish(0.0, 0.0)

        x, y, yaw = self._pose
        position = np.array([x, y])
        idx, t, nearest, dist = nearest_point_on_path(
            position, self.path, self._seed, self.search_window
        )

        if dist > self.max_path_error:
            # Steering back in blind from this far off is worse than stopping.
            # Without the seed, recovery starts with a global search.
            self.get_logger().warn(
                f'{dist:.2f} m off the path (limit {self.max_path_error} m) - stopping',
                throttle_duration_sec=1.0,
            )
            self._seed = None
            return self.publish(0.0, 0.0)

        if self._last_idx is not None and idx < self._last_idx - len(self.path) // 2:
            self._laps += 1
            self.get_logger().info(f'lap {self._laps}')
        self._last_idx = self._seed = idx

        lookahead = float(np.clip(
            self.lookahead_distance + self.lookahead_gain * self._speed,
            self.lookahead_min,
            self.lookahead_max,
        ))
        _, _, target = lookahead_point_on_path(position, self.path, idx, t, lookahead)
        steer = float(np.clip(
            compute_steering(position, yaw, target, self.wheelbase),
            -self.max_steering_angle,
            self.max_steering_angle,
        ))
        speed = self._target_speed(idx, t, steer)
        self.publish(steer, speed)

        if self.publish_debug:
            cte = cross_track_error(position, yaw, nearest)
            self._publish_debug(target, [cte, dist, lookahead, steer, speed, self._speed])

    def _target_speed(self, idx: int, t: float, steer: float) -> float:
        """The profile at this point of the path, eased off with the wheel turned."""
        v = sample_at(self._profile, idx, t)

        if self.steering_speed_factor < 1.0 and self.max_steering_angle > 0.0:
            # Apart from the profile's curvature cap: this one answers to
            # tracking error, not to the plan.
            frac = abs(steer) / self.max_steering_angle
            v *= 1.0 - (1.0 - self.steering_speed_factor) * frac

        v = max(0.0, v)
        if 0.0 < v < self.min_speed:
            v = self.min_speed  # not a crawl the motor cannot deliver
        return v

    def publish(self, steer: float, speed: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub.publish(msg)

    def _publish_debug(self, target, diagnostics) -> None:
        point = PointStamped()
        point.header.frame_id = self.path_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x, point.point.y = float(target[0]), float(target[1])
        self.target_pub.publish(point)
        # [cross-track error, distance to the path, lookahead, steer, commanded speed, speed]
        self.diag_pub.publish(Float32MultiArray(data=diagnostics))

    def _on_parameter_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name not in LIVE_TUNABLE:
                return SetParametersResult(
                    successful=False, reason=f"'{p.name}' can only be set at startup"
                )
        for p in params:
            setattr(self, p.name, p.value)
        self._update_profile()
        return SetParametersResult(successful=True)

    def brake_and_stop(self) -> None:
        """Send a zero command on the way out, if the context still lets us."""
        try:
            for _ in range(3):
                self.publish(0.0, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.brake_and_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
