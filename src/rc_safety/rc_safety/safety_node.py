#!/usr/bin/env python3
"""Automatic emergency braking on the time to collision along the car's path.

Every scan, the car's body is swept along the arc its commanded steering gives,
to the first scan points it would touch (collision.py); the time to collision
is that distance over the car's speed. Obstacles count as standing still and
are checked again on every scan, as in Nav2's Collision Monitor: anticipating
a moving one takes tracking, and on the outdoor bag every tracker-free
estimate of obstacle motion braked more often for nothing than it helped.

Relays drive_cmd to /drive untouched while the way is clear. It brakes instead
when a collision is near, when the e-stop is active or silent, and when the
scan, the odometry or drive_cmd goes stale. The brake - zero speed, zero
steering - is the only command this node writes itself; limits belong to the
backend on the other end of /drive (the ESP32 firmware, or the sim).
"""

import math

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from rc_safety.collision import Body, distance_to_collision, scan_points

# Read-only once the node runs, so no `ros2 param set` can loosen a limit on a
# moving car: change them in the YAML and relaunch.
PARAMETERS = [
    ('scan_topic', 'scan'),
    ('odom_topic', 'odom'),
    ('drive_cmd_topic', 'drive_cmd'),
    # Absolute: the ESP32 firmware and the sim bridge subscribe to exactly this name.
    ('drive_topic', '/drive'),
    ('ttc_threshold', 0.5),       # [s] brake below this
    ('ttc_release', 1.2),         # [s] release only above this
    ('min_hold_s', 0.5),          # [s] shortest time the brake stays on
    ('min_beams', 5),             # echoes in the path it takes: a wall, not one noisy echo
    ('min_speed', 0.3),           # [m/s] no collision braking below this
    # The car around its rear axle (base_link). The defaults are this car's.
    ('wheelbase', 0.33),          # [m]
    ('laser_x', 0.285),           # [m] lidar ahead of the rear axle
    ('body_front', 0.425),        # [m] front bumper ahead of the rear axle
    ('body_rear', 0.135),         # [m] rear bumper behind it
    ('body_half_width', 0.15),    # [m]
    ('margin', 0.05),             # [m] kept clear around the body
    ('odom_timeout_s', 0.5),
    ('brake_on_stale_odom', True),
    ('drive_cmd_timeout_s', 0.5),
    ('brake_on_stale_drive_cmd', True),
    ('scan_timeout_s', 0.5),
    ('use_estop', True),
    ('estop_timeout_s', 0.5),
    ('publish_debug', True),
]


class Safety(Node):
    def __init__(self):
        super().__init__('safety_node')

        # One descriptor each: rclpy writes the parameter's type into it.
        self.declare_parameters('', [
            (name, value, ParameterDescriptor(read_only=True)) for name, value in PARAMETERS
        ])
        for name, _ in PARAMETERS:
            setattr(self, name, self.get_parameter(name).value)
        self.body = Body(
            self.body_front + self.margin,
            self.body_rear + self.margin,
            self.body_half_width + self.margin,
        )

        self.speed = 0.0     # [m/s] along the car, negative in reverse
        self.steering = 0.0  # [rad] the latest command, positive left
        self._odom_stamp = None
        self._drive_cmd_stamp = None
        self._scan_stamp = None
        self._estop_stamp = None
        self._estop_stop = True
        self._frame_id = ''
        self._braking = False
        self._brake_since = 0.0

        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            AckermannDriveStamped, self.drive_cmd_topic, self.drive_cmd_callback, 10
        )
        self.create_subscription(Bool, 'estop', self.estop_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.bool_pub = self.create_publisher(Bool, '~/brake_bool', 10)
        self.ttc_pub = self.create_publisher(Float32, '~/min_ttc', 10)

        self.create_timer(0.05, self.check_callback)

        self.get_logger().info(
            f'braking below {self.ttc_threshold} s to collision along the steering arc'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stale(self, stamp, timeout_s: float) -> bool:
        """Nothing has arrived yet, or the last one is older than timeout_s."""
        return stamp is None or (self.get_clock().now() - stamp).nanoseconds * 1e-9 > timeout_s

    def _estop_is_active(self) -> bool:
        return self.use_estop and (
            self._estop_stop or self._stale(self._estop_stamp, self.estop_timeout_s)
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.speed = float(msg.twist.twist.linear.x)
        self._odom_stamp = self.get_clock().now()

    def estop_callback(self, msg: Bool) -> None:
        self._estop_stop = bool(msg.data)
        self._estop_stamp = self.get_clock().now()

    def drive_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        self._drive_cmd_stamp = self.get_clock().now()
        self.steering = msg.drive.steering_angle
        # While braking, engage() owns /drive.
        if not self._braking and not self._estop_is_active():
            self.drive_pub.publish(msg)

    def check_callback(self) -> None:
        """Brake on the e-stop or a silent scan: on a timer, so a dead lidar cannot delay it."""
        if self._estop_is_active():
            self.get_logger().warn('e-stop active', throttle_duration_sec=2.0)
            self.engage()
        elif self._stale(self._scan_stamp, self.scan_timeout_s):
            self.get_logger().warn('scan is too old', throttle_duration_sec=2.0)
            self.engage()

    def scan_callback(self, scan: LaserScan) -> None:
        self._scan_stamp = self.get_clock().now()
        self._frame_id = scan.header.frame_id

        if self._estop_is_active():
            self.engage()  # before the release logic below can let go
            return

        if self._stale(self._odom_stamp, self.odom_timeout_s):
            # No speed, no time to collision: unknown has to mean stop.
            self.get_logger().warn('odometry is too old', throttle_duration_sec=2.0)
            if self.brake_on_stale_odom:
                self.engage()
            return

        if self._stale(self._drive_cmd_stamp, self.drive_cmd_timeout_s):
            self.get_logger().warn('drive_cmd is too old', throttle_duration_sec=2.0)
            if self.brake_on_stale_drive_cmd:
                self.engage()
            return

        held = self._now() - self._brake_since >= self.min_hold_s
        if abs(self.speed) < self.min_speed:
            # Nearly stationary: hand control back, once the hold time has passed.
            if not self._braking:
                self.publish_bool(False)
            elif held:
                self.release()
            else:
                self.engage()
            return

        ttc = self.time_to_collision(scan)
        if self._braking:
            if held and ttc > self.ttc_release:
                self.release()
            else:
                self.engage(ttc)
        elif ttc < self.ttc_threshold:
            self.get_logger().warn(f'EMERGENCY BRAKE, {ttc:.2f} s to collision')
            self.engage(ttc)
        else:
            self.publish_bool(False)
            self.publish_ttc(ttc)

    def time_to_collision(self, scan: LaserScan) -> float:
        """Along the commanded arc, with the car's body; inf while the way is clear."""
        curvature = math.tan(self.steering) / self.wheelbase
        # Anything farther is more than ttc_release away; negative when reversing.
        horizon = self.speed * self.ttc_release
        points = scan_points(scan, self.laser_x)
        gap = distance_to_collision(points, curvature, self.body, horizon, self.min_beams)
        return gap / abs(self.speed)

    def engage(self, ttc: float = math.inf) -> None:
        """Publish the brake. Repeated on every call, so /drive never goes quiet under it."""
        if not self._braking:
            self._braking = True
            self._brake_since = self._now()
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.drive.speed = msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)
        self.publish_bool(True)
        self.publish_ttc(ttc)

    def release(self) -> None:
        self._braking = False
        self.get_logger().info('brake released')
        self.publish_bool(False)

    def publish_bool(self, braking: bool) -> None:
        self.bool_pub.publish(Bool(data=braking))

    def publish_ttc(self, ttc: float) -> None:
        if self.publish_debug:
            self.ttc_pub.publish(Float32(data=ttc if math.isfinite(ttc) else 1e9))


def main(args=None):
    rclpy.init(args=args)
    node = Safety()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
