#!/usr/bin/env python3
"""Follow mode: wall following, falling back to follow-the-gap.

Follows the wall on `wall_side` while there is one and the car's own lane is
clear for `clear_distance`. Anything else - an obstacle, an opening, open
ground - is follow-the-gap. Both pick a target point and steer to it with the
pure pursuit law racing uses; the speed comes from how far the lane is free.

With publish_debug on, ~/behavior says which of the two is driving and
~/markers draws how it chose the target (lidar frame).
rc_bringup/foxglove/follow.json is a Foxglove layout built around them.
"""

import math
from typing import NamedTuple, Optional

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from rc_control.lib.follow import (
    braking_speed,
    clean_ranges,
    deepest_gap,
    extend_disparities,
    fit_wall,
    free_distance_ahead,
    gap_target,
    wall_target,
)
from rc_control.lib.pure_pursuit import compute_steering, curvature_speed_limit, pursuit_arc

# Everything but the topics, which are bound to the subscriptions at startup.
LIVE_TUNABLE = {
    'fov_deg', 'max_range', 'half_width', 'wall_side', 'wall_distance', 'lookahead',
    'clear_distance', 'stop_distance', 'max_decel', 'max_lateral_accel', 'max_speed',
    'wheelbase', 'laser_x', 'publish_debug',
}

WALL_COLOR = ColorRGBA(r=0.2, g=0.7, b=1.0, a=1.0)
GAP_COLOR = ColorRGBA(r=1.0, g=0.3, b=0.9, a=1.0)
GREEN = ColorRGBA(r=0.3, g=0.9, b=0.3, a=0.8)
RED = ColorRGBA(r=1.0, g=0.25, b=0.2, a=0.8)
ORANGE = ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.8)
GRAY = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)


class Plan(NamedTuple):
    """What one scan decided and what it rested on, in the lidar frame."""

    behavior: str                # wall | gap
    target: np.ndarray           # (x, y) the car steers for
    free: float                  # [m] the car's lane is clear this far
    wall: Optional[tuple]        # (foot, direction) of the fitted wall, if there is one
    wide: Optional[np.ndarray]   # gap mode: ranges across the FOV, every obstacle widened


class Follow(Node):
    def __init__(self):
        super().__init__('follow_node')

        self.declare_parameters('', [
            ('scan_topic', 'scan_filtered'),
            ('drive_topic', 'follow/drive'),
            ('fov_deg', 180.0),          # where follow-the-gap may aim
            ('max_range', 3.0),          # [m] anything farther counts as open
            ('half_width', 0.25),        # [m] half the car's width plus margin
            ('wall_side', 'left'),       # left | right
            ('wall_distance', 0.7),      # [m] held from the wall
            ('lookahead', 1.0),          # [m] along the wall to the target point
            ('clear_distance', 1.5),     # [m] lane free this far: follow the wall
            ('stop_distance', 0.5),      # [m] from the lidar to where the car has stopped
            ('max_decel', 2.0),          # [m/s^2] braking the speed plans for
            ('max_lateral_accel', 3.0),  # [m/s^2] cornering the speed plans for
            ('max_speed', 2.0),          # [m/s] planning cap; the firmware owns the real limit
            ('wheelbase', 0.33),         # [m]
            ('laser_x', 0.285),          # [m] lidar ahead of the rear axle
            ('publish_debug', True),
        ])
        for name in LIVE_TUNABLE:
            setattr(self, name, self.get_parameter(name).value)

        self.behavior = None
        self._angles = None  # per beam, rebuilt when the scan changes size or the FOV is retuned
        self._in_fov = None

        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.target_pub = self.create_publisher(PointStamped, '~/target', 10)
        self.behavior_pub = self.create_publisher(String, '~/behavior', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            f'{self.wall_side} wall at {self.wall_distance} m, '
            f'gap follow when the lane is blocked within {self.clear_distance} m'
        )

    def command(self, scan: LaserScan):
        """Steering, speed and the Plan behind them, for one scan."""
        n = len(scan.ranges)
        if self._angles is None or len(self._angles) != n:
            self._angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
            self._in_fov = np.abs(self._angles) <= math.radians(self.fov_deg) / 2.0

        ranges = clean_ranges(scan.ranges, self.max_range)
        free = free_distance_ahead(ranges, self._angles, self.half_width, self.max_range)
        wall = fit_wall(ranges, self._angles, self.wall_side, self.max_range)

        if wall is not None and free > self.clear_distance:
            target = wall_target(*wall, self.wall_distance, self.lookahead)
            plan = Plan('wall', target, free, wall, None)
        else:
            wide = extend_disparities(ranges, scan.angle_increment, self.half_width)[self._in_fov]
            plan = Plan('gap', gap_target(wide, self._angles[self._in_fov]), free, wall, wide)

        if plan.behavior != self.behavior:
            self.behavior = plan.behavior
            self.get_logger().info(f'{plan.behavior} follow')

        # Pure pursuit steers the rear axle, laser_x behind the lidar.
        steer = compute_steering(
            (0.0, 0.0), 0.0, plan.target + (self.laser_x, 0.0), self.wheelbase
        )
        speed = curvature_speed_limit(
            math.tan(steer) / self.wheelbase,
            self.max_lateral_accel,
            braking_speed(free, self.stop_distance, self.max_decel, self.max_speed),
        )
        return steer, float(speed), plan

    def scan_cb(self, scan: LaserScan) -> None:
        steer, speed, plan = self.command(scan)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = scan.header.frame_id
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub.publish(msg)

        if self.publish_debug:
            point = PointStamped(header=scan.header)
            point.point.x, point.point.y = float(plan.target[0]), float(plan.target[1])
            self.target_pub.publish(point)
            self.behavior_pub.publish(String(data=plan.behavior))
            self.marker_pub.publish(self.markers(scan.header, plan, speed))

    def markers(self, header, plan: Plan, speed: float) -> MarkerArray:
        """The lane, the wall or the gap, the target, and the arc the car drives to it."""
        color = WALL_COLOR if plan.behavior == 'wall' else GAP_COLOR
        hw, clear = self.half_width, self.clear_distance
        to_base = np.array([self.laser_x, 0.0])
        lane = [(0.0, hw), (plan.free, hw), (plan.free, -hw), (0.0, -hw)]
        arc = pursuit_arc(plan.target + to_base) - to_base
        text = f'{plan.behavior} follow  {speed:.1f} m/s'
        out = [
            line(header, 'lane', lane, GREEN if plan.free > clear else RED),
            line(header, 'clear_distance', [(clear, hw), (clear, -hw)], GRAY),
            line(header, 'driving_line', arc, color, 0.04),
            sphere(header, 'target', plan.target, color),
            label(header, text, -to_base - (0.4, 0.0), color),
        ]
        if plan.wall is not None:
            foot, direction = plan.wall
            out.append(line(header, 'wall', [foot, foot + self.lookahead * direction], GRAY))
            if plan.behavior == 'wall':
                # How wall_target built it: wall_distance out, then lookahead along the wall.
                corner = plan.target - self.lookahead * direction
                out.append(line(header, 'wall', [foot, corner, plan.target], color))
        if plan.wide is not None:
            angles = self._angles[self._in_fov]
            xy = np.column_stack([plan.wide * np.cos(angles), plan.wide * np.sin(angles)])
            start, end = deepest_gap(plan.wide)
            out.append(line(header, 'gap', xy, ORANGE))
            out.append(line(header, 'gap', xy[start:end], color, 0.04))
        for i, m in enumerate(out):
            m.id = i
        return MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)] + out)

    def _on_parameter_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name not in LIVE_TUNABLE:
                return SetParametersResult(
                    successful=False, reason=f"'{p.name}' can only be set at startup"
                )
            if p.name == 'wall_side' and p.value not in ('left', 'right'):
                return SetParametersResult(
                    successful=False, reason='wall_side must be left or right'
                )
        for p in params:
            setattr(self, p.name, p.value)
        self._angles = None  # rebuild the FOV mask on the next scan
        return SetParametersResult(successful=True)


def _marker(header, ns: str, kind: int, color: ColorRGBA, size: float) -> Marker:
    m = Marker(header=header, ns=ns, type=kind, action=Marker.ADD, color=color)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = size  # line width, sphere diameter, text height
    return m


def line(header, ns, points, color, width=0.02) -> Marker:
    m = _marker(header, ns, Marker.LINE_STRIP, color, width)
    m.points = [Point(x=x, y=y) for x, y in np.asarray(points, dtype=float).tolist()]
    return m


def sphere(header, ns, xy, color, diameter=0.12) -> Marker:
    m = _marker(header, ns, Marker.SPHERE, color, diameter)
    m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
    return m


def label(header, text, xy, color, height=0.15) -> Marker:
    m = _marker(header, 'label', Marker.TEXT_VIEW_FACING, color, height)
    m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
    m.text = text
    return m


def main(args=None):
    rclpy.init(args=args)
    node = Follow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
