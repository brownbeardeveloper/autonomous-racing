"""Planar (x, y, yaw) poses and how they compose.

No ROS import, so the maths can be tested without a workspace. The car only
rotates about z, which makes the 2D form exact for it.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw [rad] from a quaternion, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def inverse(pose: Pose2D) -> Pose2D:
    """The transform that undoes `pose`."""
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    return Pose2D(
        x=-(cos_yaw * pose.x + sin_yaw * pose.y),
        y=-(-sin_yaw * pose.x + cos_yaw * pose.y),
        yaw=-pose.yaw,
    )


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """The transform "a after b": apply b, then a."""
    cos_yaw, sin_yaw = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        x=a.x + cos_yaw * b.x - sin_yaw * b.y,
        y=a.y + sin_yaw * b.x + cos_yaw * b.y,
        yaw=a.yaw + b.yaw,
    )


def compose_offset(poses: np.ndarray, offset: Pose2D, out: np.ndarray) -> np.ndarray:
    """compose(row, offset) for every (x, y, yaw) row of `poses`, written into `out`."""
    cos_yaw, sin_yaw = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    out[:, 0] = poses[:, 0] + cos_yaw * offset.x - sin_yaw * offset.y
    out[:, 1] = poses[:, 1] + sin_yaw * offset.x + cos_yaw * offset.y
    out[:, 2] = poses[:, 2] + offset.yaw
    return out


def map_to_odom(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    """The map -> odom correction: composed with odom -> base_link it gives map_to_base."""
    return compose(map_to_base, inverse(odom_to_base))
