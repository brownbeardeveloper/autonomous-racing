"""Where the car's body first touches the scan, driving on along its steering arc.

No ROS imports, so it tests on its own. Everything is in base_link: the rear
axle, x forward and y left (REP 103). The car turns about the rear axle, on the
circle its steering angle gives.
"""

from dataclasses import dataclass

import numpy as np

STEPS = 100  # poses along the arc: one every 2 cm at a 2 m horizon


@dataclass(frozen=True)
class Body:
    """The car's footprint around the rear axle, margin included."""

    front: float       # [m] ahead of the rear axle
    rear: float        # [m] behind it
    half_width: float  # [m]


def scan_points(scan, laser_x: float) -> np.ndarray:
    """(N, 2) echoes of a LaserScan in base_link. NaN and inf are no echo, not an obstacle."""
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    echo = np.isfinite(r) & (r >= scan.range_min)
    return np.column_stack((laser_x + r[echo] * np.cos(a[echo]), r[echo] * np.sin(a[echo])))


def arc_poses(curvature: float, distances: np.ndarray):
    """Rear-axle (x, y, yaw) after each distance along the arc; negative is reversing."""
    yaw = curvature * distances
    if abs(curvature) < 1e-9:
        return distances, np.zeros_like(distances), yaw
    return np.sin(yaw) / curvature, (1.0 - np.cos(yaw)) / curvature, yaw


def distance_to_collision(points, curvature, body, horizon, min_points) -> float:
    """How far the car drives along the arc before its body reaches min_points of the points.

    horizon is how far to look, negative when reversing; inf means clear that far.
    """
    # The body never gets farther than this from where it is now.
    reach = abs(horizon) + np.hypot(max(body.front, body.rear), body.half_width)
    points = points[np.hypot(points[:, 0], points[:, 1]) <= reach]
    distances = np.linspace(0.0, horizon, STEPS)
    x, y, yaw = arc_poses(curvature, distances)
    # Every point in the car's own frame, at every pose along the arc: (STEPS, N).
    dx, dy = points[:, 0] - x[:, None], points[:, 1] - y[:, None]
    cos_yaw, sin_yaw = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    along, across = cos_yaw * dx + sin_yaw * dy, cos_yaw * dy - sin_yaw * dx
    inside = (along <= body.front) & (along >= -body.rear) & (np.abs(across) <= body.half_width)
    # Per point, the last pose still clear of it: never a step late.
    first = inside.argmax(axis=0)
    reached = np.where(inside.any(axis=0), np.abs(distances[np.maximum(first - 1, 0)]), np.inf)
    if len(reached) < min_points:
        return np.inf
    return float(np.partition(reached, min_points - 1)[min_points - 1])
