"""Follow-mode math: wall following, and follow-the-gap where there is no wall.

No rclpy here; follow_node.py is the ROS side. Everything is in the lidar
frame: x forward, y left, angles in radians.
"""

import math

import numpy as np

WALL_SECTOR = (math.radians(20.0), math.radians(110.0))  # beside the car, a little ahead to behind
MIN_WALL_POINTS = 30
SIDE_SIGN = {'left': 1.0, 'right': -1.0}


def clean_ranges(ranges, max_range: float) -> np.ndarray:
    """NaN and inf mean no echo: free all the way out to max_range."""
    r = np.asarray(ranges, dtype=np.float32)
    r = np.nan_to_num(r, nan=max_range, posinf=max_range, neginf=0.0)
    return np.clip(r, 0.0, max_range)


def free_distance_ahead(ranges, angles, half_width: float, max_range: float) -> float:
    """How far the car can drive straight on: the nearest hit inside its own lane."""
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    in_lane = (x > 0.0) & (np.abs(y) < half_width)
    return float(x[in_lane].min()) if in_lane.any() else max_range


def braking_speed(free: float, stop_distance: float, max_decel: float, max_speed: float) -> float:
    """Fastest speed that still stops stop_distance short of the hit: v^2 = 2 a d."""
    room = max(0.0, free - stop_distance)
    return min(max_speed, math.sqrt(2.0 * max_decel * room))


def extend_disparities(ranges, angle_increment: float, half_width: float) -> np.ndarray:
    """Widen every obstacle edge by half the car's width.

    Next to a jump in range, the far beams lead straight past the near
    obstacle's corner. Copying the near range over them keeps the car's side
    clear of it. A jump counts as an edge when it is wider than half the car.
    """
    out = ranges.copy()
    for i in np.flatnonzero(np.abs(np.diff(ranges)) > half_width):
        near = float(min(ranges[i], ranges[i + 1]))
        half_angle = math.asin(min(1.0, half_width / max(near, half_width)))
        n = int(math.ceil(half_angle / angle_increment))
        if ranges[i] < ranges[i + 1]:
            lo, hi = i + 1, i + 1 + n
        else:
            lo, hi = max(0, i + 1 - n), i + 1
        out[lo:hi] = np.minimum(out[lo:hi], near)
    return out


def largest_gap(free) -> tuple:
    """Longest run of True in `free`, as [start, end). Needs at least one True."""
    edges = np.diff(np.concatenate(([0], np.asarray(free, dtype=np.int8), [0])))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    g = int(np.argmax(ends - starts))
    return int(starts[g]), int(ends[g])


def deepest_gap(ranges) -> tuple:
    """Widest run of the deepest beams (within 10 %), as [start, end)."""
    return largest_gap(ranges >= 0.9 * ranges.max())


def gap_target(ranges, angles) -> np.ndarray:
    """Middle of the deepest gap, as an (x, y) point."""
    start, end = deepest_gap(ranges)
    i = (start + end - 1) // 2
    return np.array([ranges[i] * math.cos(angles[i]), ranges[i] * math.sin(angles[i])])


def fit_wall(ranges, angles, side: str, max_range: float):
    """Least-squares line through the wall beside the car, or None if there is none.

    Returns the wall's point nearest the lidar and its forward unit direction.
    """
    side_angle = SIDE_SIGN[side] * angles  # positive on the wall's side
    use = (side_angle > WALL_SECTOR[0]) & (side_angle < WALL_SECTOR[1]) & (ranges < max_range)
    if np.count_nonzero(use) < MIN_WALL_POINTS:
        return None
    r, a = ranges[use], angles[use]
    points = np.column_stack([r * np.cos(a), r * np.sin(a)])
    center = points.mean(axis=0)
    direction = np.linalg.svd(points - center, full_matrices=False)[2][0]  # the points' main axis
    if direction[0] < 0.0:
        direction = -direction
    if direction[0] < 0.5:
        return None  # more than 60 degrees off our heading: a wall ahead, not beside us
    foot = center - (center @ direction) * direction
    return foot, direction


def wall_target(foot, direction, distance: float, lookahead: float) -> np.ndarray:
    """Return the point `lookahead` ahead along the wall, `distance` out from it on our side."""
    toward_car = -foot / np.linalg.norm(foot)
    return foot + distance * toward_car + lookahead * direction
