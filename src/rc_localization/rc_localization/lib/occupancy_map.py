"""Occupancy grid helpers: where particles may start, and pixels to metres."""

from dataclasses import dataclass

import numpy as np

from rc_localization.lib.frame_math import Pose2D


@dataclass(frozen=True)
class MapInfo:
    resolution: float  # [m] per pixel
    origin: Pose2D     # of pixel (0, 0), in the map frame


def permissible_cells(occupancy_data, width: int, height: int) -> np.ndarray:
    """(x, y) pixel coordinates of the free cells.

    `occupancy_data` is OccupancyGrid.data: 0 free, 100 occupied, -1 unknown.
    """
    grid = np.asarray(occupancy_data).reshape((height, width))
    rows, cols = np.where(grid == 0)
    return np.column_stack((cols, rows)).astype(float)


def map_to_world(poses: np.ndarray, map_info: MapInfo) -> None:
    """Convert (x, y, theta) rows of `poses` in place, from map pixels to the map frame."""
    origin = map_info.origin
    cos_yaw, sin_yaw = np.cos(origin.yaw), np.sin(origin.yaw)
    x_px = np.copy(poses[:, 0])
    poses[:, 0] = cos_yaw * poses[:, 0] - sin_yaw * poses[:, 1]
    poses[:, 1] = sin_yaw * x_px + cos_yaw * poses[:, 1]
    poses[:, :2] *= map_info.resolution
    poses[:, 0] += origin.x
    poses[:, 1] += origin.y
    poses[:, 2] += origin.yaw
