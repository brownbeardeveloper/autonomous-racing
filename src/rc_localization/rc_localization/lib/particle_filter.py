"""Monte Carlo localization: the particle filter itself, without ROS."""

import math
from typing import Callable

import numpy as np

from rc_localization.lib.frame_math import Pose2D
from rc_localization.lib.motion_model import apply_motion_model, MotionDispersion
from rc_localization.lib.occupancy_map import map_to_world, MapInfo

EvaluateSensorModel = Callable[[np.ndarray, np.ndarray], np.ndarray]


class MonteCarloLocalizer:
    """A weighted particle cloud (N, [x, y, theta]) over a known map.

    `evaluate_sensor_model(particles, observed_ranges) -> weights` is passed
    in, so neither this class nor its tests need RangeLibc.
    """

    def __init__(
        self,
        particle_count: int,
        dispersion: MotionDispersion,
        evaluate_sensor_model: EvaluateSensorModel,
    ):
        self._dispersion = dispersion
        self._evaluate_sensor_model = evaluate_sensor_model
        self.particles = np.zeros((particle_count, 3))
        self.weights = np.full(particle_count, 1.0 / particle_count)

    @property
    def particle_count(self) -> int:
        return self.particles.shape[0]

    def initialize_global(self, permissible_cells: np.ndarray, map_info: MapInfo) -> None:
        """Spread the particles over the free cells, each with a random heading."""
        n = self.particle_count
        indices = np.random.randint(0, permissible_cells.shape[0], size=n)
        self.particles[:, 0:2] = permissible_cells[indices]
        self.particles[:, 2] = np.random.uniform(0.0, 2.0 * np.pi, size=n)
        map_to_world(self.particles, map_info)
        self.weights[:] = 1.0 / n

    def initialize_pose(self, pose: Pose2D, xy_std: float = 0.5, theta_std: float = 0.4) -> None:
        """Seed the particles in a small cloud around a known pose."""
        n = self.particle_count
        self.particles[:, 0] = pose.x + np.random.normal(0.0, xy_std, size=n)
        self.particles[:, 1] = pose.y + np.random.normal(0.0, xy_std, size=n)
        self.particles[:, 2] = pose.yaw + np.random.normal(0.0, theta_std, size=n)
        self.weights[:] = 1.0 / n

    def update(self, odometry_delta: np.ndarray, observed_ranges: np.ndarray) -> Pose2D:
        """One step: resample, move by the odometry, re-weigh against the scan.

        Returns the new expected pose.
        """
        proposal = self.particles[
            np.random.choice(self.particle_count, self.particle_count, p=self.weights)
        ]
        apply_motion_model(proposal, odometry_delta, self._dispersion)
        weights = self._evaluate_sensor_model(proposal, observed_ranges)

        self.particles = proposal
        self.weights = weights / weights.sum()
        return self.expected_pose()

    def expected_pose(self) -> Pose2D:
        """Weighted mean pose.

        The yaw is a circular mean: 0.05 and 2 pi - 0.05 rad average to 0, not to pi.
        """
        x, y = self.particles[:, :2].T @ self.weights
        yaw = math.atan2(
            np.sin(self.particles[:, 2]) @ self.weights,
            np.cos(self.particles[:, 2]) @ self.weights,
        )
        return Pose2D(x, y, yaw)
