"""Odometry motion model: how the particles move between two updates."""

from dataclasses import dataclass

import numpy as np

from rc_localization.lib.frame_math import compose, inverse, Pose2D


@dataclass(frozen=True)
class MotionDispersion:
    """Std-dev of the Gaussian noise added per update, in the car's own frame."""

    x: float      # [m]
    y: float      # [m]
    theta: float  # [rad]


def odometry_delta(previous: Pose2D, current: Pose2D) -> np.ndarray:
    """(dx, dy, dyaw) between two odometry poses, in the car's frame at `previous`."""
    relative = compose(inverse(previous), current)
    return np.array([relative.x, relative.y, relative.yaw])


def apply_motion_model(
    particles: np.ndarray, action: np.ndarray, dispersion: MotionDispersion
) -> None:
    """Move `particles` (N, [x, y, theta]) in place by `action`, plus noise.

    `action` is an odometry_delta, applied along each particle's own heading.
    The noise keeps the cloud wide enough for the sensor model to correct the
    drift that odometry alone builds up.
    """
    n = particles.shape[0]
    cos_yaw = np.cos(particles[:, 2])
    sin_yaw = np.sin(particles[:, 2])

    particles[:, 0] += cos_yaw * action[0] - sin_yaw * action[1]
    particles[:, 1] += sin_yaw * action[0] + cos_yaw * action[1]
    particles[:, 2] += action[2]

    particles[:, 0] += np.random.normal(0.0, dispersion.x, size=n)
    particles[:, 1] += np.random.normal(0.0, dispersion.y, size=n)
    particles[:, 2] += np.random.normal(0.0, dispersion.theta, size=n)
