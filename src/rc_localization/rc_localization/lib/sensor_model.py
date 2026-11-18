"""The sensor model on RangeLibc: how well each particle explains the scan.

Rays are cast on the Jetson's GPU, by RangeLibc's ray marching. Importing this
module needs the compiled range_libc extension; the rest of lib does not.
"""

from dataclasses import dataclass

import numpy as np
import range_libc

from rc_localization.lib.frame_math import compose_offset, Pose2D


@dataclass(frozen=True)
class SensorModelParams:
    z_short: float
    z_max: float
    z_rand: float
    z_hit: float
    sigma_hit: float
    max_range_px: int


def precompute_sensor_model_table(params: SensorModelParams) -> np.ndarray:
    """The [observed, expected] range probability table RangeLibc looks up at runtime."""
    width = params.max_range_px + 1
    table = np.zeros((width, width))

    for computed in range(width):
        observed = np.arange(width, dtype=float)
        z = observed - computed
        prob = params.z_hit * np.exp(-(z * z) / (2.0 * params.sigma_hit ** 2)) / (
            params.sigma_hit * np.sqrt(2.0 * np.pi)
        )
        short_reading = observed < computed
        prob[short_reading] += (
            2.0 * params.z_short * (computed - observed[short_reading]) / max(computed, 1)
        )
        prob[width - 1] += params.z_max  # a max-range reading in error
        prob[:-1] += params.z_rand / params.max_range_px  # a random reading
        table[:, computed] = prob / prob.sum()

    return table


def build_ray_caster(map_msg, max_range_px: int):
    """RangeLibc's GPU ray caster for `map_msg`, an OccupancyGrid."""
    if not range_libc.SHOULD_USE_CUDA:
        # A CPU-only build does not raise by itself: it warns and returns a
        # caster that cannot cast rays.
        raise RuntimeError(
            'RangeLibc is built without CUDA - rebuild it with WITH_CUDA=ON (see README.md)'
        )
    return range_libc.PyRayMarchingGPU(range_libc.PyOMap(map_msg), max_range_px)


def evaluate(
    ray_caster,
    particles: np.ndarray,
    mount: Pose2D,
    angles: np.ndarray,
    observed_ranges: np.ndarray,
    squash_exponent: float,
    queries: np.ndarray,
    ranges: np.ndarray,
) -> np.ndarray:
    """Weigh every particle by how well its predicted scan matches the observed one.

    Particles are base_link poses; the rays start at the lidar, `mount` away
    from each. `queries` (particles, 3) and `ranges` (particles * angles) are
    the caller's scratch buffers, reused across updates.
    """
    compose_offset(particles, mount, queries)
    weights = np.empty(particles.shape[0])
    # Two calls, not RangeLibc's fused one: on the GPU that only warns and
    # leaves `weights` uninitialized, which reaches the resampler as NaN.
    ray_caster.calc_range_repeat_angles(queries, angles, ranges)
    ray_caster.eval_sensor_model(
        observed_ranges, ranges, weights, angles.shape[0], particles.shape[0]
    )
    return np.power(weights, squash_exponent)
