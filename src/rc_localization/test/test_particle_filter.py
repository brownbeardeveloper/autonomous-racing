import numpy as np

from rc_localization.lib.frame_math import Pose2D
from rc_localization.lib.motion_model import MotionDispersion
from rc_localization.lib.occupancy_map import MapInfo, permissible_cells
from rc_localization.lib.particle_filter import MonteCarloLocalizer

ZERO_DISPERSION = MotionDispersion(x=0.0, y=0.0, theta=0.0)


def uniform_sensor_model(particles: np.ndarray, observed_ranges: np.ndarray) -> np.ndarray:
    return np.ones(particles.shape[0])


def test_initialize_pose_centers_the_cloud_on_the_given_pose():
    rng_state = np.random.get_state()
    np.random.seed(0)
    try:
        localizer = MonteCarloLocalizer(2000, ZERO_DISPERSION, uniform_sensor_model)
        localizer.initialize_pose(Pose2D(3.0, -1.0, 0.2), xy_std=0.1, theta_std=0.05)
        pose = localizer.expected_pose()
        assert np.isclose(pose.x, 3.0, atol=0.05)
        assert np.isclose(pose.y, -1.0, atol=0.05)
        assert np.isclose(pose.yaw, 0.2, atol=0.05)
        assert np.isclose(localizer.weights.sum(), 1.0)
    finally:
        np.random.set_state(rng_state)


def test_initialize_global_only_places_particles_on_permissible_cells():
    # A 3x3 grid, free (0) only in the middle column.
    occupancy = [100, 0, 100] * 3
    cells = permissible_cells(occupancy, width=3, height=3)
    map_info = MapInfo(resolution=1.0, origin=Pose2D(0.0, 0.0, 0.0))

    localizer = MonteCarloLocalizer(500, ZERO_DISPERSION, uniform_sensor_model)
    localizer.initialize_global(cells, map_info)

    assert np.allclose(localizer.particles[:, 0], 1.0)  # the only free column, x=1


def test_update_moves_by_the_odometry_delta_when_sensor_model_is_uniform():
    localizer = MonteCarloLocalizer(200, ZERO_DISPERSION, uniform_sensor_model)
    localizer.initialize_pose(Pose2D(0.0, 0.0, 0.0), xy_std=0.0, theta_std=0.0)

    pose = localizer.update(np.array([1.0, 0.0, 0.0]), observed_ranges=np.zeros(10))

    assert np.isclose(pose.x, 1.0, atol=1e-9)
    assert np.isclose(pose.y, 0.0, atol=1e-9)
    assert np.isclose(localizer.weights.sum(), 1.0)


def test_update_resamples_toward_higher_weighted_particles():
    localizer = MonteCarloLocalizer(500, ZERO_DISPERSION, uniform_sensor_model)
    # Two well-separated clusters, first half at x=0, second half at x=10.
    localizer.particles[:250] = [0.0, 0.0, 0.0]
    localizer.particles[250:] = [10.0, 0.0, 0.0]
    localizer.weights[:] = 1.0 / 500

    def favors_the_far_cluster(particles: np.ndarray, observed_ranges: np.ndarray) -> np.ndarray:
        return np.where(particles[:, 0] > 5.0, 1.0, 0.0)

    localizer._evaluate_sensor_model = favors_the_far_cluster
    pose = localizer.update(np.array([0.0, 0.0, 0.0]), observed_ranges=np.zeros(1))

    assert pose.x > 9.0


def test_expected_heading_is_a_circular_mean():
    # Both halves face +x; a plain mean of 0.05 and 2*pi - 0.05 would say pi.
    localizer = MonteCarloLocalizer(2, ZERO_DISPERSION, uniform_sensor_model)
    localizer.particles[:] = [[0.0, 0.0, 0.05], [0.0, 0.0, 2.0 * np.pi - 0.05]]
    assert np.isclose(localizer.expected_pose().yaw, 0.0, atol=1e-9)
