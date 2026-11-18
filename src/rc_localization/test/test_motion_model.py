import math

import numpy as np

from rc_localization.lib.frame_math import Pose2D
from rc_localization.lib.motion_model import MotionDispersion, apply_motion_model, odometry_delta


def test_odometry_delta_is_zero_when_the_car_did_not_move():
    pose = Pose2D(1.0, 2.0, 0.5)
    delta = odometry_delta(pose, pose)
    assert np.allclose(delta, [0.0, 0.0, 0.0])


def test_odometry_delta_expresses_the_move_in_the_previous_heading():
    # Facing +90 deg (world +y is "forward"), then moving 1 m along world +y
    # is 1 m "forward" in the car's own frame at the time.
    previous = Pose2D(0.0, 0.0, math.pi / 2)
    current = Pose2D(0.0, 1.0, math.pi / 2)
    delta = odometry_delta(previous, current)
    assert np.allclose(delta, [1.0, 0.0, 0.0], atol=1e-9)


def test_apply_motion_model_moves_a_particle_in_its_own_heading_without_noise():
    zero_dispersion = MotionDispersion(x=0.0, y=0.0, theta=0.0)
    particles = np.array([[0.0, 0.0, math.pi / 2]])  # facing +90 deg

    apply_motion_model(particles, np.array([1.0, 0.0, 0.0]), zero_dispersion)

    assert np.allclose(particles[0], [0.0, 1.0, math.pi / 2], atol=1e-9)


def test_apply_motion_model_dispersion_spreads_particles_around_the_mean():
    rng_state = np.random.get_state()
    np.random.seed(0)
    try:
        dispersion = MotionDispersion(x=0.1, y=0.1, theta=0.05)
        particles = np.zeros((2000, 3))
        apply_motion_model(particles, np.array([0.0, 0.0, 0.0]), dispersion)
        assert 0.0 < np.std(particles[:, 0]) < 0.2
        assert np.isclose(np.mean(particles[:, 0]), 0.0, atol=0.02)
    finally:
        np.random.set_state(rng_state)
