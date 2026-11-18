"""Tests for collision.py: the car's body swept along its steering arc."""

import math

import numpy as np
import pytest

from rc_safety.collision import arc_poses, Body, distance_to_collision

BODY = Body(front=0.5, rear=0.2, half_width=0.2)
FULL_LEFT = math.tan(0.36) / 0.33  # [1/m] full lock: a 0.88 m radius


def row(x, y_from, y_to, n=50):
    """Points across the path at x, from y_from to y_to."""
    return np.column_stack((np.full(n, x), np.linspace(y_from, y_to, n)))


def gap(points, curvature=0.0, horizon=3.0):
    return distance_to_collision(points, curvature, BODY, horizon, min_points=5)


def test_a_quarter_turn_left_ends_one_radius_ahead_and_one_to_the_left():
    x, y, yaw = arc_poses(1.0, np.array([math.pi / 2]))
    assert (x[0], y[0], yaw[0]) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_straight_ahead_it_is_the_free_distance_in_front_of_the_bumper():
    assert gap(row(1.5, -1.0, 1.0)) == pytest.approx(1.5 - BODY.front, abs=0.05)


def test_nothing_within_the_horizon_is_clear():
    assert math.isinf(gap(row(5.0, -1.0, 1.0)))


def test_turning_away_clears_what_a_straight_line_would_hit():
    """The classic false brake in a corner: the wall ahead the car is steering away from."""
    ahead_right = row(1.2, -0.3, -0.05)
    assert math.isfinite(gap(ahead_right))
    assert math.isinf(gap(ahead_right, FULL_LEFT))


def test_turning_toward_what_is_beside_the_car_hits_it():
    """Missed by a straight line: the car's own arc carries it into the obstacle."""
    beside_left = row(0.8, 0.4, 0.6)
    assert math.isinf(gap(beside_left))
    assert math.isfinite(gap(beside_left, FULL_LEFT))


def test_reversing_meets_what_is_behind_with_the_rear_bumper():
    behind = row(-0.8, -0.5, 0.5)
    assert math.isinf(gap(behind))
    assert gap(behind, horizon=-3.0) == pytest.approx(0.8 - BODY.rear, abs=0.05)


def test_a_few_noisy_echoes_are_not_an_obstacle():
    assert math.isinf(gap(np.array([[1.0, 0.0]] * 4)))
