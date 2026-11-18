"""Tests for the pure pursuit maths in rc_control.lib.pure_pursuit."""

import math
import os
import tempfile

import numpy as np
import pytest

from rc_control.lib.pure_pursuit import (
    Path,
    advance_by_arclength,
    compute_steering,
    cross_track_error,
    curvature_speed_limit,
    load_waypoints,
    lookahead_point_on_path,
    nearest_point_on_path,
    pursuit_arc,
    quaternion_to_yaw,
    sample_at,
    speed_profile,
)


def circle_path(radius=10.0, n=200, v=3.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    xy = np.column_stack([radius * np.cos(a), radius * np.sin(a)])
    return Path.from_arrays(xy, np.full(n, v), np.full(n, 1.0 / radius))


def square_path(side=10.0, per=25):
    pts = []
    corners = [(0, 0), (side, 0), (side, side), (0, side)]
    for i in range(4):
        a = np.array(corners[i], float)
        b = np.array(corners[(i + 1) % 4], float)
        for k in range(per):
            pts.append(a + (b - a) * k / per)
    return Path.from_arrays(np.array(pts), np.full(len(pts), 2.0))


def test_path_geometry_matches_circle():
    p = circle_path(radius=10.0, n=400)
    assert len(p) == 400
    assert p.length == pytest.approx(2 * math.pi * 10.0, rel=1e-4)
    assert p.s[0] == 0.0
    assert np.all(np.diff(p.s) > 0)


def test_duplicate_closing_point_is_dropped():
    xy = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]], float)
    p = Path.from_arrays(xy, np.arange(5, dtype=float))
    assert len(p) == 4
    assert len(p.v) == 4
    assert p.length == pytest.approx(4.0)


def test_rejects_degenerate_input():
    with pytest.raises(ValueError):
        Path.from_arrays(np.array([[0.0, 0.0], [1.0, 0.0]]))
    with pytest.raises(ValueError):
        Path.from_arrays(np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]))
    with pytest.raises(ValueError):
        Path.from_arrays(np.array([[0.0, 0.0], [1.0, np.nan], [1.0, 1.0]]))


def _write(text, suffix='.csv'):
    fd, name = tempfile.mkstemp(suffix=suffix)
    with os.fdopen(fd, 'w') as fh:
        fh.write(text)
    return name


def test_loads_pipeline_export_format():
    """The header the map_pipeline notebook writes."""
    txt = ('s_m,x_m,y_m,psi_rad,kappa_radpm,v_mps,v_crit_mps\n'
           '0.000000,0.000000,0.000000,0.000000,0.010000,5.000000,7.000000\n'
           '1.000000,1.000000,0.000000,0.000000,0.020000,4.000000,6.000000\n'
           '2.000000,1.000000,1.000000,1.570796,0.030000,3.000000,5.000000\n'
           '3.000000,0.000000,1.000000,3.141593,0.040000,2.000000,4.000000\n')
    p = load_waypoints(_write(txt))
    assert len(p) == 4
    assert p.xy[1].tolist() == [1.0, 0.0]
    assert p.v.tolist() == [5.0, 4.0, 3.0, 2.0]
    assert p.kappa[3] == pytest.approx(0.04)


def test_loads_tum_semicolon_format():
    txt = ('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n'
           '0.0; 0.0; 0.0; 0.0; 0.0; 6.0; 0.0\n'
           '1.0; 1.0; 0.0; 0.0; 0.1; 5.0; 0.0\n'
           '2.0; 1.0; 1.0; 1.57; 0.2; 4.0; 0.0\n'
           '3.0; 0.0; 1.0; 3.14; 0.3; 3.0; 0.0\n')
    p = load_waypoints(_write(txt))
    assert len(p) == 4
    assert p.v.tolist() == [6.0, 5.0, 4.0, 3.0]      # vx_mps, not the s column
    assert p.kappa[1] == pytest.approx(0.1)


def test_loads_headerless_xyv_and_xy():
    p = load_waypoints(_write('0,0,2.0\n1,0,2.5\n1,1,3.0\n0,1,3.5\n'))
    assert p.v.tolist() == [2.0, 2.5, 3.0, 3.5]
    assert np.all(p.kappa == 0.0)

    q = load_waypoints(_write('0,0\n1,0\n1,1\n0,1\n'))
    assert len(q) == 4
    assert np.all(q.v == 0.0)


def test_loading_errors_are_typed():
    with pytest.raises(OSError):
        load_waypoints('/definitely/not/here.csv')
    with pytest.raises(ValueError):
        load_waypoints(_write('\n\n'))
    with pytest.raises(ValueError):
        load_waypoints(_write('0,0\n1,0\nbroken,row\n1,1\n'))
    with pytest.raises(ValueError):
        load_waypoints(_write('alpha,beta\n0,0\n1,0\n1,1\n'))


def test_quaternion_to_yaw_roundtrip():
    for want in (0.0, 0.5, -1.2, 3.0, -3.0):
        qz, qw = math.sin(want / 2), math.cos(want / 2)
        assert quaternion_to_yaw(0.0, 0.0, qz, qw) == pytest.approx(want)


def test_nearest_point_on_circle():
    p = circle_path(radius=10.0, n=360)
    idx, t, pt, d = nearest_point_on_path([12.0, 0.0], p)
    assert d == pytest.approx(2.0, abs=1e-3)
    assert np.linalg.norm(pt - np.array([10.0, 0.0])) < 1e-3


def test_windowed_search_ignores_the_far_side():
    """Two straights 0.4 m apart: an unrestricted search snaps to the wrong one."""
    a = np.column_stack([np.linspace(0, 20, 100), np.zeros(100)])
    b = np.column_stack([np.linspace(20, 0, 100), np.full(100, 0.4)])
    p = Path.from_arrays(np.vstack([a, b]))

    query = [10.0, 0.05]
    seeded = nearest_point_on_path(query, p, seed=50, window=15)
    assert seeded[0] == pytest.approx(50, abs=15)

    # Sitting nearer the return leg, a global search jumps across.
    far = nearest_point_on_path([10.0, 0.35], p)
    near = nearest_point_on_path([10.0, 0.35], p, seed=50, window=15)
    assert far[0] > 100 and near[0] < 100


def test_lookahead_sits_on_the_circle():
    p = circle_path(radius=10.0, n=720)
    pos = np.array([10.0, 0.0])
    idx0, t0, _, _ = nearest_point_on_path(pos, p)
    for radius in (0.5, 1.0, 2.5):
        i, t, pt = lookahead_point_on_path(pos, p, idx0, t0, radius)
        assert np.linalg.norm(pt - pos) == pytest.approx(radius, abs=1e-2)


def test_lookahead_goes_forward_not_backward():
    p = square_path()
    pos = np.array([5.0, 0.0])
    idx, t, _, _ = nearest_point_on_path(pos, p)
    i, tt, pt = lookahead_point_on_path(pos, p, idx, t, 2.0)
    assert pt[0] > pos[0]              # heading +x along the bottom edge
    assert pt[1] == pytest.approx(0.0, abs=1e-6)


def test_lookahead_falls_back_when_circle_never_closes():
    """Radius larger than the whole loop still yields a usable target."""
    p = circle_path(radius=2.0, n=60)
    i, t, pt = lookahead_point_on_path([0.0, 0.0], p, 0, 0.0, 500.0)
    assert np.all(np.isfinite(pt))
    assert 0 <= i < len(p)


def test_advance_by_arclength_wraps():
    p = circle_path(radius=10.0, n=360)
    i, t, pt = advance_by_arclength(p, 0, 0.0, p.length + 1.0)
    j, u, qt = advance_by_arclength(p, 0, 0.0, 1.0)
    assert np.linalg.norm(pt - qt) < 1e-6


def test_sample_at_interpolates_and_wraps():
    v = np.array([0.0, 10.0, 20.0, 30.0])
    assert sample_at(v, 0, 0.5) == pytest.approx(5.0)
    assert sample_at(v, 3, 0.5) == pytest.approx(15.0)     # 30 -> 0


def test_steering_is_zero_when_aligned():
    assert compute_steering([0, 0], 0.0, [5.0, 0.0], 0.33) == pytest.approx(0.0)


def test_steering_sign_and_symmetry():
    left = compute_steering([0, 0], 0.0, [3.0, 1.0], 0.33)
    right = compute_steering([0, 0], 0.0, [3.0, -1.0], 0.33)
    assert left > 0 and right < 0
    assert left == pytest.approx(-right)


def test_steering_matches_geometric_arc():
    """delta = atan(L_wb / R) for a target on a circle of radius R."""
    wheelbase, R, ld = 0.33, 5.0, 2.0
    alpha = math.asin(ld / (2 * R))
    target = [ld * math.cos(alpha), ld * math.sin(alpha)]
    delta = compute_steering([0, 0], 0.0, target, wheelbase)
    assert delta == pytest.approx(math.atan(wheelbase / R), rel=1e-6)


def test_steering_handles_target_behind_and_coincident():
    assert compute_steering([0, 0], 0.0, [-3.0, 0.0], 0.33) == pytest.approx(0.0, abs=1e-9)
    assert compute_steering([0, 0], 0.0, [0.0, 0.0], 0.33) == 0.0


def test_pursuit_arc_is_the_steering_circle_through_the_target():
    target, wheelbase = np.array([2.0, 0.8]), 0.33
    arc = pursuit_arc(target)
    radius = wheelbase / math.tan(compute_steering([0, 0], 0.0, target, wheelbase))
    assert arc[0] == pytest.approx([0.0, 0.0])
    assert arc[-1] == pytest.approx(target)
    assert np.hypot(arc[:, 0], arc[:, 1] - radius) == pytest.approx(np.full(len(arc), radius))
    assert pursuit_arc([2.0, 0.0])[:, 1] == pytest.approx(np.zeros(20))


def test_cross_track_error_sign():
    assert cross_track_error([0, 0], 0.0, [0.0, 1.0]) > 0     # path to the left
    assert cross_track_error([0, 0], 0.0, [0.0, -1.0]) < 0


def test_curvature_speed_limit():
    assert curvature_speed_limit(0.0, 6.0, 8.0) == 8.0
    assert curvature_speed_limit(1 / 6.0, 6.0, 8.0) == pytest.approx(6.0)
    assert curvature_speed_limit(1 / 100.0, 6.0, 8.0) == 8.0   # cap dominates


def straight_with_slow_section():
    """1 m spacing, 8 m/s except 2 m/s at points 60-69."""
    n = 100
    xy = np.column_stack([np.linspace(0, 99, n), np.zeros(n)])
    v = np.full(n, 8.0)
    v[60:70] = 2.0
    return Path.from_arrays(xy, v)


def test_profile_brakes_before_a_slow_section():
    v = speed_profile(straight_with_slow_section(), 1.0, 10.0, 0.0, max_decel=3.0)
    # 5 m out from v=2 at 3 m/s^2: sqrt(4 + 30) = 5.83
    assert v[55] == pytest.approx(math.sqrt(4.0 + 2 * 3.0 * 5.0))
    # 10 m out the limit is exactly the straight's 8.0 - the last point at which
    # full speed is still recoverable. Anything closer must already be slower.
    assert v[50] == pytest.approx(8.0)
    assert v[10] == pytest.approx(8.0)
    assert v[65] == pytest.approx(2.0)


def test_profile_applies_speed_gain_before_braking():
    """Braking from the scaled speed, so max_decel stays max_decel at any gain."""
    v = speed_profile(straight_with_slow_section(), 0.5, 10.0, 0.0, max_decel=3.0)
    # 2 m out from 0.5 * 2 m/s; scaling after braking would give 0.5 * sqrt(4 + 12) = 2.0
    assert v[58] == pytest.approx(math.sqrt(1.0 ** 2 + 2 * 3.0 * 2.0))


def test_profile_braking_wraps_round_the_loop():
    p = circle_path(radius=10.0, n=120, v=3.0)
    p.v[0] = 1.0
    v = speed_profile(p, 1.0, 10.0, 0.0, max_decel=2.0)
    assert v[-1] == pytest.approx(math.sqrt(1.0 + 2 * 2.0 * p.seg[-1]))
    assert v[-1] < 3.0


def test_profile_caps_at_max_speed_and_grip():
    p = circle_path(radius=10.0, n=120, v=20.0)
    assert speed_profile(p, 1.0, 5.0, 0.0, 0.0) == pytest.approx(np.full(120, 5.0))
    # sqrt(a_lat * R) = sqrt(6 * 10)
    assert speed_profile(p, 1.0, 50.0, 6.0, 0.0) == pytest.approx(np.full(120, math.sqrt(60.0)))


def test_tracks_a_circle_in_simulation():
    """Kinematic bicycle following a circle should converge and stay on it."""
    p = circle_path(radius=10.0, n=720, v=3.0)
    wheelbase, dt = 0.33, 0.02
    pos = np.array([11.0, 0.0])            # start 1 m outside the line
    yaw = math.pi / 2
    seed, errors = None, []

    for step in range(4000):
        idx, t, near, dist = nearest_point_on_path(pos, p, seed, 40)
        seed = idx
        ld = float(np.clip(1.0 + 0.3 * 3.0, 0.5, 3.0))
        i, tt, target = lookahead_point_on_path(pos, p, idx, t, ld)
        delta = np.clip(compute_steering(pos, yaw, target, wheelbase), -0.36, 0.36)
        v = 3.0
        pos = pos + v * np.array([math.cos(yaw), math.sin(yaw)]) * dt
        yaw += v / wheelbase * math.tan(delta) * dt
        if step > 500:
            errors.append(abs(np.linalg.norm(pos) - 10.0))

    assert max(errors) < 0.25, f'max radial error {max(errors):.3f} m'
    assert np.mean(errors) < 0.15


def test_lap_completes_without_index_stalling():
    """The seeded search must advance all the way round, not stick."""
    p = circle_path(radius=10.0, n=360, v=3.0)
    wheelbase, dt = 0.33, 0.02
    pos, yaw = np.array([10.0, 0.0]), math.pi / 2
    seed, visited = None, set()

    for _ in range(6000):
        idx, t, _, _ = nearest_point_on_path(pos, p, seed, 30)
        seed = idx
        visited.add(idx)
        i, tt, target = lookahead_point_on_path(pos, p, idx, t, 1.5)
        delta = np.clip(compute_steering(pos, yaw, target, wheelbase), -0.36, 0.36)
        pos = pos + 3.0 * np.array([math.cos(yaw), math.sin(yaw)]) * dt
        yaw += 3.0 / wheelbase * math.tan(delta) * dt

    assert len(visited) > 350, f'only reached {len(visited)}/360 waypoints'
