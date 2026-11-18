import math

import numpy as np

from rc_localization.lib.frame_math import (
    Pose2D,
    compose,
    compose_offset,
    map_to_odom,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def test_quaternion_yaw_round_trip():
    for yaw in (0.0, 0.5, -1.2, math.pi / 2):
        assert math.isclose(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw, abs_tol=1e-9)


def test_identity_odometry_gives_correction_equal_to_the_estimate():
    pose = Pose2D(1.0, 2.0, 0.3)
    correction = map_to_odom(pose, Pose2D(0.0, 0.0, 0.0))
    assert math.isclose(correction.x, pose.x)
    assert math.isclose(correction.y, pose.y)
    assert math.isclose(correction.yaw, pose.yaw)


def test_correction_composed_with_odometry_reproduces_the_map_frame_estimate():
    map_to_base = Pose2D(5.0, -2.0, 1.0)
    odom_to_base = Pose2D(3.0, 1.0, 0.4)
    correction = map_to_odom(map_to_base, odom_to_base)

    cos_c, sin_c = math.cos(correction.yaw), math.sin(correction.yaw)
    x = correction.x + cos_c * odom_to_base.x - sin_c * odom_to_base.y
    y = correction.y + sin_c * odom_to_base.x + cos_c * odom_to_base.y
    yaw = correction.yaw + odom_to_base.yaw

    assert math.isclose(x, map_to_base.x, abs_tol=1e-9)
    assert math.isclose(y, map_to_base.y, abs_tol=1e-9)
    assert math.isclose(yaw, map_to_base.yaw, abs_tol=1e-9)


def test_compose_offset_matches_compose_row_by_row():
    poses = np.array([[0.0, 0.0, 0.0], [1.0, -2.0, 1.2], [-3.0, 0.5, -2.8]])
    mount = Pose2D(0.285, 0.0, 0.0)
    out = compose_offset(poses, mount, np.empty_like(poses))
    for row, got in zip(poses, out):
        want = compose(Pose2D(*row), mount)
        assert np.allclose(got, [want.x, want.y, want.yaw])
