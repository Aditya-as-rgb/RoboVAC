import math

import numpy as np
import pytest

from vacuum_vision.geometry import CameraPose, Intrinsics, pixel_to_floor, project

K = Intrinsics.from_fov(640, 480, 120.0)


@pytest.mark.parametrize('rx,ry,yaw', [(0, 0, 0), (-2.0, -0.5, 0), (1.0, 2.0, math.pi / 3), (0.5, -1.0, -2.5)])
def test_floor_point_round_trip(rx, ry, yaw):
    pose = CameraPose.from_mount(rx, ry, yaw)
    # a floor point 0.6 m straight ahead and 0.1 m to the left of the robot
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy = rx + 0.6 * c - 0.1 * s, ry + 0.6 * s + 0.1 * c
    u, v, z = project(pose, K, (wx, wy, 0.0))
    assert z > 0 and 0 <= u <= 640 and 0 <= v <= 480
    got = pixel_to_floor(pose, K, u, v)
    assert got == pytest.approx((wx, wy), abs=1e-6)


def test_floor_ahead_is_in_the_lower_half_and_left_is_left():
    pose = CameraPose.from_mount(0, 0, 0)
    u_l, v_l, _ = project(pose, K, (0.8, 0.2, 0.0))     # left of the robot (+y)
    u_r, v_r, _ = project(pose, K, (0.8, -0.2, 0.0))
    assert u_l < 320 < u_r and v_l == pytest.approx(v_r)


def test_horizon_pixel_has_no_floor():
    pose = CameraPose.from_mount(0, 0, 0)
    assert pixel_to_floor(pose, K, 320, 0) is None          # top row of the image looks above the floor


def test_bottom_row_is_close_to_the_robot():
    pose = CameraPose.from_mount(0, 0, 0)
    x, y = pixel_to_floor(pose, K, 320, 479)
    assert 0.17 < x < 0.45 and abs(y) < 1e-6


def test_quaternion_matches_mount():
    p = CameraPose.from_mount(1.0, 2.0, 0.7)
    # quaternion of R from a rotation matrix (w >= 0 branch is enough for this test)
    R = p.R
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    q2 = CameraPose.from_quaternion(*q, p.t)
    assert np.allclose(q2.R, p.R, atol=1e-9)
