"""Dirt labelling tests (no ROS, no Gazebo): pytest src/vacuum_gazebo/test"""

import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(PKG, 'scripts', name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


dl = load('dirt_labels')
dw = load('dirt_world')
K = dl.Intrinsics.from_fov(640, 480, 120.0)
SQ = [[-0.03, -0.03], [0.03, -0.03], [0.03, 0.03], [-0.03, 0.03]]


def patch(x, y, name='dirt_000', kind='crumbs', h=0.004, hull=SQ):
    return {'name': name, 'kind': kind, 'x': x, 'y': y, 'radius_m': 0.04, 'hull': hull, 'height_m': h}


def robot_pose(x=0.0, y=0.0, yaw=0.0):
    return dl.CameraPose.from_mount(x, y, yaw)


def test_nadir_camera_gives_the_analytic_box():
    # camera 1 m above the floor looking straight down: x_world -> image down, y_world -> image left
    R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, -1.0]])        # cols: optical x,y,z in world
    pose = dl.CameraPose(R, np.array([0.0, 0.0, 1.0]))
    k = dl.Intrinsics(500, 500, 320, 240, 640, 480)
    res = dl.label_patches([patch(0.0, 0.0, h=1e-6, hull=[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]])],
                           pose, k)
    (lab,) = res.labels
    assert abs(lab.w - 100) < 0.5 and abs(lab.h - 100) < 0.5
    assert abs(lab.x + lab.w / 2 - 320) < 0.5 and abs(lab.y + lab.h / 2 - 240) < 0.5


def test_patch_ahead_is_in_the_picture_and_left_is_left():
    pose = robot_pose()
    centre = dl.label_patches([patch(0.5, 0.0)], pose, K).labels[0]
    left = dl.label_patches([patch(0.5, 0.2)], pose, K).labels[0]
    right = dl.label_patches([patch(0.5, -0.2)], pose, K).labels[0]
    near = dl.label_patches([patch(0.3, 0.0)], pose, K).labels[0]
    cu = lambda l: l.x + l.w / 2
    cv = lambda l: l.y + l.h / 2
    assert abs(cu(centre) - 320) < 2
    assert cu(left) < 320 < cu(right)                 # +y is the robot's left
    assert cv(near) > cv(centre)                      # nearer = lower in the picture
    assert centre.w > centre.h                        # foreshortened: wide and short


def test_pose_follows_the_robot_yaw():
    pose = robot_pose(1.0, 2.0, math.pi / 2)          # facing +y
    lab = dl.label_patches([patch(1.0, 2.5)], pose, K).labels[0]
    assert abs(lab.x + lab.w / 2 - 320) < 2
    assert not dl.label_patches([patch(1.0, 1.0)], pose, K).labels      # behind


def test_pixel_to_floor_round_trip():
    pose = robot_pose(0.3, -0.2, 0.7)
    for (x, y) in [(1.2, 0.1), (0.9, -0.6), (1.5, 0.8)]:
        uv, z = dl.project(pose, K, np.array([[x, y, 0.0]]))
        assert z[0] > 0
        gx, gy = dl.pixel_to_floor(pose, K, uv[0, 0], uv[0, 1])
        assert abs(gx - x) < 1e-6 and abs(gy - y) < 1e-6


def test_pixel_above_horizon_has_no_floor_point():
    assert dl.pixel_to_floor(robot_pose(), K, 320, 0) is None


def test_removed_patch_is_not_labelled():
    res = dl.label_patches([patch(0.5, 0.0, 'a'), patch(0.5, 0.2, 'b')], robot_pose(), K, remaining={'b'})
    assert [l.name for l in res.labels] == ['b']
    assert ('a', 'removed') in res.ignored


def test_far_patch_is_ignored_and_does_not_spoil_the_frame():
    res = dl.label_patches([patch(2.6, 0.0)], robot_pose(), K)
    assert not res.labels and res.usable


def test_tiny_patch_close_up_makes_frame_ambiguous_but_far_one_does_not():
    tiny = [[-0.002, -0.002], [0.002, -0.002], [0.002, 0.002], [-0.002, 0.002]]
    near = dl.label_patches([patch(0.9, 0.0, hull=tiny, h=0.001)], robot_pose(), K)
    far = dl.label_patches([patch(1.8, 0.0, hull=tiny, h=0.001)], robot_pose(), K)
    assert not near.usable and far.usable


def test_patch_cut_by_image_edge_is_ambiguous():
    # far to the left so only part of the box is inside
    pose = robot_pose()
    uv, _ = dl.project(pose, K, np.array([[0.6, 0.0, 0.0]]))
    for y in np.linspace(0.3, 1.2, 40):
        res = dl.label_patches([patch(0.6, y)], pose, K)
        if res.ambiguous:
            assert 'edge' in res.ambiguous[0][1]
            return
    raise AssertionError('never saw a partially visible patch')


def test_behind_the_camera_is_ignored():
    res = dl.label_patches([patch(-0.5, 0.0)], robot_pose(), K)
    assert not res.labels and res.usable


def test_wall_in_between_makes_the_patch_ambiguous(tmp_path):
    free = np.ones((40, 40), bool)
    free[:, 20] = False                                # wall at x = 1.0 (res 0.05, origin 0)
    fmap = dw.FloorMap(free, ~free, 0.05, 0.0, 0.0)
    pose = robot_pose(0.2, 1.0, 0.0)
    wall = dl.label_patches([patch(1.4, 1.0)], pose, K, fmap=fmap)
    assert wall.usable and not wall.labels and wall.ignored[0][1].startswith('hidden')
    clear = dl.label_patches([patch(0.85, 1.0)], pose, K, fmap=fmap)
    assert clear.labels and clear.usable


def test_patch_against_a_wall_is_not_hidden_by_that_wall():
    free = np.ones((40, 40), bool)
    free[:, 24:] = False                               # wall from x = 1.2
    fmap = dw.FloorMap(free, ~free, 0.05, 0.0, 0.0)
    res = dl.label_patches([patch(1.17, 1.0)], robot_pose(0.5, 1.0), K, fmap=fmap)
    assert res.labels


def test_yolo_line_is_normalised():
    lab = dl.Label('a', 'crumbs', 100, 200, 64, 48, 1.0, 1.0)
    cls, cx, cy, w, h = lab.yolo(K, 2).split()
    assert cls == '2'
    assert (float(cx), float(cy), float(w), float(h)) == (0.20625, 0.466667, 0.1, 0.1)


def test_quaternion_matches_mount_rotation():
    p = dl.CameraPose.from_mount(0, 0, 0.0)
    # rotation of the optical frame about z by 90 degrees == yaw 90
    h = math.sqrt(0.5)
    q = dl.CameraPose.from_quaternion(0, 0, h, h, [0, 0, 0])
    rot = q.R @ np.array([1.0, 0, 0])
    assert np.allclose(rot, [0, 1, 0])
    assert np.allclose(np.linalg.det(p.R), 1.0)


def test_patch_only_partly_behind_a_post_is_ambiguous():
    free = np.ones((40, 40), bool)
    free[20, 10:12] = False                              # a thin post at x 0.5..0.6, y 1.0..1.05
    fmap = dw.FloorMap(free, ~free, 0.05, 0.0, 0.0)
    wide = [[-0.4, -0.4], [0.4, -0.4], [0.4, 0.4], [-0.4, 0.4]]
    res = dl.label_patches([patch(1.4, 1.025, hull=wide)], robot_pose(0.2, 1.025, 0.0), K, fmap=fmap)
    assert not res.usable and res.ambiguous[0][1] == 'partly occluded'
