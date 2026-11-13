"""The labeller and the detector must agree on the camera model.

Two independent copies of the pinhole geometry exist:

    vacuum_gazebo/scripts/dirt_labels.py   turns dirt positions into training boxes
    vacuum_vision/vacuum_vision/geometry.py turns detected boxes back into dirt positions

The training data is only correct while they describe the same camera from the same
mount. Nothing else in the suite compares them, so a change to one (a pitch tweak, a
transposed rotation, a swapped cx/cy) would silently poison every label. These tests
pin the contract: same intrinsics, same pose, same ray/floor round trip, and a box
labelled by one module lands back on the same patch when read by the other.
"""

import importlib.util
import math
import os

import numpy as np
import pytest

from vacuum_vision import geometry as vg

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))          # .../src
LABELS = os.path.join(SRC, 'vacuum_gazebo', 'scripts', 'dirt_labels.py')


def load_labels(path):
    spec = importlib.util.spec_from_file_location('dirt_labels', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


pytestmark = pytest.mark.skipif(not os.path.isfile(LABELS),
                                reason='vacuum_gazebo/scripts/dirt_labels.py not in this workspace')


@pytest.fixture(scope='module')
def dl():
    return load_labels(LABELS)


SQ = [[-0.03, -0.03], [0.03, -0.03], [0.03, 0.03], [-0.03, 0.03]]


def layout_patch(x, y, name='dirt_000', kind='crumbs'):
    return {'name': name, 'kind': kind, 'x': x, 'y': y, 'radius_m': 0.04,
            'hull': SQ, 'height_m': 0.004}


# ------------------------------------------------- intrinsics and pose --

def test_intrinsics_from_fov_agree(dl):
    a = dl.Intrinsics.from_fov(640, 480, 120.0)
    b = vg.Intrinsics.from_fov(640, 480, 120.0)
    for f in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
        assert getattr(a, f) == pytest.approx(getattr(b, f))


@pytest.mark.parametrize('x,y,yaw', [(0.0, 0.0, 0.0), (-2.0, -0.5, 0.7), (1.0, 2.0, math.pi)])
def test_mount_pose_agrees(dl, x, y, yaw):
    a = dl.CameraPose.from_mount(x, y, yaw)
    b = vg.CameraPose.from_mount(x, y, yaw)
    assert np.allclose(a.R, b.R, atol=1e-12)
    assert np.allclose(a.t, b.t, atol=1e-12)


@pytest.mark.parametrize('x,y,yaw', [(0.0, 0.0, 0.3), (2.0, -1.5, -1.1)])
def test_quaternion_pose_agrees(dl, x, y, yaw):
    # what the detector gets from TF: a quaternion and a translation
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    quat = (0.0, 0.0, qz, qw)
    t = (x, y, 0.07)
    a = dl.CameraPose.from_quaternion(*quat, t)
    b = vg.CameraPose.from_quaternion(*quat, t)
    assert np.allclose(a.R, b.R, atol=1e-12) and np.allclose(a.t, b.t, atol=1e-12)


# ------------------------------------------------------ ray and floor --

def test_pixel_to_floor_agrees_and_handles_the_horizon(dl):
    K_lab = dl.Intrinsics.from_fov(640, 480, 120.0)
    K_vis = vg.Intrinsics.from_fov(640, 480, 120.0)
    pose_lab = dl.CameraPose.from_mount(0.5, -0.25, 0.4)
    pose_vis = vg.CameraPose.from_mount(0.5, -0.25, 0.4)
    checked = 0
    for u in range(0, 640, 37):
        for v in range(0, 480, 23):
            a = dl.pixel_to_floor(pose_lab, K_lab, u, v)
            b = vg.pixel_to_floor(pose_vis, K_vis, u, v)
            assert (a is None) == (b is None), 'horizon test differs at (%d, %d)' % (u, v)
            if a is not None:
                assert a == pytest.approx(b, abs=1e-9)
                checked += 1
    assert checked > 50


def test_project_agrees_on_the_same_world_point(dl):
    K_lab = dl.Intrinsics.from_fov(640, 480, 120.0)
    K_vis = vg.Intrinsics.from_fov(640, 480, 120.0)
    pose_lab = dl.CameraPose.from_mount(-2.0, -0.5, 2.1)
    pose_vis = vg.CameraPose.from_mount(-2.0, -0.5, 2.1)
    pts = np.array([[x, y, 0.0] for x in (-1.0, 0.0, 1.0) for y in (-0.5, 0.5)])
    uv_lab, z_lab = dl.project(pose_lab, K_lab, pts)
    for (u, v), z, (x, y, _) in zip(uv_lab, z_lab, pts):
        u2, v2, z2 = vg.project(pose_vis, K_vis, (x, y, 0.0))
        assert (u2, v2) == pytest.approx((u, v), abs=1e-9)
        assert z2 == pytest.approx(z, abs=1e-9)


# -------------------------------------------- label -> detection -> label --

def test_a_labelled_box_centre_maps_back_to_the_patch(dl):
    """The detector's floor position for a training box must be where the patch really is.

    This is the round trip the whole dataset depends on: dirt at a known place is labelled
    by dirt_labels, the box centre is fed to the detector's pixel_to_floor, and the result
    has to land on the dirt again — including the camera mount, not a nadir idealisation.
    """
    K_lab = dl.Intrinsics.from_fov(640, 480, 120.0)
    K_vis = vg.Intrinsics.from_fov(640, 480, 120.0)
    pose_lab = dl.CameraPose.from_mount(0.4, 0.3, 0.35)
    pose_vis = vg.CameraPose.from_mount(0.4, 0.3, 0.35)

    for dist in (0.45, 0.7, 1.0):
        x = 0.4 + dist * math.cos(0.35)
        y = 0.3 + dist * math.sin(0.35)
        res = dl.label_patches([layout_patch(x, y)], pose_lab, K_lab)
        assert len(res.labels) == 1, 'patch at %.1f m was not labelled' % dist
        lab = res.labels[0]

        # what vacuum_vision's pipeline accepts (pipeline.DirtVision drops anything smaller)
        cx, cy = lab.x + lab.w / 2.0, lab.y + lab.h / 2.0
        assert lab.w >= 4 and lab.h >= 3
        got = vg.pixel_to_floor(pose_vis, K_vis, cx, cy)
        assert got is not None
        # the box spans the patch's top face too, so its centre sits a little above the
        # floor centre: ~2 cm at 1 m, growing with distance. The mount must not add to it.
        assert math.hypot(got[0] - x, got[1] - y) < 0.03       # 3 cm
