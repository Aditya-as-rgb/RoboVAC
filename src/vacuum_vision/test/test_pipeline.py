import math

import numpy as np
import pytest

from vacuum_vision.geometry import CameraPose, Intrinsics, project
from vacuum_vision.pipeline import DirtVision, draw_detections, msg_to_rgb
from vacuum_vision.tracker import SpotTracker
from vacuum_vision.yolo import Detection

K = Intrinsics.from_fov(640, 480, 120.0)
IMG = np.zeros((480, 640, 3), np.uint8)


class FakeDetector:
    """Returns boxes that sit exactly on the projection of the given floor patches."""

    def __init__(self, pose, patches, half_px=12):
        self.pose, self.patches, self.half = pose, patches, half_px

    def detect(self, img):
        out = []
        for x, y, name in self.patches:
            u, v, z = project(self.pose, K, (x, y, 0.0))
            if z > 0 and 0 <= u <= 640 and 0 <= v <= 480:
                out.append(Detection(0, name, 0.8, u - self.half, v - self.half // 2,
                                     u + self.half, v + self.half // 2))
        return out


def vision(pose, patches, **kw):
    tracker = SpotTracker(min_hits=3, clear_misses=3)
    return DirtVision(FakeDetector(pose, patches), tracker, **kw), tracker


def test_spot_lands_on_the_patch():
    pose = CameraPose.from_mount(-2.0, -0.5, 0.4)
    c, s = math.cos(0.4), math.sin(0.4)
    patch = (-2.0 + 0.7 * c - 0.15 * s, -0.5 + 0.7 * s + 0.15 * c, 'crumbs')
    v, _ = vision(pose, [patch])
    for i in range(3):
        res = v.process(IMG, pose, K, i * 0.2)
    assert len(res.spots) == 1
    assert (res.spots[0].x, res.spots[0].y) == pytest.approx(patch[:2], abs=0.02)


def test_far_patch_ignored():
    pose = CameraPose.from_mount(0, 0, 0)
    v, t = vision(pose, [(2.6, 0.0, 'dust')], max_range=1.5)
    res = v.process(IMG, pose, K, 0.0)
    assert res.sightings == [] and t.spots == []


def test_cleaned_patch_disappears_when_camera_looks_at_it():
    pose = CameraPose.from_mount(0, 0, 0)
    patch = (0.8, 0.0, 'spill')
    v, t = vision(pose, [patch])
    for i in range(3):
        v.process(IMG, pose, K, i * 0.2)
    assert len(t.confirmed()) == 1
    v.detector.patches = []                                   # the brush took it away
    last = None
    for i in range(3):
        last = v.process(IMG, pose, K, 1.0 + i * 0.2)
    assert len(last.cleared) == 1 and t.confirmed() == []


def test_spot_kept_while_camera_looks_elsewhere():
    near = CameraPose.from_mount(0, 0, 0)
    patch = (0.8, 0.0, 'spill')
    v, t = vision(near, [patch])
    for i in range(3):
        v.process(IMG, near, K, i * 0.2)
    away = CameraPose.from_mount(0, 0, math.pi)               # facing the other way
    v.detector.pose, v.detector.patches = away, []
    for i in range(10):
        v.process(IMG, away, K, 1.0 + i * 0.2)
    assert len(t.confirmed()) == 1


def test_clear_zone_ignores_edge_of_image_and_far_range():
    pose = CameraPose.from_mount(0, 0, 0)
    v, _ = vision(pose, [], clear_range=1.2)
    assert v._in_clear_zone(pose, K, 0.8, 0.0)
    assert not v._in_clear_zone(pose, K, 2.0, 0.0)             # too far
    assert not v._in_clear_zone(pose, K, 0.8, 3.0)             # off to the side, outside the image
    assert not v._in_clear_zone(pose, K, -1.0, 0.0)            # behind


def test_msg_to_rgb_handles_padding_and_bgr():
    h, w, step = 2, 3, 12                                       # 3 px * 3 ch = 9 bytes, rows padded to 12
    row = bytes([1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0])
    rgb = msg_to_rgb(h, w, step, 'rgb8', row * 2)
    assert rgb.shape == (2, 3, 3) and list(rgb[0, 0]) == [1, 2, 3]
    bgr = msg_to_rgb(h, w, step, 'bgr8', row * 2)
    assert list(bgr[0, 0]) == [3, 2, 1]
    assert msg_to_rgb(h, w, step, 'mono8', row * 2) is None


def test_draw_detections_changes_pixels_not_the_input():
    img = np.zeros((100, 100, 3), np.uint8)
    out = draw_detections(img, [Detection(2, 'spill', 0.7, 10, 20, 50, 60)])
    assert out.shape == img.shape and out.any() and not img.any()
