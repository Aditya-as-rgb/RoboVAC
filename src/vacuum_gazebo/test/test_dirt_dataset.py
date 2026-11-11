"""Dataset recorder tests (no ROS, no Gazebo): pytest src/vacuum_gazebo/test"""

import importlib.util
import json
import os
import sys

import numpy as np
import pytest

pytest.importorskip('PIL')
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('dirt_dataset', os.path.join(PKG, 'scripts', 'dirt_dataset.py'))
dd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dd)
dl = dd.dirt_labels
K = dl.Intrinsics.from_fov(640, 480, 120.0)      # real size so the min-box rules behave as in the sim
SQ = [[-0.03, -0.03], [0.03, -0.03], [0.03, 0.03], [-0.03, 0.03]]
IMG = np.zeros((48, 64, 3), np.uint8)


def layout(patches):
    return {'seed': 1, 'world': 'w.world', 'environment': None, 'patches': patches}


def patch(x, y, name='dirt_000', kind='spill'):
    return {'name': name, 'kind': kind, 'x': x, 'y': y, 'radius_m': 0.04, 'hull': SQ, 'height_m': 0.004}


def pose(x=0.0, y=0.0, yaw=0.0):
    return dl.CameraPose.from_mount(x, y, yaw)


def test_saves_image_and_yolo_label_with_the_right_class(tmp_path):
    rec = dd.DatasetRecorder(str(tmp_path), 'ep', layout([patch(0.5, 0.0, kind='spill')]))
    assert rec.offer(0.0, IMG, pose(), K) is not None
    rec.close()
    lines = open(tmp_path / 'ep' / 'labels' / '000000.txt').read().split('\n')[:-1]
    assert len(lines) == 1 and lines[0].split()[0] == str(dd.CLASSES.index('spill'))
    assert all(0 <= float(v) <= 1 for v in lines[0].split()[1:])
    assert (tmp_path / 'ep' / 'images' / '000000.jpg').exists()
    assert open(tmp_path / 'classes.txt').read().split() == dd.CLASSES
    info = json.load(open(tmp_path / 'ep' / 'episode.json'))
    assert info['frames_saved'] == 1 and info['with_dirt'] == 1


def test_near_duplicate_and_too_fast_frames_are_dropped(tmp_path):
    rec = dd.DatasetRecorder(str(tmp_path), 'ep', layout([patch(0.5, 0.0)]))
    assert rec.offer(0.0, IMG, pose(), K)
    assert rec.offer(0.1, IMG, pose(0.5, 0.0), K) is None                 # too soon
    assert rec.offer(1.0, IMG, pose(0.01, 0.0), K) is None                # has not moved
    assert rec.offer(2.0, IMG, pose(0.0, 0.0, 0.5), K)                    # turned ~29 degrees
    assert rec.dropped['too_soon'] == 1 and rec.dropped['too_close_to_last'] == 1


def test_ambiguous_frames_are_dropped(tmp_path):
    rec = dd.DatasetRecorder(str(tmp_path), 'ep', layout([patch(0.9, 0.0)]), policy=dd.FramePolicy(neg_prob=1.0))
    tiny = dict(patch(0.9, 0.0), hull=[[-0.001, -0.001], [0.001, -0.001], [0.001, 0.001], [-0.001, 0.001]],
                height_m=0.0005)
    rec2 = dd.DatasetRecorder(str(tmp_path), 'ep2', layout([tiny]), policy=dd.FramePolicy(neg_prob=1.0))
    assert rec2.offer(0.0, IMG, pose(), K) is None
    assert rec2.dropped['ambiguous'] == 1 and rec2.saved == 0
    rec.close(), rec2.close()


def test_background_frames_are_kept_only_sometimes(tmp_path):
    never = dd.DatasetRecorder(str(tmp_path), 'a', layout([patch(5.0, 5.0)]), policy=dd.FramePolicy(neg_prob=0.0))
    always = dd.DatasetRecorder(str(tmp_path), 'b', layout([patch(5.0, 5.0)]), policy=dd.FramePolicy(neg_prob=1.0))
    assert never.offer(0.0, IMG, pose(), K) is None
    assert always.offer(0.0, IMG, pose(), K) is not None
    assert open(tmp_path / 'b' / 'labels' / '000000.txt').read() == ''
    assert always.negatives == 1


def test_cleaned_patch_is_not_labelled(tmp_path):
    rec = dd.DatasetRecorder(str(tmp_path), 'ep', layout([patch(0.5, 0.0, 'a'), patch(0.5, 0.2, 'b')]))
    res = rec.offer(0.0, IMG, pose(), K, remaining={'b'})
    assert [l.name for l in res.labels] == ['b']


def test_image_conversion():
    class Msg:
        encoding, width, height, step = 'bgr8', 2, 1, 6
        data = bytes([1, 2, 3, 4, 5, 6])
    assert image_to_list(Msg) == [[3, 2, 1], [6, 5, 4]]


def image_to_list(msg):
    return dd.image_msg_to_rgb(msg)[0].tolist()
