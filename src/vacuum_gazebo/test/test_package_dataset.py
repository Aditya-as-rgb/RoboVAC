"""Dataset packaging tests: pytest src/vacuum_gazebo/test"""

import importlib.util
import json
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), 'tools', 'dirt_dataset', 'package_dataset.py')
spec = importlib.util.spec_from_file_location('package_dataset', TOOL)
pk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pk)


def make_episodes(root, n_eps, frames=10, boxes_every=2):
    root.mkdir()
    (root / 'classes.txt').write_text('crumbs\ndust\nspill\nhair\n')
    for e in range(n_eps):
        for sub in ('images', 'labels'):
            (root / ('ep%d' % e) / sub).mkdir(parents=True)
        for f in range(frames):
            (root / ('ep%d' % e) / 'images' / ('%06d.jpg' % f)).write_bytes(b'jpg')
            lab = '2 0.5 0.5 0.1 0.1\n1 0.3 0.3 0.1 0.1\n' if f % boxes_every == 0 else ''
            (root / ('ep%d' % e) / 'labels' / ('%06d.txt' % f)).write_text(lab)


def test_split_is_by_episode_with_enough_episodes(tmp_path):
    make_episodes(tmp_path / 'src', 10)
    r = pk.package(str(tmp_path / 'src'), str(tmp_path / 'out'), seed=1, max_background=1.0)
    assert r['split'] == 'by_episode'
    seen = {}
    for sp in ('train', 'val', 'test'):
        for fn in os.listdir(tmp_path / 'out' / 'images' / sp):
            seen.setdefault(fn.split('_')[0], set()).add(sp)
    assert all(len(v) == 1 for v in seen.values())            # no episode in two splits
    assert {s for v in seen.values() for s in v} == {'train', 'val', 'test'}


def test_few_episodes_fall_back_to_time_blocks(tmp_path):
    make_episodes(tmp_path / 'src', 2, frames=20)
    r = pk.package(str(tmp_path / 'src'), str(tmp_path / 'out'), max_background=1.0)
    assert 'time_block' in r['split']
    assert r['images'] == {'train': 28, 'val': 6, 'test': 6}


def test_single_class_collapses_kinds_and_yaml_is_written(tmp_path):
    make_episodes(tmp_path / 'src', 5)
    pk.package(str(tmp_path / 'src'), str(tmp_path / 'out'), 'single', max_background=1.0)
    lab = next((tmp_path / 'out' / 'labels' / 'train').glob('*.txt'))
    for ln in (tmp_path / 'out' / 'labels' / 'train').glob('*.txt'):
        for row in ln.read_text().split('\n'):
            if row:
                assert row.split()[0] == '0'
    assert 'names:\n  0: dirt' in (tmp_path / 'out' / 'data.yaml').read_text()
    pk.package(str(tmp_path / 'src'), str(tmp_path / 'out2'), 'kind', max_background=1.0)
    assert '2: spill' in (tmp_path / 'out2' / 'data.yaml').read_text()
    kinds = {row.split()[0] for f in (tmp_path / 'out2' / 'labels' / 'train').glob('*.txt')
             for row in f.read_text().split('\n') if row}
    assert kinds == {'1', '2'}


def test_background_share_is_capped_and_zip_written(tmp_path):
    make_episodes(tmp_path / 'src', 5, frames=20, boxes_every=10)       # 90 % background
    r = pk.package(str(tmp_path / 'src'), str(tmp_path / 'out'), max_background=0.2, make_zip=True)
    n_img = sum(r['images'].values())
    n_bg = sum(1 for sp in ('train', 'val', 'test')
               for f in (tmp_path / 'out' / 'labels' / sp).glob('*.txt') if not f.read_text())
    assert n_bg / n_img <= 0.21
    assert os.path.exists(r['zip'])
    assert json.load(open(tmp_path / 'out' / 'dataset_report.json'))['background_dropped'] > 0
