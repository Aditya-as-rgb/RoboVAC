"""Dirt cleaner tests (no ROS, no Gazebo): pytest src/vacuum_gazebo/test"""

import csv
import importlib.util
import json
import math
import os
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(PKG, 'scripts', name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


dc = load('dirt_cleaner')
dw = load('dirt_world')

WORLD = os.path.join(PKG, 'worlds', 'living_room.world')
MAP = os.path.join(PKG, 'maps', 'living_room.yaml')


def patch(name='d0', x=0.0, y=0.0, scrub=0.05, kind='crumbs', stubborn=False):
    return dc.Patch(name, kind, x, y, 0.08, scrub, stubborn)


def drive(field, points, dt=0.1, t0=0.0):
    """Feed positions to the field; returns every patch cleaned on the way."""
    out = []
    for i, (x, y) in enumerate(points):
        out += field.update(x, y, t0 + i * dt)
    return out


def line(x0, y0, x1, y1, step=0.05):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


# ------------------------------------------------------------ geometry ---

def test_chord_through_the_centre_is_the_diameter():
    assert dc.chord_length((-1, 0), (1, 0), (0, 0), 0.2) == pytest.approx(0.4)


def test_chord_of_an_offset_line():
    off = 0.12
    assert dc.chord_length((-1, off), (1, off), (0, 0), 0.2) == pytest.approx(2 * math.sqrt(0.2 ** 2 - off ** 2))


def test_chord_clips_the_segment_ends():
    assert dc.chord_length((0, 0), (1, 0), (0, 0), 0.2) == pytest.approx(0.2)        # starts at the centre
    assert dc.chord_length((0.1, 0), (0.15, 0), (0, 0), 0.2) == pytest.approx(0.05)   # fully inside
    assert dc.chord_length((0.5, 0), (1.0, 0), (0, 0), 0.2) == 0.0                    # misses
    assert dc.chord_length((-1, 0.3), (1, 0.3), (0, 0), 0.2) == 0.0                   # passes by
    assert dc.chord_length((0, 0), (0, 0), (0, 0), 0.2) == 0.0                        # not moving


# --------------------------------------------------------------- field ---

def test_ordinary_patch_comes_off_on_the_first_pass():
    f = dc.DirtField([patch(x=1.0, y=0.0)], clean_radius=0.2)
    done = drive(f, line(0, 0, 2, 0))
    assert [p.name for p in done] == ['d0'] and f.remaining == 0
    assert 0.0 < done[0].removed_at < 2.0 * 10


def test_a_patch_the_brush_misses_stays():
    f = dc.DirtField([patch(x=1.0, y=0.45)], clean_radius=0.2)
    assert drive(f, line(0, 0, 2, 0)) == [] and f.remaining == 1


def test_brush_radius_decides_what_is_reached():
    p = lambda: patch(x=1.0, y=0.19)                                 # noqa: E731
    assert drive(dc.DirtField([p()], 0.20), line(0, 0, 2, 0))
    assert not drive(dc.DirtField([p()], 0.15), line(0, 0, 2, 0))


def test_stubborn_patch_needs_more_than_one_pass():
    f = dc.DirtField([patch(x=1.0, y=0.12, scrub=0.5, kind='spill', stubborn=True)], clean_radius=0.2)
    chord = 2 * math.sqrt(0.2 ** 2 - 0.12 ** 2)                       # about 0.32 m per pass
    assert drive(f, line(0, 0, 2, 0)) == []
    assert f.patches[0].scrubbed == pytest.approx(chord, abs=0.01)
    done = drive(f, line(2, 0, 0, 0))                                 # and back again
    assert [p.name for p in done] == ['d0']
    assert f.patches[0].scrubbed >= 0.5


def test_standing_still_or_jitter_does_not_scrub():
    f = dc.DirtField([patch(x=0.0, y=0.0, scrub=0.05)], clean_radius=0.2, min_step=0.02)
    jitter = [(0.003 * ((-1) ** i), 0.002 * ((-1) ** i)) for i in range(200)]
    assert drive(f, jitter) == [] and f.patches[0].scrubbed == 0.0


def test_a_localisation_jump_is_not_driving():
    f = dc.DirtField([patch(x=1.0, y=0.0)], clean_radius=0.2, max_step=1.0)
    assert drive(f, [(0.0, 0.0), (2.0 + 0.5, 0.0)]) == []             # 2.5 m in one tick, straight over it
    assert f.remaining == 1


def test_fast_driving_still_counts_the_whole_segment():
    f = dc.DirtField([patch(x=0.5, y=0.0)], clean_radius=0.2)
    assert [p.name for p in drive(f, [(0.0, 0.0), (0.9, 0.0)])] == ['d0']   # one big step over the patch


def test_removed_patches_are_not_cleaned_twice():
    f = dc.DirtField([patch(x=1.0, y=0.0)], clean_radius=0.2)
    drive(f, line(0, 0, 2, 0))
    assert drive(f, line(2, 0, 0, 0)) == []


def test_summary_counts():
    f = dc.DirtField([patch('a', 1.0, 0.0), patch('b', 1.0, 1.0, kind='spill', stubborn=True, scrub=0.5)],
                     clean_radius=0.2)
    drive(f, line(0, 0, 2, 0))
    s = f.summary()
    assert (s['total'], s['removed'], s['remaining']) == (2, 1, 1)
    assert s['by_kind']['crumbs'] == {'total': 1, 'removed': 1}
    assert s['by_kind']['spill'] == {'total': 1, 'removed': 0}
    assert s['stubborn'] == {'total': 1, 'removed': 0}
    assert s['remaining_patches'][0]['name'] == 'b'
    assert s['timeline'][0]['name'] == 'a'


# ----------------------------------------------------------- gazebo call ---

def fake_runner(calls, answers):
    def run(cmd, capture_output=True, text=True, timeout=10):
        calls.append(cmd)
        ok = answers.pop(0) if answers else True
        return SimpleNamespace(returncode=0 if ok else 1, stdout='data: true\n' if ok else '',
                               stderr='' if ok else 'Service call timed out')
    return run


def test_remove_command_is_the_documented_gz_service_call():
    cmd = dc.gz_remove_cmd('default', 'dirt_007')
    assert cmd[:4] == ['gz', 'service', '-s', '/world/default/remove']
    assert cmd[cmd.index('--reqtype') + 1] == 'gz.msgs.Entity'
    assert cmd[cmd.index('--reptype') + 1] == 'gz.msgs.Boolean'
    assert cmd[-1] == 'name: "dirt_007" type: MODEL'


def test_remover_retries_until_gazebo_answers():
    calls, sleeps, logs = [], [], []
    r = dc.Remover('default', fake_runner(calls, [False, False, True]), sleeps.append, logs.append,
                   retries=5, backoff=2.0)
    assert r.process('dirt_001') is True
    assert len(calls) == 3 and sleeps == [2.0, 2.0] and r.removed == ['dirt_001'] and not r.failed


def test_remover_gives_up_and_says_how_to_test_by_hand():
    calls, logs = [], []
    r = dc.Remover('default', fake_runner(calls, [False] * 9), lambda s: None, logs.append, retries=3)
    assert r.process('dirt_002') is False
    assert len(calls) == 3 and r.failed == ['dirt_002']
    assert 'gz service -s /world/default/remove' in logs[0] and 'timed out' in logs[0]


def test_remover_survives_a_missing_gz_binary():
    def boom(*a, **k):
        raise FileNotFoundError('gz')
    logs = []
    r = dc.Remover('default', boom, lambda s: None, logs.append)
    assert r.process('dirt_003') is False and 'not found' in logs[0]


# ------------------------------------------------------------- the core ---

def make_layout(tmp_path):
    res = dw.make_world(WORLD, MAP, 12, seed=3, spawn=(-2.0, -0.5), out_dir=str(tmp_path / 'dirt'))
    return res['current'], res['patches']


def test_layout_round_trip_with_the_generator(tmp_path):
    current, patches = make_layout(tmp_path)
    loaded, meta = dc.load_layout(current)
    assert [p.name for p in loaded] == [p['name'] for p in patches]
    assert [p.scrub_m for p in loaded] == [p['scrub_m'] for p in patches]
    assert meta['seed'] == 3 and meta['frame'] == 'world == map'
    assert all(p.scrub_m == dw.SCRUB_STUBBORN for p in loaded if p.kind == 'spill')


def test_old_layouts_without_scrub_fields_still_load(tmp_path):
    f = tmp_path / 'old.json'
    f.write_text(json.dumps({'patches': [{'name': 'dirt_000', 'kind': 'dust', 'x': 1, 'y': 2, 'radius_m': 0.05}]}))
    (p,), _ = dc.load_layout(str(f))
    assert p.scrub_m == dc.DEFAULT_SCRUB_M and p.stubborn is False


def test_core_cleans_what_the_robot_passes_and_writes_the_files(tmp_path):
    current, patches = make_layout(tmp_path)
    calls = []
    core = dc.CleanerCore(current, clean_radius=0.20, world='default', runner=fake_runner(calls, []),
                          sleep=lambda s: None, stamp='T')
    target = next(p for p in patches if not p['stubborn'])
    t = 100.0
    for x, y in line(target['x'] - 0.6, target['y'], target['x'] + 0.6, target['y']):
        core.step(x, y, t)
        t += 0.1
    core.remover.stop()
    cleaned = [p for p in core.patches if p.removed_at is not None]
    assert target['name'] in [p.name for p in cleaned]
    assert any(c[-1] == 'name: "%s" type: MODEL' % target['name'] for c in calls)

    rows = list(csv.DictReader(open(core.events_path)))
    assert [r['name'] for r in rows] == [p.name for p in sorted(cleaned, key=lambda q: q.removed_at)]
    assert int(rows[-1]['remaining']) == core.field.remaining
    assert float(rows[0]['t_s']) < 2.0                       # time counts from the first pose

    s = json.load(open(core.summary_path))
    assert s['total'] == 12 and s['removed'] == len(cleaned) and s['remaining'] == 12 - len(cleaned)
    assert s['gazebo_removed'] == len(cleaned) and s['gazebo_removal_failed'] == []
    assert os.path.dirname(core.summary_path) == os.path.dirname(current)


def test_core_without_gazebo_removal_still_counts(tmp_path):
    current, patches = make_layout(tmp_path)
    core = dc.CleanerCore(current, remove=False, stamp='T2')
    assert core.remover is None
    p = patches[0]
    t = 0.0
    for x, y in line(p['x'] - 0.5, p['y'], p['x'] + 0.5, p['y']):
        core.step(x, y, t)
        t += 0.1
    assert core.field.remaining < 12 or p['stubborn']
    core.close()
    assert 'gazebo_removed' not in json.load(open(core.summary_path))


def test_empty_layout_is_an_error(tmp_path):
    f = tmp_path / 'e.json'
    f.write_text(json.dumps({'patches': []}))
    with pytest.raises(ValueError):
        dc.CleanerCore(str(f), remove=False)


def test_remaining_names_shrinks_as_patches_come_off():
    f = dc.DirtField([patch('a', 0.0, 0.0, scrub=0.05), patch('b', 5.0, 5.0, scrub=0.05)], clean_radius=0.2)
    assert f.remaining_names() == ['a', 'b']
    f.update(-0.5, 0.0, 0.0)
    f.update(0.5, 0.0, 1.0)
    assert f.remaining_names() == ['b']
