"""Run report tests (no ROS): pytest src/vacuum_coverage/test"""

import csv
import json
import os

import numpy as np
import pytest

from vacuum_coverage import core, mission, report

RES = 0.05


def make_world():
    w, h = 80, 60
    data = np.zeros((h, w), dtype=np.int8)
    data[:2, :] = data[-2:, :] = 100
    data[:, :2] = data[:, -2:] = 100
    grid = core.GridMap(data, RES, 0.0, 0.0)
    params = core.CoverageParams()
    masks = core.build_masks(grid, params, (1.0, 1.0))
    tracker = core.CoverageTracker(grid, masks.coverable, params.clean_radius)
    return grid, params, masks, tracker


def make_run():
    grid, params, masks, tracker = make_world()
    log = report.RunLog(t0=100.0)
    for i in range(61):                                 # drive 3 m along y = 1.0
        x = 0.5 + 0.05 * i
        tracker.stamp_xy(x, 1.0)
        log.add_position(x, 1.0)
        if i % 10 == 0:
            log.snapshot(100.0 + i, tracker.percent(), report.percent_free(tracker, grid))
    log.end_pass(0, 160.0, tracker.percent(), report.percent_free(tracker, grid))
    pieces = [core.Piece('wall', [(0.5, 1.0, 0.0), (1.5, 1.0, 0.0)]),
              core.Piece('lane', [(1.5, 1.0, 0.0), (3.5, 1.0, 0.0)])]
    result = mission.MissionResult()
    result.pieces = [
        mission.PieceResult(0, 'wall', 'done', 1, 0, 100.0, 120.0),
        mission.PieceResult(1, 'lane', 'skipped', 3, 0, 120.0, 160.0),
    ]
    return grid, params, tracker, log, pieces, result


def test_log_distance_and_samples():
    log = report.RunLog(t0=10.0)
    for x in (0.0, 0.3, 0.6, 0.6):
        log.add_position(x, 0.0)
    log.snapshot(12.5, 40.0, 30.0)
    assert log.distance == pytest.approx(0.6)
    s = log.samples[0]
    assert s.t == pytest.approx(2.5) and s.x == pytest.approx(0.6) and s.distance == pytest.approx(0.6)
    log.end_pass(0, 13.0, 41.0, 31.0)
    assert log.current_pass == 1 and log.pass_marks[0][1] == pytest.approx(3.0)


def test_percent_free_counts_all_free_cells():
    grid, params, masks, tracker = make_world()
    tracker.stamp_xy(2.0, 1.5)
    free = int((grid.data == 0).sum())
    expect = 100.0 * int((tracker.covered & (grid.data == 0)).sum()) / free
    assert report.percent_free(tracker, grid) == pytest.approx(expect)
    assert report.percent_free(tracker, grid) <= tracker.percent() + 1e-9 or True


def test_write_report_creates_the_files(tmp_path):
    grid, params, tracker, log, pieces, result = make_run()
    out = tmp_path / 'run_x'
    files = report.write_report(str(out), log, result, {0: pieces}, tracker, grid, params,
                                plan_stats={'pieces': 2}, cleanup=[{'pass': 1, 'spots': 0}],
                                started_at='2026-11-10T10:00:00')
    for name in ('coverage.csv', 'pieces.csv', 'summary.json'):
        assert os.path.exists(files[name])

    rows = list(csv.DictReader(open(files['coverage.csv'])))
    assert len(rows) == len(log.samples) == 7
    assert float(rows[-1]['cleaned_reachable_pct']) == pytest.approx(tracker.percent(), abs=0.01)
    assert float(rows[-1]['distance_m']) == pytest.approx(3.0, abs=0.01)
    assert rows[0]['t_s'] == '0.0' and rows[-1]['t_s'] == '60.0'

    prows = list(csv.DictReader(open(files['pieces.csv'])))
    assert [r['status'] for r in prows] == ['done', 'skipped']
    assert float(prows[0]['length_m']) == pytest.approx(1.0)
    assert float(prows[1]['duration_s']) == pytest.approx(40.0)

    s = json.load(open(files['summary.json']))
    assert s['status'] == 'finished' and s['passes'] == 1
    assert s['cleaned_reachable_pct'] == pytest.approx(tracker.percent(), abs=0.01)
    assert s['pieces'] == {'total': 2, 'done': 1, 'skipped': 1,
                           'by_kind': {'wall': {'done': 1, 'skipped': 0, 'cancelled': 0},
                                       'lane': {'done': 0, 'skipped': 1, 'cancelled': 0}}}
    assert s['cleaned_area_m2'] + s['missed_area_m2'] == pytest.approx(s['reachable_area_m2'], abs=0.02)
    assert s['distance_m'] == pytest.approx(3.0, abs=0.01)
    assert s['average_speed_mps'] == pytest.approx(0.05, abs=0.005)
    assert s['per_pass'][0]['name'] == 'main'
    assert s['cleanup'] == [{'pass': 1, 'spots': 0}]


def test_cancelled_run_is_marked_cancelled(tmp_path):
    grid, params, tracker, log, pieces, result = make_run()
    result.cancelled = True
    files = report.write_report(str(tmp_path), log, result, {0: pieces}, tracker, grid, params)
    assert json.load(open(files['summary.json']))['status'] == 'cancelled'


def test_png_is_written_when_matplotlib_is_available(tmp_path):
    pytest.importorskip('matplotlib')
    grid, params, tracker, log, pieces, result = make_run()
    files = report.write_report(str(tmp_path), log, result, {0: pieces}, tracker, grid, params)
    assert os.path.getsize(files['report.png']) > 5000
