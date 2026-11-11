"""Clean-up pass planner tests (no ROS): pytest src/vacuum_coverage/test"""

import math
import os

import numpy as np
import pytest

from vacuum_coverage import core, reclean, sim, validate

RES = 0.05


def make_room(w_m=4.0, h_m=3.0, box=None, border=2):
    w, h = int(w_m / RES), int(h_m / RES)
    data = np.zeros((h, w), dtype=np.int8)
    data[:border, :] = 100
    data[-border:, :] = 100
    data[:, :border] = 100
    data[:, -border:] = 100
    if box:
        x0, y0, x1, y1 = [int(v / RES) for v in box]
        data[y0:y1, x0:x1] = 100
    return core.GridMap(data, RES, 0.0, 0.0)


def setup_room(**kw):
    grid = make_room(**kw)
    params = core.CoverageParams()
    masks = core.build_masks(grid, params, (1.0, 1.0))
    return grid, params, masks


def stamp_spots(grid, params, masks, covered, plan):
    tr = core.CoverageTracker(grid, masks.coverable, params.clean_radius)
    tr.covered = covered.copy()
    for p in plan.pieces:
        x, y, _ = p.poses[0]
        tr.stamp_xy(x, y)
    return tr


def patch(grid, masks, x0, y0, x1, y1):
    """Boolean mask of the coverable cells inside a rectangle given in metres."""
    m = np.zeros(masks.coverable.shape, dtype=bool)
    r0, c0 = grid.world_to_cell(x0, y0)
    r1, c1 = grid.world_to_cell(x1, y1)
    m[r0:r1, c0:c1] = True
    return m & masks.coverable


def test_window_gain_matches_brute_force():
    rng = np.random.default_rng(3)
    mask = rng.random((23, 31)) < 0.3
    r = 3.4
    gain = reclean.window_gain(mask, r)
    for row in (0, 5, 11, 22):
        for col in (0, 7, 15, 30):
            expect = 0
            for rr in range(mask.shape[0]):
                for cc in range(mask.shape[1]):
                    if mask[rr, cc] and math.hypot(rr - row, cc - col) <= r + 1e-9:
                        expect += 1
            assert gain[row, col] == expect


def test_nothing_to_do_when_everything_is_cleaned():
    grid, params, masks = setup_room()
    plan = reclean.plan_reclean(grid, masks, params, masks.coverable.copy(), (1.0, 1.0))
    assert plan.pieces == []
    assert plan.missed_m2 == 0.0 and plan.left_m2 == 0.0


def test_spots_clean_a_missed_patch():
    grid, params, masks = setup_room()
    missed = patch(grid, masks, 1.5, 1.0, 2.2, 1.7)
    assert missed.sum() > 100
    covered = masks.coverable & ~missed
    plan = reclean.plan_reclean(grid, masks, params, covered, (0.5, 0.5))
    assert 1 <= len(plan.pieces) <= 12
    assert all(p.kind == 'spot' and len(p.poses) == 1 for p in plan.pieces)
    tr = stamp_spots(grid, params, masks, covered, plan)
    assert tr.percent() == pytest.approx(100.0, abs=0.01)
    assert plan.left_m2 == 0.0
    assert plan.missed_m2 == pytest.approx(missed.sum() * RES * RES)


def test_spots_stay_on_cells_the_robot_can_reach():
    grid, params, masks = setup_room(box=(1.5, 1.0, 2.5, 2.0))
    covered = np.zeros_like(masks.coverable)           # nothing cleaned at all
    plan = reclean.plan_reclean(grid, masks, params, covered, (0.5, 0.5), max_spots=200)
    assert plan.pieces
    for p in plan.pieces:
        row, col = grid.world_to_cell(p.poses[0][0], p.poses[0][1])
        assert masks.reach[row, col]


def test_ignores_specks_below_min_area():
    grid, params, masks = setup_room()
    speck = np.zeros_like(masks.coverable)
    speck[20, 40] = True                               # one 5 cm cell
    assert (speck & masks.coverable).sum() == 1
    plan = reclean.plan_reclean(grid, masks, params, masks.coverable & ~speck, (1.0, 1.0))
    assert plan.pieces == []
    assert plan.left_m2 == pytest.approx(RES * RES)


def test_visits_the_nearest_patch_first_and_heads_towards_it():
    grid, params, masks = setup_room(w_m=6.0)
    left = patch(grid, masks, 0.5, 1.0, 1.1, 1.6)
    right = patch(grid, masks, 4.6, 1.0, 5.2, 1.6)
    covered = masks.coverable & ~left & ~right
    plan = reclean.plan_reclean(grid, masks, params, covered, (4.0, 1.3))
    xs = [p.poses[0][0] for p in plan.pieces]
    assert xs[0] > 3.5 and xs[-1] < 2.0                # right patch first, then the left one
    assert abs(plan.pieces[0].poses[0][2]) < 1.2       # first leg points towards +x


def test_max_spots_is_respected():
    grid, params, masks = setup_room()
    plan = reclean.plan_reclean(grid, masks, params, np.zeros_like(masks.coverable),
                                (1.0, 1.0), max_spots=5)
    assert len(plan.pieces) == 5
    assert plan.left_m2 > 0


def find_real_map():
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        for c in (os.path.join(here, 'vacuum_gazebo', 'maps', 'living_room.yaml'),
                  os.path.join(here, 'src', 'vacuum_gazebo', 'maps', 'living_room.yaml')):
            if os.path.exists(c):
                return c
        here = os.path.dirname(here)
    return None


def test_living_room_run_with_dropped_lanes_is_repaired():
    path = find_real_map()
    if path is None:
        pytest.skip('living_room map not found')
    grid = core.load_map_yaml(path)
    start = (-2.0, -0.5)
    plan, _ = validate.plan_validated(grid, core.CoverageParams(), start)
    lanes = [i for i, p in enumerate(plan.pieces) if p.kind == 'lane']
    drop = set(lanes[3:5] + lanes[-2:])                 # as if these pieces had been skipped
    kept = core.Plan([p for i, p in enumerate(plan.pieces) if i not in drop],
                     plan.masks, plan.params, plan.lane_axis)
    _, tracker, *_ = validate.evaluate(grid, kept, (start[0], start[1], 0.0), sim.PursuitParams())
    before = tracker.percent()
    assert before < 99.0
    extra = reclean.plan_reclean(grid, plan.masks, plan.params, tracker.covered, start)
    assert 0 < len(extra.pieces) <= 40
    for p in extra.pieces:
        tracker.stamp_xy(p.poses[0][0], p.poses[0][1])
    assert tracker.percent() > before + 2.0
    assert tracker.percent() > 96.0
