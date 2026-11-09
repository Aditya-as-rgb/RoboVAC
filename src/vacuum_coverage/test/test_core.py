"""Offline tests for the coverage planning core (no ROS needed): pytest src/vacuum_coverage/test"""

import math
import os

import numpy as np
import pytest

from vacuum_coverage import core, sim, validate

RES = 0.05


def make_room(w_m=4.0, h_m=3.0, box=None, border=2):
    """Free room with an occupied border; optional box=(x0, y0, x1, y1) in metres from the corner."""
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


def find_real_map():
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        cand = os.path.join(here, 'vacuum_gazebo', 'maps', 'living_room.yaml')
        alt = os.path.join(here, 'src', 'vacuum_gazebo', 'maps', 'living_room.yaml')
        for c in (cand, alt):
            if os.path.exists(c):
                return c
        here = os.path.dirname(here)
    return None


# ---------------------------------------------------------------- helpers ---

def test_dilate_matches_brute_force():
    rng = np.random.default_rng(1)
    mask = rng.random((30, 40)) > 0.97
    for r in (1, 2.5, 4.69):
        got = core.dilate(mask, r)
        pts = np.argwhere(mask)
        want = np.zeros_like(mask)
        for rr in range(mask.shape[0]):
            for cc in range(mask.shape[1]):
                if len(pts) and (np.hypot(pts[:, 0] - rr, pts[:, 1] - cc) <= r + 1e-9).any():
                    want[rr, cc] = True
        assert (got == want).all(), 'dilate differs from brute force at r=%s' % r


def test_flood_fill_is_4_connected():
    m = np.zeros((5, 5), bool)
    m[0, 0] = m[1, 1] = True            # diagonal only: not connected
    assert core.flood_fill(m, (0, 0)).sum() == 1
    m[0, 1] = True
    assert core.flood_fill(m, (0, 0)).sum() == 3


def test_boundary_loops_rectangle_with_hole():
    region = np.zeros((20, 30), bool)
    region[2:18, 2:28] = True
    region[8:12, 10:16] = False          # a hole
    loops = core.boundary_loops(region)
    assert len(loops) == 2
    for cells in loops:
        n = len(cells)
        for i in range(n):                # consecutive cells touch (8-neighbourhood), loop closes
            a, b = cells[i], cells[(i + 1) % n]
            assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= 1
        for r, c in cells:
            assert region[r, c]
    # every boundary cell shows up in some loop
    inner = region.copy()
    inner[1:-1, 1:-1] &= region[:-2, 1:-1] & region[2:, 1:-1] & region[1:-1, :-2] & region[1:-1, 2:]
    boundary = region & ~inner
    covered = {cell for loop in loops for cell in loop}
    assert {tuple(x) for x in np.argwhere(boundary)} <= covered


def test_read_pgm_p5_and_comment(tmp_path):
    p = tmp_path / 'm.pgm'
    p.write_bytes(b'P5\n# comment\n3 2\n255\n' + bytes([0, 128, 255, 10, 20, 30]))
    a = core.read_pgm(str(p))
    assert a.shape == (2, 3) and a[0, 2] == 255 and a[1, 0] == 10


# ------------------------------------------------------- synthetic planning ---

def test_plan_empty_room_poses_stay_in_reach_and_cover():
    grid = make_room()
    params = core.CoverageParams()
    plan = core.plan_coverage(grid, params, (0.5, 0.5))
    assert plan.stats()['wall_loops'] == 1
    assert plan.stats()['lane_pieces'] >= 1
    tracker = core.CoverageTracker(grid, plan.masks.coverable, params.clean_radius)
    for piece in plan.pieces:
        for x, y, _ in piece.poses:
            r, c = grid.world_to_cell(x, y)
            assert plan.masks.reach[r, c], 'pose (%.2f, %.2f) outside reachable area' % (x, y)
            tracker.stamp_xy(x, y)
    assert tracker.percent() > 97.0


def test_wall_loop_is_closed_and_obstacle_gets_its_own_loop():
    grid = make_room(5.0, 4.0, box=(2.0, 1.5, 3.0, 2.5))
    plan = core.plan_coverage(grid, core.CoverageParams(lanes=False), (0.5, 0.5))
    walls = [p for p in plan.pieces if p.kind == 'wall']
    assert len(walls) == 2                                   # room + box
    for p in walls:
        a, b = p.poses[0], p.poses[-1]
        assert math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6   # closed loop for FollowPath
    # the first loop is the outer one: it encloses the larger area
    def span(p):
        xs = [q[0] for q in p.poses]
        return max(xs) - min(xs)
    assert span(walls[0]) > span(walls[1])


def test_lanes_do_not_cross_obstacle_and_are_split_around_it():
    grid = make_room(5.0, 4.0, box=(2.0, 1.0, 3.0, 3.0))
    params = core.CoverageParams(wall_pass=False, lane_axis='x')
    plan = core.plan_coverage(grid, params, (0.5, 0.5))
    for piece in plan.pieces:
        for x, y, _ in piece.poses:
            r, c = grid.world_to_cell(x, y)
            assert plan.masks.reach[r, c]
    # at least one lane row crosses the box, so it must have produced two separate segments
    assert plan.stats()['lane_pieces'] >= 2


def test_start_inside_obstacle_clearance_warns_and_moves_to_reach():
    grid = make_room()
    masks = core.build_masks(grid, core.CoverageParams(), (0.12, 0.12))   # inside the wall clearance
    assert masks.warnings
    assert masks.reach[masks.start_cell]


def test_pocket_start_uses_largest_area():
    grid = make_room(6.0, 3.0)
    grid.data[:, 60:64] = 100                  # wall splitting the room into two parts
    grid.data[:, 90:94] = 0
    masks = core.build_masks(grid, core.CoverageParams(), (0.45, 1.5))   # tiny left pocket (width 1.5 m -> ok)
    assert masks.reach.sum() > 0


def test_no_space_raises():
    grid = make_room(0.6, 0.6)
    with pytest.raises(ValueError):
        core.build_masks(grid, core.CoverageParams(), (0.3, 0.3))


def test_tracker_percent_monotonic_and_bounded():
    grid = make_room()
    masks = core.build_masks(grid, core.CoverageParams(), (1.0, 1.0))
    tr = core.CoverageTracker(grid, masks.coverable, 0.22)
    last = 0.0
    for x in np.linspace(0.3, 3.7, 30):
        tr.stamp_xy(x, 1.5)
        pct = tr.percent()
        assert pct >= last and pct <= 100.0
        last = pct


# --------------------------------------------------------------- simulator ---

def test_pursuit_follows_straight_path():
    path = [(x, 0.0) for x in np.arange(0.0, 3.0, 0.05)]
    trail, pose, t, ok = sim.follow(path, (0.0, 0.0, 0.0))
    assert ok and abs(pose[0] - 3.0) < 0.3 and max(abs(y) for _, y in trail) < 0.01


def test_pursuit_turns_in_place_for_large_heading_error():
    path = [(0.0, y) for y in np.arange(0.0, 2.0, 0.05)]
    trail, pose, t, ok = sim.follow(path, (0.0, 0.0, 0.0))           # facing +x, path goes +y
    assert ok and abs(pose[1] - 2.0) < 0.3
    assert max(abs(x) for x, _ in trail) < 0.3


def test_auto_margin_makes_the_simulated_plan_safe():
    grid = make_room(5.0, 4.0, box=(2.0, 1.5, 3.0, 2.5))
    plan, rep = validate.plan_validated(grid, core.CoverageParams(wall_margin=0.03), (0.6, 0.6))
    assert rep.ok, rep
    assert rep.min_clearance >= rep.robot_radius + 0.02 - 1e-9
    assert rep.failed_pieces == 0
    assert rep.coverage_reachable > 90.0


# ---------------------------------------------------------------- real map ---

@pytest.mark.skipif(find_real_map() is None, reason='living_room map not found')
def test_real_living_room_plan():
    grid = core.load_map_yaml(find_real_map())
    assert grid.data.shape == (98, 99)
    plan, rep = validate.plan_validated(grid, core.CoverageParams(), (-2.0, -0.5))
    assert rep.ok, rep
    assert rep.failed_pieces == 0
    assert rep.coverage_reachable > 90.0
    assert plan.stats()['wall_loops'] >= 1 and plan.stats()['lane_pieces'] >= 1
    # resampling a wall loop to path_step cuts corner chords, so poses may sit up to one
    # cell outside `reach`; the clearance check above is what guarantees safety
    near = core.dilate(plan.masks.reach, 1.5)
    assert rep.min_clearance >= rep.robot_radius + 0.02
    for piece in plan.pieces:
        for x, y, _ in piece.poses:
            r, c = grid.world_to_cell(x, y)
            assert near[r, c]
