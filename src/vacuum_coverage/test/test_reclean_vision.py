"""Vision-guided clean-up pass (no ROS): pytest src/vacuum_coverage/test

The camera publishes confirmed spots on /dirt/detections; the clean-up pass has to turn
those into the same kind of spot pieces as the map pass, but only for the dirt it saw.
"""

import json
import math

import numpy as np
import pytest

from vacuum_coverage import core, reclean

from test_reclean import RES, patch, setup_room


def det(x, y, conf=0.8, kind='crumbs', hits=5):
    return reclean.VisionSpot(x, y, kind, conf, hits)


def payload(spots, stamp=12.5):
    return json.dumps({'stamp': stamp, 'spots': spots})


# ---------------------------------------------------------------- parsing --

def test_parse_detections_reads_the_detector_payload():
    text = payload([{'id': 1, 'x': 1.25, 'y': -0.5, 'kind': 'spill', 'conf': 0.91, 'hits': 7}])
    stamp, spots = reclean.parse_detections(text)
    assert stamp == pytest.approx(12.5)
    assert len(spots) == 1
    assert (spots[0].x, spots[0].y) == pytest.approx((1.25, -0.5))
    assert spots[0].kind == 'spill' and spots[0].conf == pytest.approx(0.91)
    assert spots[0].hits == 7


def test_parse_detections_filters_weak_and_broken_spots():
    text = payload([
        {'x': 1.0, 'y': 1.0, 'conf': 0.9, 'hits': 5},
        {'x': 2.0, 'y': 1.0, 'conf': 0.1, 'hits': 5},     # below min_conf
        {'x': 3.0, 'y': 1.0, 'conf': 0.9, 'hits': 1},     # not confirmed by the tracker
        {'x': 'not a number', 'y': 1.0},                 # broken
        {'x': 4.0},                                      # no y
        'nonsense',                                      # not a dict
        {'x': float('nan'), 'y': 1.0},                   # not finite
    ])
    _, spots = reclean.parse_detections(text, min_conf=0.3, min_hits=3)
    assert [(s.x, s.y) for s in spots] == [(1.0, 1.0)]


def test_parse_detections_rejects_a_payload_that_is_not_spots():
    with pytest.raises(ValueError):
        reclean.parse_detections('{not json')
    with pytest.raises(ValueError):
        reclean.parse_detections('{"spots": 3}')


# ------------------------------------------------------------------ masks --

def test_detection_mask_covers_a_disc_on_the_floor():
    grid, params, masks = setup_room()
    m = reclean.detection_mask(grid, masks, [det(1.5, 1.2)], params.clean_radius)
    assert m.sum() > 0
    rows, cols = np.nonzero(m)
    for r, c in zip(rows, cols):
        x, y = grid.cell_to_world(r, c)
        assert math.hypot(x - 1.5, y - 1.2) <= params.clean_radius + RES
        assert masks.coverable[r, c]
    # a wall cell is never part of the mask
    assert not m[0, :].any()


def test_detection_mask_is_empty_for_far_away_or_missing_spots():
    grid, params, masks = setup_room()
    assert not reclean.detection_mask(grid, masks, [], params.clean_radius).any()
    assert not reclean.detection_mask(grid, masks, [det(99.0, 99.0)], params.clean_radius).any()


# ------------------------------------------------------------- the planner --

def test_vision_spots_are_planned_only_where_the_camera_saw_dirt():
    grid, params, masks = setup_room(w_m=6.0)
    left = patch(grid, masks, 0.5, 1.0, 1.1, 1.6)       # missed by the map pass...
    right = patch(grid, masks, 4.6, 1.0, 5.2, 1.6)      # ...and so is this one
    covered = masks.coverable & ~left & ~right
    plan = reclean.plan_reclean_vision(grid, masks, params, [det(0.8, 1.3)], covered, (4.0, 1.3))
    assert plan is not None and plan.source == 'vision' and plan.detections == 1
    assert plan.pieces
    # every spot goes to the camera-detected patch, none to the other missed patch
    assert all(p.poses[0][0] < 2.0 for p in plan.pieces)
    # the map pass would have chased both patches: that is the difference being measured
    blind = reclean.plan_reclean(grid, masks, params, covered, (4.0, 1.3))
    assert any(p.poses[0][0] > 3.5 for p in blind.pieces)
    # dirt the map knows about but the camera never reported is counted, not chased:
    # unseen = missed cells outside every sighting disc (including the corners of the
    # detected patch the model did not see)
    seen = reclean.detection_mask(grid, masks, [det(0.8, 1.3)], params.clean_radius)
    assert plan.unseen_m2 == pytest.approx(((left | right) & ~seen).sum() * RES * RES)
    assert plan.unseen_m2 > right.sum() * RES * RES


def test_vision_plan_stamps_the_sightings_so_the_tracker_agrees():
    grid, params, masks = setup_room()
    missed = patch(grid, masks, 1.5, 1.0, 2.2, 1.7)
    covered = masks.coverable & ~missed
    plan = reclean.plan_reclean_vision(grid, masks, params, [det(1.85, 1.35)], covered, (0.5, 0.5))
    assert plan is not None
    # the pass plans for what was seen: one cleaning radius around the sighting, not the
    # whole (larger) patch the map calls dirty
    assert plan.missed_m2 == pytest.approx(math.pi * params.clean_radius ** 2, rel=0.1)
    seen = reclean.detection_mask(grid, masks, [det(1.85, 1.35)], params.clean_radius)
    tr = core.CoverageTracker(grid, masks.coverable, params.clean_radius)
    tr.covered = covered.copy()
    for p in plan.pieces:
        tr.stamp_xy(p.poses[0][0], p.poses[0][1])
    assert (tr.covered & seen).sum() == seen.sum()      # everything seen gets cleaned
    assert plan.left_m2 == 0.0


def test_vision_plan_ignores_what_the_coverage_tracker_thinks_is_clean():
    """A stubborn patch the brush left behind is 'covered' on the map but still dirty."""
    grid, params, masks = setup_room()
    plan = reclean.plan_reclean_vision(grid, masks, params, [det(1.5, 1.2)], masks.coverable.copy(),
                                       (0.5, 0.5))
    assert plan is not None and plan.pieces
    assert plan.unseen_m2 == 0.0


def test_vision_spot_under_furniture_is_driven_to_from_reachable_floor():
    grid, params, masks = setup_room(box=(1.5, 1.0, 2.5, 2.0))
    plan = reclean.plan_reclean_vision(grid, masks, params, [det(1.4, 1.0)], None, (0.5, 0.5))
    assert plan is not None and plan.pieces
    for p in plan.pieces:
        row, col = grid.world_to_cell(p.poses[0][0], p.poses[0][1])
        assert masks.reach[row, col]
        assert not grid.data[row, col] == 100


def test_no_vision_spots_means_the_caller_falls_back_to_the_map():
    grid, params, masks = setup_room()
    covered = np.zeros_like(masks.coverable)          # the map pass sees plenty to do
    assert reclean.plan_reclean_vision(grid, masks, params, [], covered, (1.0, 1.0)) is None
    assert reclean.plan_reclean_vision(grid, masks, params, [det(1.5, 1.2, conf=0.05)],
                                       covered, (1.0, 1.0), min_conf=0.3) is None
    # a lone speck below min_area is not worth a trip either
    assert reclean.plan_reclean_vision(grid, masks, params, [det(-5.0, -5.0)], covered,
                                       (1.0, 1.0)) is None


def test_vision_spots_are_visited_nearest_first():
    grid, params, masks = setup_room(w_m=6.0)
    near, far = det(1.0, 1.3), det(5.0, 1.3)
    plan = reclean.plan_reclean_vision(grid, masks, params, [far, near], None, (0.6, 1.3))
    xs = [p.poses[0][0] for p in plan.pieces]
    assert xs[0] < 1.5 and xs[-1] > 4.5
