"""ROS-free planner for the clean-up pass: visit the spots the main run left dirty.

After the wall pass and lane sweep, some floor is usually still uncleaned (the robot stayed
a little too far from a wall, a piece was skipped, a lane was cut short by furniture).
``plan_reclean`` takes the cells the coverage tracker has not marked yet and picks a small
set of robot positions that together clean them (greedy set cover on the grid), then
orders them nearest-first. Each position is a one-pose ``Piece('spot', ...)``: the mission
only has to drive there, and the trip itself cleans the floor on the way.
"""

from dataclasses import dataclass, field
import json
import math

import numpy as np

from . import core


@dataclass
class RecleanPlan:
    pieces: list = field(default_factory=list)   # Piece('spot', [(x, y, yaw)])
    missed_m2: float = 0.0       # uncleaned reachable floor before this pass
    left_m2: float = 0.0         # what the spots will still not reach (specks below min_area)
    source: str = 'map'          # 'map' (every missed cell) or 'vision' (only dirt the camera saw)
    detections: int = 0          # vision: spots that went into the plan
    unseen_m2: float = 0.0       # vision: missed floor the camera did not report, left alone


@dataclass
class VisionSpot:
    """One confirmed dirt spot as published by vacuum_vision (/dirt/detections)."""
    x: float
    y: float
    kind: str = 'dirt'
    conf: float = 0.0
    hits: int = 0


def window_gain(mask, r_cells):
    """For every cell: how many True cells of ``mask`` lie within a disc of radius ``r_cells``."""
    n = int(math.floor(r_cells + 1e-9))
    hw = core.disc_row_halfwidths(r_cells)
    h, w = mask.shape
    cs = np.zeros((h, w + 1), dtype=np.int32)
    cs[:, 1:] = np.cumsum(mask, axis=1)
    cols = np.arange(w)
    gain = np.zeros((h, w), dtype=np.int32)
    for dy, half in zip(range(-n, n + 1), hw):
        lo = np.clip(cols - half, 0, w)
        hi = np.clip(cols + half + 1, 0, w)
        rowsum = cs[:, hi] - cs[:, lo]            # window sum along x, for every row
        r0, r1 = max(0, -dy), h - max(0, dy)
        if r0 < r1:
            gain[r0:r1] += rowsum[r0 + dy:r1 + dy]
    return gain


def _clear_disc(mask, row, col, n, hw):
    h, w = mask.shape
    for dy, half in zip(range(-n, n + 1), hw):
        rr = row + dy
        if 0 <= rr < h:
            c0, c1 = max(0, col - half), min(w, col + half + 1)
            if c0 < c1:
                mask[rr, c0:c1] = False


def plan_reclean(grid, masks, params, covered, start_xy, slack=0.08, min_area=0.015,
                 max_spots=40):
    """Pick robot positions that clean what ``covered`` (bool grid) is still missing.

    slack     metres: spots are chosen for a slightly smaller cleaning radius, because Nav2
              stops anywhere inside its goal tolerance, not exactly on the spot
    min_area  m^2: clusters of missed floor smaller than this are ignored (map speckle)
    """
    res = grid.resolution
    missed = masks.coverable & ~covered
    cell_m2 = res * res
    out = RecleanPlan(missed_m2=float(missed.sum()) * cell_m2)
    min_cells = max(1, int(round(min_area / cell_m2)))
    remaining = missed.copy()
    spots = []

    # first with the safe (reduced) radius, then with the full one for what is left
    for radius in (max(params.clean_radius - slack, res), params.clean_radius):
        r_cells = radius / res
        n = int(math.floor(r_cells + 1e-9))
        hw = core.disc_row_halfwidths(r_cells)
        while len(spots) < max_spots:
            gain = window_gain(remaining, r_cells)
            gain[~masks.reach] = 0
            k = int(np.argmax(gain))
            row, col = divmod(k, gain.shape[1])
            if gain[row, col] < min_cells:
                break
            spots.append((row, col))
            _clear_disc(remaining, row, col, n, hw)

    # what no spot will clean even if the robot stops exactly on it (only specks are left)
    left = missed.copy()
    n_full = int(math.floor(params.clean_radius / res + 1e-9))
    hw_full = core.disc_row_halfwidths(params.clean_radius / res)
    for row, col in spots:
        _clear_disc(left, row, col, n_full, hw_full)
    out.left_m2 = float(left.sum()) * cell_m2

    # nearest-first order, heading = direction of travel
    cur = (float(start_xy[0]), float(start_xy[1]))
    todo = [grid.cell_to_world(r, c) for r, c in spots]
    while todo:
        i = min(range(len(todo)), key=lambda k: math.hypot(todo[k][0] - cur[0], todo[k][1] - cur[1]))
        x, y = todo.pop(i)
        yaw = math.atan2(y - cur[1], x - cur[0]) if math.hypot(x - cur[0], y - cur[1]) > 1e-6 else 0.0
        out.pieces.append(core.Piece('spot', [(x, y, yaw)]))
        cur = (x, y)
    return out


# --------------------------------------------------------------- vision ---

def parse_detections(text, min_conf=0.0, min_hits=0):
    """Parse the JSON on /dirt/detections into ``(stamp, [VisionSpot, ...])``.

    Malformed JSON or a payload without a ``spots`` list raises ValueError; a single
    broken spot is skipped instead of failing the whole frame. ``min_conf`` / ``min_hits``
    filter out weak sightings the camera has not confirmed yet.
    """
    try:
        body = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError('bad detection JSON: %s' % exc) from exc
    if not isinstance(body, dict) or not isinstance(body.get('spots'), list):
        raise ValueError('detection JSON needs a "spots" list')
    try:
        stamp = float(body.get('stamp', 0.0))
    except (TypeError, ValueError):
        stamp = 0.0
    spots = []
    for s in body['spots']:
        if not isinstance(s, dict):
            continue
        try:
            x, y = float(s['x']), float(s['y'])
            conf = float(s.get('conf', 0.0))
            hits = int(s.get('hits', 0))
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y)) or conf < min_conf or hits < min_hits:
            continue
        spots.append(VisionSpot(x, y, str(s.get('kind', 'dirt')), conf, hits))
    return stamp, spots


def detection_mask(grid, masks, spots, radius):
    """Cells of ``masks.coverable`` within ``radius`` metres of a camera sighting.

    A sighting is dirt, not a place to stand: the mask says what a cleaning pass would
    have to reach. ``plan_reclean`` then puts the spot itself on a reachable cell, which
    also snaps a patch seen against a wall or under furniture to floor the robot can use.
    """
    out = np.zeros(masks.coverable.shape, dtype=bool)
    if radius <= 0 or not spots:
        return out
    res = grid.resolution
    r_cells = radius / res
    n = int(math.floor(r_cells + 1e-9))
    hw = core.disc_row_halfwidths(r_cells)
    for s in spots:
        row, col = grid.world_to_cell(s.x, s.y)
        if not grid.in_bounds(row, col):
            continue
        for dy, half in zip(range(-n, n + 1), hw):
            rr = row + dy
            if not (0 <= rr < grid.height):
                continue
            c0, c1 = max(0, col - half), min(grid.width, col + half + 1)
            if c0 < c1:
                out[rr, c0:c1] = True
    return out & masks.coverable


def plan_reclean_vision(grid, masks, params, spots, covered, start_xy, slack=0.08,
                        min_area=0.015, max_spots=40, min_conf=0.0):
    """Clean-up spots for the dirt the camera reported, or None when there is nothing to do.

    Only floor within a cleaning radius of a sighting counts as dirty, so this pass chases
    what the robot's own camera still sees. ``covered`` (the coverage tracker) is
    deliberately *not* used to filter the plan: a stubborn patch is geometrically "cleaned"
    the moment the robot passes near it, yet the camera keeps seeing it, and that is exactly
    what this pass exists for. A spot disappears from the input only when the detector looks
    at it and finds nothing (see vacuum_vision SpotTracker), so the loop terminates on its
    own; ``reclean_passes`` bounds it anyway.

    Floor the map calls missed but the camera never reported is left alone and reported as
    ``unseen_m2``, so blind and vision runs stay comparable.

    A detection is dirt, not a target pose: the greedy set cover runs on the sighting mask
    and places each spot on a reachable cell, nearest-first, exactly like the map pass.
    """
    dets = [s for s in spots if s.conf >= min_conf]
    if not dets:
        return None
    res = grid.resolution
    cell_m2 = res * res
    seen = detection_mask(grid, masks, dets, params.clean_radius)
    dirty = seen
    if int(dirty.sum()) * cell_m2 < min_area:
        return None
    missed_map = masks.coverable if covered is None else masks.coverable & ~covered
    # the map pass on "everything that is not camera-dirty" plans exactly the dirty cells
    out = plan_reclean(grid, masks, params, masks.coverable & ~dirty, start_xy,
                       slack=slack, min_area=min_area, max_spots=max_spots)
    out.source = 'vision'
    out.detections = len(dets)
    out.unseen_m2 = float((missed_map & ~seen).sum()) * cell_m2
    return out
