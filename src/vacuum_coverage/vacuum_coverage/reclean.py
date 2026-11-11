"""ROS-free planner for the clean-up pass: visit the spots the main run left dirty.

After the wall pass and lane sweep, some floor is usually still uncleaned (the robot stayed
a little too far from a wall, a piece was skipped, a lane was cut short by furniture).
``plan_reclean`` takes the cells the coverage tracker has not marked yet and picks a small
set of robot positions that together clean them (greedy set cover on the grid), then
orders them nearest-first. Each position is a one-pose ``Piece('spot', ...)``: the mission
only has to drive there, and the trip itself cleans the floor on the way.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from . import core


@dataclass
class RecleanPlan:
    pieces: list = field(default_factory=list)   # Piece('spot', [(x, y, yaw)])
    missed_m2: float = 0.0       # uncleaned reachable floor before this pass
    left_m2: float = 0.0         # what the spots will still not reach (specks below min_area)


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
