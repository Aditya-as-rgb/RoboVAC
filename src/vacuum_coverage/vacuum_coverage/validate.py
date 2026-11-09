"""Self-check for coverage plans: simulate the controller, widen the margin if needed.

``plan_validated`` plans, drives the plan with the pure-pursuit approximation in
sim.py, and raises ``wall_margin`` step by step until the simulated robot stays
at least ``robot_radius + safety`` away from every occupied/unknown cell. The
result carries the numbers (coverage, clearance, time) so they can be logged and
shown on a slide.

The simulation is an approximation of Nav2's controller (see sim.py), so keep a
sensible ``safety`` (default 2 cm) and check the first real run.
"""

from dataclasses import dataclass, replace

from . import core, sim


@dataclass
class Report:
    wall_margin: float
    min_clearance: float
    robot_radius: float
    coverage_reachable: float     # % of the floor a robot centre can reach and clean
    coverage_free: float          # % of ALL free floor cells in the map
    est_time_s: float
    failed_pieces: int
    ok: bool                      # clearance >= robot_radius + safety
    iterations: int


def evaluate(grid, plan, start_pose, pursuit):
    trails, total, failures = sim.run_plan(plan, start_pose, pursuit)
    tracker = core.CoverageTracker(grid, plan.masks.coverable, plan.params.clean_radius)
    pts = []
    for tr in trails:
        for x, y in tr:
            tracker.stamp_xy(x, y)
        pts.extend(tr[::4])
    clearance = sim.min_clearance(pts, grid, plan.masks.blocked) if pts else float('inf')
    free = int((grid.data == 0).sum())
    cov_free = 100.0 * int((tracker.covered & (grid.data == 0)).sum()) / max(1, free)
    return trails, tracker, clearance, cov_free, total, failures


def plan_validated(grid, params, start_xy, pursuit=None, safety=0.02,
                   step=0.01, max_extra=0.20):
    """Return (plan, Report). ``params.wall_margin`` is the starting margin."""
    pursuit = pursuit or sim.PursuitParams()
    start_pose = (start_xy[0], start_xy[1], 0.0)
    margin = params.wall_margin
    limit = params.wall_margin + max_extra
    iterations = 0
    best = None
    while True:
        iterations += 1
        p = replace(params, wall_margin=round(margin, 4))
        try:
            plan = core.plan_coverage(grid, p, start_xy)
        except ValueError:
            break                                   # margin too large, nothing fits
        if not plan.pieces:
            break
        trails, tracker, clearance, cov_free, total, failures = evaluate(
            grid, plan, start_pose, pursuit)
        ok = clearance >= p.robot_radius + safety
        report = Report(p.wall_margin, clearance, p.robot_radius, tracker.percent(),
                        cov_free, total, failures, ok, iterations)
        best = (plan, report)
        if ok or margin >= limit:
            break
        margin += step
    if best is None:
        raise ValueError('could not build any coverage plan for this map')
    return best
