"""ROS-free run report: what a cleaning run did, as files you can put in a presentation.

A finished (or cancelled) run writes one folder, ``run_YYYYmmdd_HHMMSS``:

    coverage.csv   one row per second: time, robot x/y, % cleaned, distance driven so far
    pieces.csv     one row per wall loop / lane / clean-up spot: status, time, length
    summary.json   the headline numbers (coverage, area, time, distance, passes, skips)
    report.png     coverage-vs-time curve and the final map (cleaned / missed / robot trail)

``RunLog`` collects the samples while the mission runs; ``write_report`` writes the files.
Only the PNG needs matplotlib, everything else is plain Python.
"""

import csv
from dataclasses import dataclass
import json
import math
import os

import numpy as np


@dataclass
class Sample:
    t: float
    x: float
    y: float
    percent: float          # % of the reachable floor that is clean
    percent_free: float     # % of ALL free floor cells in the map
    distance: float         # metres driven so far
    pass_no: int


class RunLog:
    """Collects the robot trail and the coverage curve while a mission runs."""

    def __init__(self, t0):
        self.t0 = float(t0)
        self.samples = []
        self.pass_marks = []        # (pass_no, t, percent, percent_free) when a pass ended
        self.trail = []             # (x, y), every position tick
        self.distance = 0.0
        self.current_pass = 0

    def add_position(self, x, y):
        """Call at every position tick: keeps the trail and the driven distance."""
        if self.trail:
            self.distance += math.hypot(x - self.trail[-1][0], y - self.trail[-1][1])
        self.trail.append((float(x), float(y)))

    def snapshot(self, t, percent, percent_free):
        """Record one row of the coverage curve (call about once a second)."""
        x, y = self.trail[-1] if self.trail else (0.0, 0.0)
        self.samples.append(Sample(float(t) - self.t0, x, y, float(percent), float(percent_free),
                                   self.distance, self.current_pass))

    def end_pass(self, pass_no, t, percent, percent_free):
        self.pass_marks.append((int(pass_no), float(t) - self.t0, float(percent), float(percent_free)))
        self.current_pass = pass_no + 1


def percent_free(tracker, grid):
    """% of all free floor cells (not only the reachable ones) that is clean."""
    free = grid.data == 0
    total = max(1, int(free.sum()))
    return 100.0 * int((tracker.covered & free).sum()) / total


# ---------------------------------------------------------------- files ---

def write_coverage_csv(path, log):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'x_m', 'y_m', 'cleaned_reachable_pct', 'cleaned_free_pct',
                    'distance_m', 'pass'])
        for s in log.samples:
            w.writerow(['%.1f' % s.t, '%.3f' % s.x, '%.3f' % s.y, '%.2f' % s.percent,
                        '%.2f' % s.percent_free, '%.2f' % s.distance, s.pass_no])


def write_pieces_csv(path, result, pass_pieces, t0):
    """pass_pieces: {pass_no: [Piece, ...]} so each result can be matched to its piece."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pass', 'piece', 'kind', 'status', 'attempts', 'start_s', 'end_s',
                    'duration_s', 'length_m', 'start_x_m', 'start_y_m'])
        for r in result.pieces:
            piece = pass_pieces.get(r.pass_no, [None] * (r.index + 1))[r.index]
            length = piece.length if piece is not None else 0.0
            x0, y0 = (piece.poses[0][0], piece.poses[0][1]) if piece is not None else (0.0, 0.0)
            w.writerow([r.pass_no, r.index + 1, r.kind, r.status, r.attempts,
                        '%.1f' % (r.t_start - t0), '%.1f' % (r.t_end - t0),
                        '%.1f' % (r.t_end - r.t_start), '%.2f' % length, '%.2f' % x0, '%.2f' % y0])


def build_summary(log, result, tracker, grid, params, plan_stats=None, cleanup=None,
                  started_at='', t_end=None):
    """Headline numbers of the run as a plain dict (written to summary.json)."""
    res = grid.resolution
    cell = res * res
    covered = int((tracker.covered & tracker.coverable).sum())
    duration = (log.samples[-1].t if log.samples else 0.0) if t_end is None else float(t_end) - log.t0
    kinds = {}
    for r in result.pieces:
        k = kinds.setdefault(r.kind, {'done': 0, 'skipped': 0, 'cancelled': 0})
        k[r.status] = k.get(r.status, 0) + 1
    marks = []
    for pass_no, t, pct, pct_free in log.pass_marks:
        marks.append({'pass': pass_no, 'name': 'main' if pass_no == 0 else 'clean-up %d' % pass_no,
                      'end_s': round(t, 1), 'cleaned_reachable_pct': round(pct, 2),
                      'cleaned_free_pct': round(pct_free, 2)})
    return {
        'status': 'cancelled' if result.cancelled else 'finished',
        'started_at': started_at,
        'duration_s': round(duration, 1),
        'duration_min': round(duration / 60.0, 2),
        'cleaned_reachable_pct': round(float(tracker.percent()), 2),
        'cleaned_free_pct': round(percent_free(tracker, grid), 2),
        'reachable_area_m2': round(tracker.total * cell, 2),
        'cleaned_area_m2': round(covered * cell, 2),
        'missed_area_m2': round((tracker.total - covered) * cell, 2),
        'distance_m': round(log.distance, 2),
        'average_speed_mps': round(log.distance / duration, 3) if duration > 0 else 0.0,
        'passes': result.passes,
        'pieces': {'total': len(result.pieces), 'done': result.done, 'skipped': result.skipped,
                   'by_kind': kinds},
        'per_pass': marks,
        'cleanup': cleanup or [],
        'settings': {'wall_margin_m': round(params.wall_margin, 3), 'clean_radius_m': params.clean_radius,
                     'lane_spacing_m': params.lane_spacing, 'robot_radius_m': params.robot_radius},
        'plan': plan_stats or {},
        'note': ('cleaned_reachable_pct counts floor a robot of this size can reach; '
                 'cleaned_free_pct counts every free cell of the map'),
    }


def plot_report(path, log, grid, tracker, summary):
    """Coverage curve + final map. Returns False when matplotlib is not installed."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.8), dpi=110,
                                   gridspec_kw={'width_ratios': [1.0, 1.05]})
    # --- coverage over time ---
    if log.samples:
        t = np.array([s.t for s in log.samples]) / 60.0
        ax1.plot(t, [s.percent for s in log.samples], color='#1971c2', lw=2.2,
                 label='reachable floor')
        ax1.plot(t, [s.percent_free for s in log.samples], color='#868e96', lw=1.6, ls='--',
                 label='all free floor')
    for pass_no, tt, pct, _ in log.pass_marks:
        ax1.axvline(tt / 60.0, color='#d9480f', lw=1.0, ls=':')
        ax1.annotate('main pass done' if pass_no == 0 else 'clean-up %d done' % pass_no,
                     (tt / 60.0, pct), textcoords='offset points', xytext=(-4, -14),
                     ha='right', fontsize=8, color='#d9480f')
    ax1.set_xlabel('time [min]')
    ax1.set_ylabel('cleaned [%]')
    ax1.set_ylim(0, 102)
    ax1.grid(alpha=0.25)
    ax1.legend(loc='lower right', fontsize=9)
    ax1.set_title('coverage over time', fontsize=11)

    # --- final map ---
    res = grid.resolution
    h, w = grid.data.shape
    extent = (grid.origin_x, grid.origin_x + w * res, grid.origin_y, grid.origin_y + h * res)
    img = np.full((h, w, 3), 0.80)
    img[grid.data == 0] = (1.0, 1.0, 1.0)
    img[grid.data == 100] = (0.15, 0.15, 0.15)
    img[tracker.covered & (grid.data == 0)] = (0.78, 0.92, 0.80)
    img[tracker.coverable & ~tracker.covered] = (0.95, 0.45, 0.45)
    ax2.imshow(img, origin='lower', extent=extent, interpolation='nearest')
    if log.trail:
        a = np.asarray(log.trail)
        ax2.plot(a[::3, 0], a[::3, 1], '-', color='#1971c2', lw=0.6, alpha=0.7)
        ax2.plot(a[0, 0], a[0, 1], 'o', color='#2b8a3e', ms=6)
        ax2.plot(a[-1, 0], a[-1, 1], 's', color='#d9480f', ms=5)
    ax2.set_aspect('equal')
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('y [m]')
    ax2.set_title('green: cleaned   red: missed   blue: robot trail', fontsize=11)

    fig.suptitle('%s: %.1f %% of reachable floor (%.1f %% of all free floor), %.1f min, %.1f m driven, '
                 '%d pass%s' % (summary['status'], summary['cleaned_reachable_pct'],
                                summary['cleaned_free_pct'], summary['duration_min'],
                                summary['distance_m'], summary['passes'],
                                '' if summary['passes'] == 1 else 'es'), fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path)
    plt.close(fig)
    return True


def write_report(out_dir, log, result, pass_pieces, tracker, grid, params, plan_stats=None,
                 cleanup=None, started_at=''):
    """Write the four report files into ``out_dir``; returns {name: path} of what was written."""
    os.makedirs(out_dir, exist_ok=True)
    files = {}
    files['coverage.csv'] = os.path.join(out_dir, 'coverage.csv')
    write_coverage_csv(files['coverage.csv'], log)
    files['pieces.csv'] = os.path.join(out_dir, 'pieces.csv')
    write_pieces_csv(files['pieces.csv'], result, pass_pieces, log.t0)
    summary = build_summary(log, result, tracker, grid, params, plan_stats, cleanup, started_at)
    files['summary.json'] = os.path.join(out_dir, 'summary.json')
    with open(files['summary.json'], 'w') as f:
        json.dump(summary, f, indent=2)
    png = os.path.join(out_dir, 'report.png')
    if plot_report(png, log, grid, tracker, summary):
        files['report.png'] = png
    return files
