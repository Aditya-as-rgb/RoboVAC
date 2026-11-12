# Copyright 2026 Aditya Choure
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Reading ``~/vacuum_runs/run_*/`` back in: summary, coverage curve, comparison.

Plain Python. The report files are written by ``vacuum_coverage``:
``summary.json``, ``coverage.csv``, ``pieces.csv`` and (with matplotlib) ``report.png``.
"""

import csv
import json
import os
from dataclasses import dataclass, field
from pathlib import Path

from . import paths


@dataclass
class RunEntry:
    path: Path
    name: str
    mtime: float
    summary: dict = field(default_factory=dict)
    error: str = ''

    @property
    def label(self):
        if self.summary:
            pct = self.summary.get('cleaned_reachable_pct', 0.0)
            mins = self.summary.get('duration_min', 0.0)
            return '%s  -  %.1f %% / %.1f min' % (self.name, pct, mins)
        return self.name + ('  (unreadable)' if self.error else '')


def list_runs(root=None, limit=200):
    root = Path(os.path.expanduser(str(root or paths.runs_dir())))
    if not root.is_dir():
        return []
    entries = []
    for child in root.iterdir():
        if not child.is_dir() or not child.name.startswith('run_'):
            continue
        summary, error = {}, ''
        path = child / 'summary.json'
        if path.is_file():
            try:
                summary = json.loads(path.read_text())
            except (OSError, ValueError) as exc:
                error = str(exc)
        entries.append(RunEntry(child, child.name, child.stat().st_mtime, summary, error))
    entries.sort(key=lambda e: e.mtime, reverse=True)
    return entries[:limit]


def load_summary(path):
    path = Path(path)
    if path.is_dir():
        path = path / 'summary.json'
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def coverage_series(path):
    """coverage.csv -> {'t': [...], 'reachable': [...], 'free': [...], 'distance': [...]}."""
    path = Path(path)
    if path.is_dir():
        path = path / 'coverage.csv'
    series = {'t': [], 'reachable': [], 'free': [], 'distance': []}
    if not path.is_file():
        return series
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                series['t'].append(float(row.get('t_s', 0)))
                series['reachable'].append(float(row.get('cleaned_reachable_pct', 0)))
                series['free'].append(float(row.get('cleaned_free_pct', 0)))
                series['distance'].append(float(row.get('distance_m', 0)))
            except (TypeError, ValueError):
                continue
    return series


METRICS = (
    ('cleaned_reachable_pct', 'Covered (reachable)', '%', 'up'),
    ('cleaned_free_pct', 'Covered (all free)', '%', 'up'),
    ('duration_min', 'Mission time', 'min', 'down'),
    ('distance_m', 'Distance driven', 'm', 'down'),
    ('missed_area_m2', 'Missed area', 'm2', 'down'),
    ('passes', 'Passes', '', 'down'),
)


def _get(summary, dotted):
    value = summary
    for part in dotted.split('.'):
        if not isinstance(value, dict):
            return None
        value = value.get(part)
    return value


@dataclass
class MetricRow:
    label: str
    a: object
    b: object
    unit: str = ''
    better: str = ''          # 'a' | 'b' | ''
    delta: object = None

    def text(self, value):
        if value is None:
            return '--'
        if isinstance(value, float):
            return '%.2f' % value
        return str(value)


def compare_runs(a, b):
    """Rows for the two-run table; ``better`` compares numerically when it makes sense."""
    rows = []
    for key, label, unit, direction in METRICS:
        va, vb = _get(a, key), _get(b, key)
        better, delta = '', None
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            delta = float(vb) - float(va)
            if abs(delta) > 1e-9:
                better = 'b' if ((delta > 0) == (direction == 'up')) else 'a'
        rows.append(MetricRow(label, va, vb, unit, better, delta))
    cleanup_a = a.get('cleanup') or []
    cleanup_b = b.get('cleanup') or []
    rows.append(MetricRow('Clean-up spots', _spots(cleanup_a), _spots(cleanup_b), ''))
    rows.append(MetricRow('Unseen dirty floor', _unseen(cleanup_a), _unseen(cleanup_b), 'm2'))
    pieces_a = (a.get('pieces') or {})
    pieces_b = (b.get('pieces') or {})
    rows.append(MetricRow('Pieces done / skipped',
                          '%s / %s' % (pieces_a.get('done', '--'), pieces_a.get('skipped', '--')),
                          '%s / %s' % (pieces_b.get('done', '--'), pieces_b.get('skipped', '--'))))
    return rows


def _spots(cleanup):
    return sum(int(c.get('spots', 0)) for c in cleanup)


def _unseen(cleanup):
    values = [c.get('unseen_m2') for c in cleanup if c.get('unseen_m2') is not None]
    return round(sum(values), 2) if values else None


def dirt_left(run_dir):
    """Patches still on the floor, from the dirt summary written at shutdown (if any)."""
    from .experiments import find_dirt_summary
    return find_dirt_summary(run_dir)


def describe(entry):
    s = entry.summary
    if not s:
        return 'No summary.json - the mission did not finish (or the run was killed).'
    pieces = s.get('pieces') or {}
    return (
        'Status: %s\n'
        'Coverage: %.1f %% of reachable floor, %.1f %% of all free floor\n'
        'Time: %.1f min   Distance: %.1f m   Passes: %s\n'
        'Pieces: %s done, %s skipped of %s\n'
        'Clean-up: %s\n'
        'Started: %s'
        % (s.get('status', '?'), s.get('cleaned_reachable_pct', 0),
           s.get('cleaned_free_pct', 0), s.get('duration_min', 0), s.get('distance_m', 0),
           s.get('passes', 0), pieces.get('done', 0), pieces.get('skipped', 0),
           pieces.get('total', 0),
           ', '.join('%s=%s spots' % (c.get('source', '?'), c.get('spots', 0))
                     for c in (s.get('cleanup') or [])) or 'none',
           s.get('started_at', '')))
