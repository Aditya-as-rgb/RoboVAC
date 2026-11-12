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

"""Blind vs vision experiment runner.

For every (seed, mode) pair it starts a headless stack with that dirt layout, lets the
mission run with ``autostart:=true``, waits for the mission's ``run_*`` folder to appear,
stops the stack and turns the run into one row of a results table (CSV + on screen).

Modes
  none    no clean-up pass at all (``reclean:=false`` in the coverage config)
  map     clean-up spots from the coverage gaps (``reclean_source:=map``)
  vision  clean-up spots from the camera's confirmed dirt (``reclean_source:=vision``)

The last one is the feature being compared. Whatever launch argument that feature grows
later goes through :attr:`ExperimentSpec.extra_launch_args` - see ``VISION_HOOK``.

Everything except the actual process start is plain Python and unit-tested; the runner
takes a :class:`~vacuum_gui.launch_manager.ProcessManager`-shaped object so a test can
drive it against a fake stack script.
"""

import csv
import datetime
import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import paths
from .commands import (StackConfig, coverage_params_with_overrides, shell_join)
from .launch_manager import ProcessManager

MODES = {
    'none': {'label': 'no re-clean', 'reclean': False, 'reclean_source': 'map'},
    'map': {'label': 'map re-clean', 'reclean': True, 'reclean_source': 'map'},
    'vision': {'label': 'vision re-clean', 'reclean': True, 'reclean_source': 'vision'},
}

# MARK: vision hook. The vision-guided re-clean currently rides on
# ``reclean_source:=vision``. When a dedicated launch argument is added (e.g.
# ``vision_reclean:=true`` with its own thresholds), put it here and it is appended to
# every vision run without touching the rest of the runner.
VISION_HOOK = []

COLUMNS = [
    'seed', 'mode', 'dirt_total', 'dirt_left', 'removed_pct', 'status',
    'duration_min', 'distance_m', 'cleaned_reachable_pct', 'cleaned_free_pct',
    'reclean_spots', 'false_recleans', 'unseen_m2', 'run_dir', 'error',
]


@dataclass
class ExperimentSpec:
    seeds: list = field(default_factory=lambda: [1, 2, 3])
    modes: list = field(default_factory=lambda: ['none', 'map', 'vision'])
    dirt: int = 12
    dirt_scale: float = 1.0
    dirt_stubborn: float = 0.15
    world: str = 'living_room.world'
    timeout_s: float = 1800.0
    settle_s: float = 8.0
    extra_launch_args: list = field(default_factory=list)


@dataclass
class RunPlan:
    seed: int
    mode: str
    dirt: int
    dirt_scale: float
    dirt_stubborn: float
    world: str
    label: str = ''
    reclean: bool = True
    reclean_source: str = 'map'
    extra_args: list = field(default_factory=list)

    def __post_init__(self):
        info = MODES.get(self.mode)
        if info:                       # the mode is the single source of truth
            self.reclean = bool(info['reclean'])
            self.reclean_source = info['reclean_source']
        if not self.label:
            self.label = 'seed%d_%s' % (self.seed, self.mode)


def parse_seeds(text):
    """'1, 2 ,5-7' -> [1, 2, 5, 6, 7] (ranges allowed, duplicates dropped, order kept)."""
    seeds = []
    for chunk in str(text).replace(';', ',').split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        if '-' in chunk:
            try:
                lo, hi = chunk.split('-', 1)
                lo, hi = int(lo), int(hi)
            except ValueError:
                continue
            for value in range(min(lo, hi), max(lo, hi) + 1):
                if value not in seeds:
                    seeds.append(value)
        else:
            try:
                value = int(chunk)
            except ValueError:
                continue
            if value not in seeds:
                seeds.append(value)
    return seeds


def expand(spec):
    """(seed x mode) -> ordered list of RunPlan, modes in the order the user ticked them."""
    plans = []
    modes = [m for m in spec.modes if m in MODES] or ['map']
    for seed in spec.seeds:
        for mode in modes:
            info = MODES[mode]
            extra = []
            if mode == 'vision':
                extra = list(VISION_HOOK) + list(spec.extra_launch_args)
            plans.append(RunPlan(seed=int(seed), mode=mode, dirt=int(spec.dirt),
                                 dirt_scale=float(spec.dirt_scale),
                                 dirt_stubborn=float(spec.dirt_stubborn),
                                 world=spec.world, reclean=bool(info['reclean']),
                                 reclean_source=info['reclean_source'], extra_args=extra))
    return plans


def plan_config(plan, base=None):
    """Stack config for one plan: headless, autostart, that seed, that clean-up source."""
    cfg = (base or StackConfig()).copy()
    cfg.slam = False
    cfg.headless = True
    cfg.rviz = False
    cfg.teleop = False
    cfg.coverage = True
    cfg.meter = True
    cfg.autostart = True
    cfg.collect_dataset = False
    cfg.dirt = int(plan.dirt)
    cfg.dirt_seed = int(plan.seed)
    cfg.dirt_scale = float(plan.dirt_scale)
    cfg.dirt_stubborn = float(plan.dirt_stubborn)
    cfg.dirt_clean = True
    cfg.dirt_randomize = False
    cfg.world = plan.world
    cfg.reclean = bool(plan.reclean)
    cfg.reclean_source = plan.reclean_source
    cfg.detector = plan.mode == 'vision'
    return cfg


# --------------------------------------------------------------- dirt truth --

def find_dirt_summary(run_dir, dirt_dir=None):
    """The dirt summary that belongs to ``run_dir`` (newest one written around it)."""
    dirt_dir = Path(os.path.expanduser(str(dirt_dir or paths.dirt_dir())))
    if not dirt_dir.is_dir():
        return None
    try:
        run_time = Path(run_dir).stat().st_mtime
    except OSError:
        run_time = time.time()
    candidates = []
    for path in dirt_dir.glob('dirt_summary_*.json'):
        try:
            stamp = path.stat().st_mtime
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        # the summary is rewritten during the run and once at shutdown; accept the file
        # written around the run folder, and ignore leftovers from earlier experiments
        if stamp < run_time - 6 * 3600:
            continue
        candidates.append((abs(stamp - run_time), stamp, data, path))
    if not candidates:
        return None
    candidates.sort(key=lambda c: (c[0], c[1]))
    data = candidates[0][2]
    data['_path'] = str(candidates[0][3])
    return data


def false_recleans(summary, dirt_summary):
    """Clean-up spots that did not take any dirt off the floor.

    A patch is stamped when the brush removes it, and each clean-up pass has an end time
    in ``summary['per_pass']``. For every clean-up pass we count the patches removed
    inside its time window and compare that with the number of spots the robot visited:
    ``max(0, spots - removed)``. It is a proxy (two spots can remove one big patch), so
    the table calls it ``false_recleans*`` and the README says how it is computed.
    """
    if not dirt_summary:
        return None
    cleanup = summary.get('cleanup') or []
    if not cleanup:
        return 0
    marks = sorted([(int(p.get('pass', 0)), float(p.get('end_s', 0.0)))
                    for p in summary.get('per_pass') or []], key=lambda m: m[1])
    removed_times = sorted(float(t.get('t_s', 0.0))
                           for t in (dirt_summary.get('timeline') or []))
    false = 0
    for entry in cleanup:
        pass_no = int(entry.get('pass', 0))
        spots = int(entry.get('spots', 0))
        if spots <= 0:
            continue
        start = 0.0
        end = None
        for idx, (mark_pass, mark_t) in enumerate(marks):
            if mark_pass == pass_no:
                end = mark_t
                start = marks[idx - 1][1] if idx > 0 else 0.0
                break
        if end is None:
            continue
        removed = sum(1 for t in removed_times if start <= t <= end + 1.0)
        false += max(0, spots - removed)
    return false


def results_row(plan, summary, dirt_summary=None, run_dir='', error=''):
    summary = summary or {}
    dirt = dirt_summary or {}
    row = {
        'seed': plan.seed,
        'mode': plan.mode,
        'dirt_total': dirt.get('total', plan.dirt),
        'dirt_left': dirt.get('remaining', ''),
        'removed_pct': dirt.get('removed_pct', ''),
        'status': summary.get('status', error and 'failed' or 'missing'),
        'duration_min': summary.get('duration_min', ''),
        'distance_m': summary.get('distance_m', ''),
        'cleaned_reachable_pct': summary.get('cleaned_reachable_pct', ''),
        'cleaned_free_pct': summary.get('cleaned_free_pct', ''),
        'reclean_spots': sum(int(c.get('spots', 0)) for c in summary.get('cleanup') or []),
        'false_recleans': false_recleans(summary, dirt_summary),
        'unseen_m2': round(sum(c.get('unseen_m2', 0.0) or 0.0
                               for c in summary.get('cleanup') or []), 2)
        if summary.get('cleanup') else '',
        'run_dir': str(run_dir),
        'error': error,
    }
    return row


def to_csv(rows, columns=None):
    columns = columns or COLUMNS
    import io
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=columns, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)
    return buf.getvalue()


def _fmt(value, digits=2):
    if value is None or value == '':
        return '--'
    if isinstance(value, float):
        return ('%.' + str(digits) + 'f') % value
    return str(value)


def table_text(rows, columns=None):
    """Plain-text table for the panel (no Qt)."""
    columns = columns or ['seed', 'mode', 'dirt_left', 'duration_min', 'distance_m',
                          'cleaned_reachable_pct', 'reclean_spots', 'false_recleans', 'status']
    widths = []
    for col in columns:
        widths.append(max([len(col)] + [len(_fmt(r.get(col))) for r in rows]))
    lines = ['  '.join(col.ljust(widths[i]) for i, col in enumerate(columns)),
             '  '.join('-' * w for w in widths)]
    for row in rows:
        lines.append('  '.join(_fmt(row.get(col)).ljust(widths[i])
                               for i, col in enumerate(columns)))
    return '\n'.join(lines)


def write_results(rows, out_dir, name='results.csv'):
    out_dir = Path(os.path.expanduser(str(out_dir)))
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / name
    path.write_text(to_csv(rows))
    return path


# ------------------------------------------------------------------- runner --

class ExperimentRunner:
    """Runs plans one after another. Call :meth:`run` from a worker thread.

    ``on_event(kind, payload)`` gets ``start`` (plan), ``log`` (line text),
    ``row`` (result dict), ``done`` (list of rows) and ``error`` (text).
    """

    def __init__(self, plans, out_dir, on_event=None, timeout_s=1800.0, settle_s=8.0,
                 base_config=None, runs_root=None, ws_map=None, stop_event=None,
                 manager=None):
        self.plans = list(plans)
        self.out_dir = Path(os.path.expanduser(str(out_dir)))
        self.on_event = on_event or (lambda kind, payload: None)
        self.timeout_s = float(timeout_s)
        self.settle_s = float(settle_s)
        self.base_config = base_config or StackConfig()
        self.runs_root = Path(os.path.expanduser(str(runs_root or paths.runs_dir())))
        self.ws_map = ws_map
        self.stop_event = stop_event
        self.rows = []
        self.manager = manager or ProcessManager()

    # ------------------------------------------------------------------ utils --
    def _emit(self, kind, payload):
        try:
            self.on_event(kind, payload)
        except Exception:      # noqa: BLE001 - a bad callback must not abort an experiment
            pass

    def _log(self, text):
        self._emit('log', text)

    def _stopped(self):
        return bool(self.stop_event and self.stop_event.is_set())

    def _run_dirs(self):
        if not self.runs_root.is_dir():
            return set()
        return {p.name for p in self.runs_root.iterdir()
                if p.is_dir() and p.name.startswith('run_')}

    def _new_run(self, before, since):
        if not self.runs_root.is_dir():
            return None
        for path in sorted(self.runs_root.iterdir(), key=lambda p: p.stat().st_mtime
                           if p.exists() else 0):
            if not path.is_dir() or not path.name.startswith('run_'):
                continue
            if path.name in before:
                continue
            summary = path / 'summary.json'
            if summary.is_file() and summary.stat().st_mtime >= since - 5:
                return path
        return None

    # ------------------------------------------------------------------- run --
    def run(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        params_dir = self.out_dir / 'params'
        results_path = self.out_dir / 'results.csv'
        self._log('experiment folder: %s' % self.out_dir)
        self._log('%d runs: %s' % (len(self.plans), ', '.join(p.label for p in self.plans)))
        for index, plan in enumerate(self.plans, 1):
            if self._stopped():
                self._log('stopped before run %d' % index)
                break
            self._emit('start', plan)
            self._log('--- run %d/%d: %s' % (index, len(self.plans), plan.label))
            row = self._run_one(plan, params_dir)
            self.rows.append(row)
            write_results(self.rows, self.out_dir)
            self._emit('row', row)
            self._log('    ' + ', '.join('%s=%s' % (k, row.get(k))
                                         for k in ('dirt_left', 'duration_min', 'distance_m',
                                                   'cleaned_reachable_pct', 'false_recleans')))
        write_results(self.rows, self.out_dir)
        self._log('results: %s' % results_path)
        self._emit('done', self.rows)
        return self.rows

    def _run_one(self, plan, params_dir):
        cfg = plan_config(plan, self.base_config)
        params_file = None
        try:
            base_cfg = paths.coverage_config()
            params_file = params_dir / ('%s.yaml' % plan.label)
            coverage_params_with_overrides(base_cfg, {'reclean': plan.reclean,
                                                      'reclean_source': plan.reclean_source},
                                           params_file)
        except (OSError, ValueError) as exc:
            self._log('could not write coverage params (%s); using the package defaults' % exc)
            params_file = None

        argv = cfg.stack_command(self.ws_map) + list(plan.extra_args)
        before = self._run_dirs()
        since = time.time()
        self._log('launch: ' + shell_join(argv))
        self.manager.log.listen(lambda entry: self._log('%s: %s' % (entry[1], entry[3])))
        if not self.manager.start('sim', argv):
            return results_row(plan, {}, None, '', 'could not start the stack')

        self._sleep(min(self.settle_s, 5.0))
        if cfg.coverage:
            self.manager.start('coverage', cfg.coverage_command(
                self.ws_map, params_file=str(params_file) if params_file else None,
                autostart=True))
        if plan.mode == 'vision':
            self.manager.start('detector', cfg.detector_command())

        run_dir, error = None, ''
        deadline = time.time() + self.timeout_s
        while time.time() < deadline:
            if self._stopped():
                error = 'stopped by the user'
                break
            run_dir = self._new_run(before, since)
            if run_dir is not None:
                break
            if not self.manager.running('sim') and not self.manager.running('coverage'):
                error = 'the stack exited before the mission wrote a report'
                break
            time.sleep(2.0)

        if run_dir is None and not error:
            error = 'timed out after %.0f min' % (self.timeout_s / 60.0)
        # let the report finish writing, then take everything down
        self._sleep(3.0)
        self.manager.stop_all(int_wait=10.0)
        self.manager.wait_dead(timeout=10.0)
        self._sleep(2.0)

        summary = {}
        dirt = None
        if run_dir is not None:
            try:
                summary = json.loads((run_dir / 'summary.json').read_text())
            except (OSError, ValueError) as exc:
                error = error or str(exc)
            dirt = find_dirt_summary(run_dir)
        return results_row(plan, summary, dirt, str(run_dir) if run_dir else '', error)

    def _sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if self._stopped():
                return
            time.sleep(0.2)

    def stop(self):
        self.manager.stop_all(int_wait=10.0)


def experiment_dir(prefix='experiments', root=None):
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    root = Path(os.path.expanduser(str(root or paths.runs_dir())))
    return root / ('%s_%s' % (prefix, stamp))
