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

"""Experiment matrix, the results table and the sequential runner state machine."""

import json
import os
import stat
import time

from vacuum_gui import experiments
from vacuum_gui.commands import StackConfig
from vacuum_gui.launch_manager import ProcessManager


def test_parse_seeds_accepts_lists_and_ranges():
    assert experiments.parse_seeds('1,2,3') == [1, 2, 3]
    assert experiments.parse_seeds(' 5-7, 2 ,5') == [5, 6, 7, 2]
    assert experiments.parse_seeds('') == []
    assert experiments.parse_seeds('a,4') == [4]


def test_expand_is_seed_major_and_drops_unknown_modes():
    spec = experiments.ExperimentSpec(seeds=[1, 2], modes=['none', 'vision', 'bogus'], dirt=9)
    plans = experiments.expand(spec)
    assert [(p.seed, p.mode) for p in plans] == [(1, 'none'), (1, 'vision'),
                                                  (2, 'none'), (2, 'vision')]
    assert all(p.dirt == 9 for p in plans)
    vision = plans[1]
    assert vision.reclean is True and vision.reclean_source == 'vision'
    none = plans[0]
    assert none.reclean is False and none.reclean_source == 'map'


def test_plan_config_sets_the_mode_without_touching_the_user_options():
    base = StackConfig(world='living_room.world', dirt_seed=99, cameras=True, map='/m.yaml')
    plan = experiments.RunPlan(seed=4, mode='vision', dirt=6, dirt_scale=2.0,
                               dirt_stubborn=0.3, world='living_room.world')
    cfg = experiments.plan_config(plan, base)
    assert cfg.dirt_seed == 4 and cfg.dirt == 6 and cfg.dirt_scale == 2.0
    assert cfg.dirt_clean is True and cfg.dirt_randomize is False
    assert cfg.headless is True and cfg.rviz is False and cfg.teleop is False
    assert cfg.autostart is True and cfg.coverage is True
    assert cfg.reclean_source == 'vision' and cfg.detector is True
    assert cfg.map == '/m.yaml'                       # the map is not part of the matrix
    assert base.dirt_seed == 99                       # the base config is untouched


def test_false_recleans_counts_spots_that_removed_nothing():
    summary = {'cleanup': [{'pass': 1, 'spots': 3}, {'pass': 2, 'spots': 1}],
               'per_pass': [{'pass': 0, 'end_s': 100.0}, {'pass': 1, 'end_s': 200.0},
                            {'pass': 2, 'end_s': 260.0}]}
    dirt = {'timeline': [{'t_s': 150.0}, {'t_s': 160.0}]}
    # pass 1: 3 spots, 2 patches -> 1 false; pass 2: 1 spot, 0 patches -> 1 false
    assert experiments.false_recleans(summary, dirt) == 2
    assert experiments.false_recleans(summary, None) is None
    assert experiments.false_recleans({'cleanup': []}, dirt) == 0


def test_results_row_reads_the_summary_and_the_dirt_truth():
    summary = {'status': 'finished', 'duration_min': 12.5, 'distance_m': 88.0,
               'cleaned_reachable_pct': 96.7, 'cleaned_free_pct': 75.7,
               'cleanup': [{'source': 'vision', 'spots': 2, 'unseen_m2': 0.4}]}
    dirt = {'total': 12, 'remaining': 1, 'removed_pct': 91.7,
            'timeline': [{'t_s': 10.0}]}
    plan = experiments.RunPlan(seed=3, mode='vision', dirt=12, dirt_scale=1.0,
                               dirt_stubborn=0.15, world='w.world')
    row = experiments.results_row(plan, summary, dirt, '/runs/run_1')
    assert row['seed'] == 3 and row['mode'] == 'vision'
    assert row['dirt_left'] == 1 and row['reclean_spots'] == 2
    assert row['cleaned_reachable_pct'] == 96.7 and row['unseen_m2'] == 0.4
    assert row['run_dir'] == '/runs/run_1'
    missing = experiments.results_row(plan, {}, None, '', 'timed out')
    assert missing['status'] == 'failed' and missing['error'] == 'timed out'


def test_csv_and_text_table_have_stable_columns(tmp_path):
    rows = [{'seed': 1, 'mode': 'map', 'dirt_left': 2, 'duration_min': 8.0,
             'distance_m': 40.0, 'cleaned_reachable_pct': 90.0, 'reclean_spots': 3,
             'false_recleans': 1, 'unseen_m2': 0.1, 'status': 'finished'}]
    path = experiments.write_results(rows, tmp_path / 'exp')
    header = path.read_text().splitlines()[0].split(',')
    assert header[:5] == ['seed', 'mode', 'dirt_total', 'dirt_left', 'removed_pct']
    text = experiments.table_text(rows)
    assert 'dirt_left' in text and 'map' in text
    assert '2' in text.splitlines()[2]


def test_find_dirt_summary_ignores_old_runs(tmp_path):
    runs = tmp_path / 'runs'
    run = runs / 'run_20260101_120000'
    run.mkdir(parents=True)
    dirt = tmp_path / 'dirt'
    dirt.mkdir()
    old = dirt / 'dirt_summary_20250101_000000.json'
    old.write_text(json.dumps({'remaining': 99}))
    os.utime(old, (time.time() - 20 * 3600,) * 2)
    new = dirt / 'dirt_summary_20260101_120100.json'
    new.write_text(json.dumps({'remaining': 2, 'total': 12}))
    found = experiments.find_dirt_summary(run, dirt)
    assert found and found['remaining'] == 2
    assert experiments.find_dirt_summary(run, tmp_path / 'nope') is None


def test_runner_detects_the_report_folder_it_just_produced(tmp_path):
    runs = tmp_path / 'runs'
    runs.mkdir()
    done = runs / 'run_20200101_000000'
    done.mkdir()
    (done / 'summary.json').write_text('{}')
    runner = experiments.ExperimentRunner([], tmp_path / 'out', runs_root=runs)
    since = time.time() - 1
    assert runner._new_run({'run_20200101_000000'}, since) is None
    fresh = runs / 'run_20260101_000000'
    fresh.mkdir()
    (fresh / 'summary.json').write_text('{}')
    assert runner._new_run({'run_20200101_000000'}, since) == fresh


def test_runner_walks_a_fake_stack_to_a_results_table(tmp_path):
    """End to end with a fake ``ros2`` that writes a run folder, then sleeps."""
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    runs = tmp_path / 'runs'
    runs.mkdir()
    script = fake_bin / 'ros2'
    script.write_text(
        '#!/bin/sh\n'
        'if [ "$1" = "launch" ]; then\n'
        '  d="%s/run_$(date +%%Y%%m%%d_%%H%%M%%S)_$$"\n'
        '  mkdir -p "$d"\n'
        '  printf \'{"status": "finished", "duration_min": 1.5, "distance_m": 9.0, '
        '"cleaned_reachable_pct": 88.0, "cleaned_free_pct": 70.0, "passes": 2, '
        '"per_pass": [], "cleanup": [{"source": "map", "spots": 2, "unseen_m2": 0.0}]}\' > '
        '"$d/summary.json"\n'
        '  sleep 60\n'
        'fi\n' % runs)
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    env = dict(os.environ)
    env['PATH'] = str(fake_bin) + os.pathsep + env.get('PATH', '')
    manager = ProcessManager(base_env=env)

    events = []
    plan = experiments.RunPlan(seed=1, mode='map', dirt=4, dirt_scale=1.0,
                               dirt_stubborn=0.1, world='living_room.world')
    runner = experiments.ExperimentRunner(
        [plan], tmp_path / 'out', on_event=lambda k, p: events.append((k, p)),
        timeout_s=20.0, settle_s=0.2, runs_root=runs, manager=manager)
    rows = runner.run()

    assert len(rows) == 1
    assert rows[0]['status'] == 'finished'
    assert rows[0]['duration_min'] == 1.5
    assert rows[0]['reclean_spots'] == 2
    assert (tmp_path / 'out' / 'results.csv').is_file()
    kinds = [k for k, _ in events]
    assert kinds[0] == 'log' and 'start' in kinds
    assert kinds.index('start') < kinds.index('row')
    assert kinds[-1] == 'done'
    assert runner.manager.running_names() == []
