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

"""Reading ``~/vacuum_runs/run_*`` folders and comparing two of them."""

import csv
import json
import os
import time

from vacuum_gui import run_report


def make_run(root, name, summary=None, coverage_rows=None, mtime=None):
    path = root / name
    path.mkdir(parents=True)
    if summary is not None:
        (path / 'summary.json').write_text(json.dumps(summary))
    if coverage_rows is not None:
        with open(path / 'coverage.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_s', 'x_m', 'y_m', 'cleaned_reachable_pct',
                             'cleaned_free_pct', 'distance_m', 'pass'])
            for row in coverage_rows:
                writer.writerow(row)
    if mtime:
        os.utime(path, (mtime, mtime))
    return path


SUMMARY_A = {'status': 'finished', 'duration_min': 12.0, 'distance_m': 90.0,
             'cleaned_reachable_pct': 95.0, 'cleaned_free_pct': 74.0,
             'missed_area_m2': 1.2, 'passes': 2,
             'pieces': {'total': 10, 'done': 9, 'skipped': 1},
             'cleanup': [{'source': 'map', 'spots': 3, 'unseen_m2': 0.5}]}
SUMMARY_B = {'status': 'finished', 'duration_min': 15.0, 'distance_m': 80.0,
             'cleaned_reachable_pct': 97.0, 'cleaned_free_pct': 76.0,
             'missed_area_m2': 0.8, 'passes': 2,
             'pieces': {'total': 10, 'done': 10, 'skipped': 0},
             'cleanup': [{'source': 'vision', 'spots': 2, 'unseen_m2': 0.1}]}


def test_list_runs_sorts_newest_first_and_skips_other_folders(tmp_path):
    make_run(tmp_path, 'run_20260101_100000', SUMMARY_A, mtime=time.time() - 100)
    make_run(tmp_path, 'run_20260102_100000', SUMMARY_B, mtime=time.time())
    (tmp_path / 'not_a_run').mkdir()
    (tmp_path / 'experiments_1').mkdir()
    runs = run_report.list_runs(tmp_path)
    assert [r.name for r in runs] == ['run_20260102_100000', 'run_20260101_100000']
    assert '%' in runs[0].label


def test_missing_summary_is_reported_not_raised(tmp_path):
    make_run(tmp_path, 'run_x')
    runs = run_report.list_runs(tmp_path)
    assert runs[0].summary == {} and runs[0].error == ''
    assert 'unreadable' in runs[0].label or 'run_x' in runs[0].label
    assert run_report.list_runs(tmp_path / 'nope') == []


def test_corrupt_summary_is_captured(tmp_path):
    path = make_run(tmp_path, 'run_y')
    (path / 'summary.json').write_text('{oops')
    entry = run_report.list_runs(tmp_path)[0]
    assert entry.error and entry.summary == {}


def test_coverage_series_parses_csv_and_tolerates_bad_rows(tmp_path):
    path = make_run(tmp_path, 'run_z', SUMMARY_A,
                    [(0.0, 0, 0, 0.0, 0.0, 0.0, 0),
                     (60.0, 1, 0, 50.5, 40.0, 22.5, 0),
                     ('x', 0, 0, 'NaN', 0, 0, 0),
                     (120.0, 2, 0, 95.0, 74.0, 90.0, 1)])
    series = run_report.coverage_series(path)
    assert series['t'] == [0.0, 60.0, 120.0]
    assert series['reachable'][-1] == 95.0
    assert series['distance'][-1] == 90.0
    assert run_report.coverage_series(tmp_path / 'nope')['t'] == []


def test_compare_runs_marks_which_run_is_better():
    rows = {r.label: r for r in run_report.compare_runs(SUMMARY_A, SUMMARY_B)}
    coverage = rows['Covered (reachable)']
    assert coverage.a == 95.0 and coverage.b == 97.0 and coverage.better == 'b'
    duration = rows['Mission time']
    assert duration.better == 'a'                       # 12 min beats 15 min
    assert rows['Clean-up spots'].a == 3 and rows['Clean-up spots'].b == 2
    assert rows['Unseen dirty floor'].b == 0.1
    assert rows['Pieces done / skipped'].a == '9 / 1'


def test_compare_runs_handles_missing_values():
    rows = run_report.compare_runs({}, SUMMARY_A)
    first = rows[0]
    assert first.a is None and first.text(first.a) == '--'
    assert first.better == ''


def test_describe_summarises_a_run():
    text = run_report.describe(run_report.RunEntry(None, 'run_a', 0.0, SUMMARY_A))
    assert 'Coverage: 95.0 %' in text and '12.0 min' in text
    empty = run_report.describe(run_report.RunEntry(None, 'run_b', 0.0, {}))
    assert 'No summary.json' in empty
