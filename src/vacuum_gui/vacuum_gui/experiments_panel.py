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

"""The blind-vs-vision experiment runner: seeds x modes, headless, one table."""

import subprocess
import threading

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMessageBox,
                               QPlainTextEdit, QPushButton, QSpinBox, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from . import experiments, paths
from .commands import StackConfig
from .ui_common import hint

TABLE_COLUMNS = ['seed', 'mode', 'dirt_left', 'duration_min', 'distance_m',
                 'cleaned_reachable_pct', 'reclean_spots', 'false_recleans', 'unseen_m2',
                 'status']


class RunnerThread(QThread):
    event = Signal(str, object)

    def __init__(self, runner, parent=None):
        super().__init__(parent)
        self.runner = runner

    def run(self):
        try:
            self.runner.run()
        except Exception as exc:      # noqa: BLE001 - report instead of killing the thread
            self.event.emit('error', str(exc))

    def stop(self):
        try:
            self.runner.stop()
        except Exception:      # noqa: BLE001
            pass


class ExperimentsPanel(QWidget):
    def __init__(self, base_config=None, parent=None):
        super().__init__(parent)
        self.base_config = base_config or StackConfig()
        self.thread = None
        self.out_dir = None
        self.rows = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        form_box = QGroupBox('Experiment matrix')
        form = QFormLayout(form_box)
        self.seeds = QLineEdit('1,2,3')
        self.seeds.setToolTip('Seeds, comma separated; ranges work (1-5)')
        form.addRow('Seeds', self.seeds)
        modes_row = QWidget()
        modes_layout = QHBoxLayout(modes_row)
        modes_layout.setContentsMargins(0, 0, 0, 0)
        self.mode_checks = {}
        for key, label in (('none', 'no re-clean'), ('map', 'map re-clean'),
                           ('vision', 'vision re-clean')):
            check = QCheckBox(label)
            check.setChecked(True)
            self.mode_checks[key] = check
            modes_layout.addWidget(check)
        modes_layout.addStretch(1)
        form.addRow('Modes', modes_row)
        self.dirt = QSpinBox()
        self.dirt.setRange(1, 200)
        self.dirt.setValue(12)
        form.addRow('Dirt patches', self.dirt)
        self.dirt_scale = QDoubleSpinBox()
        self.dirt_scale.setRange(0.3, 5.0)
        self.dirt_scale.setValue(1.0)
        self.dirt_scale.setSingleStep(0.1)
        form.addRow('Dirt scale', self.dirt_scale)
        self.dirt_stubborn = QDoubleSpinBox()
        self.dirt_stubborn.setRange(0.0, 1.0)
        self.dirt_stubborn.setValue(0.15)
        self.dirt_stubborn.setSingleStep(0.05)
        form.addRow('Stubborn share', self.dirt_stubborn)
        self.timeout = QSpinBox()
        self.timeout.setRange(2, 240)
        self.timeout.setSuffix(' min')
        self.timeout.setValue(30)
        form.addRow('Timeout per run', self.timeout)
        layout.addWidget(form_box)

        note = hint('Runs one seed and mode after another, headless, with autostart:=true, '
                    'and waits for the mission report. Vision-guided clean-up goes through '
                    'reclean_source:=vision; the hook for a future launch argument is '
                    'experiments.VISION_HOOK. false_recleans* counts clean-up spots that '
                    'removed no patch in that pass (approximation - see the README).')
        layout.addWidget(note)

        buttons = QHBoxLayout()
        self.run_button = QPushButton('Run experiment')
        self.run_button.setObjectName('primary')
        self.run_button.clicked.connect(self.start)
        self.stop_button = QPushButton('Stop')
        self.stop_button.setObjectName('danger')
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop)
        self.open_button = QPushButton('Open results folder')
        self.open_button.clicked.connect(self._open_folder)
        buttons.addWidget(self.run_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.open_button)
        buttons.addStretch(1)
        self.progress = QLabel('idle')
        buttons.addWidget(self.progress)
        layout.addLayout(buttons)

        self.table = QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(TABLE_COLUMNS)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table, 2)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        layout.addWidget(self.log, 1)

    # ------------------------------------------------------------------- run --
    def start(self):
        seeds = experiments.parse_seeds(self.seeds.text())
        modes = [key for key, check in self.mode_checks.items() if check.isChecked()]
        if not seeds or not modes:
            QMessageBox.warning(self, 'Nothing to run', 'Pick at least one seed and one mode.')
            return
        plans = experiments.expand(experiments.ExperimentSpec(
            seeds=seeds, modes=modes, dirt=int(self.dirt.value()),
            dirt_scale=float(self.dirt_scale.value()),
            dirt_stubborn=float(self.dirt_stubborn.value()),
            timeout_s=float(self.timeout.value()) * 60.0))
        self.out_dir = experiments.experiment_dir()
        runner = experiments.ExperimentRunner(
            plans, self.out_dir,
            on_event=lambda kind, payload: self.thread.event.emit(kind, payload),
            timeout_s=float(self.timeout.value()) * 60.0,
            base_config=self.base_config, runs_root=paths.runs_dir(),
            ws_map=str(paths.default_map()))
        thread = RunnerThread(runner, self)
        runner.on_event = lambda kind, payload: thread.event.emit(kind, payload)
        thread.event.connect(self._on_event)
        thread.finished.connect(self._on_finished)
        self.thread = thread
        self.rows = []
        self.table.setRowCount(0)
        self.log.clear()
        self._log('running %d launches into %s' % (len(plans), self.out_dir))
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.progress.setText('starting…')
        self.thread.start()

    def stop(self):
        if self.thread is not None:
            self.progress.setText('stopping after this run…')
            threading.Thread(target=self.thread.stop, daemon=True).start()

    def _on_finished(self):
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.progress.setText('done - %d run(s)' % len(self.rows))

    def _log(self, text):
        self.log.appendPlainText(text)

    def _on_event(self, kind, payload):
        if kind == 'log':
            self._log(str(payload))
        elif kind == 'start':
            self.progress.setText('running %s' % payload.label)
            self._log('=== %s' % payload.label)
        elif kind == 'row':
            self.rows.append(payload)
            self._add_row(payload)
        elif kind == 'error':
            self._log('ERROR: %s' % payload)
            self.progress.setText('failed: %s' % payload)
            self.run_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        elif kind == 'done':
            self._log('all runs finished; CSV in %s' % self.out_dir)

    def _add_row(self, row):
        index = self.table.rowCount()
        self.table.insertRow(index)
        for col, key in enumerate(TABLE_COLUMNS):
            value = row.get(key, '')
            if isinstance(value, float):
                value = '%.2f' % value
            item = QTableWidgetItem(str(value))
            item.setToolTip(str(row.get('run_dir', '')))
            self.table.setItem(index, col, item)
        self.table.resizeColumnsToContents()

    def _open_folder(self):
        if self.out_dir is not None:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            subprocess.Popen(['xdg-open', str(self.out_dir)])

    def apply_base_config(self, cfg):
        self.base_config = cfg
