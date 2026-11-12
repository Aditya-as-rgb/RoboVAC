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

"""Dataset recording: start/stop the recorder, count what it produced, build the zip."""

import json
import os
import subprocess
from pathlib import Path

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                               QMessageBox, QPlainTextEdit, QPushButton, QVBoxLayout,
                               QWidget)

from . import paths
from .commands import package_dataset_command
from .ui_common import hint


class DatasetPanel(QWidget):
    start_recording_requested = Signal()
    stop_recording_requested = Signal()
    package_requested = Signal(str, str, str)     # source, output, classes

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recording = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.status = QLabel('recorder: stopped')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        buttons = QHBoxLayout()
        self.start_button = QPushButton('Start recording')
        self.start_button.setObjectName('primary')
        self.start_button.clicked.connect(self.start_recording_requested.emit)
        self.stop_button = QPushButton('Stop recording')
        self.stop_button.setObjectName('danger')
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_recording_requested.emit)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        layout.addWidget(hint('The recorder is vacuum_gazebo/dirt_dataset.py, started on its '
                              'own so it can be stopped without touching the sim. It needs '
                              'dirt>0 and the camera; keep dirt_clean off so the dirt stays '
                              'visible. Recording ends by itself after dataset_max_s sim '
                              'seconds or when the mission ends.'))

        counts_box = QGroupBox('What is on disk')
        counts = QVBoxLayout(counts_box)
        self.counts = QLabel('no episodes yet')
        self.counts.setWordWrap(True)
        self.episode_detail = QPlainTextEdit()
        self.episode_detail.setReadOnly(True)
        self.episode_detail.setMaximumHeight(110)
        counts.addWidget(self.counts)
        counts.addWidget(self.episode_detail)
        layout.addWidget(counts_box)

        package_box = QGroupBox('Build the zip for Colab')
        package = QVBoxLayout(package_box)
        source_row = QHBoxLayout()
        self.source = QLineEdit(str(paths.dataset_dir()))
        self.output = QLineEdit(str(paths.runs_dir() / 'dirt_yolo'))
        self.classes = QComboBox()
        self.classes.addItem('by kind (crumbs/dust/spill/hair)', 'kind')
        self.classes.addItem('single "dirt" class', 'single')
        source_row.addWidget(QLabel('from'))
        source_row.addWidget(self.source, 1)
        package.addLayout(source_row)
        out_row = QHBoxLayout()
        out_row.addWidget(QLabel('to'))
        out_row.addWidget(self.output, 1)
        out_row.addWidget(self.classes)
        self.package_button = QPushButton('Package…')
        self.package_button.clicked.connect(self._package)
        out_row.addWidget(self.package_button)
        package.addLayout(out_row)
        layout.addWidget(package_box)

        self.output_log = QPlainTextEdit()
        self.output_log.setReadOnly(True)
        self.output_log.setMaximumBlockCount(500)
        layout.addWidget(self.output_log, 1)

        self._timer = QTimer(self)
        self._timer.setInterval(3000)
        self._timer.timeout.connect(self.refresh_counts)
        self._timer.start()
        self.refresh_counts()

    def set_recording(self, recording):
        self.recording = bool(recording)
        self.start_button.setEnabled(not recording)
        self.stop_button.setEnabled(recording)
        self.status.setText('recorder: running' if recording else 'recorder: stopped')

    def set_dataset_dir(self, path):
        self.source.setText(path)

    def refresh_counts(self):
        root = Path(self.source.text().strip() or paths.dataset_dir()).expanduser()
        if not root.is_dir():
            self.counts.setText('no dataset folder yet (%s)' % root)
            return
        episodes = sorted([p for p in root.iterdir() if p.is_dir()])
        images = 0
        labels = 0
        lines = []
        for episode in episodes[-8:]:
            count = len(list((episode / 'images').glob('*'))) if (episode / 'images').is_dir() else 0
            images += count
            labels += len(list((episode / 'labels').glob('*.txt'))) if (episode / 'labels').is_dir() else 0
            meta = episode / 'episode.json'
            info = ''
            if meta.is_file():
                try:
                    data = json.loads(meta.read_text())
                    info = 'dirt seed %s, %s frames' % (data.get('seed', '?'),
                                                        data.get('frames', '?'))
                except ValueError:
                    info = ''
            lines.append('%s: %d images  %s' % (episode.name, count, info))
        # older episodes are not re-counted for speed; say so instead of lying
        self.counts.setText('%d episode(s) in %s - newest %d hold %d images / %d label files'
                            % (len(episodes), root, min(len(episodes), 8), images, labels))
        self.episode_detail.setPlainText('\n'.join(lines))

    def set_output(self, text):
        self.output_log.appendPlainText(text.rstrip())

    def _package(self):
        self.package_requested.emit(self.source.text().strip(), self.output.text().strip(),
                                    self.classes.currentData())

    def run_package(self, source, output, classes):
        """Called by the window after the user confirms the paths."""
        cmd = package_dataset_command(source, output, classes)
        self.output_log.appendPlainText('$ ' + ' '.join(cmd))
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.output_log.appendPlainText('failed: %s' % exc)
            return
        self.output_log.appendPlainText((result.stdout or '') + (result.stderr or ''))
        self.output_log.appendPlainText('exit code %s' % result.returncode)
