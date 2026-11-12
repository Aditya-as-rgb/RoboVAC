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

"""Coverage mission controls, map saving and the per-piece list."""

import time
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QListWidget, QProgressBar,
                               QPushButton, QVBoxLayout, QWidget)

from . import paths
from .ui_common import hint


class MissionPanel(QWidget):
    plan_requested = Signal()
    start_requested = Signal()
    stop_requested = Signal()
    reset_meter_requested = Signal()
    save_map_requested = Signal(str)
    load_map_requested = Signal(str)
    reclean_toggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        head = QHBoxLayout()
        self.status = QLabel('mission: not running')
        self.status.setWordWrap(True)
        head.addWidget(self.status, 1)
        layout.addLayout(head)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat('%p % of reachable floor')
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        buttons = QHBoxLayout()
        self.plan = QPushButton('Plan (preview)')
        self.plan.setToolTip('/coverage_mission/plan - draws the plan, the robot stays put')
        self.plan.clicked.connect(self.plan_requested.emit)
        self.start = QPushButton('Start cleaning')
        self.start.setObjectName('primary')
        self.start.setToolTip('/coverage_mission/start')
        self.start.clicked.connect(self.start_requested.emit)
        self.stop = QPushButton('Stop')
        self.stop.setObjectName('danger')
        self.stop.setToolTip('/coverage_mission/stop')
        self.stop.clicked.connect(self.stop_requested.emit)
        self.reset = QPushButton('Reset meter')
        self.reset.setToolTip('/coverage_meter/reset - clears the cleaned area and percentages')
        self.reset.clicked.connect(self.reset_meter_requested.emit)
        for button in (self.plan, self.start, self.stop):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        layout.addWidget(self.reset)

        self.reclean = QCheckBox('Re-clean passes after the main plan')
        self.reclean.setToolTip('Restarts the mission node with reclean:=false when unticked '
                                '(a live change, the sim keeps running)')
        self.reclean.toggled.connect(self.reclean_toggled.emit)
        layout.addWidget(self.reclean)

        pieces_box = QGroupBox('Pieces (from /coverage/plan_markers)')
        pieces_layout = QVBoxLayout(pieces_box)
        self.pieces = QListWidget()
        self.pieces.setMaximumHeight(150)
        self.pieces_info = QLabel('no plan yet')
        self.pieces_info.setObjectName('hint')
        pieces_layout.addWidget(self.pieces)
        pieces_layout.addWidget(self.pieces_info)
        layout.addWidget(pieces_box)

        map_box = QGroupBox('Maps')
        map_layout = QFormLayout(map_box)
        save_row = QWidget()
        save_h = QHBoxLayout(save_row)
        save_h.setContentsMargins(0, 0, 0, 0)
        self.map_name = QLineEdit()
        self.map_name.setPlaceholderText('living_room')
        self.save_button = QPushButton('Save map')
        self.save_button.setToolTip('ros2 run vacuum_bringup save_map.sh <name>; writes into '
                                    '<workspace>/maps (needs the sim and a map topic)')
        self.save_button.clicked.connect(
            lambda: self.save_map_requested.emit(self.map_name.text().strip()))
        save_h.addWidget(self.map_name, 1)
        save_h.addWidget(self.save_button)
        map_layout.addRow('Save as', save_row)
        load_row = QWidget()
        load_h = QHBoxLayout(load_row)
        load_h.setContentsMargins(0, 0, 0, 0)
        self.map_pick = QComboBox()
        self.map_pick.setEditable(True)
        self.refresh_maps()
        browse = QPushButton('…')
        browse.clicked.connect(self._browse)
        load_h.addWidget(self.map_pick, 1)
        load_h.addWidget(browse)
        map_layout.addRow('Load', load_row)
        layout.addWidget(map_box)

        self.hint = hint('Plan first to check the route, then Start. Coverage needs AMCL '
                         '(slam:=False) and a finished map.')
        layout.addWidget(self.hint)
        layout.addStretch(1)

        self._history = []
        self._plan_pieces = []

    def refresh_maps(self):
        current = self.map_pick.currentText()
        self.map_pick.clear()
        try:
            for path in sorted(Path(paths.maps_dir()).glob('*.yaml')):
                self.map_pick.addItem(str(path))
        except OSError:
            pass
        if current:
            self.map_pick.setEditText(current)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Pick a map', str(paths.maps_dir()),
                                              'Map yaml (*.yaml *.yml)')
        if path:
            self.map_pick.setEditText(path)
            self.load_map_requested.emit(path)

    def set_coverage(self, pct, free_pct=None):
        self.progress.setValue(int(max(0.0, min(100.0, pct))))
        text = '%.1f %% of reachable floor' % pct
        if free_pct is not None:
            text += '   (%.1f %% of all free floor)' % free_pct
        self.progress.setFormat(text + '  [%p%]')

    def set_status(self, text):
        self._history.append('%.0f  %s' % (time.time(), text))
        self._history = self._history[-6:]
        self.status.setText(text)
        self.status.setToolTip('\n'.join(self._history[-6:]))

    def set_plan_pieces(self, markers, stamp=0.0):
        counts = {}
        entries = []
        for marker in markers:
            key = marker.label or marker.kind
            ns = (marker.label or '').split('_')[-1] or marker.kind
            if marker.kind == 'polyline':
                entries.append('%s  %.2f m' % (ns, self._polyline_length(marker.points)))
            counts[ns] = counts.get(ns, 0) + 1
        self.pieces.clear()
        for line in entries[:200]:
            self.pieces.addItem(line)
        if counts:
            summary = ', '.join('%s: %d' % (k, v) for k, v in sorted(counts.items()))
            self.pieces_info.setText('%d pieces - %s' % (len(entries), summary))
        else:
            self.pieces_info.setText('no plan yet')
        self._plan_pieces = entries

    @staticmethod
    def _polyline_length(points):
        total = 0.0
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            total += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        return total

    def set_running(self, running):
        for button in (self.plan, self.start, self.stop, self.reset):
            button.setEnabled(running)
        if not running:
            self.status.setText('mission: not running')

    def sync_reclean(self, value, blocked=True):
        self.reclean.blockSignals(True)
        self.reclean.setChecked(bool(value))
        self.reclean.blockSignals(blocked)
