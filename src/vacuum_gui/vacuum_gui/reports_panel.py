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

"""Browse ``~/vacuum_runs/run_*``, compare two runs, plot coverage over time."""

import os
import subprocess

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QListWidget,
                               QListWidgetItem, QMessageBox, QPushButton, QSplitter,
                               QTableWidget, QTableWidgetItem, QTextBrowser, QVBoxLayout,
                               QWidget)

from . import run_report
from .coverage_plot import CoveragePlot


class ReportsPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        header = QHBoxLayout()
        header.addWidget(QLabel('Runs in ~/vacuum_runs'))
        self.refresh_button = QPushButton('Refresh')
        self.refresh_button.clicked.connect(self.refresh)
        self.open_button = QPushButton('Open report.png')
        self.open_button.clicked.connect(self._open_png)
        self.folder_button = QPushButton('Open folder')
        self.folder_button.clicked.connect(self._open_folder)
        header.addStretch(1)
        header.addWidget(self.refresh_button)
        header.addWidget(self.folder_button)
        header.addWidget(self.open_button)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.list = QListWidget()
        self.list.currentRowChanged.connect(self._selected)
        left_layout.addWidget(self.list)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.detail = QTextBrowser()
        self.detail.setMaximumHeight(130)
        right_layout.addWidget(self.detail)
        self.plot = CoveragePlot()
        right_layout.addWidget(self.plot, 1)
        compare_row = QHBoxLayout()
        compare_row.addWidget(QLabel('compare A'))
        self.compare_a = QComboBox()
        compare_row.addWidget(self.compare_a, 1)
        compare_row.addWidget(QLabel('B'))
        self.compare_b = QComboBox()
        compare_row.addWidget(self.compare_b, 1)
        self.compare_button = QPushButton('Compare')
        self.compare_button.clicked.connect(self._compare)
        compare_row.addWidget(self.compare_button)
        right_layout.addLayout(compare_row)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['metric', 'A', 'B', 'better'])
        self.table.verticalHeader().setVisible(False)
        self.table.setMaximumHeight(220)
        right_layout.addWidget(self.table)
        splitter.addWidget(right)
        splitter.setSizes([260, 620])
        layout.addWidget(splitter, 1)

        self.runs = []
        self.refresh()

    def refresh(self):
        self.runs = run_report.list_runs()
        self.list.clear()
        for entry in self.runs:
            item = QListWidgetItem(entry.label)
            item.setToolTip(str(entry.path))
            self.list.addItem(item)
        for combo in (self.compare_a, self.compare_b):
            combo.clear()
            for entry in self.runs:
                combo.addItem(entry.name)
        if self.runs:
            self.list.setCurrentRow(0)
            if len(self.runs) > 1:
                self.compare_b.setCurrentIndex(1)
        else:
            self.detail.setHtml('<p>No runs yet. Every finished mission writes '
                                '<code>run_YYYYmmdd_HHMMSS/</code> here.</p>')
            self.plot.clear()

    def _selected(self, row):
        if not (0 <= row < len(self.runs)):
            return
        entry = self.runs[row]
        text = run_report.describe(entry).replace('\n', '<br>')
        path = str(entry.path)
        self.detail.setHtml('<p>%s<br><code>%s</code></p>' % (text, path))
        series = run_report.coverage_series(entry.path)
        self.plot.set_series([
            {'label': 'reachable floor', 't': series['t'], 'y': series['reachable']},
            {'label': 'all free floor', 't': series['t'], 'y': series['free']},
        ])

    def _compare(self):
        index_a = self.compare_a.currentIndex()
        index_b = self.compare_b.currentIndex()
        if not (0 <= index_a < len(self.runs) and 0 <= index_b < len(self.runs)):
            return
        a, b = self.runs[index_a], self.runs[index_b]
        rows = run_report.compare_runs(a.summary, b.summary)
        self.table.setRowCount(len(rows))
        for row, metric in enumerate(rows):
            self.table.setItem(row, 0, QTableWidgetItem(metric.label))
            self.table.setItem(row, 1, QTableWidgetItem(
                metric.text(metric.a) + (' ' + metric.unit if metric.unit else '')))
            self.table.setItem(row, 2, QTableWidgetItem(
                metric.text(metric.b) + (' ' + metric.unit if metric.unit else '')))
            better = QTableWidgetItem('B' if metric.better == 'b' else 'A' if metric.better == 'a' else '')
            if metric.better:
                better.setForeground(Qt.GlobalColor.darkGreen)
            self.table.setItem(row, 3, better)
        series_a = run_report.coverage_series(a.path)
        series_b = run_report.coverage_series(b.path)
        self.plot.set_series([
            {'label': 'A ' + a.name[-6:] + ' reachable', 't': series_a['t'],
             'y': series_a['reachable']},
            {'label': 'B ' + b.name[-6:] + ' reachable', 't': series_b['t'],
             'y': series_b['reachable']},
        ])
        self.table.setToolTip('A = %s\nB = %s\n"better" compares numerically: coverage up, '
                              'time/distance/missed area down.' % (a.path, b.path))

    def _current(self):
        row = self.list.currentRow()
        if 0 <= row < len(self.runs):
            return self.runs[row]
        return None

    def _open_png(self):
        entry = self._current()
        if not entry:
            return
        png = entry.path / 'report.png'
        if not png.is_file():
            QMessageBox.information(self, 'No report.png',
                                    'This run has no report.png (matplotlib missing?).\n'
                                    'summary.json and coverage.csv are still there.')
            return
        subprocess.Popen(['xdg-open', str(png)])

    def _open_folder(self):
        entry = self._current()
        if entry:
            subprocess.Popen(['xdg-open', str(entry.path)])
