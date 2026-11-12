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

"""Health lights: one row per node and per key topic, with the reason on click."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QAbstractItemView, QDialog, QDialogButtonBox, QHBoxLayout,
                               QHeaderView, QLabel, QPushButton, QTableWidget,
                               QTableWidgetItem, QTextBrowser, QVBoxLayout, QWidget)

from . import status
from .ui_common import Dot


class HealthPanel(QWidget):
    """Green/amber/red rows; clicking a row explains what to do about it."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        header = QHBoxLayout()
        self.summary_dot = Dot()
        self.summary = QLabel('nothing to check yet')
        self.summary.setWordWrap(True)
        self.recheck = QPushButton('Re-check')
        self.recheck.setToolTip('Run the pre-flight checks again')
        header.addWidget(self.summary_dot)
        header.addWidget(self.summary, 1)
        header.addWidget(self.recheck)
        layout.addLayout(header)

        self.table = QTableWidget(0, 3, self)
        self.table.setHorizontalHeaderLabels(['', 'Item', 'State'])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setShowGrid(False)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        self.table.setColumnWidth(0, 26)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.table.cellDoubleClicked.connect(self._show_detail)
        self.table.cellClicked.connect(self._show_detail)
        layout.addWidget(self.table, 1)

        self.hint = QLabel('Double-click a red light to see the fix hint.')
        self.hint.setObjectName('hint')
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        self._items = []

    def update_health(self, items):
        self._items = list(items)
        self.table.setRowCount(len(items))
        for row, item in enumerate(items):
            dot = self.table.cellWidget(row, 0)
            if dot is None:
                dot = Dot(item.state)
                self.table.setCellWidget(row, 0, dot)
            else:
                dot.set_state(item.state)
            name = QTableWidgetItem(item.label)
            name.setToolTip((item.hint or '') + '\n' + item.detail)
            state = QTableWidgetItem(item.detail)
            state.setToolTip(item.hint or item.detail)
            if item.state == status.FAIL:
                state.setForeground(Qt.GlobalColor.red)
            elif item.state == status.WARN:
                state.setForeground(Qt.GlobalColor.darkYellow)
            self.table.setItem(row, 1, name)
            self.table.setItem(row, 2, state)
        state, text = status.summarise_health(items)
        self.summary_dot.set_state(state)
        self.summary.setText(text)
        self.summary.setToolTip('\n'.join('%s: %s' % (i.label, i.detail) for i in items))

    def _show_detail(self, row, _column=0):
        if 0 <= row < len(self._items):
            item = self._items[row]
            dialog = QDialog(self)
            dialog.setWindowTitle(item.label)
            layout = QVBoxLayout(dialog)
            browser = QTextBrowser(dialog)
            browser.setHtml(
                '<h3>%s</h3><p><b>%s</b>: %s</p><p>%s</p>'
                % (item.label, item.state.upper(), item.detail,
                   item.hint.replace('\n', '<br>') if item.hint else 'No extra hint.'))
            layout.addWidget(browser)
            buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
            buttons.rejected.connect(dialog.reject)
            layout.addWidget(buttons)
            dialog.resize(460, 240)
            dialog.exec()
