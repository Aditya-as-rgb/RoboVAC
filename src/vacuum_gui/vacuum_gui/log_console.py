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

"""Log console: one tab per managed process, plus All, with colours, filter and save."""

import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (QCheckBox, QFileDialog, QHBoxLayout, QLabel, QLineEdit,
                               QMessageBox, QPlainTextEdit, QPushButton, QTabWidget,
                               QVBoxLayout, QWidget)

LEVEL_COLORS = {
    'error': QColor('#ff6b6b'),
    'warn': QColor('#ffd43b'),
    'info': QColor('#9ad0f5'),
    'cmd': QColor('#9aa4af'),
    'exit': QColor('#c0c6cc'),
    'stop': QColor('#c0c6cc'),
}
ERROR_WORDS = ('error', 'exception', 'traceback', 'failed', 'fatal', 'abort')
WARN_WORDS = ('warn', 'timeout', 'not found', 'no such', 'dropp')


def classify(stream, text):
    if stream in LEVEL_COLORS:
        return stream
    low = text.lower()
    if any(word in low for word in ERROR_WORDS):
        return 'error'
    if any(word in low for word in WARN_WORDS):
        return 'warn'
    if '[info]' in low or 'info' in low:
        return 'info'
    return 'stdout'


class LogConsole(QWidget):
    MAX_BLOCKS = 4000

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        controls = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText('filter text (case-insensitive)…')
        self.filter.textChanged.connect(self._apply_filter)
        self.autoscroll = QCheckBox('follow')
        self.autoscroll.setChecked(True)
        self.save_button = QPushButton('Save…')
        self.save_button.clicked.connect(self._save)
        self.clear_button = QPushButton('Clear')
        self.clear_button.clicked.connect(self.clear)
        controls.addWidget(QLabel('Filter'))
        controls.addWidget(self.filter, 1)
        controls.addWidget(self.autoscroll)
        controls.addWidget(self.save_button)
        controls.addWidget(self.clear_button)
        layout.addLayout(controls)

        self.tabs = QTabWidget(self)
        self.tabs.setDocumentMode(True)
        layout.addWidget(self.tabs, 1)
        self._views = {}
        self._all = self._make_view('all')
        self.tabs.addTab(self._all.parentWidget() or self._all, 'All')
        self._tabs_by_name = {'all': 0}
        self._hidden = {}
        self._pending = []
        self._last_flush = 0.0

    # ------------------------------------------------------------------ views --
    def _make_view(self, name):
        page = QWidget()
        box = QVBoxLayout(page)
        box.setContentsMargins(0, 0, 0, 0)
        view = QPlainTextEdit(page)
        view.setReadOnly(True)
        view.setMaximumBlockCount(self.MAX_BLOCKS)
        view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        view.setFont(QFont('monospace', 9))
        view.setStyleSheet('background:#101317; color:#d6dbe0; border:none;')
        box.addWidget(view)
        self._views[name] = (page, view)
        return view

    def _ensure_tab(self, name):
        if name in self._views:
            return
        self._make_view(name)
        page = self._views[name][0]
        index = self.tabs.addTab(page, name)
        self._tabs_by_name[name] = index

    def append(self, proc, stream, text, level=None):
        """Queue one line; the widget flushes at most every 100 ms."""
        self._pending.append((proc, stream, text.rstrip('\n'),
                              level or classify(stream, text)))
        now = time.monotonic()
        if now - self._last_flush >= 0.1:
            self.flush()

    def flush(self):
        self._last_flush = time.monotonic()
        if not self._pending:
            return
        pending, self._pending = self._pending, []
        for proc, stream, text, level in pending:
            for name in ('all', proc):
                self._ensure_tab(name)
                view = self._views[name][1]
                fmt = QTextCharFormat()
                fmt.setForeground(LEVEL_COLORS.get(level, QColor('#d6dbe0')))
                cursor = QTextCursor(view.document())
                cursor.movePosition(QTextCursor.MoveOperation.End)
                prefix = '' if name == 'all' else ''
                label = ('[%s] ' % proc) if name == 'all' else ''
                cursor.insertText('%s%s\n' % (label, text), fmt)
        if self.autoscroll.isChecked():
            view = self._views['all'][1]
            view.verticalScrollBar().setValue(view.verticalScrollBar().maximum())
        self._apply_filter(self.filter.text())

    def _apply_filter(self, text):
        needle = text.strip().lower()
        for name, (page, view) in self._views.items():
            document = view.document()
            block = document.firstBlock()
            visible = 0
            while block.isValid():
                block.setVisible(not needle or needle in block.text().lower())
                if block.isVisible():
                    visible += 1
                block = block.next()
            document.markContentsDirty(0, document.characterCount())
            view.setVisible(visible > 0 or not needle)
        self._hidden = {}

    def clear(self):
        for _page, view in self._views.values():
            view.clear()
        self._pending = []

    def save(self, path=None):
        self.flush()
        if not path:
            path, _ = QFileDialog.getSaveFileName(
                self, 'Save log', 'vacuum_gui_%s.log'
                % time.strftime('%Y%m%d_%H%M%S'), 'Log files (*.log *.txt)')
        if not path:
            return None
        with open(path, 'w') as f:
            f.write(self._views['all'][1].toPlainText())
        return path

    def _save(self):
        try:
            path = self.save()
        except OSError as exc:
            QMessageBox.warning(self, 'Could not save the log', str(exc))
            return
        if path:
            self.window().statusBar().showMessage('log saved to %s' % path, 8000)

    def text(self):
        self.flush()
        return self._views['all'][1].toPlainText()
