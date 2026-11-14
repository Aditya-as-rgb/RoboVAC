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

"""Small notifications that float over the bottom-right corner of the window.

Every action that used to answer only in a log file or a one-line status label gets a card
here: a spinner card while it runs ("Starting stack..."), then the same card turns into a
green tick or a red cross with the outcome. They stack, fade out by themselves (errors stay
longest) and can be dismissed with a click.
"""

import time

from PySide6.QtCore import QEvent, QObject, QPropertyAnimation, Qt, QTimer
from PySide6.QtWidgets import (QFrame, QGraphicsOpacityEffect, QHBoxLayout, QLabel,
                               QSizePolicy)

try:
    from shiboken6 import isValid as _is_valid
except ImportError:      # pragma: no cover
    def _is_valid(obj):
        return obj is not None

ICONS = {'info': 'ℹ', 'ok': '✓', 'warn': '!', 'error': '✕'}
SPINNER = '◐◓◑◒'
TIMEOUTS = {'info': 3500, 'ok': 3000, 'warn': 7000, 'error': 10000}
MAX_VISIBLE = 5
WIDTH = 380
MARGIN = 16
GAP = 8


class Toast(QFrame):
    def __init__(self, manager, text, level):
        super().__init__(manager.window)
        self.manager = manager
        self.setObjectName('toast')
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedWidth(WIDTH)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(10)
        self.icon = QLabel()
        self.icon.setObjectName('toastIcon')
        self.icon.setFixedWidth(18)
        self.icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label = QLabel()
        self.label.setObjectName('toastText')
        self.label.setWordWrap(True)
        self.label.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)
        self.label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        layout.addWidget(self.icon)
        layout.addWidget(self.label, 1)
        self.effect = QGraphicsOpacityEffect(self)
        self.effect.setOpacity(1.0)
        self.setGraphicsEffect(self.effect)
        self._frame = 0
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.dismiss)
        self._fade = None
        self.level = ''
        self.created = time.monotonic()
        self.set_content(text, level)

    # ----------------------------------------------------------------- content --
    def set_content(self, text, level, timeout_ms=None):
        self.level = level
        self.label.setText(text)
        self.setProperty('level', level)
        self.icon.setText(SPINNER[0] if level == 'busy' else ICONS.get(level, 'ℹ'))
        style = self.style()
        style.unpolish(self)
        style.polish(self)
        self.adjustSize()
        self._timer.stop()
        if level == 'busy':
            return
        self.effect.setOpacity(1.0)
        ms = TIMEOUTS.get(level, 3500) if timeout_ms is None else timeout_ms
        if ms > 0:
            self._timer.start(ms)

    def spin(self):
        if self.level == 'busy':
            self._frame += 1
            self.icon.setText(SPINNER[self._frame % len(SPINNER)])

    # ----------------------------------------------------------------- closing --
    def mousePressEvent(self, event):      # noqa: N802 - Qt naming
        self.dismiss()

    def dismiss(self):
        if self._fade is not None:
            return
        self._fade = QPropertyAnimation(self.effect, b'opacity', self)
        self._fade.setDuration(220)
        self._fade.setStartValue(self.effect.opacity())
        self._fade.setEndValue(0.0)
        self._fade.finished.connect(lambda: self.manager._remove(self))
        self._fade.start()


class ToastManager(QObject):
    """Owns the stack of toasts for one window."""

    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.toasts = []
        self._last = ('', '', 0.0)
        self._spin = QTimer(self)
        self._spin.setInterval(130)
        self._spin.timeout.connect(self._spin_tick)
        window.installEventFilter(self)

    # --------------------------------------------------------------------- API --
    def show(self, text, level='info', timeout_ms=None):
        """A card that closes itself. Returns the Toast (or the one it refreshed)."""
        now = time.monotonic()
        last_text, last_level, last_at = self._last
        if text == last_text and level == last_level and now - last_at < 1.5:
            for toast in reversed(self.toasts):
                if toast.label.text() == text and toast.level == level:
                    toast.set_content(text, level, timeout_ms)
                    return toast
        self._last = (text, level, now)
        return self._add(text, level, timeout_ms)

    def busy(self, text):
        """A spinner card that stays until ``finish`` (or ``close``) is called on it."""
        return self._add(text, 'busy', None)

    def update(self, toast, text):
        if toast is not None and _is_valid(toast) and toast in self.toasts:
            toast.label.setText(text)
            toast.adjustSize()
            self.reposition()

    def finish(self, toast, text, level='ok', timeout_ms=None):
        """Turn a busy card into its outcome. A toast that is gone gets a fresh card."""
        if toast is None or not _is_valid(toast) or toast not in self.toasts:
            return self.show(text, level, timeout_ms)
        toast.set_content(text, level, timeout_ms)
        self.reposition()
        return toast

    def close(self, toast):
        if toast is not None and _is_valid(toast) and toast in self.toasts:
            toast.dismiss()

    # ------------------------------------------------------------------ layout --
    def _add(self, text, level, timeout_ms):
        toast = Toast(self, text, level)
        if level != 'busy' and timeout_ms is not None:
            toast.set_content(text, level, timeout_ms)
        self.toasts.append(toast)
        while len(self.toasts) > MAX_VISIBLE:
            old = next((t for t in self.toasts if t.level != 'busy'), self.toasts[0])
            self._remove(old)
        toast.show()
        toast.raise_()
        self.reposition()
        self._sync_spinner()
        return toast

    def _remove(self, toast):
        if toast in self.toasts:
            self.toasts.remove(toast)
        if _is_valid(toast):
            toast.hide()
            toast.deleteLater()
        self.reposition()
        self._sync_spinner()

    def reposition(self):
        window = self.window
        if not _is_valid(window):
            return
        bar = window.statusBar().height() if hasattr(window, 'statusBar') else 0
        y = window.height() - bar - MARGIN
        for toast in reversed(self.toasts):
            toast.adjustSize()
            y -= toast.height()
            toast.move(window.width() - toast.width() - MARGIN, y)
            toast.raise_()
            y -= GAP

    def _spin_tick(self):
        for toast in self.toasts:
            toast.spin()

    def _sync_spinner(self):
        busy = any(t.level == 'busy' for t in self.toasts)
        if busy and not self._spin.isActive():
            self._spin.start()
        elif not busy:
            self._spin.stop()

    def eventFilter(self, obj, event):      # noqa: N802 - Qt naming
        if obj is getattr(self, 'window', None) and event.type() in (QEvent.Type.Resize, QEvent.Type.Show):
            self.reposition()
        return False
