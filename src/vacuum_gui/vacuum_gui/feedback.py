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

"""Visible feedback for button presses.

Two things, both driven by dynamic properties the stylesheet in ``theme.py`` reacts to:

* ``PressFeedback`` - an application-wide event filter that flashes any push or tool
  button for a moment when it is clicked, so even an instant action (one that only calls a
  ROS service) visibly registers.
* ``begin`` / ``end`` - a busy state for buttons whose action takes time: the label turns
  into a spinner plus a short verb ("Starting..."), the button is disabled so it cannot be
  double-fired, and ``end`` restores it and rings it green or red for a moment.
"""

from PySide6.QtCore import QEvent, QObject, QTimer
from PySide6.QtWidgets import QPushButton, QToolButton

try:                                # PySide6 ships shiboken6; tests may stub it
    from shiboken6 import isValid as _is_valid
except ImportError:      # pragma: no cover
    def _is_valid(obj):
        return obj is not None

SPINNER = '◐◓◑◒'
FLASH_MS = 320
RESULT_MS = 1400
SPIN_MS = 130


def _repolish(widget):
    style = widget.style()
    style.unpolish(widget)
    style.polish(widget)
    widget.update()


def _set_prop(widget, name, value):
    if not _is_valid(widget):
        return
    widget.setProperty(name, value)
    _repolish(widget)


def flash(widget, ms=FLASH_MS):
    """Briefly mark ``widget`` as just-pressed (style: ``[flash="true"]``)."""
    if not _is_valid(widget):
        return
    _set_prop(widget, 'flash', True)
    QTimer.singleShot(ms, lambda w=widget: _set_prop(w, 'flash', False))


class PressFeedback(QObject):
    """Install once on the QApplication: ``app.installEventFilter(PressFeedback(app))``."""

    def eventFilter(self, obj, event):      # noqa: N802 - Qt naming
        if event.type() == QEvent.Type.MouseButtonRelease and \
                isinstance(obj, (QPushButton, QToolButton)) and obj.isEnabled():
            try:
                inside = obj.rect().contains(event.position().toPoint())
            except AttributeError:
                inside = True
            if inside and not obj.property('busy'):
                flash(obj)
        return False


class _Busy:
    def __init__(self, button, text, disable):
        self.button = button
        self.text = text
        self.base_text = button.text()
        self.was_enabled = button.isEnabled()
        self.frame = 0
        self.timer = QTimer(button)
        self.timer.setInterval(SPIN_MS)
        self.timer.timeout.connect(self._tick)
        self._tick()
        self.timer.start()
        if disable:
            button.setEnabled(False)

    def label(self):
        return '%s  %s' % (SPINNER[self.frame % len(SPINNER)], self.text)

    def _tick(self):
        if _is_valid(self.button):
            self.button.setText(self.label())
        self.frame += 1


def begin(button, text, disable=True):
    """Show ``button`` as busy with ``text`` ("Starting..."); safe to call twice."""
    if not _is_valid(button):
        return
    state = getattr(button, '_busy_state', None)
    if state is not None:
        state.text = text
        return
    button._busy_state = _Busy(button, text, disable)
    _set_prop(button, 'busy', True)
    _set_prop(button, 'result', '')


def set_text(button, text):
    """Change the busy label of a button that is already busy."""
    state = getattr(button, '_busy_state', None)
    if state is not None:
        state.text = text
        state.button.setText(state.label())


def is_busy(button):
    return getattr(button, '_busy_state', None) is not None


def end(button, result=None, enable=None):
    """Leave the busy state.

    ``result`` is ``'ok'`` / ``'fail'`` / None: the button rings green / red for a moment.
    ``enable`` True/False forces the enabled state afterwards (use it when the panel
    decides, e.g. Start is disabled once the stack runs); None restores what it was.
    """
    if not _is_valid(button):
        return
    state = getattr(button, '_busy_state', None)
    if state is not None:
        state.timer.stop()
        state.timer.deleteLater()
        button.setText(state.base_text)
        button.setEnabled(state.was_enabled if enable is None else bool(enable))
        button._busy_state = None
    elif enable is not None:
        button.setEnabled(bool(enable))
    _set_prop(button, 'busy', False)
    if result in ('ok', 'fail'):
        _set_prop(button, 'result', result)
        QTimer.singleShot(RESULT_MS, lambda b=button: _set_prop(b, 'result', ''))
    else:
        _set_prop(button, 'result', '')
