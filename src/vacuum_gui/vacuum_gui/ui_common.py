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

"""Tiny widgets shared by the panels: coloured dots, labelled values, headers."""

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout,
                               QWidget)

from . import status


class Dot(QWidget):
    """A filled circle: green / amber / red / grey, used by every health light."""

    def __init__(self, state=status.UNKNOWN, size=11, parent=None):
        super().__init__(parent)
        self._state = state
        self._size = size
        self.setFixedSize(size + 4, size + 4)

    def set_state(self, state):
        if state != self._state:
            self._state = state
            self.update()

    def state(self):
        return self._state

    def paintEvent(self, event):      # noqa: N802 - Qt naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(status.LIGHT_COLORS.get(self._state, status.LIGHT_COLORS[status.UNKNOWN]))
        painter.setBrush(color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(2, 2, self._size, self._size)
        painter.end()


class LabeledValue(QWidget):
    """A caption above a value, e.g. 'coverage' / '42.3 %'."""

    def __init__(self, caption, value='--', tooltip='', parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 2, 6, 2)
        layout.setSpacing(0)
        self.caption = QLabel(caption.upper())
        self.caption.setObjectName('caption')
        self.value = QLabel(value)
        self.value.setObjectName('value')
        self.value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.caption)
        layout.addWidget(self.value)
        if tooltip:
            self.setToolTip(tooltip)

    def set_value(self, text, tooltip=None):
        self.value.setText(str(text))
        if tooltip is not None:
            self.setToolTip(tooltip)


class SectionHeader(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text.upper(), parent)
        self.setObjectName('section')


def hline():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setFrameShadow(QFrame.Shadow.Plain)
    line.setObjectName('hline')
    return line


def hint(text, wrap=True):
    label = QLabel(text)
    label.setObjectName('hint')
    label.setWordWrap(wrap)
    label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
    return label


def row(*widgets, spacing=8, margins=(0, 0, 0, 0)):
    box = QWidget()
    layout = QHBoxLayout(box)
    layout.setContentsMargins(*margins)
    layout.setSpacing(spacing)
    for widget in widgets:
        if widget is None:
            layout.addStretch(1)
        elif isinstance(widget, str):
            layout.addWidget(QLabel(widget))
        else:
            layout.addWidget(widget)
    return box


def caption(text):
    label = QLabel(text)
    label.setObjectName('caption')
    return label
