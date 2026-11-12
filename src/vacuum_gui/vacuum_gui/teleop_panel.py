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

"""On-screen teleop pad, speed limits and the emergency stop."""

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
                               QSlider, QToolButton, QVBoxLayout, QWidget)

from .ui_common import Dot, hint


class TeleopPanel(QWidget):
    cmd_vel = Signal(float, float)          # m/s, rad/s
    estop_toggled = Signal(bool)
    gamepad_devices_changed = Signal()

    def __init__(self, linear=0.25, angular=1.0, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.estop = QPushButton('EMERGENCY STOP\n(Space)')
        self.estop.setObjectName('danger')
        self.estop.setCheckable(True)
        self.estop.setMinimumHeight(64)
        self.estop.setToolTip('Cancels the Nav2 goal and the mission and publishes zero '
                              '/cmd_vel 10x a second until released. gz DiffDrive keeps the '
                              'last command, so the zero stream has to keep coming.')
        self.estop.toggled.connect(self.estop_toggled.emit)
        layout.addWidget(self.estop)

        pad_box = QGroupBox('Drive')
        pad_layout = QVBoxLayout(pad_box)
        grid = QGridLayout()
        grid.setSpacing(4)
        self._buttons = {}
        for (label, lin, ang, row, col) in (
                ('↖', 1.0, 1.0, 0, 0), ('↑', 1.0, 0.0, 0, 1), ('↗', 1.0, -1.0, 0, 2),
                ('←', 0.0, 1.0, 1, 0), ('■', 0.0, 0.0, 1, 1), ('→', 0.0, -1.0, 1, 2),
                ('↙', -1.0, 1.0, 2, 0), ('↓', -1.0, 0.0, 2, 1), ('↘', -1.0, -1.0, 2, 2)):
            button = QToolButton()
            button.setText(label)
            button.setMinimumSize(46, 40)
            button.pressed.connect(lambda l=lin, a=ang: self._press(l, a))
            button.released.connect(self._release)
            grid.addWidget(button, row, col)
            self._buttons[label] = button
        pad_layout.addLayout(grid)
        pad_layout.addWidget(hint('Hold to drive. Release to stop. The sim keeps the last '
                                  '/cmd_vel, so the pad sends zeros when you let go.'))
        layout.addWidget(pad_box)

        speed_box = QGroupBox('Speed limits (Nav2 caps: 0.30 m/s, 1.20 rad/s)')
        speed_layout = QVBoxLayout(speed_box)
        self.linear_slider = QSlider(Qt.Orientation.Horizontal)
        self.linear_slider.setRange(5, 40)          # 0.05 .. 0.40 m/s
        self.linear_slider.setValue(int(round(float(linear) * 100)))
        self.angular_slider = QSlider(Qt.Orientation.Horizontal)
        self.angular_slider.setRange(20, 200)       # 0.20 .. 2.00 rad/s
        self.angular_slider.setValue(int(round(float(angular) * 100)))
        self.linear_label = QLabel('%.2f m/s' % linear)
        self.angular_label = QLabel('%.2f rad/s' % angular)
        self.linear_slider.valueChanged.connect(self._speed_changed)
        self.angular_slider.valueChanged.connect(self._speed_changed)
        row1 = QHBoxLayout()
        row1.addWidget(QLabel('linear'))
        row1.addWidget(self.linear_slider, 1)
        row1.addWidget(self.linear_label)
        row2 = QHBoxLayout()
        row2.addWidget(QLabel('angular'))
        row2.addWidget(self.angular_slider, 1)
        row2.addWidget(self.angular_label)
        speed_layout.addLayout(row1)
        speed_layout.addLayout(row2)
        layout.addWidget(speed_box)

        gamepad_box = QGroupBox('Gamepad')
        gamepad_layout = QHBoxLayout(gamepad_box)
        self.gamepad_dot = Dot()
        self.gamepad_label = QLabel('no /joy messages yet')
        self.gamepad_label.setWordWrap(True)
        gamepad_layout.addWidget(self.gamepad_dot)
        gamepad_layout.addWidget(self.gamepad_label, 1)
        layout.addWidget(gamepad_box)
        layout.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)
        self._direction = (0.0, 0.0)

    # ------------------------------------------------------------------ drive --
    def linear_speed(self):
        return self.linear_slider.value() / 100.0

    def angular_speed(self):
        return self.angular_slider.value() / 100.0

    def _speed_changed(self, _value):
        self.linear_label.setText('%.2f m/s' % self.linear_speed())
        self.angular_label.setText('%.2f rad/s' % self.angular_speed())

    def set_speeds(self, linear, angular):
        self.linear_slider.setValue(int(round(float(linear) * 100)))
        self.angular_slider.setValue(int(round(float(angular) * 100)))

    def _press(self, linear, angular):
        self._direction = (linear, angular)
        self._tick()
        self._timer.start()

    def _release(self):
        self._timer.stop()
        self._direction = (0.0, 0.0)
        self.cmd_vel.emit(0.0, 0.0)

    def _tick(self):
        lin, ang = self._direction
        if self.estop.isChecked():
            self.cmd_vel.emit(0.0, 0.0)
            return
        self.cmd_vel.emit(lin * self.linear_speed(), ang * self.angular_speed())

    def stop_all(self):
        self._timer.stop()
        self._direction = (0.0, 0.0)
        self.cmd_vel.emit(0.0, 0.0)

    def set_estop(self, active):
        self.estop.blockSignals(True)
        self.estop.setChecked(bool(active))
        self.estop.blockSignals(False)
        self.estop.setText('E-STOP ACTIVE - press to release' if active
                           else 'EMERGENCY STOP\n(Space)')

    def set_gamepad(self, connected, detail=''):
        self.gamepad_dot.set_state('ok' if connected else 'unknown')
        self.gamepad_label.setText(detail or ('gamepad connected (/joy is publishing)'
                                              if connected else 'no /joy messages yet'))

    def set_enabled_buttons(self, enabled):
        for button in self._buttons.values():
            button.setEnabled(enabled)
        if not enabled:
            self.stop_all()
