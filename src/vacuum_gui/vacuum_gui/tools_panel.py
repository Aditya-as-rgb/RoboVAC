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

"""Detector tuning, screenshots/GIF recording and a rosbag toggle."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
                               QLabel, QLineEdit, QPushButton, QSpinBox, QVBoxLayout,
                               QWidget)

from .settings import DETECTOR_DEFAULTS
from .ui_common import hint


class ToolsPanel(QWidget):
    params_apply_requested = Signal(dict)
    params_reset_requested = Signal()
    screenshot_requested = Signal(str)          # 'map' | 'camera'
    gif_requested = Signal(str, int, int)       # target, seconds, fps
    rosbag_toggle_requested = Signal(bool, list)

    def __init__(self, view_settings=None, parent=None):
        super().__init__(parent)
        view_settings = view_settings or {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)

        detector_box = QGroupBox('Dirt detector parameters (live)')
        form = QFormLayout(detector_box)
        self.conf = QDoubleSpinBox()
        self.conf.setRange(0.01, 0.99)
        self.conf.setSingleStep(0.05)
        self.iou = QDoubleSpinBox()
        self.iou.setRange(0.05, 0.95)
        self.iou.setSingleStep(0.05)
        self.max_range = QDoubleSpinBox()
        self.max_range.setRange(0.2, 5.0)
        self.max_range.setSingleStep(0.1)
        self.max_range.setSuffix(' m')
        self.min_hits = QSpinBox()
        self.min_hits.setRange(1, 50)
        self.clear_misses = QSpinBox()
        self.clear_misses.setRange(1, 100)
        for widget, key in ((self.conf, 'conf'), (self.iou, 'iou'),
                            (self.max_range, 'max_range'), (self.min_hits, 'min_hits'),
                            (self.clear_misses, 'clear_misses')):
            widget.setValue(DETECTOR_DEFAULTS[key])
        form.addRow('confidence', self.conf)
        form.addRow('IoU (NMS)', self.iou)
        form.addRow('max range', self.max_range)
        form.addRow('min hits', self.min_hits)
        form.addRow('clear misses', self.clear_misses)
        buttons = QHBoxLayout()
        self.apply = QPushButton('Apply to /dirt_detector')
        self.apply.clicked.connect(
            lambda: self.params_apply_requested.emit(self.values()))
        self.reset = QPushButton('Reset to defaults')
        self.reset.clicked.connect(self.params_reset_requested.emit)
        buttons.addWidget(self.apply)
        buttons.addWidget(self.reset)
        form.addRow(buttons)
        self.param_status = QLabel('waiting for the detector')
        self.param_status.setObjectName('hint')
        self.param_status.setWordWrap(True)
        form.addRow(self.param_status)
        layout.addWidget(detector_box)
        layout.addWidget(hint('These go through ros2 param set, so the detector reloads '
                              'nothing and the model stays loaded. min_hits and clear_misses '
                              'trade false spots against the re-clean loop reacting late.'))

        slides_box = QGroupBox('Screenshots and GIFs (for slides)')
        slides = QVBoxLayout(slides_box)
        shot_row = QHBoxLayout()
        map_shot = QPushButton('Screenshot map')
        map_shot.clicked.connect(lambda: self.screenshot_requested.emit('map'))
        camera_shot = QPushButton('Screenshot camera')
        camera_shot.clicked.connect(lambda: self.screenshot_requested.emit('camera'))
        shot_row.addWidget(map_shot)
        shot_row.addWidget(camera_shot)
        slides.addLayout(shot_row)
        gif_row = QHBoxLayout()
        self.gif_seconds = QSpinBox()
        self.gif_seconds.setRange(2, 120)
        self.gif_seconds.setValue(int(view_settings.get('recording_seconds', 20)))
        self.gif_seconds.setSuffix(' s')
        self.gif_fps = QSpinBox()
        self.gif_fps.setRange(1, 10)
        self.gif_fps.setValue(int(view_settings.get('recording_fps', 2)))
        self.gif_fps.setSuffix(' fps')
        self.gif_target = QLineEdit('map')
        self.gif_target.setToolTip('map or camera')
        self.gif_button = QPushButton('Record GIF')
        self.gif_button.clicked.connect(
            lambda: self.gif_requested.emit(self.gif_target.text().strip() or 'map',
                                            self.gif_seconds.value(), self.gif_fps.value()))
        gif_row.addWidget(QLabel('target'))
        gif_row.addWidget(self.gif_target)
        gif_row.addWidget(self.gif_seconds)
        gif_row.addWidget(self.gif_fps)
        gif_row.addWidget(self.gif_button)
        slides.addLayout(gif_row)
        slides.addWidget(hint('Saved to ~/vacuum_runs/screenshots. The GIF is built from the '
                              'panel as it is on screen, so pan/zoom first.'))
        layout.addWidget(slides_box)

        bag_box = QGroupBox('rosbag record')
        bag = QVBoxLayout(bag_box)
        self.topics = QLineEdit(', '.join(view_settings.get(
            'rosbag_topics', ['/scan', '/odom', '/tf', '/tf_static'])))
        bag.addWidget(QLabel('topics'))
        bag.addWidget(self.topics)
        bag_row = QHBoxLayout()
        self.bag_button = QPushButton('Start recording')
        self.bag_button.setCheckable(True)
        self.bag_button.toggled.connect(self._bag_toggled)
        self.bag_status = QLabel('stopped')
        self.bag_status.setObjectName('hint')
        bag_row.addWidget(self.bag_button)
        bag_row.addWidget(self.bag_status, 1)
        bag.addLayout(bag_row)
        bag.addWidget(hint('Writes ~/vacuum_runs/bags/bag_<time> with rosbag2. Keep the topic '
                           'list short: /camera/image is ~30 MB per minute.'))
        layout.addWidget(bag_box)
        layout.addStretch(1)

    def values(self):
        return {'conf': round(self.conf.value(), 3), 'iou': round(self.iou.value(), 3),
                'max_range': round(self.max_range.value(), 3),
                'min_hits': int(self.min_hits.value()),
                'clear_misses': int(self.clear_misses.value())}

    def defaults(self):
        return dict(DETECTOR_DEFAULTS)

    def set_values(self, values):
        mapping = {'conf': self.conf, 'iou': self.iou, 'max_range': self.max_range,
                   'min_hits': self.min_hits, 'clear_misses': self.clear_misses}
        for key, widget in mapping.items():
            if key in values:
                widget.blockSignals(True)
                widget.setValue(values[key])
                widget.blockSignals(False)
        self.param_status.setText('live: ' + ', '.join('%s=%s' % kv
                                                      for kv in sorted(values.items())))

    def set_param_status(self, text):
        self.param_status.setText(text)

    def _bag_toggled(self, active):
        topics = [t.strip() for t in self.topics.text().split(',') if t.strip()]
        self.bag_button.setText('Stop recording' if active else 'Start recording')
        self.rosbag_toggle_requested.emit(active, topics)

    def set_bag_state(self, recording, detail=''):
        self.bag_button.blockSignals(True)
        self.bag_button.setChecked(bool(recording))
        self.bag_button.setText('Stop recording' if recording else 'Start recording')
        self.bag_button.blockSignals(False)
        self.bag_status.setText(detail or ('recording' if recording else 'stopped'))

    def set_gif_enabled(self, enabled):
        self.gif_button.setEnabled(bool(enabled))
