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

"""The launch manager panel: every option, the exact command, Start/Stop/restart.

Launch-time options (map, world, camera, dirt, odometry, headless) are greyed out while
the stack runs, and changing them raises a "restart needed" badge with a one-click
"Apply & restart". The live ones (teleop, detector, coverage, RViz, recording) are separate
processes owned by the window, so switching them never restarts Gazebo.
"""

import os
from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLayout, QLineEdit, QMessageBox,
                               QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy,
                               QSpinBox, QToolButton, QVBoxLayout, QWidget)

from . import paths, preflight
from .commands import StackConfig, shell_join
from .settings import default_presets
from .ui_common import Dot, SectionHeader, hint

LAUNCH_TIME_FIELDS = ('slam', 'map', 'world', 'x_pose', 'y_pose', 'odom_source',
                      'headless', 'cameras', 'dirt', 'dirt_seed', 'dirt_scale',
                      'dirt_stubborn', 'dirt_brush', 'dirt_clean', 'dirt_randomize',
                      'dirt_dir', 'nav_delay', 'qt_platform')

FIELD_LABELS = {
    'slam': 'mode', 'map': 'map', 'world': 'world', 'x_pose': 'spawn x',
    'y_pose': 'spawn y', 'odom_source': 'odometry', 'headless': 'headless',
    'cameras': 'camera', 'dirt': 'dirt count', 'dirt_seed': 'dirt seed',
    'dirt_scale': 'dirt scale', 'dirt_stubborn': 'stubborn share',
    'dirt_brush': 'brush radius', 'dirt_clean': 'brush removes dirt',
    'dirt_randomize': 'randomise floor/light', 'dirt_dir': 'dirt folder',
    'nav_delay': 'Nav2 delay', 'qt_platform': 'Qt platform',
}


class PreflightDialog(QDialog):
    def __init__(self, checks, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Pre-flight checks')
        self.resize(640, 420)
        layout = QVBoxLayout(self)
        ok, text = preflight.summary(checks)
        head = QLabel(text)
        head.setWordWrap(True)
        head.setStyleSheet('font-weight: 600;')
        layout.addWidget(head)
        body = QPlainTextEdit()
        body.setReadOnly(True)
        lines = []
        for check in checks:
            mark = {'ok': 'OK  ', 'warn': 'WARN', 'error': 'FAIL', 'info': 'ok  '}[check.state]
            lines.append('%s  %-22s %s' % (mark, check.name, check.detail))
            if not check.ok and check.hint:
                lines.append('        fix: %s' % check.hint)
        body.setPlainText('\n'.join(lines))
        layout.addWidget(body, 1)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)


class LaunchPanel(QWidget):
    start_requested = Signal(object)          # StackConfig
    stop_requested = Signal()
    restart_requested = Signal(object)        # StackConfig
    live_toggled = Signal(str, bool)          # name, enabled
    open_gazebo_requested = Signal()
    rviz_toggled = Signal(bool)
    kill_leftovers_requested = Signal()
    presets_requested = Signal()
    save_preset_requested = Signal()
    preset_selected = Signal(str)

    def __init__(self, settings=None, parent=None):
        super().__init__(parent)
        self.settings = settings or {}
        self._running = False
        self._applied = StackConfig().to_dict()
        self._fields = {}
        self._launch_time = {}
        self._building = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(6)

        # --- presets ---------------------------------------------------------
        preset_row = QHBoxLayout()
        self.preset_combo = QComboBox()
        self.preset_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.preset_combo.activated.connect(
            lambda index: self.preset_selected.emit(self.preset_combo.itemText(index)))
        load_preset = QPushButton('Load preset')
        load_preset.clicked.connect(
            lambda: self.preset_selected.emit(self.preset_combo.currentText()))
        save_preset = QPushButton('Save as…')
        save_preset.clicked.connect(self.save_preset_requested.emit)
        preset_row.addWidget(QLabel('Preset'))
        preset_row.addWidget(self.preset_combo, 1)
        preset_row.addWidget(load_preset)
        preset_row.addWidget(save_preset)
        outer.addLayout(preset_row)
        self.set_presets(default_presets())

        # --- scrollable form -------------------------------------------------
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setMinimumHeight(70)
        form_host = QWidget()
        form_host.setMinimumWidth(330)
        form = QVBoxLayout(form_host)
        form.setContentsMargins(0, 0, 6, 0)
        form.setSpacing(3)
        # keep the natural height inside the scroll area instead of squeezing groups
        form.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)

        mode_box = QGroupBox('Mode, map and world')
        mode_layout = QFormLayout(mode_box)
        self.slam = QComboBox()
        self.slam.addItem('Localize on a saved map (slam:=False)', False)
        self.slam.addItem('Map while driving (slam:=True)', True)
        self._add(mode_layout, 'slam', 'Mode', self.slam, launch=True,
                  tip='Localising needs a map; coverage needs a finished map, so use '
                      'slam:=False for cleaning and slam:=True only for mapping.')
        map_row = QWidget()
        map_h = QHBoxLayout(map_row)
        map_h.setContentsMargins(0, 0, 0, 0)
        self.map = QComboBox()
        self.map.setEditable(True)
        self.map.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self._fill_maps()
        self.map.lineEdit().setPlaceholderText('default: %s' % paths.default_map())
        browse_map = QToolButton()
        browse_map.setText('…')
        browse_map.setToolTip('Pick a map yaml from the workspace maps/ folder')
        browse_map.clicked.connect(self._browse_map)
        map_h.addWidget(self.map, 1)
        map_h.addWidget(browse_map)
        self._add(mode_layout, 'map', 'Map', map_row, launch=True,
                  tip='Empty = <workspace>/maps/living_room.yaml')
        self.world = QComboBox()
        self.world.setEditable(True)
        self._fill_worlds()
        self._add(mode_layout, 'world', 'World', self.world, launch=True,
                  tip='A .world file in vacuum_gazebo/worlds')
        pose_row = QWidget()
        pose_h = QHBoxLayout(pose_row)
        pose_h.setContentsMargins(0, 0, 0, 0)
        self.x_pose = QDoubleSpinBox()
        self.x_pose.setRange(-50.0, 50.0)
        self.x_pose.setSingleStep(0.1)
        self.x_pose.setDecimals(2)
        self.y_pose = QDoubleSpinBox()
        self.y_pose.setRange(-50.0, 50.0)
        self.y_pose.setSingleStep(0.1)
        self.y_pose.setDecimals(2)
        pose_h.addWidget(QLabel('x'))
        pose_h.addWidget(self.x_pose)
        pose_h.addWidget(QLabel('y'))
        pose_h.addWidget(self.y_pose)
        mode_layout.addRow('Spawn pose', pose_row)
        for widget in (self.x_pose, self.y_pose):
            widget.valueChanged.connect(self._changed)
        self._launch_time['x_pose'] = self.x_pose
        self._launch_time['y_pose'] = self.y_pose
        self.odom_source = QComboBox()
        self.odom_source.addItem('Ground truth (slip-free)', 'ground_truth')
        self.odom_source.addItem('Wheel odometry (drifts)', 'robot_wheels')
        self._add(mode_layout, 'odom_source', 'Odometry', self.odom_source, launch=True,
                  tip='AMCL needs a pose that is roughly right; wheel odometry drifts in sim.')
        self.headless = QCheckBox('No Gazebo window (saves GPU, ~half the load)')
        self._add(mode_layout, 'headless', 'Headless', self.headless, launch=True,
                  tip='Draws the views here instead; Gazebo runs as a server only.')
        self.cameras = QCheckBox('Front floor camera in the URDF')
        self._add(mode_layout, 'cameras', 'Camera', self.cameras, launch=True,
                  tip='Needed by the detector and by dataset recording (they enable it '
                      'themselves). Costs ~10 %% GPU.')
        self.qt_platform = QComboBox()
        self.qt_platform.setEditable(True)
        for value, label in (('xcb', 'xcb (XWayland, safest on Wayland)'),
                             ('', 'leave unset'),
                             ('wayland', 'native Wayland (Gazebo may crash)')):
            self.qt_platform.addItem(label, value)
        self._add(mode_layout, 'qt_platform', 'Gazebo Qt platform', self.qt_platform,
                  launch=True, tip='Passed to the launch as qt_platform:=…')
        form.addWidget(mode_box)

        # --- capabilities ----------------------------------------------------
        cap_box = QGroupBox('Live toggles (no restart needed)')
        cap_layout = QVBoxLayout(cap_box)
        self.teleop = QCheckBox('Gamepad teleop (joy + teleop_twist_joy)')
        self.detector = QCheckBox('Dirt detector (needs the camera)')
        self.collect_dataset = QCheckBox('Record labelled dataset images')
        self.coverage = QCheckBox('Coverage mission + meter')
        self.meter = QCheckBox('Coverage meter')
        self.reclean = QCheckBox('Re-clean passes')
        reclean_row = QHBoxLayout()
        reclean_row.addWidget(self.reclean)
        self.reclean_source = QComboBox()
        self.reclean_source.addItem('spots from the map', 'map')
        self.reclean_source.addItem('spots from the camera (vision)', 'vision')
        reclean_row.addWidget(QLabel('source'))
        reclean_row.addWidget(self.reclean_source, 1)
        self.autostart = QCheckBox('Start cleaning as soon as Nav2 is ready')
        self.rviz = QCheckBox('RViz window')
        for widget, name in ((self.teleop, 'teleop'), (self.detector, 'detector'),
                             (self.collect_dataset, 'collect_dataset'),
                             (self.coverage, 'coverage'), (self.meter, 'meter'),
                             (self.reclean, 'reclean'), (self.reclean_source, 'reclean_source'),
                             (self.autostart, 'autostart'), (self.rviz, 'rviz')):
            self._fields[name] = widget
        cap_layout.addWidget(self.teleop)
        det_row = QHBoxLayout()
        det_row.addWidget(self.detector)
        self.detector_model = QLineEdit()
        self.detector_model.setPlaceholderText('model .onnx ("" = find it in the repo)')
        det_row.addWidget(self.detector_model, 1)
        cap_layout.addLayout(det_row)
        self._fields['detector_model'] = self.detector_model
        self.detector_model.editingFinished.connect(self._changed)
        cap_layout.addWidget(self.collect_dataset)
        ds_row = QHBoxLayout()
        self.dataset_dir = QLineEdit()
        self.dataset_dir.setPlaceholderText('~/vacuum_runs/dataset')
        self.dataset_max_s = QDoubleSpinBox()
        self.dataset_max_s.setRange(30.0, 36000.0)
        self.dataset_max_s.setSuffix(' s')
        self.dataset_max_s.setDecimals(0)
        ds_row.addWidget(QLabel('into'))
        ds_row.addWidget(self.dataset_dir, 1)
        ds_row.addWidget(QLabel('max'))
        ds_row.addWidget(self.dataset_max_s)
        cap_layout.addLayout(ds_row)
        self._fields['dataset_dir'] = self.dataset_dir
        self._fields['dataset_max_s'] = self.dataset_max_s
        for widget in (self.dataset_dir,):
            widget.editingFinished.connect(self._changed)
        self.dataset_max_s.valueChanged.connect(self._changed)
        cap_layout.addWidget(self.coverage)
        meter_row = QHBoxLayout()
        meter_row.addWidget(self.meter)
        meter_row.addStretch(1)
        cap_layout.addLayout(meter_row)
        cap_layout.addLayout(reclean_row)
        cap_layout.addWidget(self.autostart)
        rviz_row = QHBoxLayout()
        rviz_row.addWidget(self.rviz)
        self.gazebo_button = QPushButton('Open Gazebo GUI…')
        self.gazebo_button.setToolTip('Gazebo can only show its window if the stack was '
                                      'started with headless off')
        self.gazebo_button.clicked.connect(self.open_gazebo_requested.emit)
        rviz_row.addWidget(self.gazebo_button)
        cap_layout.addLayout(rviz_row)
        form.addWidget(cap_box)

        # --- dirt ------------------------------------------------------------
        dirt_box = QGroupBox('Dirt')
        dirt_layout = QFormLayout(dirt_box)
        count_row = QWidget()
        count_h = QHBoxLayout(count_row)
        count_h.setContentsMargins(0, 0, 0, 0)
        self.dirt = QSpinBox()
        self.dirt.setRange(0, 200)
        self.dirt.setToolTip('Number of patches; 0 = none')
        self.dirt_seed = QSpinBox()
        self.dirt_seed.setRange(0, 100000)
        self.dirt_seed.setToolTip('Same seed = same layout; use a new seed for a new run')
        reseed = QPushButton('New layout')
        reseed.setToolTip('Pick a new seed and restart the stack (the layout is generated '
                          'at launch time)')
        reseed.clicked.connect(self._new_dirt_layout)
        count_h.addWidget(QLabel('count'))
        count_h.addWidget(self.dirt)
        count_h.addWidget(QLabel('seed'))
        count_h.addWidget(self.dirt_seed)
        count_h.addWidget(reseed)
        count_h.addStretch(1)
        dirt_layout.addRow('Patches', count_row)
        for widget in (self.dirt, self.dirt_seed):
            widget.valueChanged.connect(self._changed)
        self._launch_time['dirt'] = self.dirt
        self._launch_time['dirt_seed'] = self.dirt_seed
        self.dirt_scale = QDoubleSpinBox()
        self.dirt_scale.setRange(0.3, 5.0)
        self.dirt_scale.setSingleStep(0.1)
        self.dirt_scale.setDecimals(2)
        self._add(dirt_layout, 'dirt_scale', 'Size multiplier', self.dirt_scale, launch=True)
        self.dirt_stubborn = QDoubleSpinBox()
        self.dirt_stubborn.setRange(0.0, 1.0)
        self.dirt_stubborn.setSingleStep(0.05)
        self.dirt_stubborn.setDecimals(2)
        self._add(dirt_layout, 'dirt_stubborn', 'Stubborn share', self.dirt_stubborn,
                  launch=True, tip='Share of patches that need several passes (spills '
                                   'always do)')
        self.dirt_brush = QDoubleSpinBox()
        self.dirt_brush.setRange(0.05, 1.0)
        self.dirt_brush.setSingleStep(0.01)
        self.dirt_brush.setDecimals(2)
        self.dirt_brush.setSuffix(' m')
        self._add(dirt_layout, 'dirt_brush', 'Brush radius', self.dirt_brush, launch=True)
        self.dirt_clean = QCheckBox('Brush removes dirt the robot drives over')
        self._add(dirt_layout, 'dirt_clean', 'Brush', self.dirt_clean, launch=True,
                  tip='Turn off while recording a dataset: the dirt has to stay visible')
        self.dirt_randomize = QCheckBox('Randomise floor colour and light for this seed')
        self._add(dirt_layout, 'dirt_randomize', 'Variation', self.dirt_randomize,
                  launch=True, tip='For dataset collection: stops a model learning one floor')
        self.dirt_dir = QLineEdit()
        self.dirt_dir.setPlaceholderText('~/vacuum_runs/dirt')
        self._add(dirt_layout, 'dirt_dir', 'Layout folder', self.dirt_dir, launch=True)
        form.addWidget(dirt_box)

        # --- misc ------------------------------------------------------------
        misc_box = QGroupBox('Startup')
        misc_layout = QFormLayout(misc_box)
        self.nav_delay = QDoubleSpinBox()
        self.nav_delay.setRange(0.0, 120.0)
        self.nav_delay.setSingleStep(1.0)
        self.nav_delay.setSuffix(' s')
        self._add(misc_layout, 'nav_delay', 'Nav2 delay', self.nav_delay, launch=True,
                  tip='Seconds Gazebo gets before Nav2 starts (needs /clock, /scan, TF)')
        form.addWidget(misc_box)
        form.addStretch(1)
        scroll.setWidget(form_host)
        outer.addWidget(scroll, 1)

        # --- command preview -------------------------------------------------
        preview_box = QGroupBox('What will run')
        preview_layout = QVBoxLayout(preview_box)
        self.preview = QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setMaximumHeight(110)
        self.preview.setMinimumHeight(54)
        self.preview.setFont(self.font())
        self.command_text = ''
        copy_button = QPushButton('Copy command')
        copy_button.clicked.connect(self._copy_command)
        preview_header = QHBoxLayout()
        preview_header.addStretch(1)
        preview_header.addWidget(copy_button)
        preview_layout.addWidget(self.preview)
        preview_layout.addLayout(preview_header)
        outer.addWidget(preview_box)

        # --- status + buttons -------------------------------------------------
        self.badge = QLabel('not running')
        self.badge.setObjectName('badgeOk')
        self.state_dot = Dot()
        self.leftovers = QLabel('')
        self.leftovers.setObjectName('hint')
        kill = QPushButton('Kill leftovers')
        kill.setToolTip('SIGTERM (then SIGKILL) any gz sim / ruby / rviz2 that outlived '
                        'a previous run')
        kill.clicked.connect(self.kill_leftovers_requested.emit)
        restart = QPushButton('Apply & restart')
        restart.clicked.connect(self._apply_and_restart)
        self.restart_button = restart
        restart.setEnabled(False)
        self.start_button = QPushButton('Start stack')
        self.start_button.setObjectName('primary')
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton('Stop stack')
        self.stop_button.setObjectName('danger')
        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.stop_button.setEnabled(False)
        preflight_button = QPushButton('Pre-flight…')
        preflight_button.clicked.connect(self.show_preflight)

        status_row = QHBoxLayout()
        status_row.addWidget(self.state_dot)
        status_row.addWidget(self.badge, 1)
        status_row.addWidget(self.leftovers)
        status_row.addWidget(kill)
        outer.addLayout(status_row)
        button_row = QHBoxLayout()
        button_row.addWidget(preflight_button)
        button_row.addStretch(1)
        button_row.addWidget(restart)
        button_row.addWidget(self.stop_button)
        button_row.addWidget(self.start_button)
        outer.addLayout(button_row)

        self._connect_live()
        self.refresh_leftovers()

    # --------------------------------------------------------------- building --
    def _add(self, layout, name, label, widget, launch=False, tip=''):
        if tip:
            widget.setToolTip(tip)
        layout.addRow(label, widget)
        self._fields[name] = widget
        if launch:
            self._launch_time[name] = widget
        signal = None
        if isinstance(widget, QCheckBox):
            signal = widget.toggled
        elif isinstance(widget, QComboBox):
            signal = widget.currentIndexChanged
        elif isinstance(widget, QDoubleSpinBox):
            signal = widget.valueChanged
        elif isinstance(widget, QSpinBox):
            signal = widget.valueChanged
        elif isinstance(widget, QLineEdit):
            signal = widget.editingFinished
        if signal is not None:
            signal.connect(self._changed)

    def _fill_maps(self):
        self.map.clear()
        self.map.addItem('')
        folder = paths.maps_dir()
        try:
            for path in sorted(folder.glob('*.yaml')):
                self.map.addItem(str(path))
        except OSError:
            pass
        self.map.setCurrentIndex(0)

    def _fill_worlds(self):
        self.world.clear()
        try:
            for path in sorted((Path(paths.gazebo_share()) / 'worlds').glob('*.world')):
                self.world.addItem(path.name)
        except OSError:
            pass
        if self.world.count() == 0:
            self.world.addItem('living_room.world')

    def _browse_map(self):
        start = str(paths.maps_dir())
        path, _ = QFileDialog.getOpenFileName(self, 'Pick a map', start,
                                              'Map yaml (*.yaml *.yml)')
        if path:
            self.map.setEditText(path)
            self._changed()

    def set_presets(self, presets):
        current = self.preset_combo.currentText()
        self.preset_combo.clear()
        self.preset_combo.addItems(list(presets))
        if current in presets:
            self.preset_combo.setCurrentText(current)
        self.preset_combo.setToolTip('\n'.join(
            '%s: %s' % (name, data.get('notes', '')) for name, data in presets.items()))

    # ------------------------------------------------------------------ values --
    def _get(self, name, default=None):
        widget = self._fields.get(name)
        if widget is None:
            return default
        if isinstance(widget, QComboBox):
            data = widget.currentData()
            if data is None or (isinstance(data, str) and data == ''):
                text = widget.currentText().strip()
                return text if text else default
            return data
        if isinstance(widget, QCheckBox):
            return widget.isChecked()
        if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
            return widget.value()
        if isinstance(widget, QLineEdit):
            return widget.text().strip()
        return default

    def _set(self, name, value):
        widget = self._fields.get(name)
        if widget is None:
            return
        widget.blockSignals(True)
        try:
            if isinstance(widget, QComboBox):
                index = widget.findData(value)
                if index >= 0:
                    widget.setCurrentIndex(index)
                elif isinstance(value, str):
                    widget.setEditText(value)
            elif isinstance(widget, QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                widget.setValue(float(value) if isinstance(widget, QDoubleSpinBox)
                                else int(value))
            elif isinstance(widget, QLineEdit):
                widget.setText(str(value or ''))
        finally:
            widget.blockSignals(False)

    def config(self):
        map_value = self.map.currentText().strip()
        return StackConfig(
            slam=bool(self._get('slam', False)),
            map=map_value,
            world=self.world.currentText().strip() or 'living_room.world',
            x_pose=float(self.x_pose.value()),
            y_pose=float(self.y_pose.value()),
            odom_source=str(self._get('odom_source', 'ground_truth') or 'ground_truth'),
            headless=self.headless.isChecked(),
            cameras=self.cameras.isChecked(),
            dirt=int(self.dirt.value()),
            dirt_seed=int(self.dirt_seed.value()),
            dirt_scale=float(self.dirt_scale.value()),
            dirt_stubborn=float(self.dirt_stubborn.value()),
            dirt_brush=float(self.dirt_brush.value()),
            dirt_clean=self.dirt_clean.isChecked(),
            dirt_randomize=self.dirt_randomize.isChecked(),
            dirt_dir=self.dirt_dir.text().strip() or '~/vacuum_runs/dirt',
            nav_delay=float(self.nav_delay.value()),
            qt_platform=str(self.qt_platform.currentData() or ''),
            teleop=self.teleop.isChecked(),
            rviz=self.rviz.isChecked(),
            detector=self.detector.isChecked(),
            detector_model=self.detector_model.text().strip(),
            coverage=self.coverage.isChecked(),
            meter=self.meter.isChecked(),
            reclean=self.reclean.isChecked(),
            reclean_source=self.reclean_source.currentData(),
            autostart=self.autostart.isChecked(),
            collect_dataset=self.collect_dataset.isChecked(),
            dataset_dir=self.dataset_dir.text().strip() or '~/vacuum_runs/dataset',
            dataset_max_s=float(self.dataset_max_s.value()),
        )

    def set_config(self, cfg):
        self._building = True
        try:
            for name, value in cfg.to_dict().items():
                if name == 'map':
                    continue
                self._set(name, value)
            self.map.blockSignals(True)
            if cfg.map:
                index = self.map.findText(cfg.map)
                if index >= 0:
                    self.map.setCurrentIndex(index)
                else:
                    self.map.setEditText(cfg.map)
            else:
                self.map.setCurrentIndex(0)
            self.map.blockSignals(False)
        finally:
            self._building = False
        self._changed()

    def apply_view(self, view):
        self.autostart.setChecked(bool(view.get('autostart', self.autostart.isChecked())))

    # ------------------------------------------------------------------ state --
    def _connect_live(self):
        live = {'teleop': self.teleop, 'detector': self.detector,
                'coverage': self.coverage, 'meter': self.meter, 'rviz': self.rviz,
                'collect_dataset': self.collect_dataset}
        for name, widget in live.items():
            widget.toggled.connect(lambda checked, key=name: self._live(key, checked))
        self.reclean.toggled.connect(lambda _: self._live('reclean', self.reclean.isChecked()))
        self.reclean_source.currentIndexChanged.connect(
            lambda _: self._live('reclean_source', self.reclean_source.currentData()))

    def _live(self, name, value):
        if self._building:
            return
        if name == 'rviz':
            self.rviz_toggled.emit(bool(value))
        if self._running:
            self.live_toggled.emit(name, value)
        self._changed()

    def _changed(self, *_args):
        if self._building:
            return
        self._update_preview()
        if self._running:
            needed = self.restart_fields()
            if needed:
                self.badge.setText('restart needed: ' + ', '.join(needed[:4])
                                   + ('…' if len(needed) > 4 else ''))
                self.badge.setObjectName('badge')
                self.restart_button.setEnabled(True)
            else:
                self.badge.setText('running')
                self.badge.setObjectName('badgeOk')
                self.restart_button.setEnabled(False)
            self.badge.style().unpolish(self.badge)
            self.badge.style().polish(self.badge)
        if self.detector.isChecked() and not self.cameras.isChecked():
            self.cameras.setToolTip('The detector switches the camera on by itself.')
        self._update_detector_hint()

    def _update_detector_hint(self):
        enabled = self._running and not self.cameras.isChecked() and self.detector.isChecked()
        self.detector.setToolTip(
            'Camera recording is off in this launch: the detector needs enable_cameras. '
            'Tick "Front floor camera" and restart.' if enabled else
            'Runs vacuum_vision/detector.launch.py as its own process')

    def restart_fields(self):
        current = self.config().to_dict()
        return [FIELD_LABELS.get(name, name) for name in LAUNCH_TIME_FIELDS
                if current.get(name) != self._applied.get(name)]

    def set_running(self, running, applied=None):
        self._running = bool(running)
        if applied is not None:
            self._applied = applied.to_dict() if isinstance(applied, StackConfig) else dict(applied)
        for name, widget in self._launch_time.items():
            widget.setEnabled(not running)
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.state_dot.set_state('ok' if running else 'unknown')
        if not running:
            self.badge.setText('not running')
            self.badge.setObjectName('badgeOk')
            self.restart_button.setEnabled(False)
        else:
            self.badge.setText('running')
            self.badge.setObjectName('badgeOk')
            self.restart_button.setEnabled(bool(self.restart_fields()))
        self.badge.style().unpolish(self.badge)
        self.badge.style().polish(self.badge)
        self._update_preview()

    def _start(self):
        self.start_requested.emit(self.config())

    def _apply_and_restart(self):
        self.restart_requested.emit(self.config())

    def _new_dirt_layout(self):
        self.dirt_seed.setValue((self.dirt_seed.value() + 1) % 100000)
        if self._running:
            answer = QMessageBox.question(
                self, 'New dirt layout',
                'The dirt layout is generated when Gazebo starts, so a new seed needs a '
                'restart. Restart the stack now?')
            if answer == QMessageBox.StandardButton.Yes:
                self.restart_requested.emit(self.config())

    # --------------------------------------------------------------- leftovers --
    def refresh_leftovers(self, exclude=(), active=False):
        from .launch_manager import find_leftovers
        if active:
            self.leftovers.setText('stack running')
            self.leftovers.setStyleSheet('')
            return
        left = find_leftovers(exclude_pids=exclude)
        if left:
            self.leftovers.setText('%d leftover process(es)' % len(left))
            self.leftovers.setStyleSheet('color: #e8a33d;')
        else:
            self.leftovers.setText('no leftovers')
            self.leftovers.setStyleSheet('')

    # ---------------------------------------------------------------- preview --
    def _update_preview(self):
        cfg = self.config()
        lines = ['# 1. simulation + Nav2 (fixed until restart)',
                 shell_join(cfg.stack_command(self._default_map()))]
        live = []
        if cfg.teleop:
            live.append('# gamepad: ' + shell_join(cfg.teleop_command()))
        if cfg.detector:
            live.append('# detector: ' + shell_join(cfg.detector_command()))
        if cfg.collect_dataset:
            live.append('# recording: ' + shell_join(cfg.dataset_command()))
        if cfg.coverage:
            live.append('# mission: ' + shell_join(cfg.coverage_command(
                self._default_map(), autostart=cfg.autostart)))
        if cfg.rviz:
            live.append('# RViz: ' + shell_join(cfg.rviz_command()))
        text = '\n'.join(lines + live)
        self.command_text = text
        if self.preview.toPlainText() != text:
            self.preview.setPlainText(text)

    def _default_map(self):
        return str(paths.default_map())

    def _copy_command(self):
        from PySide6.QtWidgets import QApplication
        QApplication.clipboard().setText(self.command_text)
        self.preview.setToolTip('copied to the clipboard')

    def show_preflight(self):
        checks = preflight.run_checks(self.config())
        PreflightDialog(checks, self).exec()

    def set_status_text(self, text, state='unknown'):
        if self._running:
            self.badge.setText(text)
        self.state_dot.set_state(state)
