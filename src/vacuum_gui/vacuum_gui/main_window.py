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

"""The main window: docks, palette, stack orchestration, shortcuts.

The window owns the process manager and the backend; the panels only emit intent. That
keeps "what is running" in one place, which is what makes the live toggles and the clean
shutdown trustworthy.
"""

import base64
import math
import os
import threading
import time
from pathlib import Path

from PySide6.QtCore import QByteArray, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QKeySequence, QShortcut
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDockWidget, QFileDialog, QHBoxLayout, QInputDialog,
                               QScrollArea,
                               QLabel, QMainWindow, QMessageBox, QPushButton,
                               QSizePolicy, QSplitter, QTabWidget, QTextBrowser, QToolBar,
                               QVBoxLayout, QWidget)

from . import feedback, paths, preflight, settings, theme
from .background import Background
from .backend import NullBackend
from .camera_view import CameraPanel
from .commands import StackConfig, shell_join
from .dashboard_bar import DashboardBar
from .dataset_panel import DatasetPanel
from .dirt_inspector import DirtInspector
from .experiments_panel import ExperimentsPanel
from .health_panel import HealthPanel
from .launch_manager import BLOCKING_KINDS, ProcessManager, find_leftovers, kill_leftovers
from .launch_panel import LaunchPanel, PreflightDialog
from .log_console import LogConsole
from .map_view import MapPanel
from .mission_panel import MissionPanel
from .reports_panel import ReportsPanel
from .teleop_panel import TeleopPanel
from .toast import ToastManager
from .tools_panel import ToolsPanel
from .ui_common import hint

LAYERS = (
    ('scan', 'LiDAR /scan'),
    ('plan_global', 'Global plan'),
    ('plan_local', 'Local plan'),
    ('coverage_plan', 'Coverage plan'),
    ('cleaned', 'Cleaned area'),
    ('dirt_detected', 'Detected dirt (camera)'),
    ('dirt_truth', 'Ground-truth dirt'),
    ('particles', 'AMCL particles'),
    ('costmaps', 'Costmaps'),
    ('trail', 'Robot trail'),
)


class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('RoboVAC control panel - help')
        self.resize(720, 620)
        layout = QVBoxLayout(self)
        browser = QTextBrowser()
        browser.setHtml("""
        <h2>What this window does</h2>
        <p>It starts the simulation and draws everything the robot knows from ROS topics,
        so Gazebo can run without a window and RViz is optional.</p>
        <h3>Panels</h3>
        <ul>
        <li><b>Stack</b> (left): every launch option, the exact command line, Start/Stop,
        pre-flight checks, leftovers. Options marked <i>restart</i> are fixed at launch;
        the live ones are separate processes and can be toggled any time.</li>
        <li><b>Map</b> (centre): occupancy grid, cleaned area, plans, LiDAR, detected and
        ground-truth dirt, particles, costmaps. Click mode chooses what a click means:
        Nav2 goal, 2D pose estimate, or inspect a dirt spot.</li>
        <li><b>Camera</b>: /camera/image and /dirt/debug_image with real frame rates.</li>
        <li><b>Health</b>: a light per node and per key topic; click a red one for the fix.</li>
        <li><b>Mission</b>: plan, start, stop, reset the meter, save/load maps, the piece list.</li>
        <li><b>Robot</b>: on-screen drive pad, speed limits, gamepad state, E-STOP.</li>
        <li><b>Dataset</b>: record labelled frames, count them, build the Colab zip.</li>
        <li><b>Runs</b>: browse run_*/summary.json, compare two runs, coverage-vs-time.</li>
        <li><b>Experiments</b>: seeds x {no re-clean, map re-clean, vision re-clean}, headless,
        one row per run, CSV.</li>
        <li><b>Tools</b>: live detector parameters, screenshots/GIF, rosbag.</li>
        <li><b>Log</b>: one tab per process, level colours, filter, save.</li>
        </ul>
        <h3>Shortcuts</h3>
        <table border="0" cellspacing="6">
        <tr><td><b>F5</b></td><td>start / stop the stack</td></tr>
        <tr><td><b>Space</b></td><td>emergency stop (press again to release)</td></tr>
        <tr><td><b>Ctrl+P</b></td><td>plan the coverage route</td></tr>
        <tr><td><b>Ctrl+Return</b></td><td>start cleaning</td></tr>
        <tr><td><b>Ctrl+.</b></td><td>stop cleaning / cancel the Nav2 goal</td></tr>
        <tr><td><b>Ctrl+M</b></td><td>fit the map</td></tr>
        <tr><td><b>Ctrl+Shift+K</b></td><td>kill leftover gz / rviz processes</td></tr>
        <tr><td><b>F1</b></td><td>this help</td></tr>
        </table>
        <h3>Where things are written</h3>
        <p>Settings, presets and the window layout live in <code>~/.config/vacuum_gui/</code>.
        Runs, dirt truth, datasets, bags and screenshots live under
        <code>~/vacuum_runs/</code>. Nothing is written into the repository.</p>
        <h3>If something looks dead</h3>
        <p>Check the <b>Health</b> panel first: it distinguishes "node not running" from
        "node running but no data" and from "TF chain broken". The Log tab per process has
        the traceback.</p>
        """)
        layout.addWidget(browser, 1)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)


class MainWindow(QMainWindow):
    def __init__(self, app, settings_data, backend, parent=None):
        super().__init__(parent)
        self.app = app
        self.settings = settings_data
        self.cfg = StackConfig.from_dict(settings_data.get('stack', {}))
        self.view_settings = settings_data.get('view', {})
        self.backend = backend
        self.manager = ProcessManager()
        self._log_index = 0
        self._stack_running = False
        self._stopping = False
        self._estop = False
        self._gif_frames = []
        self._gif_timer = None
        self._gif_target = 'map'
        self._gif_path = None
        self._last_mission_text = ''
        self._map_click_mode = self.view_settings.get('map_mode', 'goal')
        self._launched_camera = False       # was the camera in the URDF of this launch?
        self._starting = False              # Start pressed, stack not ready yet
        self._start_toast = None
        self._start_t0 = 0.0
        self._last_dash = None
        self._stop_toast = None
        self._close_after_stop = False
        self._pending = {}                  # mission service -> (toast, button, watchdog)

        self.setWindowTitle('RoboVAC control panel')
        self.setObjectName('vacuum_gui')
        self.resize(1500, 950)
        self.bg = Background(self)          # blocking work off the GUI thread
        self.toasts = ToastManager(self)    # visible answer to every action
        self._startup_timer = QTimer(self)
        self._startup_timer.setInterval(1000)
        self._startup_timer.timeout.connect(self._startup_tick)

        self._build_center()
        self._build_docks()
        self._build_toolbar()
        self._build_statusbar()
        self._connect_panels()
        self._connect_backend()
        self._build_shortcuts()

        self._log_timer = QTimer(self)
        self._log_timer.setInterval(200)
        self._log_timer.timeout.connect(self._drain_logs)
        self._log_timer.start()

        self._estop_timer = QTimer(self)
        self._estop_timer.setInterval(100)
        self._estop_timer.timeout.connect(self._estop_tick)

        for key, layer in (('scan', 'scan'), ('cleaned', 'cleaned'),
                           ('plan_global', 'plan_global'), ('plan_local', 'plan_local'),
                           ('coverage_plan', 'coverage_plan'),
                           ('dirt_detected', 'dirt_detected'),
                           ('dirt_truth', 'dirt_truth'), ('particles', 'particles'),
                           ('costmaps', 'costmaps'), ('trail', 'trail')):
            if key in self.layer_checks:
                self.map_panel.view.set_layer_visible(layer,
                                                      self.layer_checks[key].isChecked())
        self._restore_layout()
        self._prime_dock_sizes()
        self.launch_panel.set_config(self.cfg)
        self.teleop_panel.set_speeds(self.view_settings.get('teleop_linear', 0.25),
                                     self.view_settings.get('teleop_angular', 1.0))
        self.health_timer = QTimer(self)
        self.health_timer.setInterval(5000)
        self.health_timer.timeout.connect(self._refresh_leftovers)
        self.health_timer.start()

        self.backend.set_image_rate(self.view_settings.get('camera_fps', 10))
        if not self.backend.available():
            self._backend_unavailable()
        else:
            self.backend.start()
        if not self.settings.get('ui', {}).get('help_seen'):
            QTimer.singleShot(400, self.show_help)

    def showEvent(self, event):      # noqa: N802
        super().showEvent(event)
        if not getattr(self, '_sized', False):
            self._sized = True
            QTimer.singleShot(0, self._fit_map_once)

    def _fit_map_once(self):
        if self.map_panel.view.grid is not None:
            self.map_panel.view.fit()

    def _prime_dock_sizes(self):
        """Set the first-run proportions before the window is shown.

        Qt ignores resizeDocks() for a dock that has never been floated, so the log and
        the health panel are briefly floated and sized while the window is still hidden;
        the sizes stick when they dock again.
        """
        if settings.load_layout():
            return
        for dock, height in ((self.dock_log, 260), (self.dock_health, 360)):
            dock.setFloating(True)
            dock.resize(900, height)
            dock.setFloating(False)
        self.dock_log.raise_()
        self.dock_camera.raise_()

    # ------------------------------------------------------------------ UI ----
    def _build_center(self):
        self.map_panel = MapPanel(self)
        layers = QWidget()
        layers_layout = QVBoxLayout(layers)
        layers_layout.setContentsMargins(8, 8, 8, 8)
        layers_layout.setSpacing(2)
        title = QLabel('LAYERS')
        title.setObjectName('section')
        layers_layout.addWidget(title)
        self.layer_checks = {}
        for key, label in LAYERS:
            check = QCheckBox(label)
            check.setChecked(bool(self.view_settings.get(
                {'cleaned': 'show_cleaned', 'scan': 'show_scan',
                 'plan_global': 'show_plan_global', 'plan_local': 'show_plan_local',
                 'coverage_plan': 'show_plan_coverage',
                 'dirt_detected': 'show_dirt_detected', 'dirt_truth': 'show_dirt_truth',
                 'particles': 'show_particles', 'costmaps': 'show_costmaps'}.get(key, ''),
                True)))
            check.toggled.connect(
                lambda checked, layer=key: self.map_panel.view.set_layer_visible(layer, checked))
            check.toggled.connect(self._save_view_settings)
            self.layer_checks[key] = check
            layers_layout.addWidget(check)
        layers_layout.addSpacing(8)
        layers_layout.addWidget(QLabel('CLICK MEANS'))
        self.click_mode = QComboBox()
        self.click_mode.addItem('Nav2 goal', 'goal')
        self.click_mode.addItem('2D pose estimate', 'pose')
        self.click_mode.addItem('Inspect dirt', 'inspect')
        index = self.click_mode.findData(self._map_click_mode)
        self.click_mode.setCurrentIndex(max(0, index))
        self.click_mode.currentIndexChanged.connect(self._click_mode_changed)
        layers_layout.addWidget(self.click_mode)
        layers_layout.addWidget(hint('Drag to pan, wheel to zoom. Ctrl+M fits the map.'))
        layers_layout.addStretch(1)

        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        splitter.addWidget(layers)
        splitter.addWidget(self.map_panel)
        splitter.setSizes([170, 900])
        splitter.setChildrenCollapsible(False)
        self.setCentralWidget(splitter)

        view = self.map_panel.view
        view.pose_requested.connect(self._pose_clicked)
        view.goal_requested.connect(self._goal_clicked)
        view.inspect_requested.connect(self._inspect_clicked)

    def _dock(self, widget, title, area, name, scroll=True):
        dock = QDockWidget(title, self)
        dock.setObjectName(name)
        if scroll:
            # A dock's minimum height is the sum of its content; the tall forms (tools,
            # experiments, dataset) would otherwise force the whole window taller than the
            # screen. A scroll area keeps the minimum at a few rows.
            area_widget = QScrollArea()
            area_widget.setWidgetResizable(True)
            area_widget.setFrameShape(QScrollArea.Shape.NoFrame)
            area_widget.setWidget(widget)
            area_widget.setMinimumHeight(90)
            dock.setWidget(area_widget)
        else:
            dock.setWidget(widget)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable |
                         QDockWidget.DockWidgetFeature.DockWidgetFloatable |
                         QDockWidget.DockWidgetFeature.DockWidgetClosable)
        self.addDockWidget(area, dock)
        return dock

    def _build_docks(self):
        left = Qt.DockWidgetArea.LeftDockWidgetArea
        right = Qt.DockWidgetArea.RightDockWidgetArea
        bottom = Qt.DockWidgetArea.BottomDockWidgetArea

        self.launch_panel = LaunchPanel(self.settings, self)
        self.camera_panel = CameraPanel(self)
        self.health_panel = HealthPanel(self)
        self.mission_panel = MissionPanel(self)
        self.teleop_panel = TeleopPanel(self.view_settings.get('teleop_linear', 0.25),
                                        self.view_settings.get('teleop_angular', 1.0), self)
        self.dataset_panel = DatasetPanel(self)
        self.reports_panel = ReportsPanel(self)
        self.experiments_panel = ExperimentsPanel(self.cfg, self)
        self.tools_panel = ToolsPanel(self.view_settings, self)
        self.log_console = LogConsole(self)

        self.dock_launch = self._dock(self.launch_panel, 'Stack', left, 'dock_launch',
                                      scroll=False)
        self.dock_camera = self._dock(self.camera_panel, 'Camera', right, 'dock_camera',
                                      scroll=False)
        self.dock_health = self._dock(self.health_panel, 'Health', right, 'dock_health')
        self.dock_mission = self._dock(self.mission_panel, 'Mission', right, 'dock_mission')
        self.dock_teleop = self._dock(self.teleop_panel, 'Robot', right, 'dock_teleop')
        self.dock_dataset = self._dock(self.dataset_panel, 'Dataset', right, 'dock_dataset')
        self.tabifyDockWidget(self.dock_camera, self.dock_mission)
        self.tabifyDockWidget(self.dock_camera, self.dock_teleop)
        self.tabifyDockWidget(self.dock_camera, self.dock_dataset)
        self.dock_camera.raise_()

        self.dock_log = self._dock(self.log_console, 'Log', bottom, 'dock_log', scroll=False)
        self.dock_reports = self._dock(self.reports_panel, 'Runs', bottom, 'dock_reports')
        self.dock_experiments = self._dock(self.experiments_panel, 'Experiments', bottom,
                                           'dock_experiments')
        self.dock_tools = self._dock(self.tools_panel, 'Tools', bottom, 'dock_tools')
        self.tabifyDockWidget(self.dock_log, self.dock_reports)
        self.tabifyDockWidget(self.dock_log, self.dock_experiments)
        self.tabifyDockWidget(self.dock_log, self.dock_tools)
        self.dock_log.raise_()
        self.setTabPosition(Qt.DockWidgetArea.RightDockWidgetArea,
                            QTabWidget.TabPosition.North)
        self.setTabPosition(Qt.DockWidgetArea.BottomDockWidgetArea,
                            QTabWidget.TabPosition.North)

        self.resizeDocks([self.dock_launch], [420], Qt.Orientation.Horizontal)
        self.resizeDocks([self.dock_camera], [420], Qt.Orientation.Horizontal)
        self.resizeDocks([self.dock_log], [260], Qt.Orientation.Vertical)

    def _build_toolbar(self):
        bar = QToolBar('Main', self)
        bar.setObjectName('toolbar')
        bar.setMovable(False)
        self.addToolBar(bar)

        self.action_start = QAction('Start stack', self)
        self.action_start.triggered.connect(
            lambda: self.start_stack(self.launch_panel.config()))
        bar.addAction(self.action_start)

        self.action_stop = QAction('Stop stack', self)
        self.action_stop.triggered.connect(self.stop_stack)
        self.action_stop.setEnabled(False)
        bar.addAction(self.action_stop)

        self.action_estop = QAction('E-STOP', self)
        self.action_estop.setCheckable(True)
        self.action_estop.triggered.connect(self.set_estop)
        self.estop_button = QPushButton('E-STOP')
        self.estop_button.setObjectName('danger')
        self.estop_button.setCheckable(True)
        self.estop_button.setMinimumWidth(96)
        self.estop_button.clicked.connect(self.set_estop)
        bar.addWidget(self.estop_button)
        bar.addSeparator()
        self.action_plan = QAction('Plan', self)
        self.action_plan.triggered.connect(lambda: self.run_trigger('plan'))
        self.action_clean = QAction('Start cleaning', self)
        self.action_clean.triggered.connect(lambda: self.run_trigger('start'))
        self.action_stop_mission = QAction('Stop cleaning', self)
        self.action_stop_mission.triggered.connect(self.stop_mission)
        for action in (self.action_plan, self.action_clean, self.action_stop_mission):
            action.setEnabled(False)
            bar.addAction(action)
        bar.addSeparator()

        self.action_shot = QAction('Screenshot map', self)
        self.action_shot.triggered.connect(lambda: self.take_screenshot('map'))
        bar.addAction(self.action_shot)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(spacer)
        bar.addWidget(QLabel('Theme '))
        self.theme_combo = QComboBox()
        self.theme_combo.setToolTip('Colour theme - applies immediately and is remembered')
        for key, label in theme.theme_choices():
            self.theme_combo.addItem(label, key)
        stored = theme.normalize(self.settings.get('ui', {}).get('theme'))
        self.theme_combo.setCurrentIndex(max(0, self.theme_combo.findData(stored)))
        self.theme_combo.activated.connect(self._theme_chosen)
        bar.addWidget(self.theme_combo)

        help_button = QPushButton('Help')
        help_button.setFlat(True)
        help_button.clicked.connect(self.show_help)
        bar.addWidget(help_button)

    def _build_statusbar(self):
        self.dashboard = DashboardBar(self)
        self.statusBar().addPermanentWidget(self.dashboard, 1)
        self.status_connection = QLabel('ROS: starting…')
        self.statusBar().addWidget(self.status_connection, 0)
        self.status_message = QLabel('')
        self.statusBar().addWidget(self.status_message, 1)

    # -------------------------------------------------------------- signals ----
    def _connect_panels(self):
        p = self.launch_panel
        p.start_requested.connect(self.start_stack)
        p.stop_requested.connect(self.stop_stack)
        p.restart_requested.connect(self.restart_stack)
        p.live_toggled.connect(self.live_toggle)
        p.rviz_toggled.connect(self.rviz_toggle)
        p.kill_leftovers_requested.connect(self.do_kill_leftovers)
        p.preset_selected.connect(self.load_preset)
        p.save_preset_requested.connect(self.save_preset)
        p.open_gazebo_requested.connect(self.open_gazebo_gui)

        m = self.mission_panel
        m.plan_requested.connect(lambda: self.run_trigger('plan'))
        m.start_requested.connect(lambda: self.run_trigger('start'))
        m.stop_requested.connect(self.stop_mission)
        m.reset_meter_requested.connect(lambda: self.run_trigger('reset_meter'))
        m.save_map_requested.connect(self.save_map)
        m.load_map_requested.connect(self._load_map_path)
        m.reclean_toggled.connect(self._reclean_toggled)

        t = self.teleop_panel
        t.cmd_vel.connect(self.backend.publish_cmd_vel)
        t.estop_toggled.connect(self.set_estop)

        d = self.dataset_panel
        d.start_recording_requested.connect(self.start_recording)
        d.stop_recording_requested.connect(self.stop_recording)
        d.package_requested.connect(self.package_dataset)

        tools = self.tools_panel
        tools.params_apply_requested.connect(self.backend.set_detector_params)
        tools.params_reset_requested.connect(
            lambda: self.backend.set_detector_params(tools.defaults()))
        tools.screenshot_requested.connect(self.take_screenshot)
        tools.gif_requested.connect(self.record_gif)
        tools.rosbag_toggle_requested.connect(self.rosbag_toggle)

    def _connect_backend(self):
        b = self.backend
        b.connected.connect(self._backend_connected)
        b.map_received.connect(self._map_received)
        b.coverage_received.connect(self.map_panel.view.set_coverage)
        b.costmap_received.connect(self.map_panel.view.set_costmap)
        b.image_received.connect(self._image_received)
        b.pose_received.connect(self.map_panel.view.set_pose)
        b.scan_received.connect(self.map_panel.view.set_scan)
        b.path_received.connect(self.map_panel.view.set_path)
        b.markers_received.connect(self._markers_received)
        b.particles_received.connect(self.map_panel.view.set_particles)
        b.detections_received.connect(self.map_panel.view.set_detections)
        b.joy_received.connect(self._joy_received)
        b.dashboard_received.connect(self._dashboard_received)
        b.health_received.connect(self.health_panel.update_health)
        b.nav2_received.connect(self._nav2_received)
        b.mission_status_received.connect(self.mission_panel.set_status)
        b.detector_params_received.connect(self.tools_panel.set_values)
        b.notice.connect(self._notice)
        b.trigger_done.connect(self._trigger_done)

    def _build_shortcuts(self):
        def shortcut(sequence, slot, app_wide=False):
            sc = QShortcut(QKeySequence(sequence), self)
            sc.setContext(Qt.ShortcutContext.ApplicationShortcut if app_wide
                          else Qt.ShortcutContext.WindowShortcut)
            sc.activated.connect(slot)
            return sc

        shortcut('F5', self._toggle_stack)
        shortcut('Ctrl+P', lambda: self.run_trigger('plan'))
        shortcut('Ctrl+Return', lambda: self.run_trigger('start'))
        shortcut('Ctrl+.', self.stop_mission)
        shortcut('Ctrl+M', self.map_panel.view.fit)
        shortcut('Ctrl+Shift+K', self.do_kill_leftovers)
        shortcut('F1', self.show_help)
        shortcut('Ctrl+Shift+S', lambda: self.take_screenshot('map'))

    # --------------------------------------------------------------- actions ---
    def start_stack(self, cfg):
        if self._stack_running or self._starting:
            return
        # Instant feedback: the button turns into a spinner before any check or dialog.
        self.launch_panel.set_starting('Checking')
        started = False
        try:
            started = self._start_stack(cfg)
        finally:
            if not started:
                self.launch_panel.clear_starting()

    def _start_stack(self, cfg):
        """Pre-flight, then spawn ``ros2 launch``. True when the launch process started."""
        checks = preflight.run_checks(cfg)
        if preflight.blocking(checks):
            dialog = PreflightDialog(checks, self,
                                     recheck=lambda: preflight.run_checks(cfg),
                                     killer=self._kill_stale_now)
            dialog.exec()
            checks = preflight.run_checks(cfg)      # the dialog can have cleaned up
            blocked = preflight.blocking(checks)
            if blocked:
                self._notice('error', 'start blocked by %d pre-flight check(s): %s'
                             % (len(blocked), ', '.join(c.name for c in blocked)))
                return False
        # rows that only deserve a line (no gamepad paired, ...) never open a dialog
        for check in checks:
            if not check.ok and not check.modal:
                self.toasts.show('%s: %s' % (check.name, check.detail), 'warn')
        warnings = [c for c in checks if not c.ok and c.modal]
        if warnings:
            if self.settings.get('ui', {}).get('preflight_warnings', True):
                dialog = QMessageBox(self)
                dialog.setWindowTitle('Pre-flight warnings')
                dialog.setText('%d check(s) would like attention before starting.'
                               % len(warnings))
                dialog.setInformativeText('\n'.join('%s: %s' % (c.name, c.detail)
                                                    for c in warnings[:6]))
                dialog.setStandardButtons(QMessageBox.StandardButton.Ok |
                                          QMessageBox.StandardButton.Cancel)
                again = QCheckBox("Don't ask about warnings again (they still show as notices)")
                dialog.setCheckBox(again)
                answer = dialog.exec()
                if again.isChecked():
                    self.settings.setdefault('ui', {})['preflight_warnings'] = False
                if answer != QMessageBox.StandardButton.Ok:
                    return False
            else:
                for check in warnings[:4]:
                    self.toasts.show('%s: %s' % (check.name, check.detail), 'warn')
        self.cfg = cfg
        self.settings['stack'] = cfg.to_dict()
        try:
            settings.save(self.settings)
        except OSError:
            pass
        env = self.manager.env()
        if cfg.qt_platform:
            env['QT_QPA_PLATFORM'] = cfg.qt_platform
        ok = self.manager.start('sim', cfg.stack_command(str(paths.default_map())), env=env,
                                on_exit=self._sim_exited)
        if not ok:
            self.toasts.show('Could not start ros2 launch - see the Log tab', 'error')
            QMessageBox.critical(self, 'Could not start',
                                 'ros2 launch did not start. See the Log tab.')
            return False
        self._launched_camera = cfg.camera_needed
        self._stack_running = True
        self._stopping = False
        self._last_dash = None
        self.launch_panel.set_running(True, cfg)
        self._set_actions_running(True)
        self._expectations_from_config(cfg)
        self.view_settings['map_mode'] = self._map_click_mode
        self.experiments_panel.apply_base_config(cfg)
        # the button keeps spinning and a card tracks the stage until Nav2 is up
        self._starting = True
        self._start_t0 = time.monotonic()
        self._start_toast = self.toasts.busy('Starting stack: launching Gazebo and Nav2…')
        self.launch_panel.set_start_stage('Launching')
        self._startup_timer.start()
        QTimer.singleShot(1500, lambda: self._start_live_parts(cfg))
        return True

    def _startup_tick(self):
        """1 Hz while the stack comes up: stage text on the button and the card."""
        if not self._starting:
            self._startup_timer.stop()
            return
        elapsed = time.monotonic() - self._start_t0
        if not self.backend.available():
            if elapsed >= 6.0:
                self._finish_starting(
                    True, 'Stack launched. This panel has no ROS link, so readiness is '
                          'not tracked.')
            return
        dash = self._last_dash
        nav_up = bool(dash and dash.nav2_state in ('active', 'partially active'))
        sim_up = bool(dash and dash.sim_time and dash.sim_time > 0)
        if nav_up:
            self._finish_starting(True, 'Stack ready: Gazebo and Nav2 are up (%.0f s)' % elapsed)
        elif elapsed > 150.0:
            self._finish_starting(False, 'Still not ready after 2.5 min - open the Log tab '
                                         'and check the sim process')
        else:
            stage = 'Waiting for Nav2' if sim_up else 'Launching Gazebo'
            self.launch_panel.set_start_stage(stage)
            self.toasts.update(self._start_toast,
                               'Starting stack: %s… (%d s)' % (stage.lower(), elapsed))

    def _finish_starting(self, ok, text):
        self._starting = False
        self._startup_timer.stop()
        self.launch_panel.clear_starting('ok' if ok else 'fail')
        self.toasts.finish(self._start_toast, text, 'ok' if ok else 'warn')
        self._start_toast = None

    def _cancel_starting(self):
        """Stop / exit while still starting: drop the spinner without a verdict."""
        if not self._starting:
            return
        self._starting = False
        self._startup_timer.stop()
        self.launch_panel.clear_starting()
        self.toasts.close(self._start_toast)
        self._start_toast = None

    def _start_live_parts(self, cfg):
        if not self._stack_running:
            return
        if cfg.teleop:
            self._start_teleop(cfg)
        if cfg.detector:
            self._start_detector(cfg)
        if cfg.collect_dataset:
            self.start_recording()
        if cfg.coverage:
            self._start_coverage(cfg)
        if cfg.rviz:
            self.rviz_toggle(True)

    def restart_stack(self, cfg):
        self.stop_stack(then_start=cfg)

    LIVE_PARTS = ('rosbag', 'dataset', 'coverage', 'detector', 'teleop', 'rviz')

    def stop_stack(self, checked=False, then_start=None):
        if self._stopping:
            return
        if not self._stack_running and not self.manager.running_names():
            if then_start is not None:
                self.start_stack(then_start)
            return
        self._cancel_starting()
        self._stopping = True
        self._set_actions_running(False)
        self.teleop_panel.stop_all()
        self.launch_panel.set_stopping('Stopping')
        self._stop_toast = self.toasts.busy(
            'Restarting: stopping the running stack first…' if then_start is not None
            else 'Stopping stack… (a few seconds)')
        sim = self.manager.get('sim')
        session = sim.pgid if sim is not None else None
        # the signal waits can take many seconds: never on the GUI thread
        self.bg.run(lambda: self._stop_processes(session),
                    lambda killed, error: self._stop_finished(killed, error, then_start),
                    name='stop-stack')

    def _stop_processes(self, sim_session):
        """Blocking: stop every managed process. Worker thread only - no widgets in here."""
        threads = []
        for name in self.LIVE_PARTS:
            if self.manager.running(name):
                thread = threading.Thread(target=self.manager.stop, args=(name,),
                                          kwargs={'int_wait': 6.0, 'term_wait': 3.0},
                                          daemon=True)
                thread.start()
                threads.append(thread)
        for thread in threads:      # the mission first, so nothing sends goals as Nav2 dies
            thread.join()
        self.manager.stop('sim', int_wait=10.0, term_wait=5.0)
        self.manager.wait_dead(timeout=8.0)
        # Anything left with the launch's session id came from our tree but escaped its
        # process group (a gz bridge re-execs, for example). It is ours: kill it without
        # asking.
        ours = find_leftovers(session=sim_session) if sim_session else []
        if ours:
            kill_leftovers(ours)
        return len(ours)

    def _stop_finished(self, killed, error, then_start):
        self._stack_running = False
        self._stopping = False
        self.launch_panel.clear_stopping('fail' if error else 'ok')
        self.launch_panel.set_running(False)
        self.mission_panel.set_running(False)
        self.dataset_panel.set_recording(False)
        self.tools_panel.set_bag_state(False)
        self._expectations_from_config(None)
        self.camera_panel.set_reason('stack stopped')
        self.map_panel.view.clear_trail()
        self.launch_panel.refresh_leftovers()
        leftovers = find_leftovers(only=BLOCKING_KINDS)
        if error is not None:
            self.toasts.finish(self._stop_toast, 'Stopping failed: %s' % error, 'error')
        elif killed:
            self.toasts.finish(self._stop_toast, 'Stack stopped; killed %d process(es) that '
                                                 'escaped the launch group' % killed, 'warn')
        elif leftovers:
            self.toasts.finish(self._stop_toast, 'Stack stopped, but %d leftover gz/bridge '
                                                 'process(es) remain' % len(leftovers), 'warn')
        else:
            self.toasts.finish(self._stop_toast, 'Stack stopped cleanly', 'ok')
        self._stop_toast = None
        if leftovers:
            self.prompt_kill_leftovers(leftovers)
        if then_start is not None:
            QTimer.singleShot(500, lambda: self.start_stack(then_start))
        if self._close_after_stop:
            self._close_after_stop = False
            self.close()

    def _kill_stale_now(self):
        """Used by the pre-flight dialog: reap every stack process and wait for it."""
        self.manager.stop_all(int_wait=5.0, term_wait=3.0)
        leftovers = find_leftovers(only=BLOCKING_KINDS)
        if leftovers:
            kill_leftovers(leftovers, wait=4.0)
        self.manager.wait_dead(timeout=6.0)
        self.launch_panel.refresh_leftovers()

    def prompt_kill_leftovers(self, leftovers=None):
        leftovers = leftovers or find_leftovers(only=BLOCKING_KINDS)
        if not leftovers:
            self.toasts.show('No leftover gz / bridge processes', 'ok')
            return
        names = '\n'.join('%s (pid %d) %s' % (l.name, l.pid, l.cmdline[:90])
                          for l in leftovers[:8])
        answer = QMessageBox.question(
            self, 'Leftover processes',
            '%d Gazebo / bridge process(es) are still alive:\n\n%s\n\nKill them? '
            '(If one belongs to another project, answer No and close that one yourself.)'
            % (len(leftovers), names))
        if answer == QMessageBox.StandardButton.Yes:
            result = kill_leftovers(leftovers)
            self.toasts.show('Killed %d leftover process(es)' % len(result), 'ok')
            self.launch_panel.refresh_leftovers()

    def do_kill_leftovers(self):
        self.prompt_kill_leftovers()

    def _sim_exited(self, proc, code):
        """Called from the process-wait thread: hop to the GUI thread before any widget."""
        self.bg.post(lambda: self._sim_exited_gui(code))

    def _sim_exited_gui(self, code):
        if not self._stack_running or self._stopping:
            return
        self._cancel_starting()
        self._stack_running = False
        self.launch_panel.set_running(False)
        self._set_actions_running(False)
        self.dataset_panel.set_recording(False)
        self.toasts.show('Gazebo / ros2 launch exited with code %s - see the sim tab in the '
                         'Log' % code, 'error')
        self._notice('error', 'Gazebo/launch exited with code %s - see the sim log tab' % code)
        names = [n for n in ('coverage', 'detector', 'dataset', 'teleop', 'rviz')
                 if self.manager.running(n)]
        if names:
            self.bg.run(lambda: [self.manager.stop(n, int_wait=4.0, term_wait=2.0)
                                 for n in names], name='stop-orphans')

    # -------------------------------------------------------------- live toggles
    def live_toggle(self, name, value):
        cfg = self.launch_panel.config()
        self.cfg = cfg
        self.settings['stack'] = cfg.to_dict()
        if name == 'teleop':
            self._start_teleop(cfg) if value else self._stop_async('teleop', 'gamepad teleop')
        elif name == 'detector':
            self._start_detector(cfg) if value else self._stop_async('detector', 'detector')
        elif name in ('coverage', 'meter', 'autostart'):
            self._restart_coverage(cfg)
        elif name in ('reclean', 'reclean_source'):
            if cfg.coverage:
                self._restart_coverage(cfg)
        elif name == 'collect_dataset':
            self.start_recording() if value else self.stop_recording()
        self._expectations_from_config(cfg)
        # keep the applied snapshot: a live toggle must not hide a pending restart
        self.launch_panel.set_running(True)

    def _stop_async(self, name, label, then=None, **kwargs):
        """Stop one managed process without freezing the window; a card shows the outcome."""
        if not self.manager.running(name):
            if then is not None:
                then()
            return
        toast = self.toasts.busy('Stopping %s…' % label)

        def done(_result, error):
            if error is not None:
                self.toasts.finish(toast, 'Could not stop %s: %s' % (label, error), 'error')
            else:
                self.toasts.finish(toast, '%s stopped' % label[:1].upper() + label[1:], 'ok',
                                   2500)
            if then is not None:
                then()

        self.bg.run(lambda: self.manager.stop(name, **kwargs), done, name='stop-' + name)

    def _start_proc(self, name, label, argv, env=None):
        """Start a managed process and say so; False when it could not be started."""
        if self.manager.running(name):
            return True
        ok = self.manager.start(name, argv, env=env)
        if ok:
            self.toasts.show('%s started' % label, 'ok', 2500)
        else:
            self.toasts.show('Could not start %s - see the Log tab' % label.lower(), 'error')
        return ok

    def _start_teleop(self, cfg):
        env = self.manager.env()
        if cfg.qt_platform:
            env['QT_QPA_PLATFORM'] = cfg.qt_platform
        self._start_proc('teleop', 'Gamepad teleop', cfg.teleop_command(), env=env)

    def _start_detector(self, cfg):
        # Judge by what the *running* launch has, not by the panel: ticking the detector
        # makes cfg.camera_needed true by itself, but the URDF of this launch is fixed.
        if not self._launched_camera:
            self._notice('warn', 'the camera was disabled at launch; tick the camera option '
                                 'and restart the stack before starting the detector')
            box = self.launch_panel.detector
            box.blockSignals(True)
            box.setChecked(False)
            box.blockSignals(False)
            self.launch_panel._changed()
            return
        self._start_proc('detector', 'Dirt detector', cfg.detector_command())

    def _coverage_params_file(self, cfg):
        if cfg.reclean:
            return None
        target = paths.config_dir() / 'coverage_params.yaml'
        try:
            from .commands import coverage_params_with_overrides
            return str(coverage_params_with_overrides(
                paths.coverage_config(), {'reclean': False, 'reclean_source':
                                          cfg.reclean_source}, target))
        except (OSError, ValueError) as exc:
            self._notice('warn', 'could not write coverage params: %s' % exc)
            return None

    def _start_coverage(self, cfg):
        params = self._coverage_params_file(cfg)
        self._start_proc('coverage', 'Coverage mission',
                         cfg.coverage_command(str(paths.default_map()), params,
                                              autostart=cfg.autostart))
        self.mission_panel.set_running(True)

    def _restart_coverage(self, cfg):
        def start():
            if cfg.coverage:
                self._start_coverage(cfg)
            else:
                self.mission_panel.set_running(False)

        self._stop_async('coverage', 'coverage mission', then=start,
                         int_wait=5.0, term_wait=3.0)

    def rviz_toggle(self, enabled):
        if enabled:
            env = self.manager.env()
            env['QT_QPA_PLATFORM'] = self.cfg.qt_platform or 'xcb'
            rviz_config = paths.install_share('vacuum_bringup') / 'rviz' / 'navigation.rviz'
            source = Path(paths.workspace_root()) / 'src' / 'vacuum_bringup' / 'rviz' / \
                'navigation.rviz'
            config = str(source if source.is_file() else rviz_config)
            self._start_proc('rviz', 'RViz', self.cfg.rviz_command(config), env=env)
        else:
            self._stop_async('rviz', 'RViz')

    def open_gazebo_gui(self):
        if not self._stack_running:
            QMessageBox.information(
                self, 'Gazebo GUI',
                'The stack is not running. Start it with "No Gazebo window (headless)" '
                'unticked to get the Gazebo window.')
            return
        if self.cfg.headless:
            answer = QMessageBox.question(
                self, 'Gazebo GUI',
                'This stack runs headless, so Gazebo has no window to open. Restart it '
                'with headless off? (This window keeps drawing everything either way.)')
            if answer == QMessageBox.StandardButton.Yes:
                cfg = self.launch_panel.config()
                cfg.headless = False
                self.launch_panel.set_config(cfg)
                self.restart_stack(cfg)
        else:
            QMessageBox.information(self, 'Gazebo GUI',
                                    'The Gazebo window is part of the running stack; check '
                                    'your task bar.')

    def _reclean_toggled(self, value):
        self.launch_panel.reclean.setChecked(value)
        if self._stack_running:
            self.live_toggle('reclean', value)

    def _load_map_path(self, path):
        self.launch_panel.map.setEditText(path)
        self.launch_panel._changed()
        self._notice('info', 'map for the next start: %s' % path)

    # ------------------------------------------------------------------ misc ----
    def _pose_clicked(self, x, y):
        if not self.backend.available():
            return
        yaw = 0.0
        if self.map_panel.view._pose is not None:
            yaw = self.map_panel.view._pose.yaw
        self.backend.request_initial_pose(x, y, yaw)

    def _goal_clicked(self, x, y):
        if not self.backend.available():
            return
        self.backend.send_goal(x, y)

    def _inspect_clicked(self, x, y):
        spot = self.map_panel.view.nearest_detection(x, y)
        truth, distance = self.map_panel.view.nearest_truth(x, y)
        if spot is None and truth is None:
            self._notice('info', 'nothing detected near (%.2f, %.2f)' % (x, y))
            return
        DirtInspector(spot, truth, distance, self).exec()

    def _click_mode_changed(self, _index):
        self._map_click_mode = self.click_mode.currentData()
        self.map_panel.view.set_click_mode(self._map_click_mode)
        self.view_settings['map_mode'] = self._map_click_mode
        self._save_view_settings()

    def _save_view_settings(self):
        self.view_settings['teleop_linear'] = self.teleop_panel.linear_speed()
        self.view_settings['teleop_angular'] = self.teleop_panel.angular_speed()
        self.view_settings['show_scan'] = self.layer_checks['scan'].isChecked()
        self.view_settings['show_cleaned'] = self.layer_checks['cleaned'].isChecked()
        self.view_settings['show_plan_global'] = self.layer_checks['plan_global'].isChecked()
        self.view_settings['show_plan_local'] = self.layer_checks['plan_local'].isChecked()
        self.view_settings['show_plan_coverage'] = self.layer_checks['coverage_plan'].isChecked()
        self.view_settings['show_dirt_detected'] = self.layer_checks['dirt_detected'].isChecked()
        self.view_settings['show_dirt_truth'] = self.layer_checks['dirt_truth'].isChecked()
        self.view_settings['show_particles'] = self.layer_checks['particles'].isChecked()
        self.view_settings['show_costmaps'] = self.layer_checks['costmaps'].isChecked()
        self.settings['view'] = self.view_settings

    def _expectations_from_config(self, cfg):
        if cfg is None:
            self.backend.set_expectations({'running': False})
            return
        self.backend.set_expectations({
            'running': True, 'slam': bool(cfg.slam), 'coverage': bool(cfg.coverage),
            'detector': bool(cfg.detector), 'teleop': bool(cfg.teleop),
            'camera': bool(cfg.camera_needed), 'dirt': cfg.dirt > 0,
        })
        self.backend.set_image_rate(self.view_settings.get('camera_fps', 10))

    def _set_actions_running(self, running):
        self.action_start.setEnabled(not running)
        self.action_stop.setEnabled(running)
        self.action_plan.setEnabled(running)
        self.action_clean.setEnabled(running)
        self.action_stop_mission.setEnabled(running)

    # ------------------------------------------------------------ ros results ---
    def _refresh_leftovers(self):
        self.launch_panel.refresh_leftovers(
            exclude=list(self.manager.pids().values()), active=self._stack_running)

    def _backend_connected(self, ok):
        self.status_connection.setText('ROS: connected' if ok else 'ROS: not connected')
        if not ok and self.backend.reason():
            self.status_connection.setToolTip(self.backend.reason())

    def _backend_unavailable(self):
        reason = getattr(self.backend, 'reason', lambda: 'ROS is not available')()
        self.status_connection.setText('ROS: unavailable')
        self.status_connection.setToolTip(reason)
        self.camera_panel.set_reason(reason)
        for action in (self.action_plan, self.action_clean, self.action_stop_mission):
            action.setEnabled(False)
        self._notice('warn', reason)

    def _map_received(self, grid):
        self.map_panel.set_grid(grid)
        self.map_panel.view.set_click_mode(self._map_click_mode)

    def _image_received(self, kind, frame):
        self.camera_panel.on_image(kind, frame)

    def _markers_received(self, kind, marker_set):
        self.map_panel.view.set_markers(kind, marker_set)
        if kind == 'coverage':
            self.mission_panel.set_plan_pieces(marker_set.markers)

    def _joy_received(self, joy):
        self.teleop_panel.set_gamepad(True, 'gamepad: %d axes, %d buttons (deadman LB)'
                                      % (len(joy.axes), len(joy.buttons)))

    def _dashboard_received(self, dash):
        self._last_dash = dash
        self.dashboard.update_dashboard(dash)
        self.mission_panel.set_coverage(dash.coverage_pct, dash.coverage_free_pct)
        if dash.mission_status and dash.mission_status != self._last_mission_text:
            self._last_mission_text = dash.mission_status
            self.mission_panel.set_status(dash.mission_status)
        if self._stack_running and not self.backend.available():
            self.status_connection.setText('ROS: unavailable')

    def _nav2_received(self, nav2_status):
        pass

    def _notice(self, level, text):
        if threading.current_thread() is not threading.main_thread():
            self.bg.post(lambda: self._notice(level, text))      # widgets: GUI thread only
            return
        self.status_message.setText(text)
        self.log_console.append('gui', level if level in ('warn', 'error') else 'info', text)
        if level == 'error':
            self.status_message.setStyleSheet('color: #e03131;')
        elif level == 'warn':
            self.status_message.setStyleSheet('color: #e8a33d;')
        else:
            self.status_message.setStyleSheet('')
        if level in ('warn', 'error'):
            self.toasts.show(text, level)

    # ------------------------------------------------------------ mission services
    TRIGGER_LABELS = {'plan': ('Planning route', 'Planning'),
                      'start': ('Starting cleaning', 'Starting'),
                      'stop': ('Stopping mission', 'Stopping'),
                      'reset_meter': ('Resetting meter', 'Resetting')}

    def _trigger_button(self, service):
        m = self.mission_panel
        return {'plan': m.plan, 'start': m.start, 'stop': m.stop,
                'reset_meter': m.reset}.get(service)

    def run_trigger(self, service):
        """Call a mission/meter service and show, at once and at the end, what happened."""
        label, verb = self.TRIGGER_LABELS.get(service, (service, service))
        if service in self._pending:
            self.toasts.show('%s: still waiting for the previous request' % label, 'warn')
            return
        if not self.backend.available():
            self.toasts.show('%s: ROS is not connected (%s)'
                             % (label, self.backend.reason()), 'error')
            button = self._trigger_button(service)
            if button is not None:
                feedback.end(button, 'fail')      # red ring: the press did register
            return
        toast = self.toasts.busy('%s…' % label)
        button = self._trigger_button(service)
        if button is not None:
            feedback.begin(button, verb)
        watchdog = QTimer(self)
        watchdog.setSingleShot(True)
        watchdog.timeout.connect(lambda: self._trigger_done(
            service, False, 'no answer after 10 s - is the mission node running?'))
        watchdog.start(10000)
        self._pending[service] = (toast, button, watchdog)
        self.backend.call_trigger(service)

    def _trigger_done(self, service, ok, message):
        pending = self._pending.pop(service, None)
        label = self.TRIGGER_LABELS.get(service, (service, service))[0]
        if pending is None:         # e.g. the e-stop's own stop call: just log it
            self._notice('info' if ok else 'warn', '%s: %s' % (service, message))
            return
        toast, button, watchdog = pending
        watchdog.stop()
        watchdog.deleteLater()
        if button is not None:
            feedback.end(button, 'ok' if ok else 'fail')
        text = '%s: %s' % (label, message) if message else (
            '%s - done' % label if ok else '%s failed' % label)
        self.toasts.finish(toast, text, 'ok' if ok else 'error')
        self.log_console.append('gui', 'info' if ok else 'error', text)

    # -------------------------------------------------------------- teleop/e-stop
    def set_estop(self, active):
        self._estop = bool(active)
        self.action_estop.setChecked(self._estop)
        self.estop_button.setChecked(self._estop)
        self.estop_button.setText('RELEASE E-STOP' if self._estop else 'E-STOP')
        self.teleop_panel.set_estop(self._estop)
        if self._estop:
            self.backend.cancel_nav_goal()
            self.backend.call_trigger('stop')
            self._estop_timer.start()
            self._estop_tick()
            self._notice('warn', 'EMERGENCY STOP: zero /cmd_vel until released')
        else:
            self._estop_timer.stop()
            self.backend.publish_cmd_vel(0.0, 0.0)
            self._notice('info', 'e-stop released')

    def _estop_tick(self):
        self.backend.publish_cmd_vel(0.0, 0.0)

    def stop_mission(self):
        if self._estop:
            self.set_estop(False)
        self.backend.cancel_nav_goal()
        self.run_trigger('stop')

    # ---------------------------------------------------------------- records ---
    def start_recording(self):
        cfg = self.launch_panel.config()
        if cfg.dirt <= 0:
            self._notice('warn', 'recording needs dirt>0 (labels come from the dirt layout)')
            return
        if not self._launched_camera:
            self._notice('warn', 'recording needs the camera in the URDF; tick the camera '
                                 'option and restart')
            return
        if self.manager.running('dataset'):
            return
        ok = self.manager.start(
            'dataset', cfg.dataset_command(),
            on_exit=lambda proc, rc: self.bg.post(
                lambda: self.dataset_panel.set_recording(False)))
        if not ok:
            self.toasts.show('Could not start the dataset recorder - see the Log tab', 'error')
            return
        self.dataset_panel.set_recording(True)
        self.dataset_panel.set_dataset_dir(cfg.dataset_path())
        self.toasts.show('Dataset recording started into %s' % cfg.dataset_path(), 'ok')

    def stop_recording(self):
        self._stop_async('dataset', 'dataset recording', int_wait=10.0, term_wait=4.0)
        self.dataset_panel.set_recording(False)

    def package_dataset(self, source, output, classes):
        self._notice('info', 'packaging %s -> %s' % (source, output))
        threading.Thread(target=self.dataset_panel.run_package,
                         args=(source, output, classes), daemon=True).start()

    def save_map(self, name):
        name = (name or '').strip() or time.strftime('map_%Y%m%d_%H%M%S')
        button = self.mission_panel.save_button
        feedback.begin(button, 'Saving')
        started = self.manager.start(
            'save_map', self.cfg.save_map_command(name),
            on_exit=lambda proc, rc: self.bg.post(lambda: self._map_saved(name, rc)))
        if not started:
            feedback.end(button, 'fail')
            self.toasts.show('Could not save the map (a save is already running?)', 'error')

    def _map_saved(self, name, code):
        ok = code == 0
        feedback.end(self.mission_panel.save_button, 'ok' if ok else 'fail')
        if ok:
            self.toasts.show('Map saved as "%s" in %s' % (name, paths.maps_dir()), 'ok')
        else:
            self.toasts.show('Saving map "%s" failed (exit code %s) - is the sim up and '
                             'publishing /map? See the Log tab' % (name, code), 'error')
        self.mission_panel.refresh_maps()

    # ---------------------------------------------------------------- extras ----
    def take_screenshot(self, target='map'):
        folder = paths.screenshots_dir()
        folder.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = folder / ('%s_%s.png' % (target, stamp))
        if target == 'camera':
            ok = self.camera_panel.grab_png(str(path))
        else:
            ok = self.map_panel.view.grab_png(str(path))
        self._notice('info' if ok else 'warn',
                     'screenshot saved %s' % path if ok else 'screenshot failed')
        if ok:
            self.toasts.show('Screenshot saved: %s' % path, 'ok')

    def record_gif(self, target, seconds, fps):
        if self._gif_timer is not None:
            self._notice('warn', 'a GIF is already being recorded')
            return
        folder = paths.screenshots_dir()
        folder.mkdir(parents=True, exist_ok=True)
        self._gif_path = folder / ('%s_%s.gif' % (target, time.strftime('%Y%m%d_%H%M%S')))
        self._gif_target = target
        self._gif_frames = []
        self._gif_timer = QTimer(self)
        self._gif_timer.setInterval(max(50, int(1000 / max(1, fps))))
        self._gif_total = max(1, int(seconds * fps))
        self._gif_timer.timeout.connect(self._gif_frame)
        self._gif_timer.start()
        self._notice('info', 'recording a %d s GIF of the %s at %d fps'
                     % (seconds, target, fps))

    def _gif_frame(self):
        view = (self.camera_panel.active_view() if self._gif_target == 'camera'
                else self.map_panel.view.viewport())
        pixmap = view.grab()
        self._gif_frames.append(pixmap.toImage().copy())
        if len(self._gif_frames) >= self._gif_total:
            timer = self._gif_timer
            if timer is not None:
                timer.stop()
            self._gif_timer = None
            frames, self._gif_frames = self._gif_frames, []
            path = self._gif_path
            threading.Thread(target=self._write_gif, args=(frames, path),
                             daemon=True).start()

    def _write_gif(self, frames, path):
        try:
            from PIL import Image
            images = []
            for image in frames:
                image = image.convertToFormat(image.Format.Format_RGBA8888)
                buffer = image.bits().tobytes()
                images.append(Image.frombytes('RGBA', (image.width(), image.height()),
                                              buffer).convert('P', palette=Image.Palette.ADAPTIVE))
            if images:
                images[0].save(path, save_all=True, append_images=images[1:], duration=500,
                               loop=0)
                self._notice('info', 'GIF saved %s' % path)
                self.bg.post(lambda: self.toasts.show('GIF saved: %s' % path, 'ok'))
        except Exception as exc:      # noqa: BLE001 - PIL may be missing
            self._notice('warn', 'GIF failed: %s' % exc)

    def rosbag_toggle(self, active, topics):
        if active:
            folder = paths.runs_dir() / 'bags'
            folder.mkdir(parents=True, exist_ok=True)
            out = folder / ('bag_%s' % time.strftime('%Y%m%d_%H%M%S'))
            argv = ['ros2', 'bag', 'record', '-o', str(out)] + list(topics)
            if self.manager.start('rosbag', argv):
                self.tools_panel.set_bag_state(True, 'recording to %s' % out)
                self.toasts.show('Recording a rosbag to %s' % out, 'ok')
            else:
                self.tools_panel.set_bag_state(False, 'could not start rosbag')
        else:
            self._stop_async('rosbag', 'rosbag recording', int_wait=8.0, term_wait=4.0)
            self.tools_panel.set_bag_state(False, 'stopped')

    # -------------------------------------------------------------- presets ----
    def load_preset(self, name):
        presets = settings.load_presets()
        preset = presets.get(name)
        if not preset:
            return
        cfg = StackConfig.from_dict(preset.get('stack', {}))
        self.launch_panel.set_config(cfg)
        view = preset.get('view') or {}
        for key, layer in (('show_scan', 'scan'), ('show_cleaned', 'cleaned'),
                           ('show_plan_global', 'plan_global'),
                           ('show_plan_local', 'plan_local'),
                           ('show_plan_coverage', 'coverage_plan'),
                           ('show_dirt_detected', 'dirt_detected'),
                           ('show_dirt_truth', 'dirt_truth'),
                           ('show_particles', 'particles'),
                           ('show_costmaps', 'costmaps')):
            if key in view:
                self.layer_checks[layer].setChecked(bool(view[key]))
        self._save_view_settings()
        self._notice('info', 'preset loaded: %s - %s' % (name, preset.get('notes', '')))

    def save_preset(self):
        name, ok = QInputDialog.getText(self, 'Save preset', 'Name for this configuration:',
                                        text='My preset')
        if not ok or not name.strip():
            return
        presets = settings.load_presets()
        presets[name.strip()] = settings.custom_preset(
            name.strip(), self.launch_panel.config(), self.view_settings,
            notes='saved from the panel on %s' % time.strftime('%Y-%m-%d %H:%M'))
        settings.save_presets(presets)
        self.launch_panel.set_presets(presets)
        self._notice('info', 'preset saved: %s' % name.strip())

    # ---------------------------------------------------------------- theme ----
    def _theme_chosen(self, index):
        key = self.theme_combo.itemData(index)
        self.settings.setdefault('ui', {})['theme'] = key
        applied = theme.apply(self.app, key)
        name = theme.label(key)
        if key == theme.SYSTEM:
            name += ' (%s)' % theme.label(applied)
        self.toasts.show('Theme: %s' % name, 'info', 1800)
        try:
            settings.save(self.settings)
        except OSError:
            pass

    def show_help(self):
        HelpDialog(self).exec()
        self.settings.setdefault('ui', {})['help_seen'] = True
        try:
            settings.save(self.settings)
        except OSError:
            pass

    # ---------------------------------------------------------------- layout ---
    def _restore_layout(self):
        data = settings.load_layout()
        try:
            if data.get('geometry'):
                self.restoreGeometry(QByteArray(base64.b64decode(data['geometry'])))
            if data.get('state'):
                self.restoreState(QByteArray(base64.b64decode(data['state'])))
        except (ValueError, TypeError):
            pass
        # a stale or offscreen geometry can be smaller than the layout's minimum, which
        # makes the form unreadable until the user resizes; fall back to a sane size
        minimum = self.minimumSizeHint()
        if self.width() < max(1200, minimum.width()) or self.height() < max(760, minimum.height()):
            self.resize(max(1500, minimum.width()), max(950, minimum.height()))
        self.map_panel.view.set_click_mode(self._map_click_mode)

    def _save_layout(self):
        try:
            settings.save_layout(
                base64.b64encode(bytes(self.saveGeometry())).decode('ascii'),
                base64.b64encode(bytes(self.saveState())).decode('ascii'))
        except OSError:
            pass

    # ------------------------------------------------------------------ logs ---
    def _drain_logs(self):
        entries, index = self.manager.log.since(self._log_index)
        self._log_index = index
        for entry in entries:
            _stamp, proc, stream, text = entry
            self.log_console.append(proc, stream, text)

    # ----------------------------------------------------------------- close ---
    def closeEvent(self, event):      # noqa: N802
        if self._stopping:
            self._close_after_stop = True
            self.toasts.show('Still stopping the stack - the window closes when it is done',
                             'warn')
            event.ignore()
            return
        if self._stack_running:
            dialog = QMessageBox(self)
            dialog.setWindowTitle('The stack is running')
            dialog.setText('Close the panel and leave Gazebo running?')
            dialog.setInformativeText(
                'Stop stack: sends SIGINT to the process group and waits (no orphan gz).\n'
                'Leave running: the sim keeps going; use "Kill leftovers" or Stop next time.')
            stop = dialog.addButton('Stop stack and quit', QMessageBox.ButtonRole.AcceptRole)
            leave = dialog.addButton('Leave running', QMessageBox.ButtonRole.DestructiveRole)
            cancel = dialog.addButton(QMessageBox.StandardButton.Cancel)
            dialog.exec()
            if dialog.clickedButton() is cancel:
                event.ignore()
                return
            if dialog.clickedButton() is stop:
                # the window is going away anyway: stop here, in line, so nothing is orphaned
                sim = self.manager.get('sim')
                self._stopping = True
                self._cancel_starting()
                self._stop_processes(sim.pgid if sim is not None else None)
                self._stack_running = False
                self._stopping = False
            elif dialog.clickedButton() is leave:
                self._notice('warn', 'leaving the stack running')
        self._save_view_settings()
        self.settings['stack'] = self.launch_panel.config().to_dict()
        try:
            settings.save(self.settings)
        except OSError:
            pass
        self._save_layout()
        self.backend.stop()
        self.manager.stop_all(int_wait=4.0, term_wait=2.0)
        event.accept()

    def _toggle_stack(self):
        if self._stack_running:
            self.stop_stack()
        else:
            self.start_stack(self.launch_panel.config())

    # Space is the e-stop, but not while typing in a field
    def keyPressEvent(self, event):      # noqa: N802
        if event.key() == Qt.Key.Key_Space and not event.modifiers():
            focus = self.focusWidget()
            if focus is None or not focus.inherits('QLineEdit'):
                self.set_estop(not self._estop)
                event.accept()
                return
        super().keyPressEvent(event)
