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
                               QSizePolicy, QSplitter, QTextBrowser, QToolBar, QVBoxLayout,
                               QWidget)

from . import paths, preflight, settings, theme
from .backend import NullBackend
from .camera_view import CameraPanel
from .commands import StackConfig, shell_join
from .dashboard_bar import DashboardBar
from .dataset_panel import DatasetPanel
from .dirt_inspector import DirtInspector
from .experiments_panel import ExperimentsPanel
from .health_panel import HealthPanel
from .launch_manager import ProcessManager, find_leftovers, kill_leftovers
from .launch_panel import LaunchPanel, PreflightDialog
from .log_console import LogConsole
from .map_view import MapPanel
from .mission_panel import MissionPanel
from .reports_panel import ReportsPanel
from .teleop_panel import TeleopPanel
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

        self.setWindowTitle('RoboVAC control panel')
        self.setObjectName('vacuum_gui')
        self.resize(1500, 950)

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
        self.action_plan.triggered.connect(lambda: self.backend.call_trigger('plan'))
        self.action_clean = QAction('Start cleaning', self)
        self.action_clean.triggered.connect(lambda: self.backend.call_trigger('start'))
        self.action_stop_mission = QAction('Stop cleaning', self)
        self.action_stop_mission.triggered.connect(self.stop_mission)
        for action in (self.action_plan, self.action_clean, self.action_stop_mission):
            action.setEnabled(False)
            bar.addAction(action)
        bar.addSeparator()

        self.action_shot = QAction('Screenshot map', self)
        self.action_shot.triggered.connect(lambda: self.take_screenshot('map'))
        bar.addAction(self.action_shot)

        theme_button = QPushButton()
        theme_button.setFlat(True)
        theme_button.setText('Theme')
        theme_button.setToolTip('Follow the desktop, or force dark/light')
        theme_button.clicked.connect(self.cycle_theme)
        bar.addWidget(theme_button)

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
        m.plan_requested.connect(lambda: self.backend.call_trigger('plan'))
        m.start_requested.connect(lambda: self.backend.call_trigger('start'))
        m.stop_requested.connect(self.stop_mission)
        m.reset_meter_requested.connect(lambda: self.backend.call_trigger('reset_meter'))
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

    def _build_shortcuts(self):
        def shortcut(sequence, slot, app_wide=False):
            sc = QShortcut(QKeySequence(sequence), self)
            sc.setContext(Qt.ShortcutContext.ApplicationShortcut if app_wide
                          else Qt.ShortcutContext.WindowShortcut)
            sc.activated.connect(slot)
            return sc

        shortcut('F5', self._toggle_stack)
        shortcut('Ctrl+P', lambda: self.backend.call_trigger('plan'))
        shortcut('Ctrl+Return', lambda: self.backend.call_trigger('start'))
        shortcut('Ctrl+.', self.stop_mission)
        shortcut('Ctrl+M', self.map_panel.view.fit)
        shortcut('Ctrl+Shift+K', self.do_kill_leftovers)
        shortcut('F1', self.show_help)
        shortcut('Ctrl+Shift+S', lambda: self.take_screenshot('map'))

    # --------------------------------------------------------------- actions ---
    def start_stack(self, cfg):
        if self._stack_running:
            return
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
                return
        warnings = [c for c in checks if not c.ok]
        if warnings:
            dialog = QMessageBox(self)
            dialog.setWindowTitle('Pre-flight warnings')
            dialog.setText('%d check(s) would like attention before starting.'
                           % len(warnings))
            dialog.setInformativeText('\n'.join('%s: %s' % (c.name, c.detail)
                                                for c in warnings[:6]))
            dialog.setStandardButtons(QMessageBox.StandardButton.Ok |
                                      QMessageBox.StandardButton.Cancel)
            if dialog.exec() != QMessageBox.StandardButton.Ok:
                return
        self.cfg = cfg
        self.settings['stack'] = cfg.to_dict()
        settings.save(self.settings)
        env = self.manager.env()
        if cfg.qt_platform:
            env['QT_QPA_PLATFORM'] = cfg.qt_platform
        ok = self.manager.start('sim', cfg.stack_command(str(paths.default_map())), env=env,
                                on_exit=self._sim_exited)
        if not ok:
            QMessageBox.critical(self, 'Could not start',
                                 'ros2 launch did not start. See the Log tab.')
            return
        self._stack_running = True
        self._stopping = False
        self.launch_panel.set_running(True, cfg)
        self._set_actions_running(True)
        self._expectations_from_config(cfg)
        self.view_settings['map_mode'] = self._map_click_mode
        self.experiments_panel.apply_base_config(cfg)
        self._notice('info', 'stack started')
        QTimer.singleShot(1500, lambda: self._start_live_parts(cfg))

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
        self._notice('info', 'restarting the stack (this takes a few seconds)')
        self.stop_stack(then_start=cfg)

    def stop_stack(self, checked=False, then_start=None):
        if not self._stack_running and not self.manager.running_names():
            if then_start is not None:
                self.start_stack(then_start)
            return
        self._stopping = True
        self._set_actions_running(False)
        self._notice('info', 'stopping…')
        self.teleop_panel.stop_all()
        for name in ('rosbag', 'dataset', 'coverage', 'detector', 'teleop', 'rviz'):
            if self.manager.running(name):
                self.manager.stop(name, int_wait=6.0, term_wait=3.0)
        sim = self.manager.get('sim')
        sim_session = sim.pgid if sim is not None else None
        self.manager.stop('sim', int_wait=10.0, term_wait=5.0)
        self.manager.wait_dead(timeout=8.0)
        # Anything left with the launch's session id came from our tree but escaped its
        # process group (a gz bridge re-execs, for example). It is ours: kill it without
        # asking, then report what survived.
        ours = find_leftovers(session=sim_session) if sim_session else []
        if ours:
            kill_leftovers(ours)
            self._notice('warn', 'killed %d process(es) that escaped the launch group'
                         % len(ours))
        self._stack_running = False
        self._stopping = False
        self.launch_panel.set_running(False)
        self.mission_panel.set_running(False)
        self.dataset_panel.set_recording(False)
        self.tools_panel.set_bag_state(False)
        self._expectations_from_config(None)
        self.camera_panel.set_reason('stack stopped')
        self.map_panel.view.clear_trail()
        self.launch_panel.refresh_leftovers()
        leftovers = find_leftovers()
        if leftovers:
            self._notice('warn', '%d leftover process(es) from an earlier run' % len(leftovers))
            self.prompt_kill_leftovers(leftovers)
        else:
            self._notice('info', 'stopped cleanly, no leftovers')
        if then_start is not None:
            QTimer.singleShot(500, lambda: self.start_stack(then_start))

    def _kill_stale_now(self):
        """Used by the pre-flight dialog: reap every stack process and wait for it."""
        self.manager.stop_all(int_wait=5.0, term_wait=3.0)
        leftovers = find_leftovers()
        if leftovers:
            kill_leftovers(leftovers, wait=4.0)
        self.manager.wait_dead(timeout=6.0)
        self.launch_panel.refresh_leftovers()

    def prompt_kill_leftovers(self, leftovers=None):
        leftovers = leftovers or find_leftovers()
        if not leftovers:
            self._notice('info', 'no leftovers')
            return
        names = '\n'.join('%s (pid %d) %s' % (l.name, l.pid, l.cmdline[:90])
                          for l in leftovers[:8])
        answer = QMessageBox.question(
            self, 'Leftover processes',
            '%d process(es) from a previous run are still alive:\n\n%s\n\nKill them?'
            % (len(leftovers), names))
        if answer == QMessageBox.StandardButton.Yes:
            result = kill_leftovers(leftovers)
            self._notice('info', 'killed leftovers: %s' % result)
            self.launch_panel.refresh_leftovers()

    def do_kill_leftovers(self):
        self.prompt_kill_leftovers()

    def _sim_exited(self, proc, code):
        if not self._stack_running or self._stopping:
            return
        self._stack_running = False
        self.launch_panel.set_running(False)
        self._set_actions_running(False)
        self.dataset_panel.set_recording(False)
        self._notice('error', 'Gazebo/launch exited with code %s - see the sim log tab' % code)
        for name in ('coverage', 'detector', 'dataset', 'teleop', 'rviz'):
            if self.manager.running(name):
                self.manager.stop(name, int_wait=4.0, term_wait=2.0)

    # -------------------------------------------------------------- live toggles
    def live_toggle(self, name, value):
        cfg = self.launch_panel.config()
        self.cfg = cfg
        self.settings['stack'] = cfg.to_dict()
        if name == 'teleop':
            self._start_teleop(cfg) if value else self.manager.stop('teleop')
        elif name == 'detector':
            self._start_detector(cfg) if value else self.manager.stop('detector')
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

    def _start_teleop(self, cfg):
        env = self.manager.env()
        if cfg.qt_platform:
            env['QT_QPA_PLATFORM'] = cfg.qt_platform
        self.manager.start('teleop', cfg.teleop_command(), env=env)

    def _start_detector(self, cfg):
        if not cfg.camera_needed:
            self._notice('warn', 'the camera was disabled at launch; tick the camera option '
                                 'and restart the stack before starting the detector')
            return
        self.manager.start('detector', cfg.detector_command())

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
        self.manager.start('coverage',
                           cfg.coverage_command(str(paths.default_map()), params,
                                                autostart=cfg.autostart))
        self.mission_panel.set_running(True)

    def _restart_coverage(self, cfg):
        if self.manager.running('coverage'):
            self.manager.stop('coverage', int_wait=5.0, term_wait=3.0)
        if cfg.coverage:
            self._start_coverage(cfg)
        else:
            self.mission_panel.set_running(False)

    def rviz_toggle(self, enabled):
        if enabled:
            env = self.manager.env()
            env['QT_QPA_PLATFORM'] = self.cfg.qt_platform or 'xcb'
            rviz_config = paths.install_share('vacuum_bringup') / 'rviz' / 'navigation.rviz'
            source = Path(paths.workspace_root()) / 'src' / 'vacuum_bringup' / 'rviz' / \
                'navigation.rviz'
            config = str(source if source.is_file() else rviz_config)
            self.manager.start('rviz', self.cfg.rviz_command(config), env=env)
        else:
            self.manager.stop('rviz')

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
        self.status_message.setText(text)
        self.log_console.append('gui', level if level in ('warn', 'error') else 'info', text)
        if level == 'error':
            self.status_message.setStyleSheet('color: #e03131;')
        elif level == 'warn':
            self.status_message.setStyleSheet('color: #e8a33d;')
        else:
            self.status_message.setStyleSheet('')

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
        self.backend.call_trigger('stop')

    # ---------------------------------------------------------------- records ---
    def start_recording(self):
        cfg = self.launch_panel.config()
        if cfg.dirt <= 0:
            self._notice('warn', 'recording needs dirt>0 (labels come from the dirt layout)')
            return
        if not cfg.camera_needed:
            self._notice('warn', 'recording needs the camera in the URDF; tick the camera '
                                 'option and restart')
            return
        self.manager.start('dataset', cfg.dataset_command(),
                           on_exit=lambda proc, rc: self.dataset_panel.set_recording(False))
        self.dataset_panel.set_recording(True)
        self.dataset_panel.set_dataset_dir(cfg.dataset_path())
        self._notice('info', 'dataset recording started into %s' % cfg.dataset_path())

    def stop_recording(self):
        if self.manager.running('dataset'):
            self.manager.stop('dataset', int_wait=10.0, term_wait=4.0)
        self.dataset_panel.set_recording(False)

    def package_dataset(self, source, output, classes):
        self._notice('info', 'packaging %s -> %s' % (source, output))
        threading.Thread(target=self.dataset_panel.run_package,
                         args=(source, output, classes), daemon=True).start()

    def save_map(self, name):
        name = (name or '').strip() or time.strftime('map_%Y%m%d_%H%M%S')
        self.manager.start('save_map', self.cfg.save_map_command(name))
        self._notice('info', 'saving map as %s' % name)
        QTimer.singleShot(4000, self.mission_panel.refresh_maps)

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
            else:
                self.tools_panel.set_bag_state(False, 'could not start rosbag')
        else:
            self.manager.stop('rosbag', int_wait=8.0, term_wait=4.0)
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
    def cycle_theme(self):
        modes = ['system', 'dark', 'light']
        current = self.settings.get('ui', {}).get('theme', 'system')
        index = (modes.index(current) + 1) % len(modes) if current in modes else 0
        mode = modes[index]
        self.settings.setdefault('ui', {})['theme'] = mode
        applied = theme.apply(self.app, mode)
        self._notice('info', 'theme: %s (%s)' % (mode, applied))
        settings.save(self.settings)

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
                self.stop_stack()
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
