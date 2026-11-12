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

"""Entry point: Qt application, theme, backend and the main window."""

import argparse
import os
import signal
import sys

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QApplication, QMessageBox


def _qt_platform(args, ui_settings):
    """Wayland and X11 both work with the default platform plugin.

    Native Wayland is the default on a Wayland session. ``--xcb`` (or the stored
    ui.qt_platform setting) forces XWayland, which is what the Gazebo and RViz windows
    need - the panel itself does not, but a machine with a broken Qt Wayland plugin does.
    ``QT_QPA_PLATFORM`` set in the environment always wins, so nothing here surprises a
    user who configured it.
    """
    if os.environ.get('QT_QPA_PLATFORM'):
        return None
    if args.xcb:
        return 'xcb'
    stored = str(ui_settings.get('qt_platform') or '').strip()
    if stored:
        return stored
    if args.offscreen:
        return 'offscreen'
    return None


def build_parser():
    parser = argparse.ArgumentParser(
        prog='vacuum_gui',
        description='Control panel for the RoboVAC ROS 2 simulation.')
    parser.add_argument('--xcb', action='store_true',
                        help='force Qt through XWayland (QT_QPA_PLATFORM=xcb)')
    parser.add_argument('--theme', choices=['system', 'dark', 'light'],
                        help='override the stored theme for this run')
    parser.add_argument('--no-ros', action='store_true',
                        help='open the window without starting the ROS backend')
    parser.add_argument('--offscreen', action='store_true',
                        help='render without a display (used by the smoke test)')
    parser.add_argument('--reset-layout', action='store_true',
                        help='ignore the saved dock layout')
    parser.add_argument('--screenshot', metavar='PATH',
                        help='with --offscreen: save a window screenshot and exit')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    from . import settings, theme
    data = settings.load()
    platform = _qt_platform(args, data.get('ui', {}))
    if platform:
        os.environ['QT_QPA_PLATFORM'] = platform
    if args.reset_layout:
        try:
            os.remove(settings.paths.layout_path())
        except OSError:
            pass

    app = QApplication(sys.argv[:1])
    app.setApplicationName('vacuum_gui')
    app.setApplicationDisplayName('RoboVAC Control Panel')
    app.setDesktopFileName('vacuum_gui')
    app.setOrganizationName('RoboVAC')
    mode = (args.theme or os.environ.get('VACUUM_GUI_THEME')
            or data.get('ui', {}).get('theme', 'system'))
    applied = theme.apply(app, mode)
    theme.connect_system_theme(
        app, lambda: theme.apply(app, data.get('ui', {}).get('theme', 'system')))

    backend = None
    no_ros = args.no_ros or os.environ.get('VACUUM_GUI_NO_ROS', 'false') == 'true'
    if not no_ros:
        try:
            from .ros_backend import RosBackend
            backend = RosBackend()
        except Exception as exc:      # noqa: BLE001 - missing rclpy or unsourced ROS
            backend = None
            reason = ('ROS not available (%s). Source /opt/ros/jazzy/setup.zsh and '
                      'install/setup.zsh, then reopen.' % exc)
    if backend is None:
        from .backend import NullBackend
        backend = NullBackend('ROS not available: %s'
                              % ('started with --no-ros' if args.no_ros
                                 else 'rclpy is not importable in this Python'))

    from .main_window import MainWindow
    window = MainWindow(app, data, backend)
    window.showMaximized() if not args.offscreen else window.show()
    if applied != mode:
        window.statusBar().showMessage('theme: %s (desktop)' % applied, 4000)

    if args.screenshot:
        def snap():
            window.grab().save(args.screenshot, 'PNG')
            app.quit()
        QTimer.singleShot(1500, snap)

    # Ctrl+C in the terminal that started it should close the window cleanly
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.start(300)
    timer.timeout.connect(lambda: None)

    try:
        return app.exec()
    finally:
        try:
            backend.stop()
        except Exception:      # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
