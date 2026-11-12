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

"""Pre-flight checks before Start, each with a fix hint.

Pure Python: subprocess-free where it can be, so the tests can point it at a temporary
workspace and assert which checks fail. The panel shows the failing rows and refuses to
start on an ``error`` level row.
"""

import glob
import importlib.util
import os
import shutil
from dataclasses import dataclass
from pathlib import Path

from . import paths
from .launch_manager import find_leftovers, port_free

ERROR, WARN, INFO = 'error', 'warn', 'info'


@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ''
    hint: str = ''
    level: str = ERROR

    @property
    def state(self):
        return 'ok' if self.ok else self.level


def _packages_in(ws):
    found = set()
    for prefix in (Path(ws) / 'install', Path('/opt/ros/jazzy')):
        if not prefix.is_dir():
            continue
        for entry in prefix.iterdir():
            pkg_xml = entry / 'share' / entry.name / 'package.xml'
            if pkg_xml.is_file():
                found.add(entry.name)
    return found


def check_ros_environment():
    if not os.environ.get('AMENT_PREFIX_PATH'):
        return Check('ROS environment', False, 'AMENT_PREFIX_PATH is empty',
                     'source /opt/ros/jazzy/setup.zsh and then install/setup.zsh '
                     'before starting the GUI (the .desktop launcher does it for you)')
    if not shutil.which('ros2'):
        return Check('ROS environment', False, 'ros2 not on PATH',
                     'source /opt/ros/jazzy/setup.zsh; if it is a venv shell, run the GUI '
                     'with PATH=/usr/bin:/bin:$PATH')
    return Check('ROS environment', True, 'ROS_DISTRO=%s' % os.environ.get('ROS_DISTRO', '?'))


def check_packages(ws=None):
    ws = Path(ws or paths.workspace_root())
    wanted = ['vacuum_bringup', 'vacuum_coverage', 'vacuum_vision', 'vacuum_gazebo',
              'vacuum_description', 'vacuum_teleop']
    found = _packages_in(ws)
    missing = [p for p in wanted if p not in found]
    if missing:
        return Check('Workspace packages', False, 'missing: ' + ', '.join(missing),
                     'cd %s && colcon build --symlink-install && source install/setup.zsh' % ws)
    if 'vacuum_gui' not in found:
        return Check('Workspace packages', True, 'all sim packages found (vacuum_gui not '
                                                 'installed yet - running from source)',
                     level=INFO)
    return Check('Workspace packages', True, 'all packages found in %s' % ws)


def check_python_deps():
    problems = []
    try:
        import numpy
        if int(str(numpy.__version__).split('.')[0]) >= 2:
            problems.append('numpy %s (onnxruntime needs <2)' % numpy.__version__)
    except ImportError:
        problems.append('numpy missing')
    if importlib.util.find_spec('onnxruntime') is None:
        problems.append('onnxruntime missing')
    if importlib.util.find_spec('PySide6') is None:
        problems.append('PySide6 missing')
    if importlib.util.find_spec('rclpy') is None:
        problems.append('rclpy missing (ROS not sourced in this shell)')
    if problems:
        return Check('Python dependencies', False, '; '.join(problems),
                     '/usr/bin/python3 -m pip install --user --break-system-packages '
                     'PySide6 onnxruntime "numpy<2"  (then source /opt/ros/jazzy/setup.zsh)')
    return Check('Python dependencies', True, 'numpy, onnxruntime, PySide6, rclpy importable')


def check_detector_model(cfg, ws=None):
    if not (cfg.detector or cfg.reclean_source == 'vision'):
        return Check('Detector model', True, 'not needed for this configuration', level=INFO)
    if cfg.detector_model:
        path = Path(os.path.expanduser(cfg.detector_model))
    else:
        path = paths.detector_model(ws)
    if path.is_file():
        return Check('Detector model', True, '%s (%.1f MB)' % (path, path.stat().st_size / 1e6))
    return Check('Detector model', False, 'not found: %s' % path,
                 'put dirt_yolo11n.onnx in <ws>/models/dirt_detector/ (the repo ships it) '
                 'or set the model path in the launch panel')


def check_map(cfg, ws=None):
    if cfg.slam:
        return Check('Map file', True, 'slam:=True builds the map while driving', level=INFO)
    path = Path(cfg.resolved_map(paths.default_map(ws)))
    if path.is_file():
        return Check('Map file', True, str(path))
    return Check('Map file', False, 'not found: %s' % path,
                 'save one with the Mission tab (ros2 run vacuum_bringup save_map.sh <name>), '
                 'or pick another map, or switch to slam:=True')


def check_world(cfg, ws=None):
    world = Path(paths.gazebo_share(ws)) / 'worlds' / str(cfg.world)
    if world.is_file():
        return Check('World file', True, str(world))
    known = [p.name for p in (Path(paths.gazebo_share(ws)) / 'worlds').glob('*.world')]
    return Check('World file', False, 'not found: %s' % world,
                 'worlds in vacuum_gazebo/worlds: ' + (', '.join(known) or 'none'))


def check_dirt(cfg, ws=None):
    if cfg.dirt <= 0:
        return Check('Dirt layout', True, 'no dirt (dirt:=0)', level=INFO)
    if cfg.slam and not cfg.map:
        return Check('Dirt layout', False, 'dirt:=%d needs a map of the world to place patches '
                                           'on reachable floor' % cfg.dirt,
                     'dirt_map defaults to `map`, which slam:=True ignores; pass a saved map '
                     'in the launch panel or use slam:=False')
    path = Path(cfg.resolved_map(paths.default_map(ws)))
    if not path.is_file():
        return Check('Dirt layout', False, 'dirt needs %s' % path, 'see the map check above')
    return Check('Dirt layout', True, '%d patches, seed %d' % (cfg.dirt, cfg.dirt_seed))


def check_camera(cfg):
    if cfg.camera_needed:
        return Check('Camera', True, 'camera enabled in the URDF (adds ~10 %% GPU load)',
                     level=INFO)
    return Check('Camera', True, 'camera off: the Camera panel will show "no signal"',
                 level=INFO)


def check_gamepad(cfg):
    devices = sorted(glob.glob('/dev/input/js*'))
    if not cfg.teleop:
        return Check('Gamepad', True, 'teleop off', level=INFO)
    if devices:
        return Check('Gamepad', True, 'found ' + ', '.join(devices))
    return Check('Gamepad', False, 'no /dev/input/js* device',
                 'pair the pad (or use the on-screen teleop pad in the Robot tab); '
                 'check with: ros2 run joy joy_enumerate_devices', level=WARN)


def check_leftovers():
    left = find_leftovers()
    if not left:
        return Check('Stale processes', True, 'no leftover gz / bridge / Nav2 / RViz')
    pids = ', '.join('%s (pid %d)' % (l.name, l.pid) for l in left[:4])
    hint = ('a previous run is still alive: a second gz cannot start on one world, and a '
            'leftover ros_gz_bridge publishes a second /clock which makes AMCL lose the '
            'TF chain. Use "Kill leftovers".')
    return Check('Stale processes', False, '%d process(es): %s' % (len(left), pids), hint)


def check_ports(check=True):
    if not check:
        return Check('Ports', True, 'skipped', level=INFO)
    busy = [p for p in (11345,) if not port_free(p)]
    if busy:
        return Check('Ports', False, 'TCP %s busy' % ', '.join(str(p) for p in busy),
                     'another Gazebo is probably running; kill leftovers first')
    return Check('Ports', True, 'gz transport port 11345 free')


def check_writable(path, label):
    target = Path(os.path.expanduser(str(path)))
    try:
        target.mkdir(parents=True, exist_ok=True)
        probe = target / '.vacuum_gui_probe'
        probe.write_text('ok')
        probe.unlink()
        return Check(label, True, str(target))
    except OSError as exc:
        return Check(label, False, 'cannot write %s: %s' % (target, exc),
                     'check permissions, or point the path somewhere under $HOME')


def check_dataset_tools(ws=None):
    script = paths.package_dataset_script(ws)
    if script.is_file():
        return Check('Dataset tools', True, str(script), level=INFO)
    return Check('Dataset tools', False, str(script) + ' missing',
                 'tools/dirt_dataset/package_dataset.py is part of the repo; '
                 'pull it or skip packaging', level=WARN)


def run_checks(cfg, ws=None, check_port=True, gamepad=True):
    """All checks, in the order the panel shows them."""
    ws = ws or paths.workspace_root()
    checks = [
        check_ros_environment(),
        check_python_deps(),
        check_packages(ws),
        check_map(cfg, ws),
        check_world(cfg, ws),
        check_dirt(cfg, ws),
        check_detector_model(cfg, ws),
        check_camera(cfg),
        check_leftovers(),
        check_ports(check_port),
        check_writable(cfg.dirt_path(), 'Dirt folder'),
        check_writable(cfg.dataset_path(), 'Dataset folder'),
        check_dataset_tools(ws),
    ]
    if gamepad:
        checks.append(check_gamepad(cfg))
    return checks


def blocking(checks):
    return [c for c in checks if not c.ok and c.level == ERROR]


def summary(checks):
    bad = [c for c in checks if not c.ok]
    if not bad:
        return True, 'all %d checks passed' % len(checks)
    return False, '%d check(s) need attention: %s' % (
        len(bad), ', '.join(c.name for c in bad))
