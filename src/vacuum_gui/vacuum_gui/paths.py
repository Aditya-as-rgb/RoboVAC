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

"""Where the workspace, maps, models and run folders live.

Plain Python (no ROS, no Qt) so the launch panel and the tests can use it. The
workspace root is found from ``AMENT_PREFIX_PATH`` (``<ws>/install/<pkg>``) or, failing
that, by walking up from this file looking for a ``src`` sibling. ``VACUUM_WS`` wins.
"""

import os
from pathlib import Path

APP_NAME = 'vacuum_gui'


def workspace_root(start=None):
    """Best guess for the colcon workspace root (the folder that holds ``src/``)."""
    env = os.environ.get('VACUUM_WS', '').strip()
    if env:
        return Path(env).expanduser()
    for entry in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        entry = entry.strip()
        if not entry:
            continue
        p = Path(entry)
        # <ws>/install/<pkg>  ->  <ws>
        if p.name != 'install' and p.parent.name == 'install':
            if (p.parent.parent / 'src').is_dir():
                return p.parent.parent
        if p.name == 'install' and (p.parent / 'src').is_dir():
            return p.parent
    here = Path(start or __file__).resolve()
    for parent in [here] + list(here.parents):
        if (parent / 'src').is_dir() and (parent / 'src' / 'vacuum_gui').is_dir():
            return parent
    for parent in [here] + list(here.parents):
        if (parent / 'src').is_dir() and any((parent / 'src').glob('vacuum_*')):
            return parent
    return Path.cwd()


def install_share(pkg, ws=None):
    """Installed share directory of a workspace package (may not exist)."""
    ws = Path(ws or workspace_root())
    return ws / 'install' / pkg / 'share' / pkg


def _first_dir(*candidates):
    for c in candidates:
        if c and Path(c).is_dir():
            return Path(c)
    return Path(candidates[0]) if candidates else Path('.')


def gazebo_share(ws=None):
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'src' / 'vacuum_gazebo', install_share('vacuum_gazebo', ws))


def maps_dir(ws=None):
    """Where saved maps are read from and written to (``<ws>/maps`` first)."""
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'maps', gazebo_share(ws) / 'maps')


def default_map(ws=None):
    return maps_dir(ws) / 'living_room.yaml'


def models_dir(ws=None):
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'models' / 'dirt_detector', ws / 'models')


def detector_model(ws=None):
    return models_dir(ws) / 'dirt_yolo11n.onnx'


def package_dataset_script(ws=None):
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'tools' / 'dirt_dataset', ws) / 'package_dataset.py'


def runs_dir():
    return Path(os.path.expanduser('~/vacuum_runs'))


def dirt_dir():
    return runs_dir() / 'dirt'


def dataset_dir():
    return runs_dir() / 'dataset'


def screenshots_dir():
    return runs_dir() / 'screenshots'


def config_dir():
    return Path(os.path.expanduser('~/.config')) / APP_NAME


def settings_path():
    return config_dir() / 'settings.json'


def layout_path():
    return config_dir() / 'layout.json'


def presets_path():
    return config_dir() / 'presets.json'


def coverage_config(ws=None):
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'src' / 'vacuum_coverage' / 'config',
                      install_share('vacuum_coverage', ws) / 'config') / 'coverage.yaml'


def navigation_config(ws=None):
    ws = Path(ws or workspace_root())
    return _first_dir(ws / 'src' / 'vacuum_bringup' / 'config',
                      install_share('vacuum_bringup', ws) / 'config') / 'navigation.yaml'
