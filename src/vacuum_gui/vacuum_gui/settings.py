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

"""Settings and presets, as plain JSON under ``~/.config/vacuum_gui/``.

Nothing is ever written inside the repository. Missing keys fall back to the defaults,
so an old settings file stays usable after an upgrade.
"""

import copy
import json
import os

from . import paths
from .commands import StackConfig

VIEW_DEFAULTS = {
    'show_scan': True,
    'show_plan_global': True,
    'show_plan_local': False,
    'show_plan_coverage': True,
    'show_cleaned': True,
    'show_dirt_detected': True,
    'show_dirt_truth': False,
    'show_particles': False,
    'show_costmaps': False,
    'camera_fps': 10,
    'map_fps': 2,
    'teleop_linear': 0.25,
    'teleop_angular': 1.0,
    'map_mode': 'goal',           # goal | pose | inspect
    'rosbag_topics': ['/scan', '/odom', '/tf', '/tf_static', '/camera/image', '/dirt/detections'],
    'recording_seconds': 20,
    'recording_fps': 2,
}

UI_DEFAULTS = {
    'theme': 'system',            # system | dark | light
    'start_page': 'map',
    'confirm_close_running': True,
    'help_seen': False,
    'preflight_warnings': True,   # False: skip the "pre-flight warnings" confirmation
    'qt_platform': '',            # '' = let Qt decide; 'xcb' forces XWayland
}

DETECTOR_DEFAULTS = {
    'conf': 0.25,
    'iou': 0.5,
    'max_range': 1.5,
    'min_hits': 3,
    'clear_misses': 10,
}

DEFAULTS = {
    'stack': StackConfig().to_dict(),
    'view': dict(VIEW_DEFAULTS),
    'ui': dict(UI_DEFAULTS),
    'detector': dict(DETECTOR_DEFAULTS),
    'experiments': {
        'seeds': '1,2,3',
        'dirt': 12,
        'dirt_scale': 1.0,
        'dirt_stubborn': 0.15,
        'modes': ['none', 'map', 'vision'],
        'timeout_min': 30,
    },
}


def default_presets():
    """One-click configurations for the demos the project actually does."""
    demo = StackConfig(map='', slam=False, headless=True, cameras=True, detector=True,
                       coverage=True, meter=True, reclean=True, reclean_source='map',
                       dirt=12, dirt_seed=3, dirt_scale=1.5, dirt_clean=True,
                       autostart=False, teleop=True, rviz=False, collect_dataset=False)
    mapping = StackConfig(slam=True, headless=True, cameras=False, detector=False,
                          coverage=False, meter=False, dirt=0, autostart=False,
                          teleop=True, rviz=False)
    dataset = StackConfig(slam=False, headless=True, cameras=True, detector=False,
                          collect_dataset=True, dataset_max_s=3600.0,
                          dirt=15, dirt_seed=100, dirt_scale=1.0, dirt_clean=False,
                          dirt_randomize=True, coverage=True, meter=True, autostart=True,
                          teleop=False, rviz=False)
    vision = StackConfig(slam=False, headless=True, cameras=True, detector=True,
                         coverage=False, meter=False, dirt=8, dirt_seed=5,
                         dirt_scale=2.0, dirt_clean=False, autostart=False,
                         teleop=False, rviz=False)
    return {
        'Demo (localise + coverage + detector)': {
            'stack': demo.to_dict(),
            'view': dict(VIEW_DEFAULTS),
            'notes': 'Saved map + AMCL, dirt scattered, camera and detector on, '
                     'coverage plan ready to start. Headless: about half the GPU load of a '
                     'Gazebo window.'},
        'Mapping': {
            'stack': mapping.to_dict(),
            'view': dict(VIEW_DEFAULTS, show_scan=True, show_dirt_detected=False),
            'notes': 'slam:=True, no coverage: drive with the gamepad (or the on-screen pad) '
                     'and save the map from the Mission tab when the room looks complete.'},
        'Dataset collection (headless)': {
            'stack': dataset.to_dict(),
            'view': dict(VIEW_DEFAULTS, show_dirt_detected=False),
            'notes': 'Dirt stays on the floor (dirt_clean:=false), floor colour and light are '
                     'randomised per seed, recording runs while the mission cleans.'},
        'Vision test': {
            'stack': vision.to_dict(),
            'view': dict(VIEW_DEFAULTS, show_dirt_truth=True),
            'notes': 'Detector only, no mission: drive with the pad and compare the camera boxes '
                     '(blue) with the ground-truth dirt (orange). Big patches, close range.'},
        'Hardware later': {
            'stack': StackConfig(slam=False, headless=True, cameras=True, detector=True,
                                 coverage=True, meter=True, dirt=0, teleop=False).to_dict(),
            'view': dict(VIEW_DEFAULTS),
            'notes': 'Placeholder for the real robot: the panels talk to the backend interface '
                     'in vacuum_gui/backend.py, so a serial/micro-ROS backend can replace the '
                     'Gazebo one without touching the widgets.'},
    }


def _merge(base, override):
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def load(path=None):
    path = path or paths.settings_path()
    data = {}
    try:
        with open(path) as f:
            data = json.load(f) or {}
    except (OSError, ValueError):
        data = {}
    merged = _merge(DEFAULTS, data)
    merged = {key: merged[key] for key in DEFAULTS}
    merged['stack'] = StackConfig.from_dict(merged.get('stack')).to_dict()
    from . import theme
    merged['ui']['theme'] = theme.normalize(merged['ui'].get('theme'))
    return merged


def save(settings, path=None):
    path = path or paths.settings_path()
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    tmp = str(path) + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(settings, f, indent=2, sort_keys=False)
    os.replace(tmp, path)
    return path


def load_presets(path=None):
    path = path or paths.presets_path()
    data = {}
    try:
        with open(path) as f:
            data = json.load(f) or {}
    except (OSError, ValueError):
        data = {}
    merged = default_presets()
    for name, preset in data.items():
        if isinstance(preset, dict):
            merged[name] = _merge(merged.get(name, {'stack': DEFAULTS['stack'],
                                                     'view': VIEW_DEFAULTS, 'notes': ''}), preset)
    return merged


def save_presets(presets, path=None):
    path = path or paths.presets_path()
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(presets, f, indent=2)
    return path


def custom_preset(name, cfg, view=None, notes=''):
    return {str(name): {'stack': cfg.to_dict(), 'view': dict(view or VIEW_DEFAULTS),
                        'notes': notes}}


def load_layout(path=None):
    path = path or paths.layout_path()
    try:
        with open(path) as f:
            data = json.load(f) or {}
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_layout(geometry_hex, state_hex, path=None):
    path = path or paths.layout_path()
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump({'geometry': geometry_hex, 'state': state_hex}, f)
    return path
