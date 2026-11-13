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

"""Every command line the panel runs, and the options behind them.

No ROS import and no Qt import: this module is unit-tested on its own. The panel shows
``shell_join(cfg.stack_command())`` so what is previewed is exactly what runs.

The GUI owns teleop, detector, coverage mission, dataset recording and RViz as *separate*
managed processes. That is what makes them live-togglable without restarting Gazebo:
``vacuum.launch.py`` is started with ``teleop:=none coverage:=false meter:=false
detector:=false collect_dataset:=false rviz:=false`` and the GUI starts those nodes
itself when the matching switch is on. The camera (``enable_cameras``) is baked into the
URDF at launch time, so it stays a launch-time option - and so does everything the world,
the map, the dirt and the odometry need.
"""

import os
import shlex
from dataclasses import asdict, dataclass, fields
import yaml

TRUTHY = ('1', 'true', 'yes', 'on', 'True')

# Options that are baked into the URDF / world / map when the stack starts. Changing one
# while the stack runs needs a restart; everything else is a separate process.
LAUNCH_TIME_FIELDS = ('slam', 'map', 'world', 'x_pose', 'y_pose', 'odom_source',
                      'headless', 'cameras', 'dirt', 'dirt_seed', 'dirt_scale',
                      'dirt_stubborn', 'dirt_brush', 'dirt_clean', 'dirt_randomize',
                      'dirt_dir', 'nav_delay', 'qt_platform')


def b(value):
    """Python bool -> the 'true'/'false' spelling the launch files declare."""
    return 'true' if bool(value) else 'false'


def tf(value):
    """Python bool -> 'True'/'False' (slam/map choices in vacuum.launch.py)."""
    return 'True' if bool(value) else 'False'


def shell_join(argv):
    return ' '.join(shlex.quote(str(a)) for a in argv)


def expand(path):
    return os.path.expanduser(str(path)) if path else ''


@dataclass
class StackConfig:
    """Everything the launch panel can set. Saved in settings.json as a dict."""

    # --- launch-time, fixed for the lifetime of a run -----------------------
    slam: bool = False                       # False: AMCL on `map`; True: map while driving
    map: str = ''                            # '' = <ws>/maps/living_room.yaml
    world: str = 'living_room.world'
    x_pose: float = -2.0
    y_pose: float = -0.5
    odom_source: str = 'ground_truth'
    headless: bool = True
    cameras: bool = False                    # front floor camera in the URDF
    dirt: int = 0
    dirt_seed: int = 1
    dirt_scale: float = 1.0
    dirt_stubborn: float = 0.15
    dirt_brush: float = 0.20
    dirt_clean: bool = True
    dirt_randomize: bool = False
    dirt_dir: str = '~/vacuum_runs/dirt'
    nav_delay: float = 8.0
    qt_platform: str = 'xcb'                 # Gazebo/RViz through XWayland

    # --- live, each one its own managed process ----------------------------
    teleop: bool = True
    rviz: bool = False
    detector: bool = False
    detector_model: str = ''                 # '' = let vacuum_vision find its model
    coverage: bool = True
    meter: bool = True
    reclean: bool = True                     # re-clean passes on/off (coverage param)
    reclean_source: str = 'map'              # map | vision
    autostart: bool = False
    collect_dataset: bool = False
    dataset_dir: str = '~/vacuum_runs/dataset'
    dataset_max_s: float = 3600.0

    def __post_init__(self):
        self.dirt = int(self.dirt)
        self.dirt_seed = int(self.dirt_seed)
        self.reclean_source = 'vision' if str(self.reclean_source) == 'vision' else 'map'
        self.odom_source = ('robot_wheels' if str(self.odom_source) == 'robot_wheels'
                            else 'ground_truth')
        if self.dirt_clean in (0, 1):
            self.dirt_clean = bool(self.dirt_clean)
        if not str(self.map or '').strip():
            self.map = ''

    # ------------------------------------------------------------- helpers --
    @property
    def camera_needed(self):
        """The camera is in the URDF only if something asks for it at launch."""
        return bool(self.cameras or self.detector or self.collect_dataset
                    or self.reclean_source == 'vision')

    def resolved_map(self, ws_map=None):
        return expand(self.map) if self.map else str(ws_map or '')

    def dataset_path(self):
        return expand(self.dataset_dir)

    def dirt_path(self):
        return expand(self.dirt_dir)

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            return cls()
        names = {f.name for f in fields(cls)}
        clean = {}
        for k, v in data.items():
            if k not in names:
                continue
            current = getattr(cls(), k)
            try:
                if isinstance(current, bool):
                    clean[k] = str(v).lower() in TRUTHY if isinstance(v, str) else bool(v)
                elif isinstance(current, int):
                    clean[k] = int(float(v))
                elif isinstance(current, float):
                    clean[k] = float(v)
                else:
                    clean[k] = str(v)
            except (TypeError, ValueError):
                continue
        return cls(**clean)

    def copy(self):
        return StackConfig.from_dict(self.to_dict())

    # ------------------------------------------------------------ commands --
    def launch_args(self, ws_map=None):
        """Arguments for vacuum_bringup/vacuum.launch.py (sim + Nav2 only)."""
        args = [
            'slam:=' + tf(self.slam),
            'world:=' + str(self.world),
            'x_pose:=%g' % float(self.x_pose),
            'y_pose:=%g' % float(self.y_pose),
            'odom_source:=' + self.odom_source,
            'headless:=' + b(self.headless),
            'cameras:=' + b(self.camera_needed),
            # The GUI runs these itself so they can be switched without a restart.
            'teleop:=none',
            'detector:=' + b(False),
            'collect_dataset:=' + b(False),
            'coverage:=' + b(False),
            'meter:=' + b(False),
            'rviz:=' + b(False),
            'nav_delay:=%g' % float(self.nav_delay),
            'qt_platform:=' + str(self.qt_platform or ''),
            'dirt:=%d' % self.dirt,
            'dirt_seed:=%d' % self.dirt_seed,
            'dirt_scale:=%g' % float(self.dirt_scale),
            'dirt_stubborn:=%g' % float(self.dirt_stubborn),
            'dirt_brush:=%g' % float(self.dirt_brush),
            'dirt_clean:=' + b(self.dirt_clean),
            'dirt_randomize:=' + b(self.dirt_randomize),
            'dirt_dir:=' + expand(self.dirt_dir),
        ]
        map_path = self.resolved_map(ws_map)
        if map_path:
            args.insert(1, 'map:=' + map_path)
        return args

    def stack_command(self, ws_map=None):
        return ['ros2', 'launch', 'vacuum_bringup', 'vacuum.launch.py'] + self.launch_args(ws_map)

    def teleop_command(self):
        return ['ros2', 'launch', 'vacuum_teleop', 'teleop_joy.launch.py']

    def detector_command(self):
        cmd = ['ros2', 'launch', 'vacuum_vision', 'detector.launch.py']
        if self.detector_model:
            cmd.append('model:=' + expand(self.detector_model))
        return cmd

    def coverage_command(self, ws_map=None, params_file=None, autostart=None):
        cmd = ['ros2', 'launch', 'vacuum_coverage', 'coverage.launch.py',
               'autostart:=' + b(self.autostart if autostart is None else autostart),
               'meter:=' + b(self.meter),
               'reclean_source:=' + self.reclean_source]
        if params_file:
            cmd.append('params_file:=' + expand(params_file))
        return cmd

    def dataset_command(self):
        """Standalone recorder, so recording can start and stop without a restart."""
        layout = os.path.join(self.dirt_path(), 'current_layout.json')
        return ['ros2', 'run', 'vacuum_gazebo', 'dirt_dataset.py', '--ros-args',
                '-p', 'use_sim_time:=true',
                '-p', 'layout:=' + layout,
                '-p', 'out_dir:=' + self.dataset_path(),
                '-p', 'max_duration_s:=%g' % float(self.dataset_max_s)]

    def rviz_command(self, rviz_config=None):
        cmd = ['ros2', 'run', 'rviz2', 'rviz2']
        if rviz_config:
            cmd += ['-d', expand(rviz_config)]
        return cmd

    def gazebo_gui_command(self):
        """Only for a stack that was started with headless:=false (gz GUI is part of it)."""
        return ['gz', 'sim', '-g']

    def save_map_command(self, name):
        name = (name or '').strip() or 'map'
        return ['ros2', 'run', 'vacuum_bringup', 'save_map.sh', name]

    def reset_dirt_command(self, ws_map=None):
        """Regeneration needs a launch-time seed, so this is the restart flavour."""
        return self.stack_command(ws_map)


def restart_fields(current, applied):
    """Names of the launch-time options that differ between two config dicts."""
    current = current or {}
    applied = applied or {}
    return [name for name in LAUNCH_TIME_FIELDS
            if current.get(name) != applied.get(name)]


def detector_param_command(values, node='/dirt_detector'):
    """A list of ``ros2 param set`` argv lists, one per parameter."""
    out = []
    for key, value in values.items():
        if isinstance(value, bool):
            text = b(value)
        elif isinstance(value, float):
            text = '%g' % value
        else:
            text = str(value)
        out.append(['ros2', 'param', 'set', str(node), str(key), text])
    return out


def package_dataset_command(src_dir, out_dir, classes='kind', zip_it=True, ws=None):
    from . import paths
    script = paths.package_dataset_script(ws)
    cmd = ['python3', str(script), expand(src_dir), expand(out_dir), '--classes', str(classes)]
    if zip_it:
        cmd.append('--zip')
    return cmd


def child_env(extra=None, path_prefix=('/usr/bin', '/bin'), qt_platform=None):
    """Environment for the managed children.

    A venv python on PATH shadows ``/usr/bin/python3`` and the ROS nodes die on
    ``import numpy`` / ``import rclpy``; every child therefore gets the system
    directories in front. Qt apps (Gazebo GUI, RViz) go through XWayland.
    """
    env = dict(os.environ)
    parts = [p for p in path_prefix if p]
    current = [p for p in env.get('PATH', '').split(os.pathsep) if p and p not in parts]
    env['PATH'] = os.pathsep.join(parts + current)
    env['PYTHONUNBUFFERED'] = '1'
    if qt_platform:
        env['QT_QPA_PLATFORM'] = qt_platform
    if extra:
        env.update({str(k): str(v) for k, v in extra.items()})
    return env


def coverage_params_with_overrides(base_yaml, overrides, out_path):
    """Write the coverage config with ``coverage_mission`` parameters overridden.

    ``coverage.launch.py`` hands one params file to both nodes, so the base file is
    copied whole and only the requested keys change (the mission needs the rest).
    Returns ``out_path``.
    """
    with open(expand(base_yaml)) as f:
        data = yaml.safe_load(f) or {}
    mission = data.setdefault('coverage_mission', {}).setdefault('ros__parameters', {})
    for key, value in (overrides or {}).items():
        mission[key] = value
    data.setdefault('coverage_meter', {}).setdefault('ros__parameters', {})
    os.makedirs(os.path.dirname(expand(out_path)) or '.', exist_ok=True)
    with open(expand(out_path), 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path


def stacked_command(cfg, ws_map=None):
    """The human-facing one-liner for the sim launch, as the panel shows it."""
    return shell_join(cfg.stack_command(ws_map))
