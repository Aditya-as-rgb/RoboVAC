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

"""Command-line building: the preview the panel shows must be what runs."""

import os
import sys

import yaml

from vacuum_gui import commands
from vacuum_gui.commands import StackConfig


def test_stack_command_forces_gui_managed_parts_off():
    cfg = StackConfig(teleop=True, coverage=True, meter=True, detector=True,
                      collect_dataset=True, rviz=True)
    cmd = cfg.stack_command('/tmp/living_room.yaml')
    assert cmd[:4] == ['ros2', 'launch', 'vacuum_bringup', 'vacuum.launch.py']
    assert 'teleop:=none' in cmd
    assert 'coverage:=false' in cmd
    assert 'meter:=false' in cmd
    assert 'detector:=false' in cmd
    assert 'collect_dataset:=false' in cmd
    assert 'rviz:=false' in cmd
    assert 'cameras:=true' in cmd          # detector/collect need the camera in the URDF


def test_camera_needed_follows_detector_dataset_and_vision_reclean():
    assert StackConfig().camera_needed is False
    assert StackConfig(cameras=True).camera_needed is True
    assert StackConfig(detector=True).camera_needed is True
    assert StackConfig(collect_dataset=True).camera_needed is True
    assert StackConfig(reclean_source='vision').camera_needed is True
    assert StackConfig(reclean_source='map').camera_needed is False


def test_launch_args_spell_bools_the_way_the_launch_files_declare():
    cfg = StackConfig(slam=True, headless=False, dirt_clean=False, dirt=3, dirt_seed=7)
    args = cfg.launch_args()
    assert 'slam:=True' in args            # choices are 'True'/'False' in vacuum.launch.py
    assert 'headless:=false' in args
    assert 'dirt_clean:=false' in args
    assert 'dirt:=3' in args and 'dirt_seed:=7' in args


def test_map_is_only_passed_when_set():
    with_map = StackConfig(map='~/maps/room.yaml').launch_args()
    assert 'map:=' + os.path.expanduser('~/maps/room.yaml') in with_map
    without = StackConfig(map='').launch_args()
    assert all(not a.startswith('map:=') for a in without)
    # the workspace default map is used when the user has not picked one
    assert 'map:=/tmp/x.yaml' in StackConfig(map='').launch_args('/tmp/x.yaml')


def test_shell_join_quotes_arguments_with_spaces():
    cfg = StackConfig(map='/home/a b/map.yaml')
    text = commands.shell_join(cfg.stack_command())
    assert "'map:=/home/a b/map.yaml'" in text
    assert text.startswith('ros2 launch vacuum_bringup vacuum.launch.py')


def test_coverage_command_carries_live_switches_and_optional_params():
    cfg = StackConfig(meter=False, autostart=True, reclean_source='vision')
    cmd = cfg.coverage_command(params_file='/tmp/c.yaml')
    assert cmd[:4] == ['ros2', 'launch', 'vacuum_coverage', 'coverage.launch.py']
    assert 'meter:=false' in cmd
    assert 'reclean_source:=vision' in cmd
    assert 'autostart:=true' in cmd
    assert 'params_file:=/tmp/c.yaml' in cmd


def test_dataset_command_is_standalone_so_it_can_start_and_stop_live():
    cfg = StackConfig(dirt_dir='~/vacuum_runs/dirt', dataset_dir='~/vacuum_runs/dataset')
    cmd = cfg.dataset_command()
    assert cmd[:4] == ['ros2', 'run', 'vacuum_gazebo', 'dirt_dataset.py']
    assert '-p' in cmd
    assert 'layout:=' + os.path.expanduser('~/vacuum_runs/dirt/current_layout.json') in cmd
    assert 'out_dir:=' + os.path.expanduser('~/vacuum_runs/dataset') in cmd


def test_detector_param_commands_match_ros2_param_set():
    cmds = commands.detector_param_command({'conf': 0.3, 'min_hits': 4, 'debug': False})
    assert cmds[0] == ['ros2', 'param', 'set', '/dirt_detector', 'conf', '0.3']
    assert cmds[1][-2:] == ['min_hits', '4']
    assert cmds[2][-1] == 'false'


def test_save_map_uses_the_workspace_wrapper_and_a_bare_name():
    cmd = StackConfig().save_map_command(' demo_room ')
    assert cmd == ['ros2', 'run', 'vacuum_bringup', 'save_map.sh', 'demo_room']
    assert StackConfig().save_map_command('')[-1] == 'map'


def test_child_env_puts_the_system_python_first_and_sets_qt_platform():
    env = commands.child_env({'ROS_DOMAIN_ID': '4'}, qt_platform='xcb')
    assert env['PATH'].split(os.pathsep)[:2] == ['/usr/bin', '/bin']
    assert env['QT_QPA_PLATFORM'] == 'xcb'
    assert env['ROS_DOMAIN_ID'] == '4'
    assert env['PYTHONUNBUFFERED'] == '1'


def test_stack_config_round_trip_and_coercion():
    cfg = StackConfig(dirt=5, dirt_scale=1.5)
    data = cfg.to_dict()
    data['dirt'] = '9'
    data['dirt_scale'] = '2'
    data['headless'] = 'false'
    data['whatever'] = 1
    back = StackConfig.from_dict(data)
    assert back.dirt == 9 and back.dirt_scale == 2.0
    assert back.headless is False
    assert not hasattr(back, 'whatever')


def test_coverage_params_override_keeps_the_rest_of_the_file(tmp_path):
    base = tmp_path / 'coverage.yaml'
    base.write_text(yaml.safe_dump({
        'coverage_mission': {'ros__parameters': {'lane_spacing': 0.4, 'reclean': True,
                                                 'report_dir': '~/vacuum_runs'}},
        'coverage_meter': {'ros__parameters': {'rate': 10.0}}}))
    out = tmp_path / 'sub' / 'params.yaml'
    commands.coverage_params_with_overrides(base, {'reclean': False}, out)
    data = yaml.safe_load(out.read_text())
    mission = data['coverage_mission']['ros__parameters']
    assert mission['reclean'] is False
    assert mission['lane_spacing'] == 0.4
    assert data['coverage_meter']['ros__parameters']['rate'] == 10.0


def test_restart_fields_lists_only_launch_time_options():
    base = StackConfig(world='living_room.world', dirt=3, teleop=True, coverage=True)
    current = base.to_dict()
    current['world'] = 'kitchen.world'
    current['dirt'] = 5
    current['teleop'] = False          # live switch: never needs a restart
    current['coverage'] = False
    current['unknown_future_key'] = 1
    assert commands.restart_fields(current, base.to_dict()) == ['world', 'dirt']
    assert commands.restart_fields(base.to_dict(), base.to_dict()) == []
    assert commands.restart_fields({}, base.to_dict()) == [
        name for name in commands.LAUNCH_TIME_FIELDS if base.to_dict().get(name) is not None]


def test_stack_config_rejects_unknown_reclean_source():
    assert StackConfig(reclean_source='teleport').reclean_source == 'map'
    assert StackConfig(odom_source='magic').odom_source == 'ground_truth'


if __name__ == '__main__':
    sys.exit('run with: python3 -m pytest test -q')
