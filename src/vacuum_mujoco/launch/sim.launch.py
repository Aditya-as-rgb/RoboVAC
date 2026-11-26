"""Run RoboVAC with MuJoCo physics and the Jazzy ros2_control system."""

import importlib.util
import json
import math
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _numbers(*values):
    return ' '.join(str(value) for value in values)


def _add_dirt(scene, gazebo, world, spawn, arg):
    """Build collision-free MJCF visuals from the same patches used by Gazebo."""
    script = gazebo / 'scripts' / 'dirt_world.py'
    spec = importlib.util.spec_from_file_location('dirt_world', script)
    dirt_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dirt_world)
    map_file = Path(arg('dirt_map') or gazebo / 'maps' / f'{Path(world).stem}.yaml')
    map_file = map_file.expanduser().absolute()
    seed = int(arg('dirt_seed'))
    scale = float(arg('dirt_scale'))
    patches = dirt_world.scatter_dirt(
        dirt_world.load_map(str(map_file)), int(arg('dirt')), seed, spawn,
        scale=scale, stubborn_fraction=float(arg('dirt_stubborn')))
    environment = dirt_world.environment_variation(seed) if arg('dirt_randomize') == 'true' else None

    worldbody = scene.find('worldbody')
    if environment:
        worldbody.find('geom[@name="floor"]').set(
            'rgba', _numbers(*environment['floor_color'], 1))
        worldbody.find('light[@name="room_light"]').set(
            'ambient', _numbers(*environment['ambient']))
    for patch in patches:
        # The mocap body's only purpose is to hold visuals and allow the plugin to hide them.
        body = ET.SubElement(worldbody, 'body', name=patch['name'], mocap='true',
                             pos=_numbers(patch['x'], patch['y'], 0.0021))
        for shape in patch['shapes']:
            kind = shape['shape']
            if kind == 'box':
                size = (shape['a'] / 2, shape['b'] / 2, shape['h'] / 2)
                height = size[2]
            elif kind == 'sphere':
                size = (shape['r'],)
                height = shape['r'] * .6
            else:
                size = (shape['r'], shape['h'] / 2)
                height = size[1]
            ET.SubElement(body, 'geom', {
                'class': 'dirt', 'type': kind, 'size': _numbers(*size),
                'pos': _numbers(shape['dx'], shape['dy'], height),
                'euler': _numbers(0, 0, shape.get('yaw', 0.)),
                'rgba': _numbers(*shape['color'], 1. - shape.get('transparency', 0.)),
            })

    out = Path(arg('dirt_dir')).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    layout = {
        'world': world, 'map': str(map_file), 'seed': seed, 'count': len(patches),
        'spawn': list(spawn), 'scale': scale, 'frame': 'world == map',
        'environment': environment,
        'patches': [{key: patch[key] for key in dirt_world.LAYOUT_KEYS} for patch in patches],
    }
    stem = f'{Path(world).stem}_dirt{len(patches)}_seed{seed}'
    serialized = json.dumps(layout, indent=2)
    (out / f'{stem}.json').write_text(serialized)
    (out / 'current_layout.json').write_text(serialized)


def _launch(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('vacuum_mujoco'))
    extension = Path(get_package_prefix('vacuum_mujoco')) / 'lib/libvacuum_mujoco_sensors.so'
    if not extension.is_file():
        raise RuntimeError('Install ros-jazzy-mujoco-ros2-control, ros-jazzy-diff-drive-controller '
                           'and ros-jazzy-joint-state-broadcaster, then rebuild vacuum_mujoco.')
    gazebo = Path(get_package_share_directory('vacuum_gazebo'))
    world = arg('world')
    world_path = share / 'mjcf' / f'{Path(world).stem}.xml'
    if world not in ('living_room.world', 'living_room.xml'):
        raise ValueError(f'MuJoCo supports living_room.world, got {world!r}')
    spawn = (float(arg('x_pose')), float(arg('y_pose')))
    if not all(math.isfinite(value) for value in spawn):
        raise ValueError('Both spawn coordinates must be finite.')
    description = Path(get_package_share_directory('vacuum_description'))
    urdf = ET.fromstring(xacro.process_file(
        str(description / 'urdf/robot.urdf.xacro')).toxml())
    for gazebo_element in urdf.findall('gazebo'):
        urdf.remove(gazebo_element)

    robot = ET.parse(share / 'mjcf/robot.xml').getroot()
    scene = ET.parse(world_path).getroot()
    robot.find('worldbody/body[@name="vacuum"]').set('pos', _numbers(*spawn, 0))
    if arg('enable_cameras') != 'true':
        body = robot.find('worldbody/body[@name="vacuum"]')
        body.remove(body.find('camera'))
    if int(arg('dirt')) > 0:
        _add_dirt(scene, gazebo, world, spawn, arg)
    robot.find('worldbody').extend(scene.find('worldbody'))

    temp = tempfile.TemporaryDirectory(prefix='vacuum_mujoco_')
    model_path = Path(temp.name) / 'scene.xml'
    ET.ElementTree(robot).write(model_path, encoding='utf-8')
    truth = arg('odom_source') == 'ground_truth'
    config = yaml.safe_load((share / 'config/controllers.yaml').read_text())
    config['vacuum_drive']['ros__parameters']['enable_odom_tf'] = not truth
    plugins = {'robo_vac': {'type': 'vacuum_mujoco/Sensors', 'ground_truth': truth}}
    if arg('enable_cameras') == 'true':
        plugins['floor_camera'] = {
            'type': 'mujoco_ros2_control_plugins/CameraPlugin',
            'camera_publish_rate': 15.0,
            'floor': {'frame_name': 'camera_optical_frame', 'image_topic': '/camera/image',
                      'info_topic': '/camera/camera_info'},
        }
    config['mujoco_ros2_control_node'] = {'ros__parameters': {'mujoco_plugins': plugins}}
    config_path = Path(temp.name) / 'controllers.yaml'
    config_path.write_text(yaml.safe_dump(config))

    control = ET.SubElement(urdf, 'ros2_control', name='RoboVACMuJoCo', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    for name, value in {'mujoco_model': str(model_path), 'headless': arg('headless')}.items():
        ET.SubElement(hardware, 'param', name=name).text = value
    for side in ('left', 'right'):
        joint = ET.SubElement(control, 'joint', name=f'wheel_{side}_joint')
        ET.SubElement(joint, 'command_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='velocity')

    odom = '/odom_wheel' if truth else '/odom'
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _context: temp.cleanup()),
        ])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ET.tostring(urdf, encoding='unicode'),
                          'use_sim_time': True}]),
        # Jazzy needs the controller manager shipped by mujoco_ros2_control.
        Node(package='mujoco_ros2_control', executable='ros2_control_node', output='screen',
             parameters=[str(config_path), {'use_sim_time': True}]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', '--controller-manager-timeout', '45']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['vacuum_drive', '--controller-manager-timeout', '45',
                        '--param-file', str(config_path),
                        '--controller-ros-args', f'--remap ~/odom:={odom}']),
    ]


def generate_launch_description():
    defaults = {
        'world': 'living_room.world', 'x_pose': '-2.0', 'y_pose': '-0.5',
        'dirt': '0', 'dirt_seed': '1', 'dirt_map': '', 'dirt_dir': '~/vacuum_runs/dirt',
        'dirt_scale': '1.0', 'dirt_stubborn': '0.15',
    }
    switches = ('headless', 'enable_cameras', 'dirt_randomize')
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        *(DeclareLaunchArgument(name, default_value='false', choices=['true', 'false'])
          for name in switches),
        DeclareLaunchArgument('odom_source', default_value='ground_truth',
                              choices=['ground_truth', 'robot_wheels']),
        OpaqueFunction(function=_launch),
    ])
