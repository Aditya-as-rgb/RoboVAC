#!/usr/bin/env python3
#
# Adapted from makerspet/oomwoo_gazebo world.launch.py
# Copyright 2023-2025 KAIA.AI, licensed under the Apache License, Version 2.0
# (http://www.apache.org/licenses/LICENSE-2.0). Modified: removed the kaiaai
# dependency, the robot description package is a plain launch argument, and the
# model directory is added to the Gazebo resource path here.
#
# Starts gz-sim with a world, bridges gz <-> ROS 2, publishes robot_description
# and spawns the robot.

import importlib.util
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# gz model name. config/gz_bridge.yaml addresses the bumper contact sensors by
# the scoped gz topic /world/default/model/vacuum/..., so this name and the
# world name ("default") must match what the bridge file says. If you rename the
# robot, change it there too.
ROBOT_NAME = 'vacuum'

XACRO_SWITCHES = ('enable_lidar', 'enable_ranges', 'enable_tof',
                  'enable_cameras', 'enable_imu')


def _arg(context: LaunchContext, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _add_dirt(context: LaunchContext, gazebo_share: str, world_path: str, count: int) -> str:
    """Write a copy of the world with ``count`` dirt patches added; returns its path.

    The patches go only where the robot can clean, so the generator needs the map of
    this world: by default maps/<world name>.yaml in this package, or dirt_map:=...
    """
    spec = importlib.util.spec_from_file_location(
        'dirt_world', os.path.join(gazebo_share, 'scripts', 'dirt_world.py'))
    dirt_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dirt_world)

    map_yaml = _arg(context, 'dirt_map')
    if not map_yaml:
        stem = os.path.splitext(_arg(context, 'world'))[0]
        map_yaml = os.path.join(gazebo_share, 'maps', stem + '.yaml')
    if not os.path.exists(map_yaml):
        raise RuntimeError(
            'dirt:=%d needs the map of this world to place dirt on reachable floor, '
            'but %s does not exist. Save one with save_map.sh, then pass '
            'dirt_map:=/path/to/map.yaml' % (count, map_yaml))
    result = dirt_world.make_world(
        world_path, map_yaml, count, seed=int(_arg(context, 'dirt_seed')),
        spawn=(float(_arg(context, 'x_pose')), float(_arg(context, 'y_pose'))),
        out_dir=_arg(context, 'dirt_dir'), scale=float(_arg(context, 'dirt_scale')),
        stubborn_fraction=float(_arg(context, 'dirt_stubborn')),
        randomize=_arg(context, 'dirt_randomize') == 'true')
    print('Dirt       : {} patches, seed {} (map {})'.format(
        len(result['patches']), _arg(context, 'dirt_seed'), map_yaml))
    print('Dirt layout: {}'.format(result['layout']))
    return result['world']


def make_nodes(context: LaunchContext):
    description_pkg = _arg(context, 'description_pkg')
    use_sim_time = _arg(context, 'use_sim_time').lower() == 'true'
    headless = _arg(context, 'headless').lower() == 'true'
    software_gl = _arg(context, 'software_gl').lower() == 'true'

    gazebo_share = get_package_share_directory('vacuum_gazebo')
    desc_share = get_package_share_directory(description_pkg)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(desc_share, 'urdf', 'robot.urdf.xacro')
    bridge_cfg = os.path.join(desc_share, 'config', 'gz_bridge.yaml')
    world_path = os.path.join(gazebo_share, 'worlds', _arg(context, 'world'))
    dirt_count = int(_arg(context, 'dirt'))
    if dirt_count > 0:
        world_path = _add_dirt(context, gazebo_share, world_path, dirt_count)

    mappings = {'odom_source': _arg(context, 'odom_source')}
    for name in XACRO_SWITCHES:
        mappings[name] = _arg(context, name)
    robot_description = xacro.process_file(urdf_path, mappings=mappings).toxml()

    print('URDF  file : {}'.format(urdf_path))
    print('World file : {}'.format(world_path))
    print('Bridge cfg : {}'.format(bridge_cfg))

    # Headless: server only (-s) + offscreen rendering, so the GPU LiDAR still
    # produces data with no display attached.
    gz_args = ('-s -r --headless-rendering -v 4 ' if headless else '-r -v 4 ') + world_path

    actions = [
        # Lets the world resolve model://... URIs from this package.
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
    ]
    # The Gazebo GUI segfaults on a Wayland session unless Qt runs through
    # XWayland (xcb). Harmless on X11. qt_platform:='' leaves it untouched.
    qt_platform = _arg(context, 'qt_platform')
    if qt_platform:
        actions.append(SetEnvironmentVariable('QT_QPA_PLATFORM', qt_platform))
    # Mesa software rendering: under headless only when the machine has no GPU
    # (no /dev/dri) — with one, hardware EGL renders offscreen fine and forcing
    # llvmpipe segfaults Mesa (EGL driCreateNewScreen3, exit 139). software_gl
    # forces it either way (GUI fallback if the window dies with OGRE/EGL error).
    if software_gl or (headless and not os.path.exists('/dev/dri')):
        actions += [
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        ]

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': gz_args,
                'on_exit_shutdown': 'true',
            }.items()),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            output='screen',
            # No use_sim_time here: this node is what bridges /clock.
            parameters=[{'config_file': bridge_cfg}]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_description}]),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-name', ROBOT_NAME,
                '-topic', 'robot_description',
                '-timeout', '180',
                '-x', _arg(context, 'x_pose'),
                '-y', _arg(context, 'y_pose'),
                # Carried over unchanged from the upstream launch file.
                '-g', ' ',
                '-allow_renaming', 'false',
            ]),
    ]
    return actions


def generate_launch_description():
    def switch(name, default, doc):
        return DeclareLaunchArgument(
            name, default_value=default, choices=['true', 'false'],
            description=doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'description_pkg', default_value='vacuum_description',
            description='Package providing urdf/robot.urdf.xacro and '
                        'config/gz_bridge.yaml (swap in your own robot here)'),
        DeclareLaunchArgument(
            'world', default_value='living_room.world',
            description='World file name in vacuum_gazebo/worlds'),
        DeclareLaunchArgument(
            'dirt', default_value='0',
            description='Number of dirt patches to scatter on the floor (0 = none). '
                        'Visual only, no collision; placed on floor the robot can reach.'),
        DeclareLaunchArgument(
            'dirt_seed', default_value='1',
            description='Random seed: the same seed gives the same dirt layout'),
        DeclareLaunchArgument(
            'dirt_map', default_value='',
            description='Map yaml of the world, used to find reachable floor '
                        '(default: maps/<world name>.yaml in vacuum_gazebo)'),
        DeclareLaunchArgument(
            'dirt_dir', default_value='~/vacuum_runs/dirt',
            description='Where the dirt world and its ground-truth layout (.json) are written'),
        DeclareLaunchArgument(
            'dirt_scale', default_value='1.0',
            description='Size multiplier for every dirt patch (2.0 = twice as big; makes '
                        'them easier for a camera to see)'),
        DeclareLaunchArgument(
            'dirt_stubborn', default_value='0.15',
            description='Share of non-spill patches that need much more scrubbing to come off '
                        '(spills always do)'),
        DeclareLaunchArgument(
            'dirt_randomize', default_value='false', choices=['true', 'false'],
            description='Random floor colour and light level for this dirt_seed (dataset '
                        'collection: stops a vision model learning one floor)'),
        DeclareLaunchArgument(
            'qt_platform', default_value='xcb',
            description='Value for QT_QPA_PLATFORM (xcb = run the Gazebo GUI '
                        'through XWayland). Empty string leaves it unset.'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                              description='Spawn x (m)'),
        DeclareLaunchArgument('y_pose', default_value='-0.5',
                              description='Spawn y (m)'),
        switch('use_sim_time', 'true', 'Use the Gazebo clock'),
        switch('headless', 'false',
               'Server only, offscreen rendering (no GUI)'),
        switch('software_gl', 'false',
               'Force Mesa software GL for the GUI (slow; use if the window '
               'dies with an OGRE/EGL error)'),
        DeclareLaunchArgument(
            'odom_source', default_value='ground_truth',
            choices=['ground_truth', 'robot_wheels'],
            description='Which odometry owns /odom + /tf. ground_truth = '
                        'slip-free model pose; robot_wheels = wheel odometry '
                        '(drifts). The other stream is on /odom_wheel or '
                        '/odom_truth.'),
        switch('enable_lidar', 'true', '2D LiDAR /scan'),
        switch('enable_ranges', 'true', 'Side distance sensors'),
        switch('enable_tof', 'true', 'Front multizone ToF (costly)'),
        switch('enable_cameras', 'false', 'Front camera (costly)'),
        switch('enable_imu', 'true', 'IMU /imu'),
        OpaqueFunction(function=make_nodes),
    ])
