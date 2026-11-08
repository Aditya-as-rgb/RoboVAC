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
    # Mesa software rendering: always when headless; optional for the GUI
    # (software_gl:=true) if the Gazebo window dies with an OGRE/EGL error.
    if headless or software_gl:
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
            'qt_platform', default_value='xcb',
            description='Value for QT_QPA_PLATFORM (xcb = run the Gazebo GUI '
                        'through XWayland). Empty string leaves it unset.'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                              description='Spawn x (m)'),
        DeclareLaunchArgument('y_pose', default_value='-0.5',
                              description='Spawn y (m)'),
        switch('use_sim_time', 'true', 'Use the Gazebo clock'),
        switch('headless', 'false',
               'Server only, offscreen rendering, software GL (no GUI)'),
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
