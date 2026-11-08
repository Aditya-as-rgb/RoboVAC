#!/usr/bin/env python3
#
# Adapted from makerspet/oomwoo-ros2-tools oomwoo_bringup navigation.launch.py
# Copyright 2023-2024 KAIA.AI, licensed under the Apache License, Version 2.0
# (http://www.apache.org/licenses/LICENSE-2.0). Heavily simplified: kaiaai
# dependency removed, uses nav2_bringup's stock bringup_launch.py only.
#
#   slam:=True  (default) map while navigating (slam_toolbox, via nav2_bringup)
#   slam:=False           localize on a saved map with AMCL (map:=/path/map.yaml)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('vacuum_bringup')
    gazebo_share = get_package_share_directory('vacuum_gazebo')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items())

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},  # avoid Wayland Qt/Ogre crashes
        condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'slam', default_value='True', choices=['True', 'False'],
            description='True: build a map while navigating. '
                        'False: localize on `map` with AMCL.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(gazebo_share, 'maps', 'living_room.yaml'),
            description='Map yaml, used only when slam:=False'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'navigation.yaml'),
            description='Nav2 (+ AMCL) parameters'),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_share, 'rviz', 'navigation.rviz')),
        nav2,
        rviz_node,
    ])
