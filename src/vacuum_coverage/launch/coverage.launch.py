"""Coverage mission + coverage meter. Needs Nav2 and a map (see vacuum_bringup/vacuum.launch.py)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vacuum_coverage')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'coverage.yaml')),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='start cleaning as soon as the map, robot and Nav2 are ready'),
        DeclareLaunchArgument('meter', default_value='true', description='run the coverage meter'),
        Node(package='vacuum_coverage', executable='coverage_mission', name='coverage_mission',
             output='screen',
             parameters=[params, {'use_sim_time': use_sim_time,
                                  'autostart': LaunchConfiguration('autostart')}]),
        Node(package='vacuum_coverage', executable='coverage_meter', name='coverage_meter',
             output='screen', condition=IfCondition(LaunchConfiguration('meter')),
             parameters=[params, {'use_sim_time': use_sim_time}]),
    ])
