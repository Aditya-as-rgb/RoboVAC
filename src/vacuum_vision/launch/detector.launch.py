"""Dirt detector node. Needs the floor camera (cameras:=true) and TF to camera_optical_frame."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vacuum_vision')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'detector.yaml')),
        DeclareLaunchArgument('model', default_value='',
                              description='path to dirt_yolo11n.onnx ("" = find it in the repo)'),
        Node(package='vacuum_vision', executable='dirt_detector', name='dirt_detector',
             output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')},
                         {'model': LaunchConfiguration('model')}]),
    ])
