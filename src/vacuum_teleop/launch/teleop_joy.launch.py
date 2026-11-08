#!/usr/bin/env python3
#
# Gamepad teleop for the vacuum robot, replacing teleop_twist_keyboard.
#
#   ros2 launch vacuum_teleop teleop_joy.launch.py
#
# Starts joy_node (joystick -> sensor_msgs/msg/Joy) and teleop_twist_joy
# (/joy -> geometry_msgs/msg/Twist on /cmd_vel), which gz_bridge forwards to
# the DiffDrive plugin. Hold LB, drive with the left stick, RB = turbo.
#
# Arguments:
#   joy_dev  SDL joystick index (default 0; list them with
#            `ros2 run joy joy_enumerate_devices`)
#   joy_vel  Twist topic (default cmd_vel)
#   config   parameter file (default config/teleop_joy.yaml)
#
# Note: Nav2's controller also publishes /cmd_vel, so don't drive with the pad
# while a Nav2 goal is executing. Remap joy_vel if you want to arbitrate the
# two through twist_mux.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_share = get_package_share_directory('vacuum_teleop')
    default_config = os.path.join(teleop_share, 'config', 'teleop_joy.yaml')

    joy_dev = LaunchConfiguration('joy_dev')
    joy_vel = LaunchConfiguration('joy_vel')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', default_value='0',
            description='SDL joystick index (see `ros2 run joy '
                        'joy_enumerate_devices`)'),
        DeclareLaunchArgument(
            'joy_vel', default_value='cmd_vel',
            description='Twist topic to publish'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='joy_node + teleop_twist_joy parameters'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='screen',
            parameters=[config, {'device_id': joy_dev}],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', output='screen',
            parameters=[config],
            remappings=[('/cmd_vel', joy_vel)],
        ),
    ])
