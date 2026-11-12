#!/usr/bin/env python3
#
# The control panel as a launch file, so it can sit next to the rest of the stack:
#
#   ros2 launch vacuum_gui gui.launch.py
#   ros2 launch vacuum_gui gui.launch.py xcb:=true          # force XWayland
#   ros2 launch vacuum_gui gui.launch.py theme:=light
#
# The panel starts the simulation itself from its Stack tab, so this file only starts the
# panel (never a second Gazebo).

import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

COMMAND = [sys.executable, '-m', 'vacuum_gui.app']


def generate_launch_description():
    xcb = LaunchConfiguration('xcb')
    theme = LaunchConfiguration('theme')
    no_ros = LaunchConfiguration('no_ros')
    return LaunchDescription([
        DeclareLaunchArgument('xcb', default_value='false', choices=['true', 'false'],
                              description='Force QT_QPA_PLATFORM=xcb (XWayland)'),
        DeclareLaunchArgument('theme', default_value='',
                              description='system, dark or light (empty = stored setting)'),
        DeclareLaunchArgument('no_ros', default_value='false', choices=['true', 'false'],
                              description='Open the panel without a ROS backend'),
        SetEnvironmentVariable('VACUUM_GUI_THEME', theme),
        ExecuteProcess(cmd=COMMAND + ['--xcb'], name='vacuum_gui', output='screen',
                       condition=IfCondition(xcb)),
        ExecuteProcess(cmd=COMMAND, name='vacuum_gui', output='screen',
                       condition=UnlessCondition(xcb)),
        # no_ros is read by the app itself; passing it as an argument keeps the env clean
        SetEnvironmentVariable('VACUUM_GUI_NO_ROS', no_ros),
    ])
