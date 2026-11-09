#!/usr/bin/env python3
#
# One command for the whole demo: Gazebo + robot, Nav2 (+ AMCL or SLAM), RViz,
# optional gamepad teleop, and the coverage mission with its coverage meter.
#
#   ros2 launch vacuum_bringup vacuum.launch.py
#   ros2 launch vacuum_bringup vacuum.launch.py autostart:=true      # clean on its own
#   ros2 launch vacuum_bringup vacuum.launch.py slam:=True coverage:=false   # drive and map
#   ros2 launch vacuum_bringup vacuum.launch.py headless:=true rviz:=false
#
# Start cleaning (if autostart:=false), or stop it, from another terminal:
#   ros2 service call /coverage_mission/start std_srvs/srv/Trigger
#   ros2 service call /coverage_mission/stop  std_srvs/srv/Trigger
#   ros2 service call /coverage_mission/plan  std_srvs/srv/Trigger   # preview only
#
# slam:=False (default) localizes on `map` with AMCL, which is what coverage
# needs: the plan is made from a finished map. Don't use coverage with slam:=True.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('vacuum_gazebo')
    bringup_share = get_package_share_directory('vacuum_bringup')
    coverage_share = get_package_share_directory('vacuum_coverage')
    teleop_share = get_package_share_directory('vacuum_teleop')

    def include(pkg_share, launch_file, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', launch_file)),
            launch_arguments=args)

    sim = include(gazebo_share, 'sim.launch.py', {
        'headless': LaunchConfiguration('headless'),
        'world': LaunchConfiguration('world'),
        'x_pose': LaunchConfiguration('x_pose'),
        'y_pose': LaunchConfiguration('y_pose'),
        'odom_source': LaunchConfiguration('odom_source'),
        'qt_platform': LaunchConfiguration('qt_platform'),
    }.items())

    navigation = include(bringup_share, 'navigation.launch.py', {
        'slam': LaunchConfiguration('slam'),
        'map': LaunchConfiguration('map'),
        'rviz': LaunchConfiguration('rviz'),
    }.items())

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='False', choices=['True', 'False'],
                              description='False: AMCL on `map`. True: map while driving.'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(gazebo_share, 'maps', 'living_room.yaml'),
            description='Map yaml (slam:=False)'),
        DeclareLaunchArgument('world', default_value='living_room.world'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('odom_source', default_value='ground_truth',
                              choices=['ground_truth', 'robot_wheels']),
        DeclareLaunchArgument('qt_platform', default_value='xcb'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('teleop', default_value='joy', choices=['joy', 'none'],
                              description='joy: start the gamepad teleop'),
        DeclareLaunchArgument('coverage', default_value='true', choices=['true', 'false'],
                              description='start the coverage mission + meter'),
        DeclareLaunchArgument('meter', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='false', choices=['true', 'false'],
                              description='begin cleaning once Nav2 and the robot pose are ready'),
        DeclareLaunchArgument('nav_delay', default_value='8.0',
                              description='seconds to let Gazebo come up before Nav2 starts'),
        sim,
        # Nav2 needs /clock, /scan and the odom TF, which appear a few seconds after start
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[navigation]),
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(teleop_share, 'launch', 'teleop_joy.launch.py')),
                condition=LaunchConfigurationEquals('teleop', 'joy'))]),
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(coverage_share, 'launch', 'coverage.launch.py')),
                launch_arguments={'autostart': LaunchConfiguration('autostart'),
                                  'meter': LaunchConfiguration('meter')}.items(),
                condition=IfCondition(LaunchConfiguration('coverage')))]),
    ])
