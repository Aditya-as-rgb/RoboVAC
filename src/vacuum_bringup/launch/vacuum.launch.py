#!/usr/bin/env python3
#
# One command for the whole demo: Gazebo + robot, Nav2 (+ AMCL or SLAM), RViz,
# optional gamepad teleop, and the coverage mission with its coverage meter.
#
#   ros2 launch vacuum_bringup vacuum.launch.py
#   ros2 launch vacuum_bringup vacuum.launch.py autostart:=true      # clean on its own
#   ros2 launch vacuum_bringup vacuum.launch.py slam:=True coverage:=false   # drive and map
#   ros2 launch vacuum_bringup vacuum.launch.py headless:=true rviz:=false
#   ros2 launch vacuum_bringup vacuum.launch.py dirt:=15 dirt_seed:=3  # scatter dirt on the floor
#
# dirt:=N adds N visual-only dirt patches (crumbs, dust, spills, hair) on floor the robot
# can reach, placed from `map` (or dirt_map:=...). The same dirt_seed gives the same layout;
# the ground truth is written to ~/vacuum_runs/dirt/<world>_dirt<N>_seed<S>.json.
#   dirt_scale:=2.0     bigger patches (easier for a camera to see)
#   dirt_stubborn:=0.3  more patches that need several passes (spills always do)
#   dirt_clean:=false   leave the dirt on the floor (default true: the robot's brush removes
#                       what it drives over; see vacuum_gazebo/scripts/dirt_cleaner.py)
#   dirt_brush:=0.20    brush radius in metres
#   dirt_randomize:=true  random floor colour + light level for this dirt_seed
#   collect_dataset:=true record labelled camera images (YOLO format) into dataset_dir
#                       (~/vacuum_runs/dataset); turns the camera on, use dirt_clean:=false
#                       and see tools/dirt_dataset/ for the batch script and Colab notebook
# What is left is counted in /dirt/remaining and in ~/vacuum_runs/dirt/dirt_summary_*.json.
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
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        'dirt': LaunchConfiguration('dirt'),
        'dirt_seed': LaunchConfiguration('dirt_seed'),
        'dirt_map': LaunchConfiguration('dirt_map'),
        'dirt_dir': LaunchConfiguration('dirt_dir'),
        'dirt_scale': LaunchConfiguration('dirt_scale'),
        'dirt_stubborn': LaunchConfiguration('dirt_stubborn'),
        'dirt_randomize': LaunchConfiguration('dirt_randomize'),
        # the dataset recorder needs the camera, so collecting switches it on by itself
        'enable_cameras': PythonExpression([
            "'true' if '", LaunchConfiguration('cameras'), "' == 'true' or '",
            LaunchConfiguration('collect_dataset'), "' == 'true' else 'false'"]),
    }.items())

    # the brush: takes dirt off the floor as the robot drives over it (only when there is dirt)
    dirt_cleaner = Node(
        package='vacuum_gazebo', executable='dirt_cleaner.py', name='dirt_cleaner', output='screen',
        parameters=[{
            'use_sim_time': True,
            'layout': [LaunchConfiguration('dirt_dir'), TextSubstitution(text='/current_layout.json')],
            'clean_radius': ParameterValue(LaunchConfiguration('dirt_brush'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "int('", LaunchConfiguration('dirt'), "') > 0 and '",
            LaunchConfiguration('dirt_clean'), "' == 'true'"])))

    # labelled dirt images for training a vision model; stops itself when the mission ends,
    # and then takes the whole launch down so a batch script can move on to the next seed
    dataset = Node(
        package='vacuum_gazebo', executable='dirt_dataset.py', name='dirt_dataset', output='screen',
        parameters=[{
            'use_sim_time': True,
            'layout': [LaunchConfiguration('dirt_dir'), TextSubstitution(text='/current_layout.json')],
            'out_dir': LaunchConfiguration('dataset_dir'),
            'max_duration_s': ParameterValue(LaunchConfiguration('dataset_max_s'), value_type=float),
        }],
        condition=IfCondition(PythonExpression([
            "int('", LaunchConfiguration('dirt'), "') > 0 and '",
            LaunchConfiguration('collect_dataset'), "' == 'true'"])))
    stop_when_dataset_done = RegisterEventHandler(OnProcessExit(
        target_action=dataset, on_exit=[EmitEvent(event=Shutdown(reason='dataset episode finished'))]))

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
        DeclareLaunchArgument('dirt', default_value='0',
                              description='number of dirt patches to scatter (0 = none)'),
        DeclareLaunchArgument('dirt_seed', default_value='1',
                              description='same seed = same dirt layout'),
        DeclareLaunchArgument('dirt_map', default_value=LaunchConfiguration('map'),
                              description='map yaml used to find reachable floor for the dirt'),
        DeclareLaunchArgument('dirt_dir', default_value='~/vacuum_runs/dirt',
                              description='where dirt layouts, events and summaries are written'),
        DeclareLaunchArgument('dirt_scale', default_value='1.0', description='dirt size multiplier'),
        DeclareLaunchArgument('dirt_stubborn', default_value='0.15',
                              description='share of patches that need several passes (spills always do)'),
        DeclareLaunchArgument('dirt_clean', default_value='true', choices=['true', 'false'],
                              description='the brush removes dirt the robot drives over'),
        DeclareLaunchArgument('dirt_brush', default_value='0.20', description='brush radius (m)'),
        DeclareLaunchArgument('dirt_randomize', default_value='false', choices=['true', 'false'],
                              description='random floor colour + light level per dirt_seed'),
        DeclareLaunchArgument('cameras', default_value='false', choices=['true', 'false'],
                              description='run the front floor camera (collect_dataset turns it on)'),
        DeclareLaunchArgument('collect_dataset', default_value='false', choices=['true', 'false'],
                              description='record labelled dirt images while the mission runs'),
        DeclareLaunchArgument('dataset_dir', default_value='~/vacuum_runs/dataset',
                              description='where the labelled images are written'),
        DeclareLaunchArgument('dataset_max_s', default_value='3600.0',
                              description='stop recording after this many sim seconds'),
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
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[dirt_cleaner]),
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[dataset]),
        stop_when_dataset_done,
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
