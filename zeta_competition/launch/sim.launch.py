# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# 
# Modified by: Rafael D.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition

this_path = get_package_share_directory('zeta_competition')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    # DeclareLaunchArgument('map', default_value=PathJoinSubstitution([this_path, 'maps', 'room_practice.yaml']),
    #                       description='The map to display in Rviz2'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('world', default_value='room_practice_v2',
                        choices=['room_practice_v2', 'new_v2'], description='Simulator World')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Replaced with the competition simulator launch.
    this_path = get_package_share_directory('zeta_competition')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [this_path, 'launch', 'newsim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [this_path, 'launch', 'turtlebot4_spawn_mod.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))]
    )

    robot_spawn_practice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '-0.1'),
            ('y', '-5.35'),
            ('z', '0.03'),
            ('yaw', LaunchConfiguration('yaw')),
            ('model', LaunchConfiguration('model')),
            ('map', PathJoinSubstitution([this_path, 'maps', "room_practice.yaml"])),
            ('nav2', 'true'),
            ('slam', 'false'),
            ('localization', 'true')],
        condition=IfCondition(PythonExpression([
            '"', LaunchConfiguration('world'), '" == "room_practice_v2"'
        ]))
    )

    robot_spawn_new = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '-1.15'),
            ('y', '-5.35'),
            ('z', '-0.00'),
            ('yaw', '-1.59'),
            ('model', LaunchConfiguration('model')),
            ('map', PathJoinSubstitution([this_path, 'maps', "new.yaml"])),
            ('nav2', 'true'),
            ('slam', 'false'),
            ('localization', 'true')],
        condition=IfCondition(PythonExpression([
            '"', LaunchConfiguration('world'), '" == "new_v2"'
        ]))
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn_practice)
    ld.add_action(robot_spawn_new)
    return ld
