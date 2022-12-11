#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os


from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch import LaunchContext
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    this_path = get_package_share_directory('zeta_competition')
    zeta_comp_launch_file_dir = os.path.join(this_path, 'launch')

    default_map_path = os.path.join(this_path, 'maps', 'room_practice.yaml')
    default_pose = os.path.join(this_path, 'config', 'sim_initial_pose.yaml')
    rviz_config_path = os.path.join(this_path, 'rviz', 'final_scoring.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('ground_truth', default_value=os.path.join(this_path, 'config/default_victims.csv')),
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('initial_pose', default_value=default_pose),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([zeta_comp_launch_file_dir, '/competition.launch.py']),
            launch_arguments=[
                ('map', LaunchConfiguration('map')),
                ('initial_pose', LaunchConfiguration('initial_pose')),
                ('use_sim_time', use_sim_time)
            ],
        ),
        Node(
            package="zeta_competition",
            executable="zeta_scorer",
            name="zeta_scorer",
            output="screen",
            arguments=[LaunchConfiguration('ground_truth')]
        ),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2_comp',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

if __name__ == "__main__":
    generate_launch_description()
