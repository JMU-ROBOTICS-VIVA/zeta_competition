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
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch import LaunchContext
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def get_node_names():
    """Return all active node names."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('__anonymous')
    rclpy.spin_once(node, timeout_sec=.2)
    names = node.get_node_names()
    node.destroy_node()
    rclpy.shutdown()
    return names


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    this_path = get_package_share_directory('zeta_competition')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')

    default_map_path = os.path.join(this_path, 'maps', 'room_practice.yaml')
    default_pose = os.path.join(this_path, 'config', 'sim_initial_pose.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('initial_pose', default_value=default_pose),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Start Aruco node detector
        Node(
            package="ros2_aruco",
            executable="aruco_node",
            output="screen",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'camera_frame': 'camera_rgb_optical_frame'}]
        ),

        # START NAV SYSTEM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation2.launch.py']),
            launch_arguments=[
                ('map', LaunchConfiguration('map')),
                ('use_sim_time', LaunchConfiguration('use_sim_time'))
            ],
        ),
        
        # Start the reporting button
        Node(
            package="zeta_competition",
            executable="report_button",
            output="screen",
        ),
        
        # Start initial pose setter
        Node(
            package="zeta_competition",
            executable="set_initial_pose",
            output="screen",
            parameters=[LaunchConfiguration('initial_pose')]
        ),

    ])
    node_names = get_node_names()
    tb3_dir = get_package_share_directory('turtlebot3_bringup')
    tb3_launch_file_dir = os.path.join(tb3_dir, 'launch')
    if 'image_republisher' not in node_names:
        ld.add_entity(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([tb3_launch_file_dir,
                                           '/image_transport.launch.py']),
            condition=UnlessCondition(LaunchConfiguration("use_sim_time"))
        ))
    return ld

if __name__ == "__main__":
    generate_launch_description()
