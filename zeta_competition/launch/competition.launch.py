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

    tb_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('jmu_turtlebot3_bringup'), 'launch')
    #nav2_launch_file_dir = os.path.join(this_path,'launch')

    default_world_path = os.path.join(this_path, 'worlds', 'room_practice.world')

    default_map_path = os.path.join(this_path, 'maps', 'room_practice.yaml')

    default_pose = os.path.join(this_path, 'config', 'sim_initial_pose.yaml')

    model_path = os.path.join(this_path, 'models/')
    tb_model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models/')

    lc = LaunchContext()
    SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
                           '/usr/share/gazebo-11:' + model_path + ":" + tb_model_path).visit(lc)
    SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                           '/usr/share/gazebo-11/models:'+ model_path + ":" + tb_model_path).visit(lc)

    #launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world_path),
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('initial_pose', default_value=default_pose),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),


        # START SIMULATOR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': default_world_path}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([tb_launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Start Aruco node detector
        Node(
            package="ros2_aruco",
            executable="aruco_node",
            output="screen",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'camera_frame': 'camera_rgb_optical_frame'}]
        ),

        # Start transform service
        Node(
            package="transform_service",
            executable="transform_service",
            output="screen",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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

if __name__ == "__main__":
    generate_launch_description()
