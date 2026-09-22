#!/usr/bin/env python3
#
# Copyright 2026 dobricaJankovic
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
# Authors: dobricaJankovic

"""The simulator plus robot_state_publisher, parameterised over a world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_isaacsim'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    world_z = LaunchConfiguration('world_z')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    headless = LaunchConfiguration('headless')

    isaacsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'isaacsim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'world_z': world_z,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'headless': headless,
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Path to an environment .usd. Empty for a ground plane'),

        DeclareLaunchArgument(
            'world_z', default_value='0.0',
            description="Metres to raise the world by. Must match build_map.py's "
                        '--world-z for the same world'),

        DeclareLaunchArgument('x_pose', default_value='0.0'),

        DeclareLaunchArgument('y_pose', default_value='0.0'),

        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Isaac Sim with no window'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Isaac Sim) clock if true'),

        isaacsim_cmd,
        robot_state_publisher_cmd,
    ])
