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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world')

    isaacsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'isaacsim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'headless': headless
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription([
        # Not a file in this package: one of Isaac Sim's stock environments,
        # named by its path under the asset root and fetched from there the
        # first time it is used. The other three in the same folder --
        # warehouse_with_forklifts, warehouse_multiple_shelves and
        # full_warehouse -- are drop-in alternatives, and any local .usd or URL
        # works here too. See UPSTREAM.md, "Worlds".
        DeclareLaunchArgument(
            'world',
            default_value='/Isaac/Environments/Simple_Warehouse/warehouse.usd',
            description='Stock environment path, a local .usd, or a URL'),

        # Beside the shelving, not at the world origin. The warehouse is a
        # 20 x 30 m hall whose middle is bare floor: the origin is 8.83 m from
        # the nearest thing a ray can hit, so a 3.5 m lidar there returns
        # nothing, /scan stays silent and AMCL has no features to match. This
        # pose sits 1.83 m off the racks, measured against the stage itself --
        # generator.get_occupied_positions(), not the .pgm. The mirror of it,
        # +7.0, is 2.38 m off the opposite wall and also works; it was the
        # default until the map's x axis was unmirrored, which moved every
        # feature the pose had been read off. See DESIGN.md, "The occupancy map
        # was flipped".
        DeclareLaunchArgument('x_pose', default_value='-7.0'),

        DeclareLaunchArgument('y_pose', default_value='0.0'),

        DeclareLaunchArgument('headless', default_value='false'),

        isaacsim_cmd,
        robot_state_publisher_cmd,
    ])

    return ld
