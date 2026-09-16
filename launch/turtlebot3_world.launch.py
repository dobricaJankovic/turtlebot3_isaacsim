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

"""world.launch.py against worlds/turtlebot3_world.usd, this package's own."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('turtlebot3_isaacsim')
    launch_file_dir = os.path.join(pkg, 'launch')
    world = os.path.join(pkg, 'worlds', 'turtlebot3_world.usd')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'world.launch.py')
            ),
            launch_arguments={
                'world': world,
                'x_pose': '-2.0',
                'y_pose': '-0.5',
            }.items()
        ),
    ])
