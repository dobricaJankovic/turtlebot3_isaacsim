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
    world_z = LaunchConfiguration('world_z')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world')

    isaacsim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'isaacsim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'world_z': world_z,
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
        # The small counterpart of warehouse.launch.py: one stock room instead
        # of a 24 x 38.8 m hall, so the whole thing fits inside the burger's
        # 3.5 m lidar and a map of it is 219 x 219 px rather than 1200 x 1200.
        # Measured on the stage: walls at x = +-4.41 and y = -3.31 (back) and
        # y = +4.85 (the window wall), so 8.82 x 8.16 m of interior, with one
        # 3.19 x 1.63 m low table centred on the origin and nothing else.
        DeclareLaunchArgument(
            'world',
            default_value='/Isaac/Environments/Simple_Room/simple_room.usd',
            description='Stock environment path, a local .usd, or a URL'),

        # This asset is authored with its floor at z = -0.7696 and ships its
        # own GroundPlane there, which is 0.77 m below the one the simulator
        # authors at z = 0. Raising the whole reference is what reconciles the
        # two: measured after the lift, the floor mesh tops out at z = +0.00003
        # and the room's own plane lands at z = +0.0001, so the robot settles
        # on the room's actual floor -- body bounding box z = [-0.00054,
        # +0.19126], and identical with the package's plane deleted, which is
        # the test that says which surface is holding it up.
        #
        # Two infinite colliders now sit within 0.1 mm of each other. Measured
        # over 2 s of contact: 0.00000 m of xy drift and 0.00000 m of z range,
        # i.e. no jitter and no creep, so they are left alone.
        #
        # The point of doing this rather than tolerating the float is that the
        # table stops being scenery. Left at its authored height its top was at
        # z = +0.0104, 1 cm into the robot and 17 cm below the beam -- an
        # obstacle neither the map nor the scan could see. Raised, it spans
        # z = 0.000 to 0.780 and the beam cuts its legs: 156 of the map's cells
        # are the four legs, and the room stops being an empty box.
        #
        # scripts/build_map.py MUST be given the same 0.7696. See README.md,
        # "Maps", which carries the command, and DESIGN.md, "Standing the world
        # on the ground plane", for why a mismatch is the dangerous kind of bug.
        DeclareLaunchArgument(
            'world_z', default_value='0.7696',
            description='Metres to raise the world by, so its floor meets z=0'),

        # Off the back corner, not at the origin, and measured against the
        # stage -- generator.get_occupied_positions(), never the .pgm, which is
        # the mistake DESIGN.md's "The occupancy map was flipped" records.
        #
        # The origin is inside the table, which after the lift is a real
        # obstacle 0.78 m tall rather than something to drive over. From here
        # the nearest obstacle is 1.275 m away, the nearest table leg 1.355 m,
        # and 64.4% of the 360 one-degree bearings get a return inside 3.5 m
        # (3043 mapped cells, 89 of them table). Corners score up to 70.0% but
        # leave only 0.53-0.63 m of clearance; (-3.0, -2.0) is the best of the
        # roomy ones at 67.8%, which is not worth the less memorable number.
        # The room is very nearly symmetric in x, so (+2.0, -2.0) is the same
        # pose mirrored and measures the same to within 0.1%; y is what matters.
        DeclareLaunchArgument('x_pose', default_value='-2.0'),

        DeclareLaunchArgument('y_pose', default_value='-2.0'),

        DeclareLaunchArgument('headless', default_value='false'),

        isaacsim_cmd,
        robot_state_publisher_cmd,
    ])

    return ld
