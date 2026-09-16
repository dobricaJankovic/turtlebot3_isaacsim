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

"""world.launch.py against Isaac Sim's replicator_kitchen stock environment.

The smallest world in the package: 5.148 x 4.352 m of interior inside 2.55 m
walls, a U of cabinets open to the south.
"""

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

    return LaunchDescription([
        # The other four in the same folder -- kitchen_l_shape, kitchen_l_island,
        # kitchen_g_shape and kitchen_peninsula -- are drop-in alternatives, with
        # one caveat: peninsula drags a 100 x 100 m outdoor backdrop behind its
        # windows, so the +-3 m bounds this world's map is built with are far too
        # tight for it. Only u_shape has been measured; the rest are named from a
        # listing of the asset root, not from a floor height anyone has checked.
        DeclareLaunchArgument(
            'world',
            default_value='/Isaac/Environments/replicator_kitchen/kitchen_u_shape.usda',
            description='Stock environment path, a local .usd, or a URL'),

        # No world_z is declared here, and that is the point worth stating out
        # loud, given that simple_room.launch.py next door has to pass 0.7696.
        #
        # A stock environment is not obliged to put its floor at z = 0, and the
        # two in this package disagree: Simple_Room is authored 0.7696 m below
        # the ground plane the simulator authors, this one is authored exactly
        # on it -- measured, the Floor collider tops out at z = 0.0000, and the
        # asset ships no GroundPlane of its own at all. So world.launch.py's
        # default of 0.0 is correct here, unchanged, and the robot settles on
        # the kitchen's own floor: body bounding box z = [-0.00064, +0.19116],
        # unchanged when the package's ground plane is deleted, which is the
        # test that says which surface holds it up.
        #
        # The practical consequence is that maps/kitchen is built with no
        # --world-z either. Check a new world's floor before assuming either
        # way; DESIGN.md, "Standing the world on the ground plane", has both
        # cases and the measurement.

        # The origin, unusually. It is the wrong answer in both other worlds --
        # the warehouse origin has nothing inside lidar range, the Simple_Room
        # origin is inside the table -- and the right one here: 1.525 m to the
        # nearest obstacle, 1.413 m of turning margin around a burger whose
        # circumscribed radius is 0.112 m, and 99.2% of the 360 one-degree
        # bearings returning inside 3.5 m. All 2064 of the map's occupied cells
        # are in range from here, so there is nowhere better to start. Measured
        # against the stage, generator.get_occupied_positions(), never the .pgm;
        # see DESIGN.md, "The occupancy map was flipped", for why that matters.
        DeclareLaunchArgument('x_pose', default_value='0.0'),

        DeclareLaunchArgument('y_pose', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'world.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
            }.items()
        ),
    ])
