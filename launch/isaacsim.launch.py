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
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def colcon_overlays():
    """Return the sourced colcon prefixes, comma separated.

    These must not follow Kit onto PYTHONPATH/LD_LIBRARY_PATH: it runs Python
    3.12 and a ROS 2 overlay built for 3.10 is not importable there. See
    DESIGN.md, "run_isaacsim.py".
    """
    prefixes = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)
    return ','.join(p for p in prefixes if p)


def launch_setup(context):
    pkg_isaacsim = get_package_share_directory('turtlebot3_isaacsim')
    pkg_bringup = get_package_share_directory('isaacsim_bringup')

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    robot = cfg('robot') or os.path.join(
        pkg_isaacsim, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL,
        'turtlebot3_' + TURTLEBOT3_MODEL + '.usd')

    # run_isaacsim ignores every other argument on the standalone path, so
    # headless and lidar are the simulator's own argv rather than its.
    standalone = [
        os.path.join(pkg_isaacsim, 'scripts', 'turtlebot3_isaacsim.py'),
        '--model', TURTLEBOT3_MODEL,
        '--robot', robot,
        '--x-pose', cfg('x_pose'),
        '--y-pose', cfg('y_pose'),
        '--z-pose', cfg('z_pose'),
        '--yaw', cfg('yaw'),
        '--physics-hz', cfg('physics_hz'),
        '--lidar-config', cfg('lidar_config'),
    ]
    if cfg('world'):
        standalone += ['--world', cfg('world')]
    if cfg('namespace'):
        standalone += ['--namespace', cfg('namespace')]
    if cfg('headless').lower() in ('true', '1'):
        standalone += ['--headless']
    if cfg('lidar').lower() in ('false', '0'):
        standalone += ['--no-lidar']

    # run_isaacsim pastes this into a shell command unquoted, so a space
    # anywhere in it would silently become two arguments.
    spaced = [arg for arg in standalone if ' ' in arg]
    if spaced:
        raise RuntimeError('arguments must not contain spaces: {}'.format(spaced))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'run_isaacsim.launch.py')
            ),
            launch_arguments={
                'standalone': ' '.join(standalone),
                'install_path': cfg('isaac_install_path'),
                'version': cfg('isaac_version'),
                'ros_distro': cfg('ros_distro'),
                'use_internal_libs': cfg('use_internal_libs'),
                'exclude_install_path': cfg('exclude_install_path'),
                'dds_type': cfg('dds_type')
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Path to an environment .usd. Empty for a ground plane'),

        DeclareLaunchArgument(
            'robot', default_value='',
            description="Path to the robot .usd. Empty for this package's own"),

        DeclareLaunchArgument('x_pose', default_value='0.0'),

        DeclareLaunchArgument('y_pose', default_value='0.0'),

        DeclareLaunchArgument('z_pose', default_value='0.01'),

        DeclareLaunchArgument('yaw', default_value='0.0'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Topic and frame prefix, for multi-robot'),

        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Isaac Sim with no window'),

        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='Publish /scan'),

        DeclareLaunchArgument(
            'lidar_config', default_value='turtlebot3_lds',
            description='RTX lidar profile from models/lidar_configs'),

        DeclareLaunchArgument('physics_hz', default_value='60.0'),

        DeclareLaunchArgument(
            'isaac_install_path', default_value='/isaac-sim',
            description='Isaac Sim install root'),

        DeclareLaunchArgument('isaac_version', default_value='6.0.1'),

        DeclareLaunchArgument(
            'ros_distro', default_value=os.environ.get('ROS_DISTRO', 'humble')),

        DeclareLaunchArgument(
            'use_internal_libs', default_value='true',
            description='Use the ROS 2 libraries bundled with Isaac Sim'),

        DeclareLaunchArgument(
            'exclude_install_path', default_value=colcon_overlays(),
            description="Overlays to keep off Isaac Sim's search paths"),

        DeclareLaunchArgument(
            'dds_type', default_value='',
            description='Empty keeps the surrounding RMW_IMPLEMENTATION'),

        OpaqueFunction(function=launch_setup),
    ])
