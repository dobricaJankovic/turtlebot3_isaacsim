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
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    # The same table the simulator's DifferentialController uses, loaded by
    # path: runtime/turtlebot3_isaacsim.py cannot be imported here because it
    # pulls in Kit, and a second copy of the wheel geometry would be a scale
    # error waiting to happen -- the forward map (/cmd_vel -> wheels) and the
    # inverse (wheels -> /odom) have to agree or the difference reads as slip.
    wheels = load_geometry(pkg_isaacsim)['WHEELS'][TURTLEBOT3_MODEL]

    # headless and lidar are the simulator's own argv rather than its.
    standalone = [
        os.path.join(pkg_isaacsim, 'runtime', 'turtlebot3_isaacsim.py'),
        '--model', TURTLEBOT3_MODEL,
        '--robot', robot,
        '--world-z', cfg('world_z'),
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
            # .launch.xml, not .launch.py: NVIDIA reimplemented the launcher as
            # XML in the IsaacSim-6.1.0 tag. Same node, same arguments, plus a
            # new python_script we do not use. AnyLaunchDescriptionSource picks
            # the frontend off the extension, so this does not care which it is.
            AnyLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'run_isaacsim.launch.xml')
            ),
            launch_arguments={
                'standalone': ' '.join(standalone),
                # An IncludeLaunchDescription inherits the surrounding
                # configurations, so every name run_isaacsim.launch.py declares
                # is one we can collide with. isaac_install_path and
                # isaac_version are renamed for that reason; headless is not so
                # lucky -- it is the same word for two different things. Here it
                # is a bool going into the simulator's own argv; there it is a
                # string naming a headless *mode* ('native', 'webrtc', or empty
                # for a window), read with .string_value. Left to leak, 'false'
                # reaches launch_ros as a parameter value, which infers BOOL
                # from it, and the node dies before Kit starts with
                #   Trying to set parameter 'headless' to 'True' of type
                #   'BOOL', expecting type 'STRING'
                # Empty is the right value regardless: on the standalone path
                # run_isaacsim ignores its own headless setting.
                'headless': '',
                'install_path': cfg('isaac_install_path'),
                'version': cfg('isaac_version'),
                'ros_distro': cfg('ros_distro'),
                'use_internal_libs': cfg('use_internal_libs'),
                'exclude_install_path': cfg('exclude_install_path'),
                'dds_type': cfg('dds_type')
            }.items(),
        ),
        # /odom and the odom -> base_footprint transform, integrated from the
        # wheels. The simulator itself publishes only its chassis-prim pose, on
        # /ground_truth/odom -- see nodes/wheel_odometry.py for why the two are
        # not the same thing and why this is a ROS node rather than an
        # OmniGraph one.
        #
        # A separate process, deliberately: it runs on the system Python with
        # the system ROS 2, where rclpy and tf2_ros exist, rather than inside
        # Kit's isolated 3.12. It is also what the real robot does -- the
        # burger's odometry comes out of turtlebot3_node, not out of the motors.
        Node(
            package='turtlebot3_isaacsim',
            executable='wheel_odometry',
            name='wheel_odometry',
            namespace=cfg('namespace') or None,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'wheel_radius': wheels['radius'],
                'wheel_separation': wheels['separation'],
            }],
        ),
    ]


def load_geometry(pkg_isaacsim):
    """runtime/geometry.py as a plain dict, without importing the runtime."""
    import importlib.util
    path = os.path.join(pkg_isaacsim, 'runtime', 'geometry.py')
    spec = importlib.util.spec_from_file_location('tb3_isaac_geometry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return vars(module)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Path to an environment .usd. Empty for a ground plane'),

        # Zero leaves the world reference untransformed, which is what every
        # world that is authored floor-at-z=0 wants -- turtlebot3_world, the
        # warehouse, the bare ground plane. Simple_Room is not one of those;
        # simple_room.launch.py passes 0.7696. Whatever is passed here must
        # also be passed to scripts/build_map.py, or the map is cut from a
        # scene at a different height than the simulated one.
        DeclareLaunchArgument(
            'world_z', default_value='0.0',
            description="Metres to raise the world by. Must match build_map.py's "
                        '--world-z for the same world'),

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

        # 240, not 60: four PhysX sub-steps per rendered frame at 60 Hz,
        # which is the conventional ratio.
        #
        # The wheels do not track their commanded velocity at 60 Hz -- they
        # chatter. Commanded a steady -1.2121 rad/s the left wheel ranges over
        # -2.91 to +1.04, and the mean of that, -0.79, is the "Isaac Sim
        # under-rotates by 30%" that measurements/ recorded from 2026-09-15.
        #
        # It is the contact solve, not the drive. Lift the robot off the ground
        # and the joints track their command EXACTLY (error 0.0000 in every
        # mode); every bit of the error appears only once there is contact, and
        # sweeping the drive damping 1e3 -> 1e7 does not move it at all. What
        # does move it is the timestep: the deficit in the pivot at 0.5 rad/s
        # goes 25.5% -> 8.7% -> 3.8% across 60 / 240 / 480 Hz, and forward
        # motion from 2.1% to 0.05%.
        #
        # 240 rather than 480 is a deliberate trade: it fixes forward motion
        # completely and leaves roughly 8.7% in the pivot, for a real-time
        # factor that makes an experiment matrix affordable. The underlying
        # cause is the CYLINDRICAL wheel collider, which no solver here rolls
        # exactly -- sub-stepping damps that symptom rather than removing it.
        # Spherical colliders would remove it, and are deliberately not used:
        # a sphere contacts at a point, and a cylinder is the more faithful
        # model of a tyre for a robot that pivots on two wheels and a skid.
        #
        # Solver iteration count is deliberately NOT raised with it. That
        # lowers the mean error further while tripling the chatter, which reads
        # as a fix only if you report the mean.
        #
        # This is PhysX sub-steps per frame (set_dt -> timeStepsPerSecond), not
        # the frame rate, so the OmniGraph and every ROS topic still tick at
        # the render rate and the interface contract is unchanged.
        #
        # docs/worknotes/2026-09-17-lane-a-physics.md in tb3_sim2real has the
        # measurements.
        DeclareLaunchArgument('physics_hz', default_value='240.0'),

        DeclareLaunchArgument(
            'isaac_install_path', default_value='/isaac-sim',
            description='Isaac Sim install root'),

        DeclareLaunchArgument('isaac_version', default_value='6.1.0'),

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
