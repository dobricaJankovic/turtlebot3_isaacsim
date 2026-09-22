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

"""The simulator. Started by launch/isaacsim.launch.py, not by hand."""

import argparse
import json
import os
import sys
import traceback

from assets import asset_layer, lift_world, resolve_world, set_pose
from geometry import WHEEL_JOINTS, WHEELS

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROBOT_PRIM = '/World/turtlebot3'
WORLD_PRIM = '/World/env'
GRAPH_PATH = '/World/ROS2Interface'
MATERIALS_PRIM = '/World/PhysicsMaterials'


SCAN_OFFSET = {
    'burger': (-0.032, 0.0, 0.182),
    'waffle': (-0.064, 0.0, 0.132),
    'waffle_pi': (-0.064, 0.0, 0.132),
}

PROFILE_PREFIX = 'omni:sensor:Core:'
PROFILE_RENAMES = {
    'reportRateBaseHz': 'patternFiringRateHz',
    'minReflectanceRange': 'minReflectionRangeM',
    'wavelengthNm': 'waveLengthNm',
}
PROFILE_TOKENS = (
    'scanType', 'intensityProcessing', 'rotationDirection', 'rayType',
    'intensityMappingType',
)
PROFILE_STRUCTURAL = ('emitterStateCount', 'emitterStates')
PROFILE_UNSUPPORTED = ('avgPowerW',)
EMITTER_STATE = 's001'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                        choices=sorted(WHEELS))
    parser.add_argument('--robot', default='')
    parser.add_argument('--world', default='')
    parser.add_argument('--world-z', type=float, default=0.0)
    parser.add_argument('--x-pose', type=float, default=0.0)
    parser.add_argument('--y-pose', type=float, default=0.0)
    parser.add_argument('--z-pose', type=float, default=0.01)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-lidar', action='store_true')
    parser.add_argument('--lidar-config', default='turtlebot3_lds')
    parser.add_argument('--physics-hz', type=float, default=240.0)
    parser.add_argument('--namespace', default='')
    args, _ = parser.parse_known_args()

    if not args.robot:
        args.robot = os.path.join(
            SHARE, 'models', 'turtlebot3_' + args.model,
            'turtlebot3_' + args.model + '.usd')
    return args


args = parse_args()

from isaacsim import SimulationApp                                   # noqa: E402

simulation_app = SimulationApp({'headless': args.headless})

import isaacsim.core.experimental.utils.app as app_utils             # noqa: E402
import isaacsim.core.experimental.utils.prim as prim_utils           # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils         # noqa: E402
import omni.graph.core as og                                         # noqa: E402
import omni.usd                                                      # noqa: E402
import usdrt.Sdf                                                     # noqa: E402
from isaacsim.core.api.materials.physics_material import PhysicsMaterial  # noqa: E402
from isaacsim.core.api.objects import GroundPlane                    # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager       # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage  # noqa: E402
from pxr import PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402


def prefixed(name):
    if not args.namespace:
        return name
    return '{}/{}'.format(args.namespace.strip('/'), name.lstrip('/'))


def articulation_root():
    """Find the prim carrying PhysicsArticulationRootAPI under the robot."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(ROBOT_PRIM)
    if not root.IsValid():
        raise RuntimeError('{} is not on the stage'.format(ROBOT_PRIM))
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError(
        'no articulation root under {}. Rebuild the robot asset with '
        'scripts/build_models.sh'.format(ROBOT_PRIM))


def build_graph(chassis):
    """Build the ROS 2 interface as one OmniGraph, ticked by playback."""
    wheels = WHEELS[args.model]
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('ComputeOdom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('ReadJointState', 'isaacsim.sensors.physics.IsaacReadJointState'),
                ('PubJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('SubTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                ('BreakLinVel', 'omni.graph.nodes.BreakVector3'),
                ('BreakAngVel', 'omni.graph.nodes.BreakVector3'),
                ('DiffController',
                 'isaacsim.robot.wheeled_robots.DifferentialController'),
                ('ArticController',
                 'isaacsim.core.nodes.IsaacArticulationController'),
            ],
            keys.CONNECT: [
                ('OnTick.outputs:tick', 'PubClock.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'ComputeOdom.inputs:execIn'),
                ('OnTick.outputs:tick', 'PubOdom.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
                ('ComputeOdom.outputs:position', 'PubOdom.inputs:position'),
                ('ComputeOdom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('ComputeOdom.outputs:linearVelocity',
                 'PubOdom.inputs:linearVelocity'),
                ('ComputeOdom.outputs:angularVelocity',
                 'PubOdom.inputs:angularVelocity'),

                ('OnTick.outputs:tick', 'ReadJointState.inputs:execIn'),
                ('ReadJointState.outputs:execOut', 'PubJointState.inputs:execIn'),
                ('ReadJointState.outputs:jointNames',
                 'PubJointState.inputs:jointNames'),
                ('ReadJointState.outputs:jointPositions',
                 'PubJointState.inputs:jointPositions'),
                ('ReadJointState.outputs:jointVelocities',
                 'PubJointState.inputs:jointVelocities'),
                ('ReadJointState.outputs:jointEfforts',
                 'PubJointState.inputs:jointEfforts'),
                ('ReadJointState.outputs:jointDofTypes',
                 'PubJointState.inputs:jointDofTypes'),
                ('ReadJointState.outputs:stageMetersPerUnit',
                 'PubJointState.inputs:stageMetersPerUnit'),
                ('SimTime.outputs:simulationTime',
                 'PubJointState.inputs:timeStamp'),

                ('OnTick.outputs:tick', 'SubTwist.inputs:execIn'),
                ('OnTick.outputs:tick', 'ArticController.inputs:execIn'),
                ('SubTwist.outputs:execOut', 'DiffController.inputs:execIn'),
                ('SubTwist.outputs:linearVelocity', 'BreakLinVel.inputs:tuple'),
                ('BreakLinVel.outputs:x', 'DiffController.inputs:linearVelocity'),
                ('SubTwist.outputs:angularVelocity', 'BreakAngVel.inputs:tuple'),
                ('BreakAngVel.outputs:z', 'DiffController.inputs:angularVelocity'),
                ('DiffController.outputs:velocityCommand',
                 'ArticController.inputs:velocityCommand'),
            ],
            keys.SET_VALUES: [
                ('PubClock.inputs:topicName', '/clock'),

                ('ComputeOdom.inputs:chassisPrim', [usdrt.Sdf.Path(chassis)]),
                ('PubOdom.inputs:topicName', prefixed('/ground_truth/odom')),
                ('PubOdom.inputs:odomFrameId', prefixed('odom')),
                ('PubOdom.inputs:chassisFrameId', prefixed('base_footprint')),

                ('PubJointState.inputs:topicName', prefixed('/joint_states')),
                ('ReadJointState.inputs:prim', [usdrt.Sdf.Path(chassis)]),

                ('SubTwist.inputs:topicName', prefixed('/cmd_vel')),
                ('DiffController.inputs:wheelRadius', wheels['radius']),
                ('DiffController.inputs:wheelDistance', wheels['separation']),
                ('ArticController.inputs:targetPrim', [usdrt.Sdf.Path(chassis)]),
                ('ArticController.inputs:jointNames', WHEEL_JOINTS),
            ],
        },
    )


def profile_attributes(profile):
    """Translate a lidar profile into OmniLidar attributes."""
    if len(profile['emitterStates']) != 1:
        raise RuntimeError(
            '{} emitter states in the profile; only {} is authored'.format(
                len(profile['emitterStates']), EMITTER_STATE))

    attributes = {}
    for key, value in profile.items():
        if key in PROFILE_STRUCTURAL or key in PROFILE_UNSUPPORTED:
            continue
        if key in PROFILE_TOKENS:
            value = value.upper()
        attributes[PROFILE_PREFIX + PROFILE_RENAMES.get(key, key)] = value

    for key, value in profile['emitterStates'][0].items():
        attributes['{}emitterState:{}:{}'.format(
            PROFILE_PREFIX, EMITTER_STATE, key)] = value

    return attributes


def attach_lidar(chassis):
    """RTX lidar -> /scan. The returned sensor must be kept alive."""
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    folder = os.path.join(SHARE, 'models', 'lidar_configs')
    config_path = os.path.join(folder, args.lidar_config + '.json')
    if not os.path.isfile(config_path):
        raise RuntimeError('no lidar profile {} in {}'.format(
            args.lidar_config, folder))

    with open(config_path) as f:
        profile = json.load(f)['profile']
    attributes = profile_attributes(profile)

    lidar = Lidar.create(
        path=chassis + '/lidar',
        attributes=attributes,
        accumulate_outputs=True,
        tick_rate=float(profile['scanRateBaseHz']),
        translations=[list(SCAN_OFFSET[args.model])],
    )

    prim = prim_utils.get_prim_at_path(lidar.paths[0])

    unknown = sorted(a for a in attributes if not prim.HasAttribute(a))
    if unknown:
        raise RuntimeError(
            'the OmniLidar schema has no {}. {} and the schema have '
            'drifted'.format(', '.join(unknown), config_path))

    scan_hz = float(prim.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get() or 0)
    firing_hz = int(prim.GetAttribute('omni:sensor:Core:patternFiringRateHz').Get() or 0)
    near = float(prim.GetAttribute('omni:sensor:Core:nearRangeM').Get() or 0)
    far = float(prim.GetAttribute('omni:sensor:Core:farRangeM').Get() or 0)
    if scan_hz <= 0 or firing_hz <= 0:
        raise RuntimeError('lidar prim has a zero scan or firing rate')

    expected = profile['scanRateBaseHz']
    if abs(scan_hz - float(expected)) > 1e-6:
        raise RuntimeError(
            'lidar resolved to another profile: {} Hz on the prim, {} Hz in '
            '{}'.format(scan_hz, expected, config_path))

    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        'RtxLidarROS2PublishLaserScan',
        topicName=prefixed('scan'),
        frameId=prefixed('base_scan'),
        horizontalFov=360.0,
        horizontalResolution=360.0 * scan_hz / firing_hz,
        depthRange=[near, far],
        rotationRate=scan_hz,
        azimuthRange=[-180.0, 180.0],
    )
    print('lidar: {} at {} Hz, {:.3f} deg/sample, range {}-{} m'.format(
        args.lidar_config, scan_hz, 360.0 * scan_hz / firing_hz, near, far),
        flush=True)
    return sensor


def build_stage():
    """Ground plane, light, world reference, robot reference."""
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, '/World')

    floor = PhysicsMaterial(prim_path=MATERIALS_PRIM + '/floor',
                            static_friction=1.0, dynamic_friction=1.0,
                            restitution=0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        floor.prim).CreateRestitutionCombineModeAttr().Set('min')
    GroundPlane(prim_path='/World/GroundPlane', physics_material=floor)
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath('/World/GroundPlane')).Bind(
            floor.material, UsdShade.Tokens.weakerThanDescendants, 'physics')

    UsdLux.DistantLight.Define(stage, '/World/DistantLight').CreateIntensityAttr(1000)

    if args.world:
        add_reference_to_stage(usd_path=resolve_world(args.world),
                               prim_path=WORLD_PRIM)
        simulation_app.update()

    if not os.path.exists(args.robot):
        raise RuntimeError(
            'no robot asset at {}. Build it with '
            'scripts/build_models.sh'.format(args.robot))
    add_reference_to_stage(usd_path=asset_layer(args.robot), prim_path=ROBOT_PRIM)
    simulation_app.update()
    while stage_utils.is_stage_loading():
        simulation_app.update()

    lift_world(WORLD_PRIM, args.world_z)

    xyz = (args.x_pose, args.y_pose, args.z_pose)
    set_pose(ROBOT_PRIM, xyz, args.yaw)
    print('world: {}{}'.format(
        args.world or 'none (ground plane)',
        ' raised {:g} m'.format(args.world_z) if args.world_z else ''), flush=True)
    print('robot: {} at {} yaw {:g}'.format(
        args.robot, [round(v, 3) for v in xyz], args.yaw), flush=True)


def check_surfaces():
    """Refuse a robot asset whose colliders carry no surface properties."""
    stage = omni.usd.get_context().get_stage()
    bad = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            'physics')
        if not material:
            bad.append('{}: no physics material'.format(prim.GetPath()))
            continue
        attr = UsdPhysics.MaterialAPI(material.GetPrim()).GetRestitutionAttr()
        if (attr.Get() if attr else None) != 0.0:
            bad.append('{}: restitution {}'.format(prim.GetPath(), attr.Get()))
    if bad:
        raise RuntimeError(
            '{} has bouncy or unset surfaces, so the robot will drift with no '
            'command given. Rebuild it with scripts/build_models.sh:\n  '
            '{}'.format(args.robot, '\n  '.join(bad)))


def main():
    app_utils.enable_extension('isaacsim.ros2.bridge')
    simulation_app.update()

    build_stage()
    check_surfaces()

    chassis = articulation_root()
    build_graph(chassis)

    lidar_sensor = None
    if not args.no_lidar:
        lidar_sensor = attach_lidar(chassis)

    SimulationManager.setup_simulation(dt=1.0 / args.physics_hz, device='cpu')
    simulation_app.update()

    app_utils.play()
    simulation_app.update()

    print('Stage loaded and simulation is playing. ROS_DOMAIN_ID={}'.format(
        os.environ.get('ROS_DOMAIN_ID', 'unset')), flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    del lidar_sensor


if __name__ == '__main__':
    status = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        status = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(status)
