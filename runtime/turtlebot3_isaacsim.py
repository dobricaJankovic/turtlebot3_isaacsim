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

"""The simulator. Started by launch/isaacsim.launch.py, not by hand.

Runs on Isaac Sim's Python, with the system ROS 2 stripped from its search
paths, so rclpy and ament_index_python are unavailable here and every path
arrives as an argument. See DESIGN.md.
"""

import argparse
import json
import os
import sys
import traceback

from assets import asset_layer, lift_world, resolve_world, set_pose

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROBOT_PRIM = '/World/turtlebot3'
WORLD_PRIM = '/World/env'
GRAPH_PATH = '/World/ROS2Interface'
MATERIALS_PRIM = '/World/PhysicsMaterials'

WHEEL_JOINTS = ['wheel_left_joint', 'wheel_right_joint']

# Wheel separation and radius, as turtlebot3_gazebo's model.sdf states them.
WHEELS = {
    'burger': {'separation': 0.160, 'radius': 0.033},
    'waffle': {'separation': 0.287, 'radius': 0.033},
    'waffle_pi': {'separation': 0.287, 'radius': 0.033},
}

# base_footprint -> base_scan, composed from turtlebot3_description's own
# base_joint (base_footprint -> base_link) and scan_joint (base_link ->
# base_scan), NOT scan_joint's origin alone -- that is relative to base_link,
# which sits 0.010 m above base_footprint on all three models. waffle and
# waffle_pi's z here were off by exactly that 0.010 m until 2026-09-17,
# caught by scripts/smoke_test.py comparing against the URDF; burger's has
# been correct throughout (verified in DESIGN.md, "Where the lidar sits").
#   burger:    base_joint (0, 0, 0.010) + scan_joint (-0.032, 0, 0.172)
#   waffle(_pi): base_joint (0, 0, 0.010) + scan_joint (-0.064, 0, 0.122)
SCAN_OFFSET = {
    'burger': (-0.032, 0.0, 0.182),
    'waffle': (-0.064, 0.0, 0.132),
    'waffle_pi': (-0.064, 0.0, 0.132),
}

# How models/lidar_configs/*.json maps onto the OmniLidar prim. Isaac Sim 6.0
# dropped the JSON profile registry from the Python API -- Lidar.create(config=)
# now names a stock USD asset out of SUPPORTED_LIDAR_CONFIGS, so a profile the
# package ships has to be authored onto the prim attribute by attribute. Most
# names carry over; these are the ones that do not.
PROFILE_PREFIX = 'omni:sensor:Core:'
PROFILE_RENAMES = {
    'reportRateBaseHz': 'patternFiringRateHz',
    'minReflectanceRange': 'minReflectionRangeM',
    'wavelengthNm': 'waveLengthNm',
}
# Enumerations are lower or mixed case in a profile, upper case on the prim.
PROFILE_TOKENS = (
    'scanType', 'intensityProcessing', 'rotationDirection', 'rayType',
    'intensityMappingType',
)
# Read from the profile's shape rather than copied across as an attribute.
PROFILE_STRUCTURAL = ('emitterStateCount', 'emitterStates')
# In the profile format, absent from the 6.0 schema. avgPowerW described the
# emitter's average power; the schema exposes peakPowerW, which is a different
# number, so it is dropped rather than guessed at.
PROFILE_UNSUPPORTED = ('avgPowerW',)
# The single emitter state the schema applies by default. A rotary 2D lidar
# needs exactly one; nothing here authors a second.
EMITTER_STATE = 's001'


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                        choices=sorted(WHEELS))
    parser.add_argument('--robot', default='')
    parser.add_argument('--world', default='')
    # Metres to raise the world reference by, so that an environment
    # whose floor is not at z=0 meets the ground plane this script
    # authors there. build_map.py takes the same argument and MUST be
    # given the same value; see assets.py.
    parser.add_argument('--world-z', type=float, default=0.0)
    parser.add_argument('--x-pose', type=float, default=0.0)
    parser.add_argument('--y-pose', type=float, default=0.0)
    parser.add_argument('--z-pose', type=float, default=0.01)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-lidar', action='store_true')
    parser.add_argument('--lidar-config', default='turtlebot3_lds')
    # 480 Hz of PhysX sub-steps, not 60. At 60 the wheels chatter rather than
    # track -- it is the contact solve, not the drive, and no gain fixes it.
    # The reasoning and the measurements are on the matching launch argument in
    # launch/isaacsim.launch.py.
    parser.add_argument('--physics-hz', type=float, default=480.0)
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
    """Find the prim carrying PhysicsArticulationRootAPI under the robot.

    Searched rather than hard coded: the URDF importer has moved it between
    releases, and a stale path yields an inert robot with no error.
    """
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
                # OnPlaybackTick is in omni.graph.action, not isaacsim.core.nodes.
                ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('ComputeOdom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubRawTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
                ('PubJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('SubTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                # ROS2SubscribeTwist emits vectord[3], DifferentialController
                # takes scalars, so the two cannot be wired directly.
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
                ('OnTick.outputs:tick', 'PubRawTF.inputs:execIn'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
                ('SimTime.outputs:simulationTime', 'PubRawTF.inputs:timeStamp'),
                ('ComputeOdom.outputs:position', 'PubOdom.inputs:position'),
                ('ComputeOdom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('ComputeOdom.outputs:linearVelocity',
                 'PubOdom.inputs:linearVelocity'),
                ('ComputeOdom.outputs:angularVelocity',
                 'PubOdom.inputs:angularVelocity'),
                ('ComputeOdom.outputs:position', 'PubRawTF.inputs:translation'),
                ('ComputeOdom.outputs:orientation', 'PubRawTF.inputs:rotation'),

                ('OnTick.outputs:tick', 'PubJointState.inputs:execIn'),
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
                ('PubOdom.inputs:topicName', prefixed('/odom')),
                ('PubOdom.inputs:odomFrameId', prefixed('odom')),
                ('PubOdom.inputs:chassisFrameId', prefixed('base_footprint')),

                # Raw, not a full transform tree: everything below
                # base_footprint belongs to robot_state_publisher.
                ('PubRawTF.inputs:topicName', '/tf'),
                ('PubRawTF.inputs:parentFrameId', prefixed('odom')),
                ('PubRawTF.inputs:childFrameId', prefixed('base_footprint')),

                ('PubJointState.inputs:topicName', prefixed('/joint_states')),
                ('PubJointState.inputs:targetPrim', [usdrt.Sdf.Path(chassis)]),

                ('SubTwist.inputs:topicName', prefixed('/cmd_vel')),
                ('DiffController.inputs:wheelRadius', wheels['radius']),
                ('DiffController.inputs:wheelDistance', wheels['separation']),
                ('ArticController.inputs:targetPrim', [usdrt.Sdf.Path(chassis)]),
                ('ArticController.inputs:jointNames', WHEEL_JOINTS),
            ],
        },
    )


def profile_attributes(profile):
    """Translate a lidar profile into OmniLidar attributes.

    The profile is the package's sensor model and stays the source of truth;
    this only restates it in the names and the case the schema uses. An
    emitterStates entry becomes the attributes of one emitter state, which is
    the schema's way of spelling the same table.
    """
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
    """RTX lidar -> /scan. The returned sensor must be kept alive.

    It owns the render product the writer draws from; letting it go out of
    scope tears that down and /scan silently never appears.
    """
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
        # Both of these are "keep whatever the asset authored" by default, which
        # is right for a stock asset and wrong for a prim built from a profile:
        # what stands instead is the schema's own default, and the schema does
        # not know this lidar turns five times a second.
        #
        # accumulateOutputs off makes the model emit each render frame's slice
        # of the sweep separately, and tickRate is 10 Hz against a 5 Hz
        # revolution, so a tick lands mid-scan and never on a whole one. Either
        # way the writer never sees a full revolution, and /scan is advertised
        # and then silent -- no error, no warning, just no messages.
        accumulate_outputs=True,
        tick_rate=float(profile['scanRateBaseHz']),
        translations=[list(SCAN_OFFSET[args.model])],
    )

    prim = prim_utils.get_prim_at_path(lidar.paths[0])

    # Lidar.create only logs a warning for an attribute the prim does not have,
    # which would leave that line of the profile quietly unapplied.
    unknown = sorted(a for a in attributes if not prim.HasAttribute(a))
    if unknown:
        raise RuntimeError(
            'the OmniLidar schema has no {}. {} and the schema have '
            'drifted'.format(', '.join(unknown), config_path))

    # Read back rather than trust the file: what the prim carries is what the
    # renderer scans with, and these numbers are what say so.
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

    # The writer does not read these off the prim, so pass them explicitly.
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

    # GroundPlane authors restitution 0.8 when handed no material, and PhysX
    # averages restitution, so the robot rocks on its caster skid and creeps
    # with nothing commanding it. 'min' makes 0 hold against any other surface.
    floor = PhysicsMaterial(prim_path=MATERIALS_PRIM + '/floor',
                            static_friction=1.0, dynamic_friction=1.0,
                            restitution=0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        floor.prim).CreateRestitutionCombineModeAttr().Set('min')
    GroundPlane(prim_path='/World/GroundPlane', physics_material=floor)
    # GroundPlane binds only its mesh collider, leaving the infinite
    # collisionPlane beside it on the PhysX fallback material.
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

    # After the loading loop, not before it: lift_world's fallback path reads
    # the xformOps the referenced layer authored, and those do not exist until
    # the reference has composed.
    lift_world(WORLD_PRIM, args.world_z)

    xyz = (args.x_pose, args.y_pose, args.z_pose)
    set_pose(ROBOT_PRIM, xyz, args.yaw)
    print('world: {}{}'.format(
        args.world or 'none (ground plane)',
        ' raised {:g} m'.format(args.world_z) if args.world_z else ''), flush=True)
    print('robot: {} at {} yaw {:g}'.format(
        args.robot, [round(v, 3) for v in xyz], args.yaw), flush=True)


def check_surfaces():
    """Refuse a robot asset whose colliders carry no surface properties.

    Such an asset loads, plays and publishes correctly and simply never
    settles, which reads as a physics-tuning problem rather than a stale build.
    """
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

    # Bound for its lifetime, not discarded: see attach_lidar().
    lidar_sensor = None
    if not args.no_lidar:
        lidar_sensor = attach_lidar(chassis)

    SimulationManager.setup_simulation(dt=1.0 / args.physics_hz, device='cpu')
    simulation_app.update()

    # Nothing publishes while the timeline is stopped.
    app_utils.play()
    simulation_app.update()

    print('Stage loaded and simulation is playing. ROS_DOMAIN_ID={}'.format(
        os.environ.get('ROS_DOMAIN_ID', 'unset')), flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    del lidar_sensor


if __name__ == '__main__':
    # Reported here because os._exit() below ends the process before Python
    # would print a traceback, making every failure look like a clean exit.
    status = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        status = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        # simulation_app.close() races a task-pool teardown and aborts.
        os._exit(status)
