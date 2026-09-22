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

"""Headless checks on a built robot asset, independent of running the sim."""

import argparse
import ast
import json
import os
import sys

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

BASE_JOINT_Z = 0.010
SCAN_JOINT_XZ = {
    'burger': (-0.032, 0.172),
    'waffle': (-0.064, 0.122),
    'waffle_pi': (-0.064, 0.122),
}

EXPECTED_LINKS = ('base_footprint', 'wheel_left_link', 'wheel_right_link')
MERGED_LINKS = ('base_link', 'base_scan', 'imu_link', 'caster_back_link')

FAILURES = []


def check(ok, message):
    print(('PASS' if ok else 'FAIL') + '  ' + message, flush=True)
    if not ok:
        FAILURES.append(message)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                        choices=sorted(SCAN_JOINT_XZ))
    parser.add_argument('--robot', default='')
    parser.add_argument('--lidar-config', default='turtlebot3_lds')
    args = parser.parse_args()
    if not args.robot:
        args.robot = os.path.join(
            SHARE, 'models', 'turtlebot3_' + args.model,
            'turtlebot3_' + args.model + '.usd')
    return args


def check_scan_offset(model):
    """Cross-check SCAN_OFFSET against the URDF, parsed with ast."""
    path = os.path.join(SHARE, 'runtime', 'turtlebot3_isaacsim.py')
    tree = ast.parse(open(path).read(), filename=path)
    offsets = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and \
                any(isinstance(t, ast.Name) and t.id == 'SCAN_OFFSET' for t in node.targets):
            offsets = ast.literal_eval(node.value)
            break
    if offsets is None:
        check(False, 'SCAN_OFFSET not found in {}'.format(path))
        return

    x, z_scan = SCAN_JOINT_XZ[model]
    expected = (x, 0.0, round(BASE_JOINT_Z + z_scan, 6))
    got = tuple(offsets.get(model, ()))
    check(got == expected,
          'SCAN_OFFSET[{!r}] == base_joint + scan_joint from the URDF: '
          'got {}, expected {}'.format(model, got, expected))


def check_lidar_profile(config_name):
    path = os.path.join(SHARE, 'models', 'lidar_configs', config_name + '.json')
    if not os.path.isfile(path):
        check(False, 'no lidar profile at {}'.format(path))
        return
    with open(path) as f:
        data = json.load(f)
    profile = data.get('profile', {})
    required = ('scanRateBaseHz', 'nearRangeM', 'farRangeM',
                'rangeResolutionM', 'rangeAccuracyM', 'emitterStates')
    missing = [k for k in required if k not in profile]
    check(not missing,
          '{} has every top-level field profile_attributes() reads: missing {}'.format(
              path, missing or 'none'))
    states = profile.get('emitterStates', [])
    check(len(states) == 1,
          '{} has exactly one emitter state (profile_attributes() only '
          'authors s001): found {}'.format(path, len(states)))
    if states:
        missing_state = [k for k in ('azimuthDeg', 'elevationDeg', 'fireTimeNs')
                          if k not in states[0]]
        check(not missing_state,
              '{} emitter state s001 has every field the schema needs: missing {}'.format(
                  path, missing_state or 'none'))


def main():
    args = parse_args()

    check_scan_offset(args.model)
    check_lidar_profile(args.lidar_config)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({'headless': True})

    from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
    from pxr import Usd, UsdPhysics
    sys.path.insert(0, os.path.join(SHARE, 'runtime'))
    from assets import asset_layer                                   # noqa: E402

    create_new_stage()
    layer = asset_layer(args.robot)
    prim_path = '/World/turtlebot3'
    add_reference_to_stage(usd_path=layer, prim_path=prim_path)
    simulation_app.update()

    import omni.usd
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    check(root_prim.IsValid(), '{} is on the stage ({})'.format(prim_path, layer))

    roots = [p for p in Usd.PrimRange(root_prim)
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check(len(roots) == 1,
          'exactly one PhysicsArticulationRootAPI prim under {}: found {}'.format(
              prim_path, [str(p.GetPath()) for p in roots]))

    names = {p.GetName() for p in Usd.PrimRange(root_prim)}
    missing = [n for n in EXPECTED_LINKS if n not in names]
    check(not missing, 'expected links present: missing {}'.format(missing or 'none'))
    present = [n for n in MERGED_LINKS if n in names]
    check(not present,
          'merge_fixed_joints-folded links absent as separate prims '
          '(present would mean the import stopped merging them): found {}'.format(
              present or 'none'))

    if roots:
        joints = [p for p in Usd.PrimRange(roots[0])
                  if p.GetName() in ('wheel_left_joint', 'wheel_right_joint')]
        check(len(joints) == 2,
              'both wheel joints present under the articulation root: found {}'.format(
                  [str(p.GetPath()) for p in joints]))
        for joint in joints:
            damping = joint.GetAttribute('drive:angular:physics:damping')
            stiffness = joint.GetAttribute('drive:angular:physics:stiffness')
            print('INFO  {} damping={} stiffness={} (not asserted -- see '
                  "tonight's F3/F4/A1/A2 in docs/worknotes/)".format(
                      joint.GetPath(),
                      damping.Get() if damping else None,
                      stiffness.Get() if stiffness else None))
        variant_sets = root_prim.GetVariantSets()
        if variant_sets.HasVariantSet('Physics'):
            print('INFO  Physics variant selected on the asset: {}'.format(
                variant_sets.GetVariantSet('Physics').GetVariantSelection()))

    simulation_app.close()

    print()
    if FAILURES:
        print('{} check(s) failed:'.format(len(FAILURES)))
        for f in FAILURES:
            print('  - ' + f)
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
