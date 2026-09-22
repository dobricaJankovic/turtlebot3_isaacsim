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

"""Build the robot asset: turtlebot3_description's URDF -> USD."""

import argparse
import os
import shutil
import sys
import traceback

SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WHEEL_LINKS = ['wheel_left_link', 'wheel_right_link']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                        choices=['burger', 'waffle', 'waffle_pi'])
    parser.add_argument('--urdf', required=True,
                        help='xacro-expanded URDF')
    parser.add_argument('--description-share', required=True,
                        help='turtlebot3_description share directory, for '
                             'resolving package:// mesh URLs')
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    if not args.output:
        args.output = os.path.join(
            SHARE, 'models', 'turtlebot3_' + args.model,
            'turtlebot3_' + args.model + '.usd')
    return args


args = parse_args()

for path, what in ((args.urdf, 'URDF'), (args.description_share, 'description share')):
    if not os.path.exists(path):
        sys.exit('error: no {} at {}'.format(what, path))

from isaacsim import SimulationApp                                   # noqa: E402

simulation_app = SimulationApp({'headless': True})

from isaacsim.core.utils.extensions import enable_extension          # noqa: E402

enable_extension('isaacsim.asset.importer.urdf')
simulation_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade            # noqa: E402


def import_robot():
    """Convert the URDF. Returns the asset's .usda entry point."""
    if os.path.isdir(args.output):
        shutil.rmtree(args.output)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    config = URDFImporterConfig(
        urdf_path=args.urdf,
        usd_path=args.output,
        ros_package_paths=[
            {'name': 'turtlebot3_description', 'path': args.description_share},
        ],
        merge_fixed_joints=True,
        fix_base=False,
        robot_type='Wheeled',
        joint_drive_type='force',
        joint_target_type='velocity',
        override_joint_damping=1.0e5,
        override_joint_stiffness=0.0,
    )
    importer = URDFImporter()
    importer.config = config
    return importer.import_urdf()


def articulation_root(stage):
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    raise RuntimeError('the import produced no articulation root')


def physics_material(stage, path, friction, friction_combine=None):
    """Define a non-bouncing surface."""
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(friction))
    api.CreateDynamicFrictionAttr().Set(float(friction))
    api.CreateRestitutionAttr().Set(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateRestitutionCombineModeAttr().Set('min')
    if friction_combine is not None:
        physx.CreateFrictionCombineModeAttr().Set(friction_combine)
    return material


def bind(prim, material):
    """Bind for the physics purpose."""
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics')


def author_surfaces(stage, root):
    """Surface properties, authored into the asset so it is self-contained."""
    materials = root.GetPath().AppendChild('PhysicsMaterials')
    wheel = physics_material(stage, materials.AppendChild('wheel'), 1.0)
    chassis = physics_material(stage, materials.AppendChild('chassis'), 0.1, 'min')

    bind(root, chassis)

    for link in WHEEL_LINKS:
        links = [p for p in Usd.PrimRange(root) if p.GetName() == link]
        if not links:
            raise RuntimeError('no {} under {}'.format(link, root.GetPath()))
        colliders = [p for m in links for p in Usd.PrimRange(m)
                     if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders:
            raise RuntimeError('{} has no collider'.format(link))
        for collider in colliders:
            bind(collider, wheel)


def verify(stage, root):
    """Fail on an import with no renderable geometry."""
    meshes = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
              if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError(
            'no mesh geometry in {}. The package:// URLs did not resolve, so '
            'check that {} is the turtlebot3_description share directory '
            'itself, the one holding meshes/'.format(
                args.output, args.description_share))

    points = 0
    for mesh in meshes:
        attr = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        points += len(attr) if attr else 0
    if points == 0:
        raise RuntimeError('{} meshes but no points'.format(len(meshes)))
    print('geometry: {} meshes, {} points'.format(len(meshes), points), flush=True)


def main():
    asset = import_robot()
    stage = Usd.Stage.Open(asset)
    root = articulation_root(stage)
    print('articulation root: {}'.format(root.GetPath()), flush=True)

    verify(stage, root)
    author_surfaces(stage, root)
    stage.Save()

    print('wrote {}'.format(args.output), flush=True)


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
