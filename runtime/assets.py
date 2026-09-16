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

"""What `--world` and `--world-z` mean, for both programs that take them.

Shared by turtlebot3_isaacsim.py and build_map.py, which have to agree about
what `--world` means or a map would be built of a different scene than the one
that gets simulated. `--world-z` is here for exactly the same reason and is the
sharper edge of the two: it MOVES the scene, so a simulator that applies it and
a map builder that does not would produce a map of the right room at the wrong
height, which ray-casts cleanly, loads in nav2 and localises the robot into a
scene that is not there. lift_world() below is the single implementation both
call with the same argument. See DESIGN.md, "Standing the world on the ground
plane".

Imported, so it must not start a SimulationApp: every Isaac Sim and USD import
here is inside the function that needs it, which runs long after the app exists.
"""

import glob
import os

REMOTE_SCHEMES = ('http://', 'https://', 'omniverse://')


def set_pose(prim_path, xyz, yaw):
    """Place a prim, coping with xformOps the reference already authored.

    XformCommonAPI cannot author a rotateXYZ over an `orient` op, and reports
    it by returning False rather than raising.

    Used on the robot and on the world reference both. pxr is imported here
    rather than at module scope because this module is imported before
    SimulationApp exists and Kit is particular about who loads USD first.
    """
    import math

    import omni.usd
    from pxr import Gf, UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    common = UsdGeom.XformCommonAPI(prim)
    if (common.SetTranslate(Gf.Vec3d(*[float(v) for v in xyz])) and
            common.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)),
                             UsdGeom.XformCommonAPI.RotationOrderXYZ)):
        return

    ops = {op.GetOpName(): op
           for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    translate = ops.get('xformOp:translate')
    if translate is None:
        raise RuntimeError('cannot place {}: no translate op'.format(prim_path))
    translate.Set(Gf.Vec3d(*[float(v) for v in xyz]))
    orient = ops.get('xformOp:orient')
    if orient is not None:
        half = yaw / 2.0
        quat = Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
        orient.Set(Gf.Quatf(quat) if orient.GetTypeName() == 'quatf' else quat)
    elif yaw:
        raise RuntimeError('cannot rotate {}: no orient op'.format(prim_path))


def lift_world(prim_path, world_z):
    """Raise the world reference by world_z metres, or leave it untouched.

    Zero authors nothing at all, deliberately: the default has to leave every
    world that was already right -- turtlebot3_world, the warehouse, the bare
    ground plane -- composing exactly as it did before this argument existed.

    Deliberately NOT set_pose(): that function also authors a rotation, and a
    zero one, because placing the robot means setting its yaw. Handed a world
    whose root prim carries a rotation of its own -- which is a stock asset's
    business, not ours -- it would silently level the scene while raising it.
    Simple_Room's root happens to be identity, so both spellings agree there
    and the difference would not have shown up until some other world hit it.
    Raising is a translation; this authors a translation and nothing else.
    """
    if not world_z:
        return

    import omni.usd
    from pxr import Gf, UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    offset = Gf.Vec3d(0.0, 0.0, float(world_z))
    if UsdGeom.XformCommonAPI(prim).SetTranslate(offset):
        return

    # Same fallback as set_pose, and for the same reason: XformCommonAPI
    # refuses a prim whose reference already authored ops it cannot express,
    # and reports it by returning False rather than raising.
    ops = {op.GetOpName(): op
           for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    translate = ops.get('xformOp:translate')
    if translate is None:
        raise RuntimeError('cannot raise {}: no translate op'.format(prim_path))
    translate.Set(offset)


def asset_layer(path):
    """Resolve a generated asset to a layer USD can actually open.

    Isaac Sim 6.0's URDF importer does not write a file: it writes an *asset
    structure*, so models/turtlebot3_burger/turtlebot3_burger.usd is a
    directory holding payloads/ beside the entry point
    turtlebot3_burger/turtlebot3_burger.usda. That is the path
    import_turtlebot3.py returns and the one to reference here -- handed the
    directory, add_reference_to_stage reports it as "wasn't found", which reads
    exactly like an asset that was never built.

    A plain file passes straight through, so a hand-authored .usd still works.
    """
    if not os.path.isdir(path):
        return path

    # The importer names the inner folder and its layer after the URDF's robot
    # name, which is the file stem for anything built by build_models.sh. The
    # glob covers an asset whose robot name differs; both patterns sit one level
    # down, which is what keeps payloads/*.usda out of the answer.
    stem = os.path.splitext(os.path.basename(path))[0]
    candidates = [os.path.join(path, stem, stem + ext)
                  for ext in ('.usda', '.usdc', '.usd')]
    candidates += sorted(glob.glob(os.path.join(path, '*', '*.usd*')))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    raise RuntimeError(
        '{} is a directory with no USD layer inside it. Rebuild the asset with '
        'scripts/build_models.sh'.format(path))


def resolve_world(path):
    """Turn a --world argument into something add_reference_to_stage can open.

    Three spellings, because a world can come from three places:

      /ws/.../turtlebot3_world.usd   a file this package generated
      /Isaac/Environments/...        one of Isaac Sim's stock environments,
                                     named by its path under the asset root
      https://.../warehouse.usd      any URL, taken as given

    The stock environments are not shipped in the container. They are resolved
    through get_assets_root_path(), which answers with the S3 bucket, a Nucleus
    server or a local asset pack depending on ISAACSIM_ASSET_ROOT -- so the same
    launch file works online and offline. See UPSTREAM.md, "Asset root
    resolution". A local path is tried first, so a file that really does live at
    /Isaac/... still wins over the asset root.
    """
    if path.startswith(REMOTE_SCHEMES):
        return path

    if os.path.exists(path):
        return asset_layer(path)

    if path.startswith(('/Isaac/', '/NVIDIA/')):
        from isaacsim.storage.native import get_assets_root_path
        root = get_assets_root_path()
        if not root:
            raise RuntimeError(
                'no Isaac Sim asset root, so {} cannot be resolved. Set '
                'ISAACSIM_ASSET_ROOT to a local asset pack, or give the '
                'container network access.'.format(path))
        return root.rstrip('/') + path

    raise RuntimeError(
        'no world at {}. Worlds are generated, see worlds/ -- or name a stock '
        'environment as /Isaac/Environments/...'.format(path))
