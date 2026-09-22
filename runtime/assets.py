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

"""What `--world` and `--world-z` mean, for both programs that take them."""

import glob
import os

REMOTE_SCHEMES = ('http://', 'https://', 'omniverse://')


def set_pose(prim_path, xyz, yaw):
    """Place a prim, coping with xformOps the reference already authored."""
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
    """Raise the world reference by world_z metres, or leave it untouched."""
    if not world_z:
        return

    import omni.usd
    from pxr import Gf, UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    offset = Gf.Vec3d(0.0, 0.0, float(world_z))
    if UsdGeom.XformCommonAPI(prim).SetTranslate(offset):
        return

    ops = {op.GetOpName(): op
           for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    translate = ops.get('xformOp:translate')
    if translate is None:
        raise RuntimeError('cannot raise {}: no translate op'.format(prim_path))
    translate.Set(offset)


def asset_layer(path):
    """Resolve a generated asset to a layer USD can actually open."""
    if not os.path.isdir(path):
        return path

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
    """Turn a --world argument into something add_reference_to_stage can open."""
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
