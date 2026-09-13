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

"""Turning a path argument into a USD layer add_reference_to_stage can open.

Shared by turtlebot3_isaacsim.py and build_map.py, which have to agree about
what `--world` means or a map would be built of a different scene than the one
that gets simulated. Imported, so it must not start a SimulationApp: the only
Isaac Sim import here is inside the one function that needs it, which runs long
after the app exists.
"""

import glob
import os

REMOTE_SCHEMES = ('http://', 'https://', 'omniverse://')


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
