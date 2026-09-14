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

"""Generate the nav2 map of a world, as a .pgm/.yaml pair.

    isaacsim-python scripts/build_map.py \
        --world /Isaac/Environments/Simple_Warehouse/warehouse.usd \
        --output maps/warehouse

A world whose floor is not at z=0 needs `--world-z`, and needs the SAME value
the simulator is given, or the map describes a scene at a different height than
the one being driven through. README.md, "Maps", carries both commands.

`turtlebot3_navigation2` ships a map for `turtlebot3_world` and for nothing
else, so any other world needs one built. This is Isaac Sim's own occupancy map
generator -- Tools > Robotics > Occupancy Map in the GUI,
`isaacsim.asset.gen.omap` here -- which is the documented substitute for a
hand-drawn map; see UPSTREAM.md, "Worlds". It ray-casts the stage's collision
geometry, so it maps what the lidar can actually hit rather than what the
renderer draws.

Runs on Isaac Sim's Python, like scripts/turtlebot3_isaacsim.py.
"""

import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True,
                        help='Same spelling as the simulator: a .usd path, '
                             '/Isaac/... under the asset root, or a URL')
    parser.add_argument('--output', required=True,
                        help='Path without extension; .pgm and .yaml are written')
    parser.add_argument('--cell-size', type=float, default=0.05,
                        help='Metres per pixel. nav2 convention is 0.05')

    # Must match the simulator's --world-z for the same world, and there is no
    # way for this script to check that: it composes its own stage. Pass the
    # two from one place -- a launch file -- or write the number down once.
    # Disagreeing by so much as the 0.77 m Simple_Room needs yields a map of
    # the right room at the wrong height, which ray-casts cleanly and is wrong
    # in exactly the silent way the mirrored map was. See assets.py.
    parser.add_argument('--world-z', type=float, default=0.0,
                        help="Raise the world by this much, as the simulator's "
                             '--world-z does. The two MUST agree')

    # The slice the map is cut from. A 2D occupancy map is a horizontal section
    # of a 3D world, and the only section that matters is the one the lidar
    # sweeps: too low and the floor fills the map, too high and it misses the
    # shelf legs the robot would hit. The default band spans both scanner
    # heights -- base_scan is at 0.182 m on a burger and 0.122 m on a waffle.
    #
    # A band wider than the beam is not free: it records anything standing
    # anywhere in that 15 cm, while the scan only ever reports what the one
    # plane hits. In a warehouse of open racks that gap is visible -- the map
    # shows a rack 2.5 m away and the beam passes cleanly between its uprights,
    # so AMCL is matching against walls the robot cannot see. Narrow the band
    # onto a single scanner to close it: --z-min 0.17 --z-max 0.19 for a burger.
    parser.add_argument('--z-min', type=float, default=0.10)
    parser.add_argument('--z-max', type=float, default=0.25)

    # How far out to map, as a box around the origin. Generous by default: a
    # cell that no ray reaches is recorded as unknown, which costs one byte and
    # is what nav2 expects outside the walls anyway.
    parser.add_argument('--bounds', type=float, default=30.0,
                        help='Half-extent in metres, or --x-min/--x-max etc.')
    parser.add_argument('--x-min', type=float)
    parser.add_argument('--x-max', type=float)
    parser.add_argument('--y-min', type=float)
    parser.add_argument('--y-max', type=float)

    args = parser.parse_args()
    for axis in ('x', 'y'):
        for end, sign in (('min', -1.0), ('max', 1.0)):
            name = '{}_{}'.format(axis, end)
            if getattr(args, name) is None:
                setattr(args, name, sign * args.bounds)
    return args


args = parse_args()

from isaacsim import SimulationApp                                   # noqa: E402

simulation_app = SimulationApp({'headless': True})

import isaacsim.core.experimental.utils.app as app_utils             # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils         # noqa: E402
import omni.physx                                                    # noqa: E402
import omni.usd                                                      # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage  # noqa: E402

from assets import lift_world, resolve_world                        # noqa: E402

# The occupancy map generator is not part of the base experience this script
# runs under, so its Python module does not exist until the extension is
# enabled -- importing it at the top fails with ModuleNotFoundError.
app_utils.enable_extension('isaacsim.asset.gen.omap')
simulation_app.update()

from isaacsim.asset.gen.omap.bindings import _omap                   # noqa: E402

# The three values generate2d() writes into the buffer. Arbitrary, but they have
# to be told apart afterwards, and 0/1/2 would collide with float rounding.
OCCUPIED, FREE, UNKNOWN = 4, 5, 6

# nav2's map_server reads a .pgm as occupancy = (255 - pixel) / 255 when
# negate is 0, then compares against the thresholds below. So black is a wall,
# white is free, and mid-grey falls between the two thresholds and is unknown.
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.196


def build():
    create_new_stage()
    add_reference_to_stage(usd_path=resolve_world(args.world),
                           prim_path='/World/env')
    simulation_app.update()
    while stage_utils.is_stage_loading():
        simulation_app.update()

    # Same prim path and same call the simulator makes, after the same loading
    # loop, so that the stage ray-cast below is the stage that gets simulated.
    lift_world('/World/env', args.world_z)

    # The generator ray-casts against PhysX, and PhysX only knows about the
    # stage once it has been stepped. Without this the map comes back entirely
    # unknown, which looks like a bad bounds argument rather than an empty
    # collision scene.
    physx = omni.physx.get_physx_interface()
    physx.start_simulation()
    physx.update_simulation(1.0 / 60.0, 0.0)

    generator = _omap.Generator(physx, omni.usd.get_context().get_stage_id())
    generator.update_settings(args.cell_size, OCCUPIED, FREE, UNKNOWN)
    # set_transform(origin, lower_bound, upper_bound): the bounds are relative
    # to the origin, so an origin of 0 makes them world coordinates.
    generator.set_transform(
        (0.0, 0.0, 0.0),
        (args.x_min, args.y_min, args.z_min),
        (args.x_max, args.y_max, args.z_max),
    )
    generator.generate2d()
    return generator


def write_map(generator):
    buffer = generator.get_buffer()
    dims = generator.get_dimensions()
    width, height = int(dims[0]), int(dims[1])
    if width <= 0 or height <= 0 or len(buffer) < width * height:
        raise RuntimeError(
            'occupancy map is {}x{} with {} cells -- nothing was '
            'mapped'.format(width, height, len(buffer)))

    value = {OCCUPIED: PGM_OCCUPIED, FREE: PGM_FREE, UNKNOWN: PGM_UNKNOWN}
    # The buffer is row-major with the row running +y from the minimum bound,
    # but x runs the OTHER way along it: NVIDIA's compute_coordinates() puts the
    # image's top-left at (max_x, min_y) and its top-right at (min_x, min_y), so
    # the first cell of a row is maximum x. Their own test_synthetic, in
    # isaacsim/asset/gen/omap/tests/test_occupancy.py, pins seven buffer indices
    # to cube positions that only fit that layout.
    #
    # nav2 reads a .pgm the other way round on both axes: the first row is the
    # TOP of the image, which is maximum y, and the first column is minimum x,
    # the one the yaml `origin` names. So rows are emitted in reverse AND each
    # row is reversed. Getting either wrong mirrors the map, and a mirrored map
    # still localises against a near-symmetric room -- it just localises the
    # robot into the mirror image of it. Measured on the warehouse: with the
    # x reversal, 99.8% of the .pgm's occupied cells land on a cell that
    # get_occupied_positions() also reports; without it, 25.3%.
    rows = []
    for row in range(height - 1, -1, -1):
        start = row * width
        rows.append(bytes(value.get(int(round(v)), PGM_UNKNOWN)
                          for v in buffer[start:start + width][::-1]))

    pgm = os.path.abspath(args.output + '.pgm')
    yaml = os.path.abspath(args.output + '.yaml')
    os.makedirs(os.path.dirname(pgm), exist_ok=True)

    with open(pgm, 'wb') as f:
        f.write(b'P5\n# generated by scripts/build_map.py\n')
        f.write('{} {}\n255\n'.format(width, height).encode())
        for row in rows:
            f.write(row)

    # origin is the world pose of the BOTTOM-LEFT pixel, which after the two
    # reversals above is the corner at both minimums. Not where the buffer
    # starts -- that is (max_x, min_y), the image's top-right.
    min_bound = generator.get_min_bound()
    with open(yaml, 'w') as f:
        f.write('image: {}\n'.format(os.path.basename(pgm)))
        f.write('mode: trinary\n')
        f.write('resolution: {}\n'.format(args.cell_size))
        f.write('origin: [{:.4f}, {:.4f}, 0.0000]\n'.format(
            min_bound[0], min_bound[1]))
        f.write('negate: 0\n')
        f.write('occupied_thresh: {}\n'.format(OCCUPIED_THRESH))
        f.write('free_thresh: {}\n'.format(FREE_THRESH))

    occupied = sum(r.count(PGM_OCCUPIED) for r in rows)
    free = sum(r.count(PGM_FREE) for r in rows)
    print('map: {}x{} px at {} m/px, origin [{:.3f}, {:.3f}]'.format(
        width, height, args.cell_size, min_bound[0], min_bound[1]), flush=True)
    print('     world {} raised {:g} m, slice z={}..{}'.format(
        args.world, args.world_z, args.z_min, args.z_max), flush=True)
    print('     {} occupied, {} free, {} unknown'.format(
        occupied, free, width * height - occupied - free), flush=True)
    print('     {}'.format(yaml), flush=True)

    if occupied == 0:
        raise RuntimeError(
            'no occupied cells: the slice z={}..{} met no collision geometry. '
            'Check the world has colliders and the slice is at lidar '
            'height.'.format(args.z_min, args.z_max))


def main():
    write_map(build())


if __name__ == '__main__':
    status = 0
    try:
        main()
    except Exception:                                    # noqa: BLE001
        import traceback
        traceback.print_exc()
        status = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        # Matches the simulator: a graceful close aborts in carb's TaskGroup
        # destructor and would mask the exit status.
        os._exit(status)
