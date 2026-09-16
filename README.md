# turtlebot3_isaacsim

Isaac Sim simulation package for the TurtleBot3.

One `ros2 launch` starts the simulator, spawns the robot and brings up
`robot_state_publisher`, presenting the same ROS 2 interface the real robot
does. Packages above it — `turtlebot3_navigation2`, `turtlebot3_cartographer`,
`turtlebot3_teleop` — run unchanged apart from `use_sim_time:=true`.

The design, and how each piece maps onto `turtlebot3_gazebo`, is in
[DESIGN.md](DESIGN.md).

What NVIDIA's Isaac Sim documentation says about each of these pieces, and
where this package deliberately differs from it, is in
[UPSTREAM.md](UPSTREAM.md).

## Requirements

- Isaac Sim 6.0
- ROS 2 Humble — this is the `humble` branch, the tested one and the default.
  A `jazzy` branch carries the untested Ubuntu 24.04 counterpart; see its
  `docker/isaacsim-ros2/Dockerfile.jazzy`.
- [`isaacsim_bringup`](https://github.com/isaac-sim/IsaacSim-ros_workspaces)
  from NVIDIA's `IsaacSim-ros_workspaces`, in the same workspace
- `turtlebot3_description`

## Install

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/dobricaJankovic/turtlebot3_isaacsim.git
cp -r /path/to/IsaacSim-ros_workspaces/humble_ws/src/isaacsim_bringup .
cd ~/turtlebot3_ws && colcon build --symlink-install
source install/setup.bash
```

## The robot asset and the worlds

Both are committed, so a clone can launch without building anything: the burger
under `models/turtlebot3_burger/` and `worlds/turtlebot3_world.usd`. They are
*generated* files kept in the repository on purpose — regenerating them needs an
NVIDIA GPU, Isaac Sim and `turtlebot3_description`, which is a bootstrap a user
of the package should not have to clear, and one CI cannot clear at all.

Regenerate after changing the model or the Isaac Sim version, not to obtain it:

```bash
export TURTLEBOT3_MODEL=burger
src/turtlebot3_isaacsim/scripts/build_models.sh burger
```

`waffle` and `waffle_pi` are not committed — only `burger` has been checked
against the URDF. Build them with the same script.

The meshes come from ROBOTIS' `turtlebot3_description` (Apache-2.0) by way of
Isaac Sim's URDF importer; this package is Apache-2.0 too. Maps are *not*
committed: `scripts/build_map.py` rebuilds them, and the warehouse `.pgm` alone
is 1.4 MB.

## Run

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_isaacsim empty_world.launch.py
ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py
ros2 launch turtlebot3_isaacsim warehouse.launch.py
ros2 launch turtlebot3_isaacsim simple_room.launch.py
ros2 launch turtlebot3_isaacsim kitchen.launch.py
```

```bash
# navigation, identical to Gazebo and to the real robot
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true map:=$HOME/map.yaml
```

`empty_world.launch.py` needs no world file and is the right first run.
`/scan` is advertised but stays silent there: the lidar is horizontal and
an empty world is an infinite floor with nothing on it, so no ray returns
anything and the writer has no scan to publish. Give it a world to see it.

`warehouse.launch.py` runs one of Isaac Sim's stock environments instead of a
generated one. It is not a file in this package: `world` defaults to
`/Isaac/Environments/Simple_Warehouse/warehouse.usd`, a path under the Isaac Sim
asset root, which is fetched on first use and cached (a few minutes, ~200 MB).
`world` also accepts a local `.usd` or a URL. The same three spellings work on
`isaacsim.launch.py`; see `runtime/assets.py`.

`simple_room.launch.py` is the same idea one size down --
`/Isaac/Environments/Simple_Room/simple_room.usd`, 8.82 x 8.16 m of interior
against the warehouse's 24 x 38.8 m, so the room fits inside the burger's 3.5 m
lidar and its map is 219 x 219 px. It spawns at `(-2.0, -2.0)`, 1.275 m off the
nearest obstacle, because the world origin is inside the room's low table.

This asset is authored with its floor at z = -0.7696 rather than 0, so the
launch file passes `world_z:=0.7696` to stand it on the ground plane the
simulator authors. Without that the robot floats at table-top height and the
table is invisible to both the map and the scan; with it the table is a 0.78 m
obstacle whose legs the lidar sees. **A map of this world must be built with the
same `--world-z`** -- see "Maps" below, and DESIGN.md, "Standing the world on
the ground plane".

`kitchen.launch.py` is the smallest of the three: 5.148 x 4.352 m of interior,
a U of cabinets, `kitchen_u_shape.usda` out of the `replicator_kitchen` pack.
`kitchen_l_shape`, `kitchen_l_island`, `kitchen_g_shape` and `kitchen_peninsula`
are drop-in `world:=` alternatives, though peninsula drags a 100 m outdoor
backdrop behind its windows and so wants far wider map bounds than the others.
Only `u_shape` has been measured — the rest are names from a listing.

It passes **no** `world_z`, and that contrast is the useful part: this asset's
floor is already at z = 0, so the default of 0.0 is right and the launch file
authors no transform at all. Two stock environments, two different answers --
check a new world's floor rather than assuming either. It spawns at the origin,
which is the best spot in a room this small: every one of the map's 2064
occupied cells is inside the burger's 3.5 m lidar from there.

## Maps

`turtlebot3_navigation2` ships a map for `turtlebot3_world` and for nothing
else, so any other world needs one built:

```bash
isaacsim-python scripts/build_map.py \
    --world /Isaac/Environments/Simple_Warehouse/warehouse.usd \
    --output maps/warehouse
```

```bash
isaacsim-python scripts/build_map.py \
    --world /Isaac/Environments/Simple_Room/simple_room.usd \
    --output maps/simple_room \
    --world-z 0.7696 \
    --x-min -5.5 --x-max 5.5 --y-min -5.5 --y-max 5.5 \
    --z-min 0.17 --z-max 0.19
```

That writes the `.pgm`/`.yaml` pair nav2 consumes, using Isaac Sim's own
occupancy map generator against the stage's collision geometry. Maps are
generated, not committed, like the worlds and the robot asset.

```bash
isaacsim-python scripts/build_map.py \
    --world /Isaac/Environments/replicator_kitchen/kitchen_u_shape.usda \
    --output maps/kitchen \
    --x-min -3.0 --x-max 3.0 --y-min -3.0 --y-max 3.0 \
    --z-min 0.17 --z-max 0.19
```

The second command shows the three arguments worth setting per world, and the
third shows the one that is conditional: the kitchen takes no `--world-z`,
because its floor is already at z = 0 and `kitchen.launch.py` passes none
either. What matters is not which value you pick but that the map builder and
the launch file pick the same one.

`--world-z 0.7696` **must be the same value `simple_room.launch.py` passes as
`world_z`.** Nothing checks this for you: the map builder composes its own
stage, so a mismatch produces a clean-looking map of the right room at the wrong
height, which loads in nav2 and localises the robot into a scene that is not
there. If you change one, change the other.

The bounds are tight around the room rather than the default +-30 m, which is
the difference between 219 x 219 px and 1200 x 1200. The slice is narrowed onto
the burger's scanner alone -- 0.182 m above `base_footprint`, which with the
world raised rests on the room's floor at z = 0.

The warehouse map used to come out mirrored about its x axis; that was fixed on
2026-09-14 and the map now matches the stage cell for cell. A map built with an
older checkout is wrong — rebuild it. See DESIGN.md, "The occupancy map was
flipped".

## Published interface

| | |
|---|---|
| `/clock` | `ROS2PublishClock` |
| `/scan` | RTX lidar, `RtxLidarROS2PublishLaserScan` |
| `/odom` | `IsaacComputeOdometry` → `ROS2PublishOdometry` |
| `/tf` `odom`→`base_footprint` | `ROS2PublishRawTransformTree` |
| `/joint_states` | `ROS2PublishJointState` |
| `/cmd_vel` | `ROS2SubscribeTwist` → `DifferentialController` |
| `/tf` below `base_footprint` | `robot_state_publisher`, from the URDF |

## Launch arguments

`turtlebot3_world.launch.py`, `empty_world.launch.py`, `warehouse.launch.py`,
`simple_room.launch.py` and `kitchen.launch.py` are examples built on the
shared `world.launch.py`, each declaring that one world's own defaults (see the
comments in each for how the numbers were measured) and including it:

| argument | default | |
|---|---|---|
| `use_sim_time` | `true` | |
| `x_pose`, `y_pose` | per world | spawn position |
| `headless` | `false` | run with no window |

The last three add `world`, defaulting to the stock environment each is named
after. `simple_room.launch.py` also takes `world_z`, the metres to raise the
world by so its floor meets z = 0; it defaults to `0.7696` there and to `0.0`
everywhere else, including `kitchen.launch.py`, whose asset needs no correction.
It has to match `build_map.py`'s `--world-z`.
`isaacsim.launch.py` additionally takes `world`, `robot`, `z_pose`, `yaw`,
`namespace`, `lidar`, `lidar_config`, `physics_hz`, `isaac_install_path`,
`isaac_version`, `ros_distro`, `use_internal_libs`, `exclude_install_path` and
`dds_type`.

## Status

Run against a GPU on Isaac Sim 6.1.0. All six launch files come up, the full
published interface above is live, and `turtlebot3_navigation2` has reached a
goal in the warehouse. What has not been re-run since the map writer was fixed
on 2026-09-14 is a nav2 goal against a corrected map, in either world. See
[DESIGN.md](DESIGN.md#status) for the measurements and for the list of what is
still unverified.
