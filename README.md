# turtlebot3_isaacsim

Isaac Sim simulation package for the TurtleBot3.

One `ros2 launch` starts the simulator, spawns the robot and brings up
`robot_state_publisher`, presenting the same ROS 2 interface the real robot
does. Packages above it — `turtlebot3_navigation2`, `turtlebot3_cartographer`,
`turtlebot3_teleop` — run unchanged apart from `use_sim_time:=true`.

## Requirements

- Isaac Sim 6.1.0 and an NVIDIA GPU
- ROS 2 Humble — this is the `humble` branch. A `jazzy` branch carries the
  untested Ubuntu 24.04 counterpart.
- `isaacsim_bringup`, from NVIDIA's
  [IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces),
  cloned into the same workspace at the tag matching the installed Isaac Sim
- `turtlebot3_description`
- `TURTLEBOT3_MODEL` set to `burger`, `waffle` or `waffle_pi`

Everything above is already in the Docker image except the two source
packages — see [Docker](#docker).

## Install

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/dobricaJankovic/turtlebot3_isaacsim.git
git clone --branch IsaacSim-6.1.0 \
    https://github.com/isaac-sim/IsaacSim-ros_workspaces.git

# NVIDIA ships a dozen other example packages in that repository, twice over
# (humble_ws and jazzy_ws). Build only the one this package needs:
cd ~/turtlebot3_ws
find src/IsaacSim-ros_workspaces/*_ws/src -name package.xml -printf '%h\n' \
  | grep -vx src/IsaacSim-ros_workspaces/humble_ws/src/isaacsim_bringup \
  | xargs -I{} touch {}/COLCON_IGNORE

colcon build --symlink-install
source install/setup.bash
```

`isaacsim_bringup` is cloned rather than copied so it stays tied to a version:
the `IsaacSim-6.1.0` tag renamed `run_isaacsim.launch.py` to
`run_isaacsim.launch.xml`, and the tag has to match the installed Isaac Sim.

## Docker

The container carries Isaac Sim 6.1, ROS 2 Humble, RViz, Nav2 and the
TurtleBot3 packages. It mounts this package and `isaacsim_bringup` from the
host, so both are edited outside and built inside.

```bash
docker login nvcr.io                      # the Isaac Sim base comes from NGC
./docker/x11-auth.sh                      # once per X session, before `up`
scripts/build_images.sh                   # base image, then this one
docker compose up -d
docker compose exec turtlebot3_isaacsim bash
```

Inside, build the workspace once and then launch as below:

```bash
colcon build --symlink-install && source install/setup.bash
```

`docker compose exec` does not run the entrypoint, so scripted commands need it
explicitly:

```bash
docker compose exec turtlebot3_isaacsim /entrypoint.sh bash -c 'ros2 topic list'
```

`isaacsim` starts the GUI and `isaacsim-python <script>` runs a standalone
script. Both go through `ros-isolate`, which is the only correct way to start
Kit from a shell that has sourced ROS 2.

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

`empty_world.launch.py` is the right first run: it needs no world file. `/scan`
stays silent there, because an empty world has nothing within lidar range.

`warehouse.launch.py`, `simple_room.launch.py` and `kitchen.launch.py` run
Isaac Sim's stock environments, fetched from the asset root on first use and
cached. `world` also takes a local `.usd` or a URL.

The robot asset under `models/turtlebot3_burger/` and
`worlds/turtlebot3_world.usd` are committed, so a clone launches without
building anything. `waffle` and `waffle_pi` are not:

```bash
export TURTLEBOT3_MODEL=waffle
scripts/build_models.sh waffle
isaacsim-python scripts/smoke_test.py --model waffle
```

## Maps

`turtlebot3_navigation2` ships a map for `turtlebot3_world` only; any other
world needs one built. Maps are generated, not committed.

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

```bash
isaacsim-python scripts/build_map.py \
    --world /Isaac/Environments/replicator_kitchen/kitchen_u_shape.usda \
    --output maps/kitchen \
    --x-min -3.0 --x-max 3.0 --y-min -3.0 --y-max 3.0 \
    --z-min 0.17 --z-max 0.19
```

`--world-z` must be the value the matching launch file passes as `world_z`:
`0.7696` for `simple_room`, nothing for the other worlds. Nothing checks this,
and a mismatch produces a good-looking map of the right room at the wrong
height.

## Published interface

| | |
|---|---|
| `/clock` | `ROS2PublishClock` |
| `/scan` | RTX lidar, `RtxLidarROS2PublishLaserScan` |
| `/odom` | `nodes/wheel_odometry.py`, integrated from `/joint_states` |
| `/tf` `odom`→`base_footprint` | `nodes/wheel_odometry.py` |
| `/ground_truth/odom` | `IsaacComputeOdometry` → `ROS2PublishOdometry` |
| `/joint_states` | `ROS2PublishJointState` |
| `/cmd_vel` | `ROS2SubscribeTwist` → `DifferentialController` |
| `/tf` below `base_footprint` | `robot_state_publisher`, from the URDF |

`/odom` is integrated from the wheels and drifts, as it does on the real robot.
The true pose is on `/ground_truth/odom`, relative to the spawn pose.

## Launch arguments

The five world launch files are wrappers around `world.launch.py`, each with its
own defaults:

| argument | default | |
|---|---|---|
| `use_sim_time` | `true` | |
| `x_pose`, `y_pose` | per world | spawn position |
| `world` | per world | stock environment, a local `.usd`, or a URL |
| `world_z` | `0.0` | metres to raise the world by; must match `build_map.py` |
| `headless` | `false` | run with no window |

`isaacsim.launch.py` additionally takes `robot`, `z_pose`, `yaw`, `namespace`,
`lidar`, `lidar_config`, `physics_hz`, `isaac_install_path`, `isaac_version`,
`ros_distro`, `use_internal_libs`, `exclude_install_path` and `dds_type`.

## Licence

Apache-2.0. The meshes come from ROBOTIS' `turtlebot3_description`
(Apache-2.0) by way of Isaac Sim's URDF importer.
