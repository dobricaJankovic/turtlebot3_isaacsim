# turtlebot3_isaacsim

Isaac Sim simulation package for the TurtleBot3.

One `ros2 launch` starts the simulator, spawns the robot and brings up
`robot_state_publisher`, presenting the same ROS 2 interface the real robot
does. Packages above it — `turtlebot3_navigation2`, `turtlebot3_cartographer`,
`turtlebot3_teleop` — run unchanged apart from `use_sim_time:=true`.

The design, and how each piece maps onto `turtlebot3_gazebo`, is in
[DESIGN.md](DESIGN.md).

## Requirements

- Isaac Sim 6.0
- ROS 2 Humble or Jazzy
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

## Build the robot asset

USD models are generated from `turtlebot3_description`, not committed. Once per
model:

```bash
export TURTLEBOT3_MODEL=burger
src/turtlebot3_isaacsim/scripts/build_models.sh burger
```

## Run

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_isaacsim empty_world.launch.py
ros2 launch turtlebot3_isaacsim turtlebot3_world.launch.py
```

```bash
# navigation, identical to Gazebo and to the real robot
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true map:=$HOME/map.yaml
```

`empty_world.launch.py` needs no world file and is the right first run.

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

`turtlebot3_world.launch.py` and `empty_world.launch.py`:

| argument | default | |
|---|---|---|
| `use_sim_time` | `true` | |
| `x_pose`, `y_pose` | per world | spawn position |
| `headless` | `false` | run with no window |

`isaacsim.launch.py` additionally takes `world`, `robot`, `z_pose`, `yaw`,
`namespace`, `lidar`, `lidar_config`, `physics_hz`, `isaac_install_path`,
`isaac_version`, `ros_distro`, `use_internal_libs`, `exclude_install_path` and
`dds_type`.

## Status

Not yet run against a GPU. See [DESIGN.md](DESIGN.md#status) for what is
verified and what is not.
