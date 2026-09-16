# Upstream reference

What NVIDIA's Isaac Sim 6.1.0 documentation says about the things this package
does, and where this package currently diverges from it.

Researched 2026-09-13 against the 6.1.0 doc set and the `v6.1.0` tag of
`github.com/isaac-sim/IsaacSim`. Claims marked **[verified here]** were checked
against the running container on this machine, not just read in the docs.
Everything else is a doc claim with its source linked at the bottom.

This file exists because several things this package builds by hand are shipped
upstream, and because the APIs it targets moved in 6.0. Read it before
"fixing" something that is actually working as designed.

---

## 0. What is on this machine

**[verified here]**, inside the `tb3_ros` container:

| | |
|---|---|
| Isaac Sim | `6.0.1-rc.7+release.42383.32955d8d` in `/isaac-sim` |
| Isaac Sim Python | 3.12.13 |
| System ROS 2 | Humble, Python 3.10.12 |
| Bundled ROS 2 libs | `/isaac-sim/exts/isaacsim.ros2.core/{humble,jazzy}/lib` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| `ROS_DOMAIN_ID` | 30 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | *unset* |
| `ISAACSIM_ASSET_ROOT` | *unset* (so assets come from the cloud root) |
| ROS 2 extensions present | `isaacsim.ros2.{bridge,core,examples,nodes,sim_control,tf_viewer,ui,urdf}` |

The Python version gap is not cosmetic. See §5.

---

## 1. The documented split: what the simulator owns, what ROS owns

This is the central architectural question for running stock `nav2` and SLAM on
top of a simulated robot, and it has a documented answer. From the ROS 2
Navigation tutorial:

> As your robot and scene assets become more complex, it will be more scalable
> and performant to publish static TFs of the robot using the default ROS 2
> `robot_state_publisher` package instead. This way, the robot state publisher
> will parse the robot URDF and publish the static TFs. Meanwhile, Isaac Sim
> will be responsible for publishing joint states for moving joints.

NVIDIA ships two Nova Carter USDs that differ by exactly this choice:
`Nova_Carter_ROS.usd` publishes TF from the simulator, and
`Nova_Carter_Joint_States_ROS.usd` drops the transform-tree graph, keeps a
single raw TF publisher for `odom -> base_link`, and adds a joint-states graph
for an external `robot_state_publisher` to consume.

| Isaac Sim publishes | Stock ROS 2 packages publish |
|---|---|
| `/clock` | |
| `/scan` | |
| `/odom` | |
| `/joint_states` | `robot_state_publisher` consumes this + `/robot_description` |
| **one** raw TF: `odom -> base_link` | everything below `base_link`, parsed from the URDF |
| subscribes `/cmd_vel` | AMCL publishes `map -> odom` |

Sensor frames that exist in hardware but not in the URDF (per-device calibrated
stereo baselines, say) stay as simulator-side static TF publishers: "Isaac Sim
acts as the hardware device driver and publishes these static transforms."

The docs contain **no** explicit warning against running both simulator TF
publishers and `robot_state_publisher`. The nearest rule governs simulator-side
contributors only: if two contributors in one aggregation group publish the same
child frame in a batch, Isaac Sim keeps the first and logs a warning. Between
Isaac Sim and an external ROS node there is no arbitration at all — tf2 takes
whichever message arrives last. Pick one owner per frame.

### `/robot_description`

There is no standalone OmniGraph node that publishes `robot_description`. The
only simulator-side publisher is the `ROS2ControlManager` node in the new
`isaacsim.ros2.control` extension, whose `publishRobotDescription` input
defaults to true and latches a URDF *synthesised from USD on each Play* — not
the original source URDF. Its own doc string says to disable it "if an external
robot_state_publisher owns it." For this package, stock `turtlebot3_bringup`
owns `/robot_description`.

---

## 2. OmniGraph nodes

### The 6.0 breaking change

`ROS2PublishTransformTree` and `ROS2PublishJointState` **no longer resolve USD
prims themselves**. `targetPrims` / `targetPrim` on those publishers are
deprecated and will be removed. They are now serializers fed by dedicated
source nodes:

- `isaacsim.core.nodes.IsaacComputeTransformTree` -> `ROS2PublishTransformTree`
- `isaacsim.sensors.physics.IsaacReadJointState` -> `ROS2PublishJointState`

Some 6.1.0 tutorial pages still show the deprecated `targetPrim` shortcut. Use
the source-node form.

### Node type strings

Note the split: the type strings are namespaced `isaacsim.ros2.bridge.*`, but
the extension that implements them is `isaacsim.ros2.nodes`.

| Node type | Notable inputs (defaults) |
|---|---|
| `isaacsim.ros2.bridge.ROS2Context` | `domain_id` 0, `useDomainIDEnvVar` **true** |
| `isaacsim.ros2.bridge.ROS2PublishClock` | `topicName` `clock` |
| `isaacsim.ros2.bridge.ROS2SubscribeTwist` | `topicName` `cmd_vel`; outputs `linearVelocity`, `angularVelocity` as `vectord[3]` |
| `isaacsim.robot.wheeled_robots.DifferentialController` | `wheelDistance`, `wheelRadius`, `maxLinearSpeed`, `maxAngularSpeed`, `dt`; output `velocityCommand` is `double[2]`, **order left, right**, rad/s |
| `isaacsim.core.nodes.IsaacArticulationController` | `targetPrim`, `jointNames` (token[]), `velocityCommand`. `robotPath`, if set, makes `targetPrim` ignored |
| `isaacsim.sensors.physics.IsaacReadJointState` | input `prim` = articulation root; outputs `jointNames`, `jointPositions`, `jointVelocities`, `jointEfforts`, `jointDofTypes`, `stageMetersPerUnit`, `sensorTime` |
| `isaacsim.ros2.bridge.ROS2PublishJointState` | `topicName` `joint_states` |
| `isaacsim.ros2.bridge.ROS2SubscribeJointState` | `topicName` `joint_command` |
| `isaacsim.core.nodes.IsaacComputeOdometry` | input `chassisPrim`; outputs position, orientation, linear/angular velocity and acceleration |
| `isaacsim.ros2.bridge.ROS2PublishOdometry` | `topicName` `odom`, `odomFrameId` `odom`, `chassisFrameId` **`base_link`** |
| `isaacsim.core.nodes.IsaacComputeTransformTree` | `targetPrims` (an articulation root expands to the whole link tree), `parentPrim` (blank means `world`) |
| `isaacsim.ros2.bridge.ROS2PublishTransformTree` | `topicName` `tf`, `staticPublisher` false |
| `isaacsim.ros2.bridge.ROS2PublishRawTransformTree` | `parentFrameId` **`odom`**, `childFrameId` **`base_link`** — the defaults are literally the odometry transform |
| `isaacsim.ros2.bridge.ROS2RtxLidarHelper` | `type` `laser_scan`, `topicName` `scan`, `frameId` `sim_lidar`, `renderProductPath`. `fullScan` is ignored in 6.1.0; `frameSkipCount` is deprecated |
| `isaacsim.core.nodes.IsaacReadSimulationTime` | `resetOnStop` — the API reference and the Clock tutorial disagree on the default, see §8 |
| `isaacsim.core.nodes.IsaacCreateRenderProduct`, `OgnIsaacRunOneSimulationFrame` | lidar/camera pipeline |
| `omni.graph.action.OnPlaybackTick`, `omni.graph.nodes.BreakVector3` | core OmniGraph |

**[verified here]** All of the above that this package would need exist in the
installed 6.0.1: `IsaacComputeOdometry`, `IsaacComputeTransformTree`,
`IsaacReadSimulationTime`, `IsaacSimulationGate`, `DifferentialController`,
`ROS2Context`, `ROS2PublishJointState`, `ROS2PublishOdometry`,
`ROS2PublishTransformTree`, `ROS2PublishLaserScan`.

### Namespace history

`omni.isaac.*` became `isaacsim.*` in 4.5. Relevant renames:
`omni.isaac.ros2_bridge` -> `isaacsim.ros2.bridge`; `omni.isaac.core_nodes` ->
`isaacsim.core.nodes`; `omni.isaac.sensor` -> `isaacsim.sensors.{camera,physics,physx,rtx}`;
`omni.importer.urdf` -> `isaacsim.asset.importer.urdf`;
`omni.isaac.ros2_bridge.robot_description` -> `isaacsim.ros2.urdf`.
Settings moved too: `/exts/omni.isaac.ros2_bridge/ros_distro` ->
`/exts/isaacsim.ros2.bridge/ros_distro`. Any tutorial using `omni.isaac.*` node
types is pre-4.5 and its node strings are wrong for us.

### Graph-building helpers

6.1.0 exposes public Python helpers in `isaacsim.ros2.nodes` for clock, generic
publisher, joint states, TF, odometry, camera, RTX lidar and RTX radar graphs —
and **removes** the equivalent API from `isaacsim.ros2.ui`.
`create_ros2_joint_states_graph` builds the migrated `IsaacReadJointState` form.

**[verified here]** 6.0.1 has no such public API. Graph creation exists only
behind `isaacsim.ros2.ui`'s menus (its test suite names
`test_clock_graph_creation`, `test_tf_graph_creation`,
`test_odometry_graph_creation`, `test_joint_states_graph_creation`,
`test_lidar_graph_creation`). This is a concrete reason to move to 6.1.0:
on 6.1.0 these graphs are a supported function call, on 6.0.1 they are ours to
hand-author.

### The graphs, as the tutorial builds them

From "Putting It All Together", which is a differential-drive robot plus nav2
plus an external `robot_state_publisher` — the same architecture this package
wants. Four graphs: `joint_states`, `odom_tf`, `scan`, `cmd_vel`.

`odom_tf`:

```python
keys.CREATE_NODES: [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
    ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
],
keys.SET_VALUES: [
    ("ComputeOdom.inputs:chassisPrim", [Sdf.Path(CHASSIS_LINK)]),
    ("PublishOdom.inputs:topicName", "odom"),
    ("PublishOdom.inputs:odomFrameId", "odom"),
    ("PublishRawTF.inputs:childFrameId", "base_link"),
    ("PublishRawTF.inputs:parentFrameId", "odom"),
    ("ComputeTransformTree.inputs:parentPrim", Sdf.Path(CHASSIS_LINK)),
    ("ComputeTransformTree.inputs:targetPrims", [Sdf.Path(SIM_LIDAR)]),
    ("PublishTF.inputs:topicName", "tf_static"),
    ("PublishTF.inputs:staticPublisher", True),
],
```

Note what the transform-tree publisher is used for here: **only the lidar's
static mount transform**, published to `tf_static`. Everything else below
`base_link` is left to `robot_state_publisher`. The tutorial says so outright:
"The remaining robot-link transforms are calculated by an external
`robot_state_publisher` that you launch separately."

`cmd_vel`:

```python
keys.CREATE_NODES: [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
    ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
    ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
],
keys.CONNECT: [
    ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
    ("OnPlaybackTick.outputs:deltaSeconds", "DiffController.inputs:dt"),
    ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
    ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
    ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
    ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
    ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
    ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
    ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
    ("Context.outputs:context", "SubscribeTwist.inputs:context"),
],
```

Two details that are easy to get wrong:

- The differential controller is ticked from `OnPlaybackTick`, **not** from the
  twist subscriber's `execOut`. The tutorial's reason: it must be ticked every
  frame regardless of when a new command arrives.
- `jointNames` must be built from **Constant Token** nodes, not Constant String
  — "OmniGraph arrays require the `token` type".

`scan`: `OnPlaybackTick -> OgnIsaacRunOneSimulationFrame ->
IsaacCreateRenderProduct(cameraPrim=<lidar prim>) -> ROS2RtxLidarHelper`, with
`CreateRenderProduct.outputs:renderProductPath` wired into the helper.

The world -> odom ground-truth transform, if wanted, goes in a **separate
scene-level graph**, not the robot graph. The docs are explicit: "Do not add the
world -> odom publisher inside the robot graph... In a full navigation stack, a
localization package such as Nav2 AMCL can publish the transform between a
global frame and the odom frame."

### TF aggregation (new in 6.0)

Isaac Sim now merges submissions from compatible TF publisher nodes into **one
`tf2_msgs/msg/TFMessage` per simulation timestamp**, grouped by ROS domain,
topic, static/dynamic and QoS. `/tf_static` is cached and republished only on
change. Disable while debugging with
`--/exts/isaacsim.ros2.nodes/tfAggregation/enabled=false`.

---

## 3. Assets

### TurtleBot3 ships upstream

**[verified here]** by HTTP range request against the cloud asset root
(206 = present, 404 = absent):

```
6.0/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd                        206
6.1/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd                        404
6.1/Isaac/Robots_Multiphysics/Turtlebot/Turtlebot3/turtlebot3_burger/…usda         206
6.0/Isaac/Samples/ROS2/Scenario/turtlebot_tutorial.usd                             206
6.1/Isaac/Samples/ROS2/Scenario/turtlebot_tutorial.usd                             206
```

Robot assets moved `Isaac/Robots/` -> `Isaac/Robots_Multiphysics/` in 6.1.0.
The asset catalogue lists the burger as 2 joints, 3 links, 2 DOF, Apache 2.0.

`turtlebot_tutorial.usd` is a complete scene: TurtleBot3 Burger in Simple_Room
at prim `/World/turtlebot3_burger_processed`, with the whole tutorial graph set
already authored in — cameras, 2D and 3D RTX lidar, TF, odometry, IMU.

**TurtleBot3 Burger is the canonical ROS 2 tutorial robot in 6.1.0.** The
tutorial series runs: URDF Import: Turtlebot -> Driving TurtleBot -> Clock ->
RTF -> Cameras -> RTX Lidar -> Transform Trees and Odometry -> Publish Rates ->
Putting It All Together. It clones upstream ROBOTIS and xacros it exactly the
way this package does:

```bash
git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3.git
namespace=""
xacro ./turtlebot3_burger.urdf "namespace:=${namespace:+$namespace/}" > tb3_burger_processed.urdf
```

There is, however, **no TurtleBot3 nav2 reference launch** anywhere upstream —
the nav2 samples use Nova Carter, iw_hub and Clearpath Dingo. And no SLAM
tutorial exists at all in 6.1.0; the nav2 pages all use AMCL against a prebuilt
map.

### Asset root resolution

Precedence, highest first: `ISAACSIM_ASSET_ROOT` env var; then
`ISAACSIM_ASSET_REGION_PROFILE` (`us`|`china`); then CLI
`--/persistent/isaac/asset_root/default=...`; then the `.kit` experience file's
`persistent.isaac.asset_root.default`; then the built-in US root.
"`ISAACSIM_ASSET_ROOT` always takes precedence."

Nucleus is **not** required: "Nucleus, Cache, and Hub are not needed to run
Isaac Sim", and the container "uses assets in the Cloud if no Nucleus server is
available." For offline use, download the asset pack and point
`ISAACSIM_ASSET_ROOT` at the directory containing `Isaac/` and `NVIDIA/`.

### Python API

```python
from isaacsim.storage.native import get_assets_root_path, get_assets_root_path_async
# get_assets_root_path(*, skip_check: bool = False, accept_unsupported: bool = True) -> str
```

Moved from `omni.isaac.nucleus` / `isaacsim.core.utils.nucleus`. Deprecated in
6.1.0: `find_nucleus_server`, `get_assets_server`, `get_isaac_asset_root_path`,
`get_nvidia_asset_root_path`.

Stage helpers live in `isaacsim.core.experimental.utils.stage`. The real
signature — the rendered API page summarises this one incorrectly:

```python
def add_reference_to_stage(
    usd_path: str, path: str, *,
    prim_type: str = "Xform",
    variants: list[tuple[str, str]] | None = None,
) -> Usd.Prim
```

`variants=` is how you select a variant at reference time, and it is the only
documented way to pick the Physics variant on an asset-structure robot:

```python
prim = stage_utils.add_reference_to_stage(
    usd_path=".../franka/franka.usda",
    path="/panda",
    variants=[("Gripper", "alternatefinger"), ("Mesh", "performance")],
)
```

### Is a robot asset plain USD, or wrapped in Python?

Plain USD. The wrapper is USD applied schemas, not Python.

The **Robot Schema** is a set of applied schemas — `IsaacRobotAPI` on the root
prim (carrying `isaac:physics:robotLinks`, `isaac:physics:robotJoints`,
`isaac:description`, `isaac:namespace`, `isaac:robotType`, `isaac:version`),
plus `IsaacLinkAPI`, `IsaacJointAPI`, `IsaacSiteAPI`, `IsaacAttachmentPointAPI`
and the typed `IsaacNamedPose` / `IsaacSurfaceGripper`. It lives in its own
`robot.usda` layer sublayered onto the asset. There is no `.robot` file. And:
"All robots in Isaac Sim's asset library and those imported through URDF
Importer Extension or MJCF Importer Extension have the Robot Schema
pre-applied."

The Python classes are optional runtime conveniences for the *scripting* path,
not a packaging format. In 6.1.0 they are:

```python
from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
```

`WheeledRobot` subclasses `Articulation`, calls `add_reference_to_stage` for
you, resolves wheel DOF indices, and adds exactly one method,
`apply_wheel_actions(velocities)`. Nothing about the robot lives in Python that
is not already in the USD. The older non-experimental paths
(`isaacsim.core.api.robots.Robot`, `isaacsim.core.prims.SingleArticulation`)
**do not exist in 6.1.0**.

The docs treat OmniGraph as the ROS-facing path and these classes as the
script-facing path, and never mix them. Graphs are authored *into the robot
USD*, parented to the prim they serve, so that "when Camera_1 is referenced into
another scene, the publisher graph travels with it." The asset-structure
convention even names an optional `asset_ros.usd` payload layer for exactly
this.

### The URDF importer

```python
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
usd_path = URDFImporter(config).import_urdf()   # returns the entry-point layer
```

`URDFImporterConfig` fields, from source at `v6.1.0`:

```python
urdf_path: str | None = None
usd_path: str | None = None            # a DIRECTORY; defaults to dirname(urdf_path)
merge_fixed_joints: bool = False
merge_mesh: bool = False
collision_from_visuals: bool = False
collision_type: str = "Convex Hull"    # | Convex Decomposition | Bounding Sphere | Bounding Cube
allow_self_collision: bool = False
ros_package_paths: list[dict[str, str]] = field(default_factory=list)
robot_type: str = "Default"            # Wheeled, Manipulator, Humanoid, ...
fix_base: bool | None = None           # None=Source, True=Fixed, False=Mobile
link_density: float | None = None
joint_drive_type: str | dict[str, str] | None = None     # force | acceleration
joint_target_type: str | dict[str, str] | None = None    # none | position | velocity
override_joint_stiffness: float | dict[str, float] | None = None
override_joint_damping: float | dict[str, float] | None = None
run_asset_transformer: bool = True
run_multi_physics_conversion: bool = True
```

The dict forms map a joint-name regex to a per-joint value. For TurtleBot3 the
tutorial uses Base Type = Mobile (`fix_base=False`), Robot Type = Wheeled.

**It writes a directory, and that is by design, on both 6.0 and 6.1.** From
`converter.py`:

```python
output_dir = importer_utils.resolve_unique_path(os.path.join(usd_path, robot_name), is_file=False)
final_path = os.path.join(output_dir, f"{robot_name}.usda")
...
return final_path
```

So `<usd_path>/<robot_name>/<robot_name>.usda` is the entry-point layer, and
`import_urdf()` hands it back directly. `resolve_unique_path` means a re-import
creates `<robot_name>_01/` rather than overwriting. Setting
`run_asset_transformer=False` writes a single flat `.usda` instead — documented
only as a one-line dataclass docstring.

With the default transformer the package contains `payloads/base.usd`,
`payloads/robot.usda`, `payloads/geometries.usd`, `payloads/materials.usda` +
`payloads/Textures/`, `payloads/Physics/{physics,physx,mujoco}.usda`,
`source_assets/` and `transform_report.json`. `base.usda` references the
geometry with `instanceable = true` — which is why a plain `Usd.PrimRange` walk
finds no meshes and you need `Usd.TraverseInstanceProxies()`. The Physics
variantSet looks like:

```usda
def Xform "inspire_hand" (
    prepend references = @payloads/base.usda@
    append variantSets = "Physics"
) {
    variantSet "Physics" = {
        "none"    { }
        "physics" ( prepend payload = @payloads/Physics/physics.usda@ ) { }
        "physx"   ( prepend payload = @payloads/Physics/physx.usda@   ) { }
    }
}
```

Prim naming follows URDF link names, nested by the URDF tree, under an inserted
`Geometry` scope — e.g., with `merge_fixed_joints` left at the documented
default, `/World/tb3_burger_processed/Geometry/base_footprint/base_link/base_scan`.

**This package sets `merge_fixed_joints=True`
(`scripts/import_turtlebot3.py:89`), diverging from what NVIDIA's own docs
recommend.** Their advice: leave it at `False`. The GUI does not expose the
option and the importer does not merge by default, "so a GUI import preserves
the sensor and inertial frames the URDF declares" — and enabling it, they warn,
destroys `base_footprint`, `base_scan` and `imu_link`, "frames nav2 needs".

That warning describes a different architecture than this package's. Nothing
here reads a sensor or joint frame off the USD stage — `robot_state_publisher`
(`launch/robot_state_publisher.launch.py`) expands `turtlebot3_description`'s
own URDF and supplies every frame from `base_footprint` down, identically on
Gazebo and Isaac Sim (`DESIGN.md`, "The three layers"). The simulator's only
frame obligation is `odom -> base_footprint`, and a sensor's mount point is not
looked up on a prim either: `SCAN_OFFSET` in `runtime/turtlebot3_isaacsim.py`
hardcodes it as `(-0.032, 0.0, 0.182)` m off `base_footprint`, which is exactly
`turtlebot3_description`'s `base_joint` (0, 0, 0.010) composed with `scan_joint`
(-0.032, 0, 0.172) — verified against the URDF, not carried over from a prim
transform.

`base_footprint` cannot be merged away by this flag regardless: it is the
URDF's root link, so it has no parent fixed joint to fold across, and the
committed asset confirms it survives as a real prim (`base_footprint` is the
one that carries `PhysicsArticulationRootAPI`). What `merge_fixed_joints=True`
does remove is `base_link`, `base_scan`, `imu_link` and `caster_back_link` as
separate named prims — their geometry folds into `base_footprint` — and the
committed asset confirms that too: only `base_footprint`, `wheel_left_link` and
`wheel_right_link` remain as named links under `Geometry`. That is a
deliberate simplification under this package's frame-ownership split, not an
accident: `scripts/import_turtlebot3.py:155`'s "Root first, as the fallback for
everything merge_fixed_joints folded into it" comment says so, and it is
harmless exactly because nothing downstream of this package looks those prims
up by name.

`package://` resolution comes from `ros_package_paths` (API) or the GUI's ROS
Package List. To import live from a running node (File > Import from ROS 2 URDF
Node, extension `isaacsim.ros2.urdf`) you must launch Isaac Sim from a terminal
with the ROS 2 workspace sourced, because "the bundled ROS 2 libraries do not
provide robot-specific packages or `package://` resource paths."

The importer applies `UsdPhysics.ArticulationRootAPI` and
`NewtonArticulationRootAPI`; "the PhysX-specific PhysxArticulationAPI is no
longer authored". Self-collision is `newton:selfCollisionEnabled`; mimic joints
use `NewtonMimicAPI`.

The TurtleBot3 tutorial also says to run the Gain Tuner and set Damping/Kd to
`10000000.0` on `wheel_left_joint` and `wheel_right_joint` — velocity drives
need zero stiffness and non-zero damping or the wheels will not track commands.

### Worlds

Stock environments, relative to the asset root:
`/Isaac/Environments/Grid/default_environment.usd` (also `gridroom_black`,
`gridroom_curved`), `/Isaac/Environments/Simple_Room/simple_room.usd`,
`/Isaac/Environments/Simple_Warehouse/{warehouse,warehouse_with_forklifts,warehouse_multiple_shelves,full_warehouse}.usd`,
`/Isaac/Environments/Hospital/hospital.usd`, `/Isaac/Environments/Office/office.usd`.

**There is no SDF importer and no Gazebo path.** Zero hits for "Gazebo" across
the entire 6.1.0 doc set. Available importers are URDF, MJCF, and CAD/mesh via
`omni.kit.asset_converter` (OBJ/STL/FBX). So `turtlebot3_world` has no
documented conversion route. The documented substitute for a nav2 map is a stock
environment plus **Tools > Robotics > Occupancy Map**
(`isaacsim.asset.gen.omap`), which emits the `.yaml` + `.png` pair nav2 consumes.

Ground plane and lighting from Python:

```python
from isaacsim.core.experimental.objects import GroundPlane, DomeLight
import isaacsim.core.experimental.utils.stage as stage_utils

stage_utils.define_prim("/World/physicsScene", "PhysicsScene")
GroundPlane("/World/groundPlane", sizes=100.0, positions=[0.0, 0.0, -100.0])
DomeLight("/World/DomeLight").set_intensities(500)
```

---

## 4. Launching: `isaacsim_bringup`

This is the sanctioned bringup path, not a demo. "The Isaac Sim launch file can
be included in other launch files to incorporate launching Isaac Sim from other
ROS 2 workflows."

The package was renamed `isaacsim` -> `isaacsim_bringup` and converted
`ament_cmake` -> `ament_python` in 6.0. In 6.1.0 every repo-owned launch file
was migrated from Python to XML, so `run_isaacsim.launch.py` becomes
**`run_isaacsim.launch.xml`**. `isaacsim.launch.py` includes it by that name
through `AnyLaunchDescriptionSource`, which selects the launch frontend from the
file extension and so is indifferent to which form a given tag ships.

Arguments (XML defaults, `humble_ws`):

| arg | default | meaning |
|---|---|---|
| `version` | `6.1.0` | which installed Isaac Sim to use |
| `install_path` | `""` | overrides `version` |
| `use_internal_libs` | **true** on Humble, false on Jazzy | use Isaac Sim's bundled ROS 2 libs |
| `dds_type` | `""` | `fastdds`\|`cyclonedds`\|`zenoh`; only overrides `RMW_IMPLEMENTATION` when set |
| `gui` | `""` | **path to a USD to open in GUI mode** |
| `standalone` | `""` | path to a Python file, standalone workflow |
| `python_script` | `""` | *new in 6.1.0*; snippet injected into the running session — must not create its own `SimulationApp`. GUI mode only, mutually exclusive with `standalone` |
| `play_sim_on_start` | false | |
| `headless` | `""` | documented as native\|webrtc\|empty — but see §8 |
| `ros_distro` | `humble` | |
| `ros_installation_path` | `""` | |
| `exclude_install_path` | `""` | paths stripped from the child env |
| `custom_args` | `""` | |

What `run_isaacsim.py` actually does: builds an isolated `child_env` (6.1.0
change; 6.0.1 mutated `os.environ` in place). With `use_internal_libs` it
prepends `{root}/exts/isaacsim.ros2.core/{ros_distro}/lib` to
`LD_LIBRARY_PATH`, sets `ROS_DISTRO`, and **strips** `/opt/ros/{distro}` and the
other distro's paths from `LD_LIBRARY_PATH`, `PYTHONPATH` and `PATH`.
`exclude_install_path` removes listed paths from the same three, applied last.
`standalone` runs `python.sh <script>`; otherwise it runs `isaac-sim.sh
--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge`, swapping in
`isaac-sim.streaming.sh` only when `headless == "webrtc"`. `gui` and
`python_script` are passed via `--exec` to `scripts/open_isaacsim_stage.py`.

Constraint: "ROS 2 Launch with Isaac Sim is only supported in Linux and Windows
with Pixi-based installation. The `isaacsim_bringup` package is not supported in
WSL2."

### The reference pattern: one launch file, sim + nav2

`carter_navigation/launch/carter_navigation_isaacsim.launch.xml`:

```xml
<launch>
  <arg name="gui" default="https://.../Isaac/6.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"/>
  <arg name="map" default="$(find-pkg-share carter_navigation)/maps/carter_warehouse_navigation.yaml"/>
  <arg name="params_file" default="$(find-pkg-share carter_navigation)/params/carter_navigation_params.yaml"/>
  <arg name="use_sim_time" default="true"/>

  <include file="$(find-pkg-share isaacsim_bringup)/launch/run_isaacsim.launch.xml">
    <arg name="version" value="6.1.0"/>
    <arg name="gui" value="$(var gui)"/>
    <arg name="play_sim_on_start" value="true"/>
  </include>
  <include file="$(find-pkg-share carter_navigation)/launch/carter_navigation.launch.xml">
    <arg name="map" value="$(var map)"/>
    <arg name="params_file" value="$(var params_file)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>
</launch>
```

Note the scene can be a URL, not just a local path.

`humble_ws/src` at the 6.1.0 tag holds: `ackermann_control`, `custom_message`,
`greenwave_monitor`, `humanoid_locomotion_policy_example`,
`isaac_compressed_image_decoder`, `isaac_ros2_messages`, `isaac_tutorials`,
`isaacsim_bringup`, `moveit`, `navigation`. The repo moved from
`NVIDIA-Omniverse/` to **`isaac-sim/IsaacSim-ros_workspaces`**.

---

## 5. Environment and container

### The Python version wall

**[verified here]** Isaac Sim Python is 3.12.13; system Humble is 3.10.12. The
docs are blunt:

> For Linux, you can not source this installation in the same terminal as
> running Isaac Sim.

and

> If you meet the following configurations, you **must** run Isaac Sim with the
> internal ROS libraries that ship with Isaac Sim: Need to use ROS Docker
> containers; Have a ROS 2 workspace built locally, but you only plan on using
> default or command ROS interfaces (for example, `std_msgs`, `geometry_msgs`,
> `nav_msgs`).

That is exactly this project: stock nav2 and SLAM message types. So Isaac Sim
runs on internal libs in one environment, and our ROS 2 nodes run with
`/opt/ros/humble` sourced in another. `isaacsim_bringup` implements precisely
this separation, which is a good reason to use it rather than hand-rolling.

Custom message packages would be the exception — those require rebuilding ROS 2
and the workspace against Python 3.12 via `./build_ros.sh -d humble -v 22.04`.
We have no custom messages, so this does not apply.

In 6.1.0 `python.sh` auto-configures the bundled libraries **when `ROS_DISTRO`
is not set**; `--no-ros-env` opts out. **[verified here]** this container exports
`ROS_DISTRO=humble` globally, which is the condition that disables that
auto-configuration.

### DDS

Fast DDS (`rmw_fastrtps_cpp`) is the default and the recommended choice. Cyclone
DDS is supported on Linux. Zenoh is "untested on Ubuntu 22.04 with ROS 2 Humble
due to Python 3.12 compilation requirements. For ROS 2 Humble, use Fast DDS or
Cyclone DDS."

For multiple machines or containers you need `FASTRTPS_DEFAULT_PROFILES_FILE`
pointing at a UDP-only profile — either the `fastdds.xml` shipped at the root of
`humble_ws`, or `~/.ros/fastdds.xml` with `<type>UDPv4</type>`,
`<useBuiltinTransports>false</useBuiltinTransports>` and
`is_default_profile="true"`. The reason: Fast DDS defaults to shared-memory
transport, which does not cross container boundaries. **[verified here]** this
variable is currently unset.

### Containers

Official image: `nvcr.io/nvidia/isaac-sim:6.1.0`, now multi-arch and **rootless
(uid 1234)** — host cache dirs need `chown -R 1234:1234`. `--network=host` is
required for WebRTC; "Docker bridge networking (`-p` port publishing) does not
work." Container root is `/isaac-sim`, entrypoint `runheadless.sh`.

**There is no official combined Isaac Sim + ROS 2 image.** The NGC image ships
Isaac Sim with bundled ROS 2 libs but no system ROS; its base is Ubuntu 24.04
noble, so installing `ros-humble-*` into it is not viable. The repo's
dockerfiles build Python-3.12 ROS 2 *without* Isaac Sim. The documented topology
is two containers on `--network=host`:

```bash
docker run -it --net=host --env="DISPLAY" --env="ROS_DOMAIN_ID" \
  -v ~/IsaacSim-ros_workspaces/humble_ws:/humble_ws \
  --name ros_ws_docker osrf/ros:humble-desktop /bin/bash
```

Isaac ROS compatibility for 6.1.0 is not stated anywhere; the matrix stops at
6.0.1 paired with Isaac ROS 4.6.

---

## 6. Sim time

`OnPlaybackTick -> ROS2PublishClock`, with
`IsaacReadSimulationTime.simulationTime -> timeStamp` and the context wired in;
`topicName` `/clock`. Every downstream node needs `use_sim_time: true`.

The FAQ names the failure mode: "If you observe that timestamps do not match, or
that TF transforms appear to jump or lag, verify that: a `/clock` topic is being
published; all relevant ROS 2 nodes have `use_sim_time` set to `true`."

Trigger choice sets the rate: `OnPlaybackTick` fires per render frame
(GPU-bound), `OnPhysicsStep` per physics step. "If you need your ROS 2 messages
to publish at the physics rate, replace On Playback Tick with On Physics Step."

`ros2 topic hz` reports wall time unless you pass `--use-sim-time`.

---

## 7. Where this package diverges today

Not all of these are wrong, but each is a deliberate difference from the
documented path and should be a decision, not an accident.

1. **We import the URDF ourselves.** `scripts/import_turtlebot3.py` and
   `scripts/build_models.sh` rebuild an asset NVIDIA ships
   (`Turtlebot/Turtlebot3/turtlebot3_burger`). Defensible only if we want
   URDF-parity with the exact upstream TurtleBot3 description rather than
   NVIDIA's conversion.
2. **`asset_layer()` recomputes a path we already had.**
   `scripts/import_turtlebot3.py:102` returns `importer.import_urdf()`, which
   per upstream source already returns `<dir>/<robot>/<robot>.usda`. Line 213
   reports `args.output`, the directory, and the launch path then has to
   rediscover the entry-point layer. Plumb the return value through instead.
3. **We do not use the documented source nodes.** Any TF or joint-state
   publishing must go through `IsaacComputeTransformTree` and
   `IsaacReadJointState`; `targetPrims` is deprecated.
4. **We publish the lidar through the Replicator writer**
   (`RtxLidarROS2PublishLaserScan` via `attach_writer`) rather than the
   `ROS2RtxLidarHelper` OmniGraph node. Both are documented — the writer form is
   what the 6.1.0 standalone RTX-lidar tutorial uses, the helper node is what the
   tutorial series uses. Worth noting the 6.1.0 writer requires explicit
   `horizontalFov`, `horizontalResolution`, `depthRange`, `rotationRate` and
   `azimuthRange` metadata read off the profile.
5. **We do not include `run_isaacsim.launch.xml`.** The documented pattern is to
   include it and pass `gui:=<scene.usd> play_sim_on_start:=true`, which also
   gets the internal-libs environment separation for free.
6. **`worlds/turtlebot3_world.usd` has no upstream path.** There is no SDF
   importer. Either hand-author the world, or use a stock environment plus the
   Occupancy Map generator for the nav2 map.

Two findings that *confirm* existing work rather than contradict it:

- The `tickRate` == `scanRateBaseHz` rule discovered empirically last session is
  the documented rule. `frameSkipCount` is deprecated in its favour.
- The asset-structure directory output is expected behaviour on both 6.0 and
  6.1, not a 6.0.1 regression.

---

## 8. Contradictions and open questions

1. **Wheel radius.** The Driving TurtleBot tutorial states `0.025`; the upstream
   TurtleBot3 burger URDF says `0.033`. Unreconciled in the docs. Ours should
   follow the URDF.
2. **`odom` child frame.** Every documented example publishes `odom ->
   base_link`, including the `ROS2PublishRawTransformTree` defaults. TurtleBot3
   and nav2 convention is `odom -> base_footprint`. `base_footprint` survives as
   a real prim regardless of `merge_fixed_joints` — it is the URDF's root link,
   so it is never a fixed joint's child (see "The URDF importer" above) — so the
   frame exists; the change would be `PublishRawTF.inputs:childFrameId` and
   `PublishOdometry.inputs:chassisFrameId`. Not addressed anywhere upstream.
3. **`IsaacReadSimulationTime.resetOnStop`.** The OGN reference says the default
   is `True`; the Clock tutorial's prose implies it is `False` and tells you to
   set it. Verify at runtime.
4. **`headless` in `run_isaacsim`.** Documented as "native, webrtc, or empty",
   but `run_isaacsim.py` only branches on `== "webrtc"`; `"native"` silently
   falls through to GUI. (This is adjacent to the type collision already fixed in
   `launch/isaacsim.launch.py` — there `headless` is a *string mode*, here it is
   a bool.)
5. **Sourcing contradiction.** The nav2 tutorial says "You must source your ROS 2
   installation from the terminal before running Isaac Sim", which directly
   contradicts `install_ros.html`'s Humble/22.04 rule. The nav2 page reads as
   written for Jazzy on 24.04.
6. **No 6.0 -> 6.1 migration guide exists.** `migration_guides/isaac_sim_6_1/`
   contains only an index and robot-policy examples. All the ROS 2 breaking
   changes landed in 6.0.
7. `resolve_asset_path` is recommended by the 6.0 migration guide but has no API
   page or signature anywhere in the 6.1.0 docs.
8. No documented convention for where a user's own USDs belong inside a ROS
   package. Local paths work; nothing is prescribed.

---

## Sources

Base: `https://docs.isaacsim.omniverse.nvidia.com/6.1.0/`

- `installation/install_ros.html` — env vars, internal vs system libs, DDS, fastdds.xml
- `installation/install_container.html`, `installation/install_workstation.html`, `installation/install_python.html`
- `installation/accessing_assets.html` — asset root precedence, offline packs
- `ros2_tutorials/index.html` and the tutorial series, especially
  `tutorial_series/tutorial_ros2_turtlebot.html`,
  `tutorial_series/tutorial_ros2_drive_turtlebot.html`,
  `tutorial_series/tutorial_ros2_tf.html`,
  `tutorial_series/tutorial_ros2_clock.html`,
  `tutorial_series/tutorial_ros2_putting_it_all_together.html`
- `ros2_tutorials/robot_control/tutorial_ros2_navigation.html` — the robot_state_publisher split
- `ros2_tutorials/bridge_configuration/tutorial_ros2_launch.html` — isaacsim_bringup args
- `ros2_tutorials/help/ros2_faq.html` — sim time, publish rates
- `migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html` — the targetPrims deprecation
- `migration_guides/isaac_sim_6_0/ros2_workspace_package_migration.html`
- `migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html`
- `migration_guides/isaac_sim_4_5/extensions_renaming.html`
- `robot_setup/asset_structure.html`, `robot_setup/asset_transformer.html`
- `omniverse_usd/robot_schema.html`
- `assets/usd_assets_robots_wheeled.html`, `assets/usd_assets_environments.html`
- `py/source/extensions/isaacsim.storage.native/docs/api.html`
- `overview/release_notes.html`
- `github.com/isaac-sim/IsaacSim` @ `v6.1.0` — `converter.py`, `config.py`, wheeled-robot classes
- `github.com/isaac-sim/IsaacSim-ros_workspaces` @ `IsaacSim-6.1.0`
