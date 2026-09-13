# Design

Why this package looks the way it does, written against `turtlebot3_gazebo`
because that is the package it has to be interchangeable with.

For the upstream side of the same questions — what the Isaac Sim docs
actually prescribe, and which of our choices diverge — see
[UPSTREAM.md](UPSTREAM.md).

- [The three layers](#the-three-layers)
- [What maps onto what](#what-maps-onto-what)
- [Two launch files that do not exist](#two-launch-files-that-do-not-exist)
- [`run_isaacsim.py`](#run_isaacsimpy)
- [The interpreter split](#the-interpreter-split)
- [robot_state_publisher](#robot_state_publisher)
- [The lidar profile](#the-lidar-profile)
- [Physics materials](#physics-materials)
- [models/](#models)
- [worlds/](#worlds)
- [Status](#status)

## The three layers

The TurtleBot3 stack splits into layers that do not know about each other:

| layer | real robot | Gazebo | Isaac Sim |
|---|---|---|---|
| hardware / physics | `turtlebot3_bringup/robot.launch.py` | `turtlebot3_gazebo` | this package |
| kinematics | `robot_state_publisher` | same node | same node |
| consumers | `turtlebot3_navigation2`, `turtlebot3_cartographer`, `turtlebot3_teleop` | identical | identical |

Only the first layer is ever replaced. The third changes exactly one thing,
`use_sim_time:=true`, which is the property the whole design exists to preserve.

The second layer is what makes the replacement invisible. `robot_state_publisher`
holds the URDF and owns everything below `base_footprint`; the simulator owns
`odom`→`base_footprint` and nothing else; AMCL owns `map`→`odom`:

```
map --(amcl)--> odom --(simulator)--> base_footprint --(robot_state_publisher)--> the rest
```

Three owners, no overlap. This is why the simulator publishes a *raw* transform
rather than a full transform tree — a `ROS2PublishTransformTree` here would
fight `robot_state_publisher` for the same frames.

## What maps onto what

| `turtlebot3_gazebo` | here |
|---|---|
| `turtlebot3_world.launch.py` | same name |
| `empty_world.launch.py` | same name |
| `gzserver.launch.py` (from `gazebo_ros`) | `isaacsim.launch.py` (wrapping `isaacsim_bringup`) |
| `gzclient.launch.py` | — `headless:=` |
| `robot_state_publisher.launch.py` | same name |
| `spawn_turtlebot3.launch.py` | — folded into the simulator |
| `models/turtlebot3_*/model.sdf` | `models/turtlebot3_*/*.usd` + the OmniGraph |
| `worlds/*.world` | `worlds/*.usd` |
| `urdf/turtlebot3_*.urdf` | — uses `turtlebot3_description` |
| `<export><gazebo_ros gazebo_model_path=…/></export>` | — USD references are absolute |

`gazebo_ros` contributes three separable things, and it is worth keeping them
apart when looking for the counterpart of any one of them:

1. **System plugins into `gzserver`.** `gzserver.launch.py` always appends
   `--init --factory --force_system`. `libgazebo_ros_init.so` runs a ROS node
   inside the `gzserver` process and publishes `/clock` — that single plugin is
   the entire reason `use_sim_time` means anything. Here `/clock` is a graph
   node, `ROS2PublishClock`.
2. **`spawn_entity.py`**, a client of the factory service. See below.
3. **Model plugins baked into `model.sdf`** — `diff_drive`, `ray_sensor`,
   `imu_sensor`, `joint_state_publisher`. These are the direct stand-ins for
   `turtlebot3_node` and `ld08_driver`, and their counterpart is the OmniGraph
   built in `scripts/turtlebot3_isaacsim.py`.

Note that `model.sdf` sets `publish_wheel_tf: false` deliberately: the wheel
transforms are left to `robot_state_publisher` to derive from `/joint_states`.
That handoff is the same here.

## Two launch files that do not exist

**`gzclient.launch.py`.** Gazebo splits server and GUI into two processes. Kit
is one process that either opens a window or does not, so the GUI is a flag on
`isaacsim.launch.py` rather than a second launch file.

**`spawn_turtlebot3.launch.py`.** This is the one real structural difference.
`gazebo_ros` ships `libgazebo_ros_factory.so`, which serves a `spawn_entity`
service, so a robot can be pushed into an already-running server by a separate
node. Isaac Sim ships no equivalent — NVIDIA's ROS 2 service surface
(`isaac_ros2_messages`) is:

```
GetPrims  GetPrimAttribute  GetPrimAttributes  SetPrimAttribute  IsaacPose
```

Inspection, attribute setting, and pose teleport. **Nothing creates a prim.**
So the robot is referenced into the stage by the simulator itself at startup,
and `x_pose`/`y_pose` are forwarded to it. The argument names are kept identical
to `turtlebot3_gazebo`'s so that muscle memory transfers.

An empty launch file that only forwarded arguments would preserve the filename
and lie about the structure, so there isn't one.

`IsaacPose` would allow a closer mirror — robot always in the stage, teleported
into place by a launch node — at the cost of a service round trip to deliver a
pose the simulator already knows at startup. Not worth it.

## `run_isaacsim.py`

`isaacsim_bringup` is NVIDIA's own "start Isaac Sim from a launch file", and is
to this package what `gazebo_ros` is to `turtlebot3_gazebo`. It has four sharp
edges, all of which come from reading `run_isaacsim.py` rather than its
documentation. `isaacsim.launch.py` handles all four.

**1. On the `standalone:=` path, every other argument is ignored.** The branch
is a bare:

```python
if args.standalone != "":
    executable_path = os.path.join(filepath_root, "python.sh")
    proc = subprocess.Popen(f"{executable_path} {args.standalone}", **popen_kwargs)
```

`headless`, `gui`, `custom_args` and `play_sim_on_start` are never consulted. So
`headless:=` on `isaacsim.launch.py` becomes `--headless` in the *simulator's*
argv, and the simulator calls `play()` itself.

**2. `standalone` is pasted into a `shell=True` string, unquoted.** That is how
arguments reach the simulator at all. It also means a path containing a space
silently becomes two arguments, so `isaacsim.launch.py` refuses one instead.

**3. `use_internal_libs` does not strip a colcon overlay.** It removes paths
that start with `/opt/ros/<distro>` and paths containing the other distro's
name. Your own workspace matches neither, so it survives on `PYTHONPATH` and
`LD_LIBRARY_PATH` and is inherited by Kit — which runs Python 3.12, where a ROS
2 overlay built against 3.10 is not importable. `exclude_install_path` is the
vendor's mechanism for this, and it defaults here to `$COLCON_PREFIX_PATH`.

**4. `install_path` must be given** whenever Isaac Sim is not at the
version-derived default under `$HOME`. It is `/isaac-sim` in a container. The
`isaac_sim_package_path` environment variable is consulted first, if you would
rather set it in an image.

Two more things worth knowing. Shutdown is `os.killpg(..., SIGKILL)` — Kit gets
no `SIGTERM` and no chance to clean up. And Kit is a *grandchild* of `ros2
launch`, so launch's process events fire for the node, never for the simulator.

NVIDIA's own `carter_navigation_isaacsim.launch.py` gates downstream nodes by
scraping stdout with `OnProcessIO` for `"Stage loaded and simulation is
playing."`, a string printed only on the GUI path by
`isaacsim_bringup/scripts/open_isaacsim_stage.py`. The simulator here prints the
same line after `play()`, so the same gate works on the standalone path.

## The interpreter split

`scripts/turtlebot3_isaacsim.py` runs on Kit's Python with the system ROS 2
stripped from its search paths. It therefore cannot import `rclpy`, cannot call
`get_package_share_directory`, and cannot run `xacro`.

**Anything Kit needs to find, ROS has to hand it.** Every path the simulator
needs arrives as an absolute command-line argument from the launch file, which
does have ament. NVIDIA solves it the same way for `open_isaacsim_stage.py`.

The same split is why `scripts/build_models.sh` exists: the xacro expansion and
the package lookup happen on the ROS 2 side, the URDF import on Kit's.

Defaults inside both scripts resolve relative to `__file__` instead, so they
stay runnable straight out of the package share for debugging.

## robot_state_publisher

`launch/robot_state_publisher.launch.py` is deliberately **not** a copy of
`turtlebot3_gazebo`'s file of the same name, which has two problems:

- It reads the URDF with `open()`. `turtlebot3_description`'s URDF is a xacro
  template carrying a `${namespace}` argument; read as plain text, every frame
  comes out literally named `${namespace}base_footprint`. (`turtlebot3_gazebo`
  gets away with it because it reads its own pre-expanded copy.)
- Its `frame_prefix` default of `''` goes through
  `PythonExpression(["'", frame_prefix, "/'"])`, which evaluates to the string
  `/` — not to an empty prefix.

Both are avoided by expanding the template with `xacro`, the way the real
robot's `turtlebot3_bringup/turtlebot3_state_publisher.launch.py` does.

`use_sim_time` matters for this node specifically because it *stamps* tf.
Stamping with wall clock while everything else is on sim clock produces tf
extrapolation errors even when the geometry is perfect.

## The lidar profile

`models/lidar_configs/turtlebot3_lds.json` models the TB3's LDS, matching
`turtlebot3_gazebo`'s `<ray>` block:

| | Gazebo `<ray>` | `turtlebot3_lds.json` |
|---|---|---|
| samples/rev | 360 | 1800 Hz ÷ 5 Hz = 360 |
| resolution | 1.0° | 1.0° |
| rate | 5 Hz | `scanRateBaseHz: 5.0` |
| range | 0.12–3.5 m | `nearRangeM` / `farRangeM` |
| range resolution | 0.015 m | `rangeResolutionM` |
| noise σ | 0.01 m | `rangeAccuracyM` |
| elevation | horizontal | `elevationDeg: [0.0]` |

It is applied by authoring it onto the `OmniLidar` prim attribute by
attribute, which is what lets the package ship its own sensor model rather than
borrow a vendor one. Isaac Sim 6.0 left no way to hand the renderer a profile by
name: `Lidar.create(config=)` resolves against `SUPPORTED_LIDAR_CONFIGS`, a
registry of stock USD assets under the Nucleus assets root, and rejects anything
else — `app.sensors.nv.lidar.profileBaseFolder` no longer reaches the Python
API. `profile_attributes()` does the translation, and the tables above it hold
the handful of names the schema spells differently (`reportRateBaseHz` is
`patternFiringRateHz`, `wavelengthNm` is `waveLengthNm`) and the one field,
`avgPowerW`, the 6.0 schema dropped. An attribute the prim does not have is a
warning inside `Lidar.create`, so the prim is checked against the profile
afterwards rather than trusted.

**Why not Isaac Sim's stock `Example_Rotary_2D`.** It is a 200 m survey lidar,
and its single emitter sits at `elevationDeg: [-2.0]` — it scans the floor. On a
bare ground plane it still returns hits, spread into a partial arc where the
tilted beam meets the ground, which Nav2 will treat as an obstacle ring.

Re-rating it by overriding `scanRateBaseHz` / `patternFiringRateHz` is **not** a
workaround. The scan pattern baked into that config still assumes its own 30 Hz
/ 32000 Hz; the plugin then warns `Multi-tick is enabled but motion BVH is not
active` and `/scan` publishes in bursts rather than steadily. Authoring a
self-consistent profile is the fix.

**`/scan` is silent in an empty world.** The writer publishes a scan only when
the sweep returned something, and a horizontal lidar over a bare ground plane
returns nothing at all: the topic is advertised and no message ever arrives. It
reads exactly like a broken sensor and is not one -- `empty_world.launch.py` has
nothing within the 3.5 m range for a ray to hit. Anything with a wall in it,
`turtlebot3_world.launch.py` included, publishes at the profile's 5 Hz.

The `RtxLidarROS2PublishLaserScan` writer is used rather than a point cloud on
purpose: a point cloud would force a `pointcloud_to_laserscan` node into every
launch file that uses this package, which neither the real robot nor Gazebo
needs.

## Physics materials

Neither the URDF nor the URDF importer supplies friction or restitution, so
every collider lands on PhysX's fallback material. Isaac Sim's `GroundPlane`
is worse than a fallback: handed no material it authors its own at restitution
0.8, and PhysX combines restitution as an *average* by default, so every
wheel-on-floor contact comes out at 0.4.

That is not cosmetic on a burger. Its centre of mass sits 4.3 mm behind the
wheel axle while the caster skid (`caster_back_link`, a 30 × 9 × 20 mm box)
clears the floor by only 0.5 mm, so the chassis permanently rests on that skid,
exactly as the real robot does. A permanently loaded elastic contact under a
rear skid is a rocking chair: the robot rocks and creeps across the floor with
the wheels commanded to nothing at all, and nothing is logged.

| surface | | why |
|---|---|---|
| wheel | μ 1.0 | `model.sdf` gives both tyres μ = μ2 = 100000, "must not slip", with upstream's own note that the number is not real data. PhysX takes a coefficient, so 1.0 is the honest spelling of the same intent. |
| chassis and caster skid | μ 0.1, combine `min` | a smooth skid dragging on the floor. A high-friction skid fights the wheels on every in-place turn. Gazebo dodges this by making `caster_back_joint` a *ball* joint, so its caster rolls where ours slides. |
| all | restitution 0, combine `min` | neither a tyre, a plastic skid nor a floor is elastic at these speeds |

`min` rather than the default average so that "does not bounce" holds against
whatever the other collider brings, rather than being averaged back up by it.

The robot's materials are authored *into the asset* by
`scripts/import_turtlebot3.py`, so the `.usd` is self-contained the way
`model.sdf` is. The floor's are authored by the simulator, since the floor is
not part of the robot. `check_surfaces()` refuses to start on an asset that has
lost them, because the symptom otherwise reads as a physics-tuning problem
rather than a stale build.

## models/

```
models/
├── lidar_configs/turtlebot3_lds.json   committed
└── turtlebot3_<model>/*.usd            generated, gitignored
```

Built by `scripts/build_models.sh`, which expands the xacro and then runs
`scripts/import_turtlebot3.py` under Isaac Sim's interpreter.

**Why there is no `urdf/` directory.** `turtlebot3_gazebo` carries a second copy
of the URDF with its mesh URLs repointed at
`turtlebot3_gazebo/models/turtlebot3_common/meshes/`. That copy exists purely so
Gazebo's `model://` resolution can find the meshes at *runtime*. Isaac Sim
resolves meshes once, at import time, and bakes the geometry into the USD —
after which the asset needs nothing from `turtlebot3_description` at all. A
second copy would buy nothing and could only drift.

**The two silent failures the importer guards against.** Passing the wrong
`--description-share` does not fail the import: the converter resolves each
unmatched `package://` URL to a bare relative path, finds nothing, and emits the
link as an empty Xform — right transform, right material binding, no geometry.
The inline `<box>`/`<cylinder>` collision primitives import either way, so the
stage loads clean, renders nothing, and collides correctly. `verify()` checks
for mesh points. Separately, the importer does not overwrite: given an existing
`turtlebot3_burger.usd` it writes `turtlebot3_burger_1` *inside* it and returns
that, so re-importing keeps working while silently accumulating copies. The
output directory is cleared first.

## worlds/

Environments, the counterpart of `turtlebot3_gazebo/worlds/*.world`. Generated
and gitignored, so a fresh clone has none and `turtlebot3_world.launch.py` fails
with a message naming the missing file until one is built. `empty_world.launch.py`
needs none.

A world does not have to be a file here. `scripts/assets.py` resolves three
spellings of `--world`, and the simulator and the map builder share it so the
map is always cut from the same scene that gets simulated:

| spelling | resolved by |
|---|---|
| `/ws/.../turtlebot3_world.usd` | the filesystem, via `asset_layer()` |
| `/Isaac/Environments/...` | `get_assets_root_path()` — fetched and cached |
| `https://.../warehouse.usd` | taken as given |

The middle one is what `warehouse.launch.py` uses, and it is the documented way
to reach the stock environments: NVIDIA ships no `turtlebot3_world`, and there
is no SDF importer, so a Gazebo world has to be generated while a stock Isaac
Sim environment only has to be named. See UPSTREAM.md, "Asset root resolution".

A world here is a *pure environment*, because the simulator composes the scene
the way `gzserver` composes a `.world` with a spawned model:

- **no robot** — referenced separately at `/World/turtlebot3`
- **no ground plane and no light** — the simulator authors both, because the
  floor needs a physics material it controls (see above)
- **Z-up, metres, origin matching the Gazebo world's origin**, so a map recorded
  against one backend is valid against the other
- colliders on everything the lidar should see

It is referenced at `/World/env`, so nothing in it may assume a prim path
outside its own subtree.

Converting an SDF world means walking its `<include>`/`<model>` placements and
referencing the same meshes at the same poses. Keeping the meshes identical
between the two backends is what makes a Gazebo run and an Isaac Sim run
comparable at all; a separately modelled world quietly destroys that.

## maps/

Nav2 needs a map, and `turtlebot3_navigation2` ships exactly one — for
`turtlebot3_world`. Every other world needs its own, so `scripts/build_map.py`
builds one with Isaac Sim's occupancy map generator (`isaacsim.asset.gen.omap`,
the Python side of Tools > Robotics > Occupancy Map). That is the documented
substitute for a hand-drawn map; UPSTREAM.md, "Worlds".

It ray-casts **collision** geometry, not what the renderer draws, so it maps
what a lidar could hit. Two consequences worth knowing:

- The extension is not in the base experience. It has to be
  `enable_extension`'d before its Python module exists, exactly like the ROS 2
  bridge, or the import fails with `ModuleNotFoundError`.
- PhysX only knows the stage after it has been stepped. Without one
  `update_simulation()` first, every cell comes back unknown — which reads like
  a bad `--bounds` rather than an empty collision scene.

The slice is a band, not a plane: `--z-min`/`--z-max` default to 0.10–0.25 m so
that both scanner heights fall inside it (burger 0.182, waffle 0.122). A band
wider than the beam records obstacles the beam can miss, which is visible in a
warehouse of open racks — the map shows a rack 2.5 m away and the scan passes
between its uprights. Narrow the band onto one scanner to close that gap.

### The occupancy map is flipped

**Open bug, do not trust `maps/warehouse.*`.** Observed 2026-09-13: the robot
spawns beside the racks in the stage, but in RViz it localises against the
mirror image of the hall — the props do not line up with the map. Nav2 still
plans and drives, because the warehouse is nearly symmetric, which is precisely
what makes this worth writing down: a mirrored map does not look broken.

There are two candidates and they are independent:

1. **The map is mirrored.** `write_map()` assumes the generator's buffer runs
   `+x` along a row and `+y` up a column, and reverses the rows so the first
   `.pgm` row is maximum y. NVIDIA's own `compute_coordinates()` in
   `isaacsim/asset/gen/omap/utils/utils.py` implies a *different* convention:
   it puts the image's top-left at `(max_x, min_y)` and its top-right at
   `(min_x, min_y)`, so across a row **x decreases**, and down a column **y
   increases**. That is the world rotated, not the layout assumed here. The
   bounds used so far are square (±30 m), so a transpose cannot be caught by
   comparing dimensions.
2. **The scan is mirrored.** `rotationDirection: CW` in the lidar profile was
   carried over from the stock profile and has never been checked against
   REP-103 ordering — it is already the first entry under "Unverified" below.
   A mirrored `LaserScan` would misalign against a perfectly good map.

To tell them apart, test each without the other:

- *Map alone, no ROS.* `generator.get_occupied_positions()` returns occupied
  cells as world coordinates. Compare that set against the world coordinates
  derived from the written `.pgm` by the pixel→world arithmetic in `write_map()`
  and `build_map.py`'s yaml `origin`. If they disagree, it is (1), and the fix
  is the row/column mapping.
- *Scan alone, no map.* Put the robot at a known pose beside an asymmetric
  feature and check the bearing of the returns against the stage geometry: a
  wall on the robot's left must appear at positive bearing. If it appears at
  negative, it is (2), and the fix is in `attach_lidar()`.

Also note, for whoever picks this up: `generate_image()` in that same NVIDIA
file tests the buffer for `1.0`/`0.0` while `update_settings()` is documented as
taking the occupied/free/unknown values to write (this package passes 4/5/6, and
4/5/6 is what comes back). Do not assume the two agree.

## Status

Run against a GPU on 2026-09-13, on Isaac Sim 6.1.0 (`isaacsim61-humble:ngc`),
with the results below. The claims under "Carried over" and "Unverified" that
this did not touch are left as they were.

**Verified live on 6.1.0:** all four existing launch files plus
`warehouse.launch.py`; `/clock`, `/odom`, `/tf` and `/joint_states` at ~45-50 Hz;
`/scan` at the profile's 5 Hz with real returns once something is in range;
`/cmd_vel` driving the robot; `odom -> base_footprint` resolving; and
`turtlebot3_navigation2` bringing up AMCL and Nav2 against a generated map,
reaching a goal 6 m away with zero recoveries. The `.pgm` that run used is
mirrored — see above — so the *navigation stack* is verified working and the
*map* is not.

**Also measured:** an empty world publishes no `/scan` at all, and neither does
the middle of the warehouse floor: the burger's 3.5 m lidar reaches nothing
there, so AMCL stops updating and Nav2 aborts the goal. Both are the sensor
behaving correctly, not a fault. Spawn and navigate near structure.

**Verified statically, before any of the above:** the package builds under
`colcon` and installs every directory where it should; the launch files load through the full include chain with
`isaacsim_bringup` present; `isaacsim.launch.py` assembles the expected
`standalone:=` string in both the empty-world and turtlebot3_world cases, with
`exclude_install_path` correctly derived from `$COLCON_PREFIX_PATH`; and the
simulator's `parse_args()` accepts negative poses and resolves its default asset
path from `TURTLEBOT3_MODEL`.

**Carried over from a working implementation** — the OmniGraph wiring, the lidar
writer attachment, the physics-material values, the `set_pose` fallback and the
`os._exit` teardown were measured live on an equivalent setup: `/clock` 60 Hz,
`/odom` 60 Hz, `/joint_states` 60 Hz, `/scan` 10 Hz, and `/cmd_vel` driving the
robot for real.

**Unverified, in rough order of how much it matters:**

- `rotationDirection` is `CW`, carried over from the stock profile rather than
  guessed at. Whether that yields REP-103 ordering in the published `LaserScan`
  is the first thing to check against Gazebo: a mirrored scan looks entirely
  plausible and is a correctness bug.
- No-return is reported as `-1.0` by the RTX writer where Gazebo publishes
  `inf`. That is the one field Nav2's obstacle layer filters on.
- Gazebo's ray sweeps `0 → 6.28` rad; the writer here is configured
  `-180° → +180°`. Self-consistent, but the two backends do not label the same
  ray with the same index.
- `author_surfaces()` writes materials into the asset rather than into a
  composed stage. The values are verified; this placement of them is not.
- `articulation_root()` searches for the root instead of hard-coding a path.
  More robust, but a different code path.
- `override_joint_damping=1.0e5` is a placeholder, not a tuned value.
- Only `burger` geometry has been checked against the URDF. `waffle` and
  `waffle_pi` wheel and scan offsets come from `turtlebot3_gazebo`'s SDF.
- No camera. `waffle_pi`'s `libgazebo_ros_camera` has no counterpart here yet.
- No IMU. The real robot and Gazebo both publish `/imu`; nothing consumes it in
  the Nav2 path used here, but the interface is incomplete without it.
