# The simulator runtime

A reference for `runtime/turtlebot3_isaacsim.py`: the program that builds the
simulated TurtleBot3 and exposes it to ROS 2.

This document is written to be read without prior Isaac Sim experience. Every
section is split the same way:

> **§x.1 Reference behaviour** — what NVIDIA's documentation and shipped
> examples prescribe for Isaac Sim 6.1.0, independent of this project.
>
> **§x.2 Implementation** — what this package actually does, and where it
> departs from §x.1.

The split is deliberate. Most of what follows is *not* a free design choice:
Isaac Sim's ROS 2 surface is narrow and opinionated, and the useful engineering
question is not "what could be built" but "where does the prescribed path stop
being adequate for a TurtleBot3, and what is the smallest honest departure".
There are four such departures in this file, and each is argued in place.

Sibling documents, not duplicated here:

- [`../DESIGN.md`](../DESIGN.md) — why the package looks the way it does,
  argued against `turtlebot3_gazebo`, the package it must be interchangeable
  with.
- [`../UPSTREAM.md`](../UPSTREAM.md) — a reference ledger of upstream API facts
  and the package's divergences from them.
- [`../launch/isaacsim.launch.py`](../launch/isaacsim.launch.py) — how this
  program is started.
- [`NVIDIA-REFERENCE.md`](NVIDIA-REFERENCE.md) — the citation appendix: every
  claim in the §x.1 sections with its source URL, plus a consolidated list of
  the places where NVIDIA's documentation is silent.

> **A note on citing Isaac Sim documentation.** The public documentation site's
> version selector tops out at 6.0.x and `latest`; there is no separate 6.1.0
> tree. Claims below are drawn from 6.0.x/`latest` pages and, where a behaviour
> mattered enough to be worth certainty, verified directly against the installed
> 6.1.0 build — its shipped `.ogn` node schemas and its
> `standalone_examples/`, which are version-exact in a way a documentation page
> is not. Where the two are cited together, that is why.

**Contents**

1. [Execution context](#1-execution-context)
2. [The world](#2-the-world)
3. [The robot](#3-the-robot)
4. [The ROS 2 interface: OmniGraph](#4-the-ros-2-interface-omnigraph)
5. [The lidar](#5-the-lidar)
6. [Summary of departures](#6-summary-of-departures)
7. [Sources](#7-sources)

---

## 1. Execution context

### 1.1 Reference behaviour

Isaac Sim is not a library that a ROS 2 node imports. It is **Omniverse Kit**, a
complete application — renderer, USD runtime, physics engine, extension system —
that embeds its own Python interpreter. NVIDIA documents three ways to drive it:

| workflow | what it is | started by |
|---|---|---|
| **GUI** | the full editor; scripting through the script editor or an extension | `isaac-sim.sh` |
| **extension** | Python loaded *into* a running Kit application | the extension manager |
| **standalone** | a Python script that *constructs* the application and owns its main loop | `python.sh <script>` |

The standalone workflow is the one appropriate to a headless, reproducible,
launch-file-driven simulation, and it carries one hard rule:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})   # MUST come first
# only now may anything else be imported
import omni.usd
from pxr import UsdGeom
```

`SimulationApp` boots Kit, loads an *experience* file (a `.kit` manifest listing
which extensions to enable), and only once that has happened do the
`isaacsim.*`, `omni.*` and `pxr` modules exist on the interpreter's path.
Importing them earlier fails, or — worse — half-succeeds against a different
USD build. This is why the file below has a block of `# noqa: E402` imports
sitting below executable code: it is not sloppiness, it is the documented
contract, and the linter suppression is the conventional way to express it.

`simulation_app.update()` runs exactly one application frame: it advances the
extension system, services asset loading, evaluates OmniGraph, renders, and —
once the timeline is playing — steps physics. Anything that needs the
application to *make progress* (a reference finishing its load, an extension
finishing its enable) is therefore spelled as a call to `update()`, not as a
sleep.

Physics rate is set through `SimulationManager`:

```python
from isaacsim.core.simulation_manager import SimulationManager
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
```

`dt` is the **physics** timestep. It is not the frame rate. NVIDIA states the
relationship directly: "the simulation can be set to run at 120 time steps per
second, while rendering is set to 60 frames per second, resulting in two physics
steps per rendered frame," and the ratio need not be a whole number, in which
case "each rendered frame may contain an uneven number of simulation
timesteps." A `dt` of `1/240` under 60 Hz rendering therefore means four physics
sub-steps per frame. `device` selects the PhysX backend; `"cpu"` is
deterministic and entirely adequate for a single small articulation.

`setup_simulation()` is the 6.x unified entry point;
`SimulationManager.set_physics_dt()` and `set_physics_sim_device()` are
deprecated, and the classic `World(physics_dt=, rendering_dt=)` class of the
Core API has been superseded for new work by the Core **Experimental** API
(`isaacsim.core.experimental.*`), which Isaac Sim 6.0's own "Getting Started"
tutorial uses in place of `World`. This file imports from
`isaacsim.core.experimental.utils.{app,prim,stage}` for that reason.

Two further pieces of 6.0 context are worth carrying into the thesis even though
this package does not use them. **Newton**, a new physics backend, is available
alongside PhysX and selected with
`SimulationManager.switch_physics_engine("newton")`; PhysX remains the default
and is what runs here. And **multi-tick rendering** decoupled per-sensor render
cadence from the simulation frame rate — before 6.0, "every camera and RTX
sensor rendered at the simulation frame rate". §5 depends on that change.

> **Not documented:** NVIDIA gives no sub-step-rate recommendation for wheeled
> robots as a category. The nearest citable guidance is the `isaac-sim/IsaacSim`
> repository's own `skills/physics-simulation/SKILL.md`, which buckets by system
> type — 60–120 Hz for standard rigid bodies, **240 Hz for contact-rich
> scenes**, 480 Hz for spinning bodies — under the general rule that "physics
> timestep must exceed 4× the highest frequency in your system". This package
> runs at 240 Hz, a figure arrived at by measurement rather than from that
> table; the two happen to agree, and the measurements are recorded on the
> `physics_hz` argument in
> [`../launch/isaacsim.launch.py`](../launch/isaacsim.launch.py).

Finally, nothing publishes while the timeline is stopped. OmniGraph's
`OnPlaybackTick` — the node that drives every ROS 2 publisher — emits only
during playback, so a standalone script must explicitly start the timeline and
then keep calling `update()`:

```python
app_utils.play()
while simulation_app.is_running():
    simulation_app.update()
app_utils.stop()
simulation_app.close()
```

### 1.2 Implementation

`runtime/turtlebot3_isaacsim.py` follows this structure exactly. `main()` reads
as the documented sequence:

```
enable_extension('isaacsim.ros2.bridge')   →  build_stage()
                                           →  check_surfaces()
                                           →  articulation_root()
                                           →  build_graph(chassis)
                                           →  attach_lidar(chassis)
                                           →  setup_simulation(dt, device)
                                           →  play()
                                           →  update() forever
```

Three details are worth drawing out.

**Arguments are parsed before Kit boots.** `parse_args()` is called at module
level, on line 120, *above* the `SimulationApp` construction. This is not
cosmetic: a bad `--model` should fail in milliseconds with an `argparse` error,
not forty seconds later after a renderer has started. It also means
`SimulationApp({'headless': args.headless})` can be configured from the command
line, which would be impossible if parsing happened after the boot.

**The script cannot see ROS 2.** It runs on Kit's Python 3.12 with the system
ROS 2 overlay deliberately stripped from `PYTHONPATH` and `LD_LIBRARY_PATH`, so
`rclpy`, `ament_index_python` and `xacro` are all unavailable inside it.
Everything it needs to locate must therefore arrive as an absolute path on the
command line, supplied by the launch file, which *does* have ament. This is the
package's "interpreter split" and it explains an otherwise odd design: the
simulator takes eleven command-line arguments rather than reading a config
through a ROS parameter server. NVIDIA solves the same problem the same way in
`isaacsim_bringup/scripts/open_isaacsim_stage.py`.

**Shutdown departs from the reference.** NVIDIA's examples end with
`simulation_app.close()`. This script ends with:

```python
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)
```

`close()` races a task-pool teardown inside Kit and aborts the process with a
non-zero status on an otherwise clean exit — which, under `ros2 launch`, is
indistinguishable from a crash. `os._exit()` ends the process immediately and
predictably. The cost is that Python's normal exception reporting never runs,
which is why `main()` is wrapped in an explicit `try/except BaseException` that
prints the traceback itself. Without that wrapper every failure in this file
would look like a silent, successful exit.

> **Departure 1 of 4.** `os._exit()` in place of `simulation_app.close()`.
> Motivated by a reproducible teardown abort; the traceback handler restores
> the diagnostics the shortcut would otherwise destroy.

---

## 2. The world

### 2.1 Reference behaviour

A scene in Isaac Sim is a **USD stage**: a tree of *prims* (primitives), each
carrying typed attributes and optional *API schemas* that add behaviour —
`UsdPhysics.CollisionAPI` makes a mesh collide, `UsdPhysics.ArticulationRootAPI`
marks the root of a jointed mechanism, and so on. A stage is assembled by
*composition*: rather than copying geometry, a stage holds **references** to
other USD layers, which compose in at a chosen prim path.

The documented way to build a scene from a standalone script is therefore:

```python
create_new_stage()
add_reference_to_stage(usd_path=assets_root + "/Isaac/Environments/...", prim_path="/background")
simulation_app.update()
```

Environments shipped by NVIDIA are not bundled with the installation. They are
resolved through `get_assets_root_path()`, which answers with a local asset
pack, a Nucleus server or NVIDIA's S3 bucket depending on how the machine is
configured, and paths are then written as `/Isaac/Environments/...` relative to
that root.

References load asynchronously. `is_stage_loading()` reports whether any are
still in flight, and the documented idiom is to pump the application until it
returns false.

For physics, two further pieces are needed that an environment asset does not
generally supply:

- **A ground plane.** `isaacsim.core.api.objects.GroundPlane` authors an
  infinite `UsdPhysics` collision plane plus a visual mesh.
- **A physics material.** Friction and restitution live on a `UsdShade.Material`
  carrying `UsdPhysics.MaterialAPI`, bound to a collider through the **`physics`
  material purpose**. A plain `Bind()` writes the *render* purpose and PhysX
  ignores it; the binding must name `'physics'` explicitly.

PhysX combines the two materials in a contact with a *combine mode* —
`average` (the default), `min`, `max` or `multiply` — declared per material
through `PhysxSchema.PhysxMaterialAPI`. The default of `average` means a
material's properties are negotiable: a restitution of 0 on one surface is
averaged upward by whatever it touches.

The asset root itself is resolved from the persistent setting
`persistent.isaac.asset_root.default`, whose shipped default is NVIDIA's public
S3 bucket, overridable to a local pack or a Nucleus server.

> **Not documented:** NVIDIA recommends no particular friction or restitution
> combine mode for robot-on-floor contact. The combine modes are documented as
> existing; which to use is left to the integrator.

### 2.2 Implementation

`build_stage()` is thirty lines and does exactly the above, in this order:
ground plane and its material, light, world reference, robot reference, wait
for loading, apply the world offset, place the robot.

Two things in it are not in the reference.

**The ground plane is given an explicit material, with combine mode `min`.**

```python
floor = PhysicsMaterial(prim_path=MATERIALS_PRIM + '/floor',
                        static_friction=1.0, dynamic_friction=1.0,
                        restitution=0.0)
PhysxSchema.PhysxMaterialAPI.Apply(
    floor.prim).CreateRestitutionCombineModeAttr().Set('min')
GroundPlane(prim_path='/World/GroundPlane', physics_material=floor)
```

Handed no material, `GroundPlane` authors one of its own **at restitution 0.8**.
Combined by PhysX's default `average` with the robot's colliders, every
wheel-on-floor contact lands at roughly 0.4 — a distinctly bouncy floor. On a
burger this is not cosmetic. Its centre of mass sits 4.3 mm behind the wheel
axle while the rear caster skid clears the floor by 0.5 mm, so the chassis rests
permanently on that skid, exactly as the real robot does. A permanently loaded
*elastic* contact under a rear skid is a rocking chair: the robot rocks and
creeps across the floor with nothing commanding it, and nothing is logged.
`restitution=0.0` with combine mode `min` makes "does not bounce" hold against
whatever the other collider brings, rather than being negotiated back up by it.

The binding then has to be repeated by hand:

```python
UsdShade.MaterialBindingAPI.Apply(
    stage.GetPrimAtPath('/World/GroundPlane')).Bind(
        floor.material, UsdShade.Tokens.weakerThanDescendants, 'physics')
```

because `GroundPlane` binds its material only to the visual mesh, leaving the
*infinite collision plane* beside it — the surface the robot actually rests on —
on the PhysX fallback material.

**A world may need lifting onto the ground plane.** `--world-z` exists because a
stock environment is under no obligation to put its floor at z = 0, and the
environments used in this project disagree about it:

| world | floor top | ships its own GroundPlane | `world_z` |
|---|---|---|---|
| `Simple_Warehouse/warehouse.usd` | 0.0 | no | 0.0 |
| `replicator_kitchen/kitchen_u_shape.usda` | 0.0000 | no | 0.0 |
| `Simple_Room/simple_room.usd` | −0.7696 | **yes**, at −0.7695 | **0.7696** |

Left alone, Simple_Room's floor sits 0.77 m *below* the package's own ground
plane. PhysX ejects anything beneath an infinite plane, so the robot settles on
the higher surface and drives three quarters of a metre above the visible floor.
The room then reads as an empty box: its only furniture, `table_low`, has its
top at z = +0.0104 — one centimetre into the robot's body and seventeen
centimetres below the lidar beam — so it is an obstacle that neither the scan
nor the generated map can see. This is a *silent* failure. It renders correctly,
it simulates without error, and it produces a map that loads in Nav2.

`lift_world()` in [`assets.py`](assets.py) raises the world reference instead of
lowering the robot, and is deliberately **not** the same function that places
the robot:

```python
def lift_world(prim_path, world_z):
    if not world_z:
        return
    ...
```

Two properties matter. Zero authors *nothing at all*, so every world that was
already correct composes exactly as it did before the argument existed. And it
authors a translation only — `set_pose()` also writes a rotation, because
placing a robot means setting its yaw, and handed a world whose root prim
carries a rotation of its own it would silently level the scene while raising
it.

It is called *after* the asset-loading loop, not before, because its fallback
path reads the `xformOps` the referenced layer authored and those do not exist
until the reference has composed.

> **The sharp edge.** `scripts/build_map.py` composes its own stage and
> ray-casts that, so nothing cross-checks the two programs. Give the simulator
> `--world-z 0.7696` and the map builder nothing, and the result is a
> correct-looking occupancy map of the right room at the wrong height. It loads
> in Nav2 and localises the robot into a scene that is not there. Both programs
> take the argument by the same name and hand it to the same `lift_world()`
> precisely so that the two *can* be given the same value.

---

## 3. The robot

### 3.1 Reference behaviour

A jointed robot in USD is an **articulation**: a tree of rigid bodies connected
by joints, with one prim in the tree carrying `UsdPhysics.ArticulationRootAPI`.
PhysX solves the whole tree as a unit, and every Isaac Sim node that drives or
reads a robot — `IsaacArticulationController`, `IsaacReadJointState`,
`IsaacComputeOdometry` — is pointed at that root prim.

The documented route from a ROS robot description to such an asset is the
**URDF importer** (`isaacsim.asset.importer.urdf`), which reads a URDF, resolves
its meshes once, and writes a USD asset. Options of consequence:

- `merge_fixed_joints` — collapses links joined by fixed joints into their
  parent, reducing solver work. The links so merged **cease to exist as prims**.
- `joint_drive_type` and the drive gains — a joint driven in velocity is a
  PhysX drive with stiffness 0 and high damping; a joint driven in position is
  the reverse.

**Drive configuration.** Joint drives are `UsdPhysics.DriveAPI`, and the two
modes are distinguished by which gain dominates:

| mode | stiffness | damping | used for |
|---|---|---|---|
| position | high | ~0 | steering, arm joints |
| **velocity** | **0** | **high** | **wheels** |

A wheel rotates continuously and has no meaningful target angle, so it is driven
as a pure damper against a target velocity. The URDF importer documentation's own
worked wheel example sets damping 15000 with stiffness left at 0. (NVIDIA's own
examples are not perfectly uniform — the mobile-robot rigging tutorial uses
damping 10000 with stiffness 100 — but the pattern "propulsion joints are
damping-dominated" is consistent.) The command itself is applied through
`IsaacArticulationController`.

**Wheel colliders.** This is worth stating carefully because the intuitive
answer is wrong. NVIDIA's guidance is a **shape-matched cylinder**, and
explicitly *against* convex decomposition for wheels:

> "any collision approximation that is not smooth and captures the exact shape
> and curvature of the wheel causes bumpy motion when attempting to drive the
> wheel" … "while convex decomposition provides tighter mesh approximation for
> general collision, it's unsuitable for wheels because the resulting geometry
> produces uneven rolling behavior."

PhysX additionally special-cases cylinder and cone geometry for smooth contact
against triangle meshes, specifically "for better wheeled simulation behaviour",
at a stated performance cost; the behaviour is controlled by the stage settings
`/physics/collisionApproximateCylinders` and `/physics/collisionApproximateCones`.

> **Not documented:** spheres as a wheel workaround. The documented sphere
> options — "Bounding Sphere" and "Sphere Approximation" — are generic,
> shape-agnostic mesh approximations, and neither is presented anywhere as a fix
> for cylindrical wheels rolling badly. NVIDIA's position is the opposite one:
> fit the cylinder better.

### 3.2 Implementation

The asset is built ahead of time by `scripts/build_models.sh` →
`scripts/import_turtlebot3.py`, not by this runtime, and the runtime's job is
only to reference it, find its articulation root and verify it.

**The articulation root is searched for, not hard-coded.**

```python
for prim in Usd.PrimRange(root):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return str(prim.GetPath())
raise RuntimeError('no articulation root under {} ...'.format(ROBOT_PRIM))
```

The importer's *current* behaviour is documented — it stamps the root link of
the URDF, and NVIDIA separately recommends "the base or the chassis of a mobile
robot" as the right anchor — but nowhere is that placement stated to be a stable
contract across versions, and in practice it has moved between releases. (In 6.0
it stamps **two** schemas on that prim: `UsdPhysics.ArticulationRootAPI` and a
new `NewtonArticulationRootAPI`, so the same asset works under either physics
backend. The search below keys on the first, which is the portable one.) A stale
hard-coded path does not raise: it yields an inert robot that loads, renders and
publishes correctly and simply never moves. A
three-line search removes a whole class of silent failure, and the explicit
`raise` converts the remaining case into a message naming the script that
rebuilds the asset.

**The asset is a directory, not a file.** Isaac Sim 6's URDF importer does not
write `turtlebot3_burger.usd`; it writes an *asset structure* — a documented
change, aligned to "USD Asset Structure 3.0", in which the output is a directory
of layers (`geometries.usd`, `materials.usda`, `instances.usda`, `robot.usda`,
`physics.usda`) plus `payloads/`, composed through references and variants. Here
that means a directory named `turtlebot3_burger.usd` containing `payloads/`
beside an entry-point layer at `turtlebot3_burger/turtlebot3_burger.usda`. Handed the directory,
`add_reference_to_stage` reports it as "wasn't found", which reads exactly like
an asset that was never built. `asset_layer()` in [`assets.py`](assets.py)
resolves the directory to its inner layer and passes a plain file through
untouched.

**`check_surfaces()` refuses an asset that has lost its materials.**

```python
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
    if not material:
        bad.append(...)
```

Neither URDF nor the importer supplies friction or restitution, so the package
authors physics materials *into* the robot asset at import time — wheels at
μ = 1.0, chassis and caster skid at μ = 0.1 with combine mode `min`, everything
at restitution 0. An asset built before that step existed still loads, still
plays and still publishes; it simply never settles. The symptom reads as a
physics-tuning problem rather than a stale build, and the cost of diagnosing it
from scratch is hours. Nine lines at startup convert it into one sentence
naming the rebuild script.

This is a recurring shape in this file, and worth naming once: **every check in
it guards a failure that is silent rather than loud.** A missing articulation
root, a stale material binding, a lidar profile that did not apply, a world at
the wrong height — none of these raise on their own, all of them produce a
simulation that looks right and is not.

---

## 4. The ROS 2 interface: OmniGraph

### 4.1 Reference behaviour

Isaac Sim's ROS 2 bridge is not an API you call. It is a set of **OmniGraph**
nodes. OmniGraph is Kit's visual dataflow system: a graph of typed nodes, wired
output-to-input, evaluated by the application. Nodes have two kinds of port —
*data* ports carrying values, and *execution* ports carrying control flow,
drawn as a distinct connection that says "now run".

An **action graph** (evaluator `execution`) is the flavour used for ROS: it is
driven by an execution source, and the canonical source is
`omni.graph.action.OnPlaybackTick`, which fires once per rendered frame while
the timeline is playing.

The documented way to build one from Python is `og.Controller.edit`, taking a
graph specification and three keyed lists:

```python
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"), ...],
        og.Controller.Keys.CONNECT:      [("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"), ...],
        og.Controller.Keys.SET_VALUES:   [("PublishClock.inputs:topicName", "clock"), ...],
    },
)
```

The nodes relevant to a differential-drive robot:

| node type | role |
|---|---|
| `omni.graph.action.OnPlaybackTick` | execution source, once per frame during playback |
| `isaacsim.core.nodes.IsaacReadSimulationTime` | the simulation clock, as a double |
| `isaacsim.ros2.bridge.ROS2PublishClock` | `/clock`, which is what makes `use_sim_time` mean anything |
| `isaacsim.ros2.bridge.ROS2SubscribeTwist` | `/cmd_vel` → two `vectord[3]` outputs |
| `isaacsim.robot.wheeled_robots.DifferentialController` | (v, ω) → wheel speeds, `double[2]`, **order left, right**, rad/s |
| `isaacsim.core.nodes.IsaacArticulationController` | applies a command to named joints of an articulation |
| `isaacsim.sensors.physics.IsaacReadJointState` | reads names, positions, velocities, efforts from an articulation |
| `isaacsim.ros2.bridge.ROS2PublishJointState` | serialises the above to `/joint_states` |
| `isaacsim.core.nodes.IsaacComputeOdometry` | pose and velocity of a chassis prim |
| `isaacsim.ros2.bridge.ROS2PublishOdometry` | serialises the above to `nav_msgs/Odometry` |
| `isaacsim.ros2.bridge.ROS2PublishTransformTree` | a TF subtree |
| `isaacsim.ros2.bridge.ROS2PublishRawTransformTree` | a single parent→child transform |

**A breaking change in Isaac Sim 6.0 matters here.** The ROS 2 publishers were
reduced to pure serialisers: they no longer resolve USD prims themselves.
`ROS2PublishJointState` still accepts a `targetPrim` input, but its schema
documents the connected form as the preferred path —

> `"jointNames": "Joint names from Isaac Read Joint State (connect instead of targetPrim for preferred path)"`

— and prim resolution moved to dedicated source nodes
(`IsaacReadJointState`, `IsaacComputeTransformTree`). NVIDIA's own 6.1.0
standalone example `standalone_examples/api/isaacsim.ros2.bridge/moveit.py`
wires the source-node form.

**On TF ownership**, NVIDIA's reference integration is explicit: the simulator
publishes the odometry transform and the *lidar's static mount*, and

>
> "The remaining robot-link transforms are calculated by an external
> `robot_state_publisher` that you launch separately."

That single sentence is, however, as far as the guidance goes.

> **Not documented, and this is a genuine silence worth recording:** NVIDIA
> gives *no* guidance anywhere on `ROS2PublishTransformTree` and a separately
> run `robot_state_publisher` conflicting over the same frames. Its tutorials
> build the entire TF tree from inside OmniGraph and simply do not discuss the
> alternative architecture — Isaac Sim publishing only `/joint_states`, with TF
> derived from the URDF on the ROS side — as either a supported option or an
> anti-pattern. The architecture in §4.2 is therefore not a departure from a
> documented recommendation; it is a decision taken where documentation
> stops.

One naming trap for anyone reading the 6.x sources: several of these nodes moved
**extension** from `isaacsim.ros2.bridge` to `isaacsim.ros2.nodes`, while their
OmniGraph **type strings** stayed `isaacsim.ros2.bridge.*`. In the installed
6.1.0 build, `ROS2PublishJointState`'s schema file lives under
`exts/isaacsim.ros2.nodes/` and declares its type as
`isaacsim.ros2.bridge.ROS2PublishJointState`. The type string is what
`og.Controller.edit` wants.

### 4.2 Implementation

`build_graph()` builds **one** action graph at `/World/ROS2Interface`. Upstream
tutorials build four (`joint_states`, `odom_tf`, `scan`, `cmd_vel`); one graph
with the same nodes in it is equivalent and easier to read as a single unit.

The graph divides into four independent chains, all triggered from the same
`OnTick`:

```
OnTick ─┬─▶ PubClock ◀── SimTime                                   /clock
        │
        ├─▶ ComputeOdom ──▶ PubOdom ◀── SimTime      /ground_truth/odom
        │
        ├─▶ ReadJointState ──▶ PubJointState ◀── SimTime     /joint_states
        │
        └─▶ SubTwist ──▶ DiffController ──▶ ArticController      /cmd_vel
                │  └── BreakLinVel.x ──┘
                └───── BreakAngVel.z ──┘
```

**The `BreakVector3` nodes are an impedance mismatch, not a design choice.**
`ROS2SubscribeTwist` emits `linearVelocity` and `angularVelocity` as
`vectord[3]`. `DifferentialController` takes two scalars. The two cannot be
wired directly, so `omni.graph.nodes.BreakVector3` decomposes each vector and
only the components a differential drive can obey — linear *x* and angular *z* —
are connected. The rest are discarded, which is the correct reading of a Twist
for a non-holonomic robot.

**`DifferentialController` is fed the geometry from a shared table.**

```python
wheels = WHEELS[args.model]
...
('DiffController.inputs:wheelRadius', wheels['radius']),
('DiffController.inputs:wheelDistance', wheels['separation']),
```

`WHEELS` lives in [`geometry.py`](geometry.py), which imports nothing, and is
read by three consumers: this graph (the forward map, `/cmd_vel` → wheels), the
odometry node (the inverse map, wheels → pose), and the launch file, which loads
the module *by path* to configure the second without importing the first. A
wheel radius that differed between the forward and the inverse map would be a
scale error presenting as wheel slip — that is, as a plausible physical
phenomenon rather than as a bug.

**`/joint_states` uses the 6.x source-node form.**

```python
('OnTick.outputs:tick',                    'ReadJointState.inputs:execIn'),
('ReadJointState.outputs:execOut',         'PubJointState.inputs:execIn'),
('ReadJointState.outputs:jointNames',      'PubJointState.inputs:jointNames'),
('ReadJointState.outputs:jointPositions',  'PubJointState.inputs:jointPositions'),
...
('ReadJointState.inputs:prim', [usdrt.Sdf.Path(chassis)]),
```

The execution edge runs *through* the read node rather than around it, so the
publisher fires once the read has data rather than one evaluation ahead of it.
One connection in upstream's example is deliberately omitted:
`ReadJointState.outputs:sensorTime` is a `float32` and, when connected, takes
over the message stamp. `IsaacReadSimulationTime` is a `double` and is the same
clock `/clock` carries. Since `wheel_odometry.py` computes Δt from consecutive
stamps, and float32 seconds quantise to roughly 6 × 10⁻⁵ s after a thousand
seconds of simulation — some 0.3 % of a 50 Hz step, injected straight into the
reported twist — every publisher in this graph is stamped from `SimTime`
instead.

**No TF is published from this graph at all.** This is the largest structural
statement in the file and it follows from a single principle: *each transform
has exactly one owner.*

```
map --(amcl)--> odom --(wheel_odometry)--> base_footprint --(robot_state_publisher)--> …
```

`robot_state_publisher` holds the URDF and owns everything below
`base_footprint`. Adding a `ROS2PublishTransformTree` here would make the
simulator a second publisher of those same frames, and two publishers of one
transform is not a redundancy — it is a race that surfaces as TF extrapolation
warnings and a robot model that flickers in RViz.

**Departure: `/odom` does not come from this graph.** `IsaacComputeOdometry`
reads the *chassis prim*. Its output is therefore the robot's true pose, and
publishing that on `/odom` would be wrong in a specific and consequential way:
on the real TurtleBot3, `/odom` is integrated from wheel encoders by
`turtlebot3_node`, and in Gazebo by `gazebo_ros_diff_drive`, and **both drift**.
Correcting that drift is the entire job of AMCL. Ground truth on `/odom` would
have made localisation trivially easy on exactly one of the three backends this
project treats as interchangeable, and would have hidden a real phenomenon: the
burger's wheels slip roughly 7 % in a pivot and 0.5 % driving straight.

So the graph publishes its chassis pose to `/ground_truth/odom` — the same topic
and meaning the Gazebo backend's P3D plugin uses — and `/odom` plus the
`odom → base_footprint` transform are produced by
[`../nodes/wheel_odometry.py`](../nodes/wheel_odometry.py), an ordinary ROS 2
node integrating `/joint_states`.

Two further notes on that choice. First, the frames are TurtleBot3's, not
upstream's: `chassisFrameId` is `base_footprint` where every NVIDIA example uses
`base_link`. Second, `IsaacComputeOdometry`'s origin is documented only in passing and only
in prose: the node's own OGN reference page says nothing but "Position vector in
meters", while the ROS 2 Transform Trees tutorial describes it as computing "the
position of the robot relative to its start location". Because the distinction
decides whether `/ground_truth/odom` can be differenced against `/odom` at all,
it was verified here rather than taken on the strength of one tutorial sentence:
spawning at (−2.0, −0.5) and reading (−0.0, −0.0) out of the node. The two
agree.

The related gap is sharper. **NVIDIA never states in so many words that
`IsaacComputeOdometry` is ground truth rather than an encoder model.** It is
inferable — the node's only data input is `chassisPrim`, it has no joint-state
input at all, and it lives in `isaacsim.core.nodes` rather than among the
sensors — but it is not written down, and a reader who assumed the node modelled
a real odometry source would get a robot whose localisation error is identically
zero and would have no warning of it. That inference is the whole reason for the
departure below.

Finally, **why a ROS node rather than a custom OmniGraph node**: NVIDIA ships
`IsaacComputeOdometry` and nothing encoder-based. A graph node doing this
arithmetic would have to be written in Kit's Python against an unstable node
API, to compute something that is not simulation at all. The real robot computes
its odometry in a ROS node too.

> **Departure 2 of 4.** `/odom` is integrated by an external ROS 2 node;
> the graph's chassis-prim odometry is relabelled `/ground_truth/odom`.
> Motivated by cross-backend equivalence and by not hiding wheel slip.
>
> **Departure 3 of 4.** No TF from the graph; `robot_state_publisher` and the
> odometry node own the tree between them. This is *stricter* than upstream's
> reference, which publishes the lidar mount to `/tf_static`; here that
> transform comes from the URDF like every other one.

---

## 5. The lidar

This section is longer than the others because the lidar is the one subsystem
that does **not** live in the action graph, and the reason why is the most
commonly misunderstood part of the Isaac Sim sensor pipeline.

### 5.1 Reference behaviour

#### 5.1.1 What an RTX lidar is

Isaac Sim offers two unrelated lidar implementations. The **PhysX** lidar
ray-casts against collision geometry — cheap, and blind to anything without a
collider. The **RTX** lidar is a *rendering* sensor: it is traced by the RTX
renderer against the visual scene, through a sensor model that reproduces beam
divergence, intensity, reflectance thresholds and per-beam noise. It is the
higher-fidelity option and the one NVIDIA documents for ROS 2 integration.

That choice has a structural consequence. Because the RTX lidar is produced by
the *renderer*, its data does not arrive on the physics tick. It arrives on the
**render** tick, through Omniverse's synthetic-data pipeline.

#### 5.1.2 The pipeline

Three objects, in a chain:

1. **The `OmniLidar` prim.** A USD prim carrying the sensor model as attributes
   under the `omni:sensor:Core:` namespace — `scanRateBaseHz`,
   `patternFiringRateHz`, `nearRangeM`, `farRangeM`, the per-emitter azimuth and
   elevation tables, and so on. This prim *is* the sensor specification.

2. **The render product.** A `UsdRender.Product` — the renderer's output target
   for that sensor, created when the prim is wrapped in a sensor object. This is
   what makes the lidar a thing the renderer draws every frame.

3. **A writer or annotator**, attached to the render product. **Annotators**
   expose the data to Python. **Writers** consume it and do something with it —
   draw it in the viewport, save it to disk, or publish it to ROS 2.

The ROS 2 publisher is therefore a *Replicator writer*, and attaching it is
literally:

```python
writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
writer.initialize(**kwargs)
writer.attach([render_product_path])
```

**This answers the question the section opened with.** The RTX lidar is *not*
outside OmniGraph. Replicator's synthetic-data pipeline *is* an OmniGraph, and
NVIDIA's own instructions for inspecting it say as much: open "Window > Graph
Editors > Action Graph, choose Edit Action Graph and open the graph named
`/Render/PostProcess/SDGPipeline`". Inside it sits
`IsaacRenderVarToCPUPointer`, which pulls the `RtxSensorCpu` buffer out of the
frame's render product, feeding lidar-specific processing nodes ("Compute RTX
Lidar Flat Scan") and finally "a writer or publisher node of some kind, like
ROS, or ROS2".

The `omni.syntheticdata` extension builds that graph per render product with
`og.Controller` — the same API `build_graph()` uses — and registers it at a
different point in the frame:

```python
# omni/syntheticdata/scripts/SyntheticData.py, abridged
_graphName = "SDGPipeline"
...
pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
...
pipelineStage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_POSTRENDER
```

So there are two graphs, not a graph and a not-graph:

| | action graph | `SDGPipeline` |
|---|---|---|
| built by | `build_graph()`, explicitly | `attach_writer()`, implicitly |
| pipeline stage | simulation | **post-render** |
| driven by | `OnPlaybackTick` | the renderer finishing a frame |
| carries | `/clock`, `/cmd_vel`, `/joint_states`, `/ground_truth/odom` | `/scan` |

The two cannot be merged, and the distinction is not bureaucratic. NVIDIA's
"multi-tick rendering" page documents the per-update ordering as three clocks:
the run loop advances the timeline; physics executes *N* sub-steps of
`physics_dt` and writes the cumulative simulated time to `/ExternalSimulationTime`;
**Hydra then reads that value once and compares it against each sensor's last
render time and its `omni:sensor:tickRate`** to decide which sensors render this
frame; and only then do OmniGraph nodes read the same time for consistent
timestamps.

Three consequences follow, and together they are the full answer to "why not
just put the lidar in the action graph":

1. A node in the simulation stage runs *before* the frame is rendered, so the
   lidar returns for that frame do not exist yet.
2. A LaserScan needs a *whole revolution*, which for a rotary sensor spans
   several render ticks. It is inherently a multi-tick, render-cadence operation,
   not a single physics tick's computation.
3. A render product is an expensive renderer-owned resource, documented as
   create-once-and-reuse; it is created by wrapping the prim, not by the graph.

This decoupling is also what lets physics run at 240 Hz while the lidar turns at
5 Hz. Before Isaac Sim 6.0 it could not: "every camera and RTX sensor rendered
at the simulation frame rate".

And so — importantly — the separation costs nothing at the ROS level. Both
graphs are evaluated by the same application, in the same process, on the same
`simulation_app.update()`.

**No additional ROS 2 node is required to publish `/scan`.** The writer
publishes `sensor_msgs/LaserScan` from inside the render pipeline, in the
simulator's own process. (The one extra ROS node in this package,
`nodes/wheel_odometry.py`, exists for odometry and has nothing to do with the
lidar.) Nor is a `pointcloud_to_laserscan` node required: the LaserScan writer
is used rather than the point-cloud one precisely so that no consumer of this
package needs a conversion node the real robot and Gazebo both do without.

#### 5.1.3 The two documented ways to attach the publisher

**(a) The OmniGraph helper node.** `isaacsim.ros2.bridge.ROS2RtxLidarHelper`
placed in the action graph, with inputs `type` (`laser_scan` or `point_cloud`),
`topicName`, `frameId` and `renderProductPath`. Its own description is *"Handles
automation of Lidar Sensor pipeline"* and its `execIn` is documented as
*"Triggering this causes the sensor pipeline to be generated"*. It is a
convenience: a graph node whose job is to attach the very writer described
above. Note that it still requires a `renderProductPath` — a render product
created outside the action graph.

**(b) The writer, attached directly from Python.** This is what NVIDIA's own
6.1.0 standalone example `standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py`
does, and it is the form appropriate to a standalone script:

```python
# NVIDIA's own example, abridged
lidar_2D = Lidar.create(path="/sensor_2D", config="Example_Rotary_2D",
                        tick_rate=10.0, translations=[[0.0, 0.0, 1.0]])
sensor_2d = LidarSensor(lidar_2D, annotators=[])
laser_scan_meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(lidar_2D.paths[0]))
sensor_2d.attach_writer("RtxLidarROS2PublishLaserScan",
                        topicName="scan", frameId="base_scan", **laser_scan_meta)
```

with, in the same file:

```python
def _read_laser_scan_metadata(prim):
    """Mirrors the metadata extraction performed by ``OgnROS2RtxLidarHelper`` so the
    LaserScan writer can be initialized directly from the prim authored by ``Lidar.create()``."""
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range    = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range     = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate   = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("LaserScan: scanRateBaseHz or patternFiringRateHz is 0 on the lidar prim")
    return {"horizontalFov": 360.0,
            "horizontalResolution": 360.0 * rotation_rate / firing_rate,
            "depthRange": [near_range, far_range],
            "rotationRate": rotation_rate,
            "azimuthRange": [-180.0, 180.0]}
```

Two documented facts are embedded there. The writer **does not read the scan
geometry off the prim**; it must be passed explicitly, replicating what the
helper node does internally. And **`tick_rate` must equal the profile's scan
rate** — NVIDIA's example carries the comment *"Example_Rotary scans at 10 Hz,
so tick_rate must be 10"*.

#### 5.1.4 The 6.0 change to custom profiles

Through Isaac Sim 4.x, a custom lidar could be described by a JSON profile
registered with the sensor extension and named by `Lidar.create(config=...)`.
From 5.0 onward that registry is no longer reachable from the Python API:
`config=` resolves against `SUPPORTED_LIDAR_CONFIGS` — a fixed set of stock USD
assets under the asset root — or a direct USD path, and is not extensible. An
NVIDIA maintainer states the supported path plainly: *instantiate a supported
config and override its sensor attributes programmatically.* The equivalent for
a sensor with no suitable stock config is to author the attributes onto the
`OmniLidar` prim directly, via `Lidar.create(attributes={...})`.

> **Not documented:** a complete enumeration of the `omni:sensor:Core:*` schema
> — names, types and defaults. The documentation page that would carry it,
> "Creating Custom RTX Sensor Profiles", is marked as still under development.
> There is consequently no authoritative list to validate a hand-authored
> profile against. §5.2 responds to exactly this.

### 5.2 Implementation

`attach_lidar()` follows path **(b)**, upstream's own standalone form, and can
be read as `rtx_lidar.py` with one substitution: a package-supplied sensor model
in place of `config="Example_Rotary_2D"`.

**Why a custom profile at all.** The obvious move is to use the stock
`Example_Rotary_2D`, and it is wrong for this robot in two ways. It is a 200 m
survey lidar, where the LDS-01 sees 3.5 m; and its single emitter sits at
`elevationDeg: [-2.0]`, so it scans the *floor*. On a bare ground plane it still
returns hits, spread into a partial arc where the tilted beam meets the ground —
which Nav2 will treat as an obstacle ring around the robot. Re-rating it by
overriding `scanRateBaseHz` and `patternFiringRateHz` is not a workaround
either: the scan pattern baked into that config still assumes its own 30 Hz /
32000 Hz, and the plugin then warns `Multi-tick is enabled but motion BVH is not
active` and publishes in bursts.

So `models/lidar_configs/turtlebot3_lds.json` models the real sensor, matched
term for term against `turtlebot3_gazebo`'s `<ray>` block so the two simulators
present the same sensor:

| | Gazebo `<ray>` | `turtlebot3_lds.json` |
|---|---|---|
| samples/rev | 360 | 1800 Hz ÷ 5 Hz = 360 |
| resolution | 1.0° | 1.0° |
| rate | 5 Hz | `scanRateBaseHz: 5.0` |
| range | 0.12–3.5 m | `nearRangeM` / `farRangeM` |
| range resolution | 0.015 m | `rangeResolutionM` |
| noise σ | 0.01 m | `rangeAccuracyM` |
| elevation | horizontal | `elevationDeg: [0.0]` |

**`profile_attributes()` translates the profile into schema attributes.** The
profile stays the source of truth; the function only restates it in the names
and the case the USD schema uses. Four small tables carry the differences:

```python
PROFILE_PREFIX      = 'omni:sensor:Core:'
PROFILE_RENAMES     = {'reportRateBaseHz': 'patternFiringRateHz',
                       'minReflectanceRange': 'minReflectionRangeM',
                       'wavelengthNm': 'waveLengthNm'}
PROFILE_TOKENS      = ('scanType', 'intensityProcessing', 'rotationDirection',
                       'rayType', 'intensityMappingType')   # UPPER CASE on the prim
PROFILE_STRUCTURAL  = ('emitterStateCount', 'emitterStates')  # read from shape, not copied
PROFILE_UNSUPPORTED = ('avgPowerW',)                          # dropped in the 6.0 schema
```

`avgPowerW` is *dropped rather than mapped*. The 6.0 schema exposes `peakPowerW`,
which is a different physical quantity; silently substituting one for the other
would put a wrong number into the sensor model and make the simulation
unfalsifiable against the datasheet.

**Two arguments to `Lidar.create` are passed that the reference example leaves
default**, and the reason is worth stating because it is the kind of thing that
only shows up at runtime as *nothing happening*:

```python
accumulate_outputs=True,
tick_rate=float(profile['scanRateBaseHz']),
```

Both default to "keep whatever the asset authored", which is right for a stock
asset and meaningless for a prim built attribute-by-attribute from a profile —
what stands instead is the schema default, and the schema does not know this
lidar turns five times a second. With `accumulate_outputs` off, the model emits
each render frame's *slice* of the sweep separately; with a 10 Hz tick against a
5 Hz revolution, a tick lands mid-scan and never on a whole one. A LaserScan
message requires a full revolution. Either way the writer never sees one, and
`/scan` is advertised and then silent — no error, no warning, no messages.

**The profile is verified against the prim, not trusted.**

```python
unknown = sorted(a for a in attributes if not prim.HasAttribute(a))
if unknown:
    raise RuntimeError('the OmniLidar schema has no {} ...')
...
if abs(scan_hz - float(expected)) > 1e-6:
    raise RuntimeError('lidar resolved to another profile: ...')
```

`Lidar.create` only *logs a warning* for an attribute the prim does not have,
which would leave that line of the profile quietly unapplied — a sensor silently
running on schema defaults. The first check turns that into a failure naming the
attribute. It is not defensiveness for its own sake: since NVIDIA publishes no
complete `omni:sensor:Core:*` attribute list (§5.1.4), the *live prim* is the
only authority on what the schema accepts, and checking the translated profile
against it is the only available substitute for the missing reference page.
`PROFILE_RENAMES` and `PROFILE_UNSUPPORTED` were each derived by running exactly
this check and reading what it rejected. The second reads the scan rate back off the prim and compares it to
the file, which catches the case where the prim resolved to some other
configuration entirely. The values then handed to the writer are the ones *read
back from the prim*, not the ones from the file, because what the prim carries
is what the renderer scans with.

**Where the lidar sits.** The sensor is created at `chassis + '/lidar'` with

```python
SCAN_OFFSET = {'burger': (-0.032, 0.0, 0.182), ...}
```

This is an offset from the articulation root — that is, from `base_footprint` —
and not from a `base_scan` prim, because `merge_fixed_joints=True` means no such
prim exists on the stage. The constant is `turtlebot3_description`'s `base_joint`
origin (0, 0, 0.010) composed with `scan_joint`'s (−0.032, 0, 0.172): exactly
the transform a `base_footprint → base_scan` chain would give. It must be,
because the `frameId` on the published scan is `base_scan`, and the TF for that
frame comes from `robot_state_publisher` reading the URDF. If the two disagree,
every range is correct and every range is in the wrong place.

> **Not documented:** any reconciliation, validation or warning when a sensor's
> `frameId` string and the robot's actual TF frame name diverge. `frameId` is a
> plain string set by hand on the writer; NVIDIA relies entirely on the
> integrator keeping the two consistent by convention. Nothing at runtime
> notices.

That silence is why the offset is checked offline instead. The composition is
the easy thing to get wrong — `scan_joint`'s origin alone is
relative to `base_link`, which sits 0.010 m above `base_footprint` on all three
models. The waffle and waffle_pi entries were off by exactly that 0.010 m until
`scripts/smoke_test.py` was extended to read `SCAN_OFFSET` back out of this file
with `ast` and compare it against the URDF. Reading it back from the source file
rather than restating the numbers in the test is the point: a test carrying its
own copy of the constant cannot catch the constant being wrong.

**A note for anyone testing this.** `/scan` is legitimately silent in
`empty_world.launch.py`. The writer publishes only when the sweep returned
something, and a horizontal beam over a bare ground plane returns nothing at
all: the topic is advertised and no message ever arrives. It reads exactly like
a broken sensor and is not one. Anything with a wall in it publishes at the
profile's 5 Hz.

> **Departure 4 of 4.** A package-authored sensor profile, applied
> attribute-by-attribute, in place of a stock `config=`. Forced by the 6.0
> removal of the JSON profile registry from the Python API, and motivated by the
> stock 2D rotary configuration being a tilted 200 m survey lidar.

---

## 6. Summary of departures

| # | Reference behaviour | This implementation | Why |
|---|---|---|---|
| 1 | `simulation_app.close()` | `os._exit()` + explicit traceback handler | `close()` races a teardown and aborts; a clean exit must not look like a crash |
| 2 | `IsaacComputeOdometry` → `/odom` | → `/ground_truth/odom`; `/odom` integrated by a ROS node from `/joint_states` | ground-truth odometry does not drift, so AMCL would have nothing to correct and wheel slip would be invisible |
| 3 | `ROS2PublishTransformTree` for the lidar mount | no TF from the graph at all | one owner per transform; the URDF already describes that mount |
| 4 | `Lidar.create(config="Example_Rotary_2D")` | `Lidar.create(attributes=profile_attributes(...))` | the stock 2D config is a tilted 200 m survey lidar; 6.0 removed the JSON profile registry |

---

## 7. Sources

The §x.1 sections are drawn from `docs.isaacsim.omniverse.nvidia.com` (6.0.x and
`latest`), the `isaac-sim/IsaacSim` GitHub repository, and the installed 6.1.0
build itself. Per-claim source URLs, together with a consolidated list of the
twelve places where the documentation was found to be silent, are collected in
[`NVIDIA-REFERENCE.md`](NVIDIA-REFERENCE.md).

Claims verified against the installed image rather than a documentation page —
because for these the version-exactness mattered — are:

| claim | verified against |
|---|---|
| `IsaacReadJointState` port names and the "preferred path" note | `exts/isaacsim.ros2.nodes/.../OgnROS2PublishJointState.ogn`, `exts/isaacsim.sensors.physics.nodes/.../OgnIsaacReadJointState.rst` |
| the source-node wiring for `/joint_states` | `standalone_examples/api/isaacsim.ros2.bridge/moveit.py` |
| the LaserScan writer form and its required metadata | `standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py` |
| `tick_rate` must equal the scan rate | the comment in the same example |
| `SDGPipeline` is an OmniGraph, registered post-render | `extscache/omni.syntheticdata-*/omni/syntheticdata/scripts/SyntheticData.py` |
| `DifferentialController` ignores `dt` with no acceleration limits set | `OgnDifferentialController.ogn` |
| `isaacsim.sensors.physics.nodes` is enabled in the standalone experience | `apps/isaacsim.exp.base.kit` |

---

Everything else in this file — the standalone lifecycle, `og.Controller.edit`,
the `OnPlaybackTick`-driven action graph, the `DifferentialController` →
`IsaacArticulationController` chain, the `IsaacReadJointState` →
`ROS2PublishJointState` pair, and the RTX lidar writer attached to a render
product — is upstream's own form, adopted deliberately under this package's rule
of preferring NVIDIA's shipped examples wherever they are adequate.
