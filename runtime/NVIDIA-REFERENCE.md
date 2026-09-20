# NVIDIA Isaac Sim Official Documentation Reference
### Target: Isaac Sim 6.1.0 (with 4.x/5.x → 6.x deltas noted) · ROS 2 Humble · TurtleBot3 Burger (diff-drive, LDS-01 2D lidar)

Compiled from official NVIDIA sources: `docs.isaacsim.omniverse.nvidia.com` (versions 4.2.0, 4.5.0, 5.0.0, 5.1.0, 6.0.0, 6.0.1, `latest`), `docs.omniverse.nvidia.com`, the `isaac-sim/IsaacSim` GitHub org (including its in-repo `skills/*/SKILL.md` reference files, which are official maintained content in the isaac-sim GitHub org), and NVIDIA Developer Forums (used only where docs are silent, always marked as forum evidence, not doc evidence).

**Version-availability note:** at the time of writing, the public docs site's version selector tops out at `6.0.x` and `latest`; no separate `6.1.0` tree was found — `latest` is assumed to track 6.1-line content. Anywhere a claim is sourced from a `6.0.x`/`latest` page, treat it as the best available proxy for 6.1.0 and cross-check against your installed build. This is flagged inline as "(6.0.x/latest)".

---

## 1. Standalone workflow & app lifecycle

### Three workflows, as NVIDIA defines them
NVIDIA's Workflows page distinguishes three ways to work in Isaac Sim:

- **GUI workflow** — "Visual, intuitive, specialized tools for populating and simulating a virtual world." Recommended for world building, robot assembly, sensor attachment, visual OmniGraph programming, and ROS bridge initialization.
- **Extension workflow** — runs asynchronously inside the Kit app; "runs asynchronously to allow interactions with the stage, hot reloading to reflect changes immediately, adaptive physics steps for real-time simulation." Recommended for testing Python snippets via the Script Editor, building interactive GUIs, custom application modules, and real-time-sensitive applications.
- **Standalone Python workflow** — a plain Python script that boots the whole app itself. Its documented strength is "control over timing of physics and rendering steps" plus the ability to run headless. NVIDIA recommends it for large-scale RL training, systematic world generation/property modification, and — notably — any case "if you need to control message publishing rates in ROS," because you fully control when `update()`/physics steps occur.
- Standalone example scripts ship under `<isaac_sim_root_dir>/standalone_examples`.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/introduction/workflows.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/python_scripting/manual_standalone_python.html

### `SimulationApp` contract
`SimulationApp` (import path: `from isaacsim import SimulationApp`) manages the lifetime of the whole Kit-based application process. NVIDIA's documentation is explicit that **all Omniverse-level imports must occur after `SimulationApp` is instantiated**: "Any Omniverse level imports **must** occur after the class is instantiated. Because APIs are provided by the extension/runtime plugin system, it must be loaded before they will be available to import." This is why `isaacsim.*`, `omni.*`, and `pxr` imports must textually follow `SimulationApp({...})` construction in every standalone script — those modules are supplied by extensions that only get registered once the Kit runtime inside `SimulationApp.__init__` has loaded.

NVIDIA's own minimal pattern:
```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

### Perform any omniverse imports here after the helper loads ###

simulation_app.update()  # Render a single frame
simulation_app.close()   # Cleanup application
```
- `simulation_app.update()` renders a single frame (pumps the Kit app / UI-render loop once).
- `simulation_app.close()` performs cleanup and shutdown, ending the application lifecycle — this is the documented shutdown sequence; scripts are expected to call it once at the end of `main()`.
- For headless runs, pass `{"headless": True}`; NVIDIA also notes that any `matplotlib` window-creation calls should be commented out in headless mode.
- Internally, `SimulationApp` loads the Carbonite framework, configures startup args (experience config file, extension paths) and initializes the Omniverse Kit application before returning control to user code.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/python_scripting/manual_standalone_python.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/index.html

### `SimulationManager` vs `SimulationContext`/`World` — the 6.x recommended entry point
In Isaac Sim 6.0.x/latest, **`SimulationManager`** (`isaacsim.core.simulation_manager`) is the documented low-level entry point for configuring physics timing and compute device, largely superseding hand-rolled `SimulationContext`/`World` setup for this purpose:

```python
SimulationManager.setup_simulation(
    dt: float | None = None,
    device: str | warp.Device | None = None,
    physics_scene: str | None = None,
) -> None
```
It "Initializes physics simulation with specified timestep and compute device. Configures the physics engine for the active stage." NVIDIA's own guidance for coherent multi-clock changes: "For an end-to-end coherent rate change, call `SimulationManager.setup_simulation(dt=...)` and `RenderingManager.set_dt(...)` with the same dt so the three rate clocks stay aligned" (the three clocks being run-loop/timeline, physics, and renderer — see Area 5 below for the full three-clock model).

Other `SimulationManager` classmethods documented: `step(render: bool = True, steps: int = 1)`, `pause()/play()/stop()`, `is_playing()/is_paused()/is_stopped()`, `get_simulation_time()`, `get_num_physics_steps()`, `get_device()`, `get_physics_simulation_view()`, `register_callback(func, event)`/`deregister_callback(uid)`, and, new in 6.0, `switch_physics_engine("newton"|"physx")` / `get_active_physics_engine()` (see Newton note below).

Two methods are explicitly **deprecated as of 1.8.0** in favor of `setup_simulation`:
- `SimulationManager.set_physics_dt(dt, physics_scene=None)` → deprecated; use `PhysicsScene.set_dt()` / `PhysxScene.set_dt()` for targeted scenes, or `setup_simulation(dt=...)`.
- `SimulationManager.set_physics_sim_device(device)` → deprecated; use `setup_simulation(device=...)`.

The **older `World(physics_dt=..., rendering_dt=..., stage_units_in_meters=...)`** pattern (`omni.isaac.core.world.World`, later `isaacsim.core.api.world.World`) is the pre-6.0 idiom: it is "the core class that enables interaction with the simulator in an easy and modular way, taking care of many time-related events such as adding callbacks, stepping physics, resetting the scene, and adding tasks." `physics_dt` set the physics step size and `rendering_dt` the rendering step size (both in seconds); `World.step()` advanced both by the configured amounts and ran registered per-step callbacks. **Not documented**: I could not retrieve NVIDIA's canonical `World` class reference page content (the fetched 4.2.0 API index only linked to it without inlining the docstring); the description above is reconstructed from usage in downstream docs (Isaac Lab's `SimulationContext`, which wraps the same World/SimulationContext pattern) rather than quoted verbatim from an Isaac Sim `World` docstring page. Treat the `World(...)` constructor parameter documentation as **secondary-sourced**, not directly quoted.

Isaac Sim 6.0's own "Getting Started" standalone example **no longer uses `World` at all** — it uses `SimulationManager.step()` + `RenderingManager.render()` + `simulation_app.update()` directly with the `isaacsim.core.experimental.prims` wrappers (`Articulation`, `XformPrim`) and `isaacsim.core.experimental.objects.GroundPlane`:
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])

asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Arm")
arm_transform = XformPrim("/World/Arm")
arm_transform.set_world_poses(positions=[0.0, 1.0, 0.0])
arm = Articulation("/World/Arm")
...
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager

arm.set_dof_positions([-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04])
for _ in range(100):
    SimulationManager.step()
    RenderingManager.render()
    simulation_app.update()
    joint_positions = arm.get_dof_positions()
```
(NVIDIA's own example; script ships at `standalone_examples/tutorials/getting_started/getting_started_robot.py`.) This is a strong signal that **`SimulationManager` + `RenderingManager` + `isaacsim.core.experimental.*` is the 6.x-forward-looking entry point**, with the classic `World` object treated as the legacy/compatibility path (still present and documented, but no longer what NVIDIA's flagship tutorial teaches).

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.simulation_manager/docs/api.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/introduction/quickstart_isaacsim_robot.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_simulation/mobile_robot_controllers.html (shows `SimulationManager.setup_simulation(dt=1.0/60.0, device=DEVICE)` used for a wheeled-robot example)

### Newton physics backend (new in 6.0) — context for the dt/device question
Isaac Sim 6.0 introduced **Newton**, "a GPU-accelerated, extensible, and differentiable physics simulation engine designed for robotics and research," built on NVIDIA Warp and integrating MuJoCo-Warp, as an **alternative** backend to PhysX. Per the docs: **PhysX remains the default physics backend in Isaac Sim 6.0**; Newton is opt-in, and "only one physics engine can be active at a time." Switching is done through `SimulationManager`:
```python
from isaacsim.core.simulation_manager import SimulationManager
SimulationManager.switch_physics_engine("newton")   # or "physx"
SimulationManager.get_active_physics_engine()
```
or at launch via `--/exts/isaacsim.core.simulation_manager/default_engine=newton` and `--/exts/isaacsim.physics.newton/auto_switch_on_startup=false`. NVIDIA states plainly: "Newton integration in Isaac Sim is experimental. The API and features may change in future releases," and that "Many Isaac Sim features and workflows that do not use the experimental core API are not yet supported with the Newton backend." Newton has documented asset-compatibility constraints (parent→child joint ordering required, no closed kinematic chains, all bodies need non-zero mass/inertia, no negative-scale collision transforms). The URDF/MJCF importers were "rebuilt on the USD Exchange SDK" specifically so imported assets are structured to work with either backend (see Area 3).

This matters for the thesis comparison: **for a TurtleBot3 on Isaac Sim 6.1 with the standard PhysX-based workflow, Newton is not required and PhysX is still the default** — but any 6.x-era doc reference to `NewtonArticulationRootAPI` on imported robots (see Area 3) is a side effect of this dual-backend architecture, not an indication the robot is running on Newton.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/physics/newton_physics.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/physics/new_physics_engine.html

### Physics timestep, rendering dt, and PhysX sub-stepping
Core relationship, per NVIDIA's Physics Simulation Fundamentals page: "simulation time differs from real-time." Physics can run faster than rendering — "the simulation can be set to run at 120 time steps per second, while rendering is set to 60 frames per second, resulting in two physics steps per rendered frame" — and the ratio need not be a whole number, in which case "each rendered frame may contain an uneven number of simulation timesteps."

- Physics step rate is a **Physics Scene** property, `physics:timeStepsPerSecond` (default 60), settable via the `PhysxSchema.PhysxSceneAPI` `TimeStepsPerSecond` attribute or `SimulationManager.setup_simulation(dt=...)`/`PhysxScene.set_dt()`.
- The stage's `timeCodesPerSecond` (Root Layer) controls the timeline's playback rate; under fixed time-stepping (the default), the sim uses `1 / timeCodesPerSecond` as its per-tick dt.
- Lowering the app's render rate via `RenderingManager.set_dt()` does **not** automatically change the number of physics substeps per rendered frame — physics rate is configured independently on the Physics Scene.
- Three event streams exist on the timeline: simulation (physics) events, pre-render events (where OmniGraph nodes typically update), and post-render events. To guarantee exactly one OmniGraph tick per physics step, NVIDIA documents using an Action Graph with the `PipelineStageOnDemand` setting and an "On Physics Step" trigger node instead of the default per-render tick.
- Contact fidelity vs. cost: "Contact Offset dictates how far from the collision geometry the simulation engine starts generating contact constraints" — larger offsets cost more compute but improve accuracy, smaller offsets risk "jittering or missed contacts or even tunneling." For fast-moving bodies, Continuous Collision Detection (CCD) is documented as the mechanism to prevent tunneling by sweeping the object's pose between steps.

**Not documented (official Isaac Sim docs):** a specific numeric sub-step-rate recommendation for *wheeled robots* as a category. The official Isaac Sim Physics Simulation Fundamentals pages (4.2.0/5.1.0/6.0.1, fetched) do not give a wheeled-robot-specific Hz recommendation.

The best citable numeric guidance found comes from the **`isaac-sim/IsaacSim` GitHub repo's own `skills/physics-simulation/SKILL.md`** (official repo, Apache-2.0, in-tree engineering reference rather than the public docs site) which gives general-purpose timestep guidance by system type: standard rigid bodies 60–120 Hz; stacking/contact-rich scenes 240 Hz; high-velocity impacts 120 Hz with 2–4 substeps + CCD; small-part vibration ≥480 Hz (≥4× the highest vibration frequency, stated as a general rule: "physics timestep must exceed 4× the highest frequency in your system"); spinning/gyroscopic bodies 480 Hz. Wheeled ground robots are not called out as their own bucket; a TurtleBot3-class differential drive would fall under "standard rigid bodies" (60–120 Hz) by this general rule, but that mapping is an inference from the repo's general categories, not a direct NVIDIA statement about wheeled robots.

Solver-iteration guidance from the same source: simple tumbling bodies 16 position / 4 velocity iterations; stacking 32/8; complex joints/articulations 64/16; stiff chains 64/32 — again general, not wheel-specific.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/physics/simulation_fundamentals.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html
- https://raw.githubusercontent.com/isaac-sim/IsaacSim/main/skills/physics-simulation/SKILL.md (isaac-sim GitHub org, not the docs site — flagged as such)

---

## 2. Stage / world construction

### Building a stage in a standalone script
NVIDIA's own pattern (from the 6.0 quickstart, using the 6.x experimental API):
```python
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
asset_path = assets_root_path + "/Isaac/Robots/.../robot.usd"
stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Arm")
```
This shows the documented idiom: `create_new_stage()` to get a clean stage, `GroundPlane(prim_path, positions=...)` (from `isaacsim.core.experimental.objects`) to add a default ground plane, and `add_reference_to_stage(usd_path=, path=)` to bring in a robot/asset USD by reference (not payload) at a given prim path.

The classic (pre-6.0-experimental, still supported) equivalent is `World.scene.add_default_ground_plane()` on a `World` instance, which is the idiom used throughout the 4.x/5.x Core API tutorials (`isaacsim.core.api`). **Not documented in what I could retrieve**: an explicit NVIDIA statement contrasting `GroundPlane("/World/GroundPlane")` (experimental object) vs. `World.scene.add_default_ground_plane()` (classic World-scene helper) as "old vs new" — both appear in current docs for their respective API generations; I found no single migration note that says one supersedes the other outside general Core→Core-Experimental guidance already covered in Area 1.

Lighting: **Not documented** — no official code snippet for standalone-script lighting setup (e.g., adding a dome/distant light programmatically) was found in the pages fetched. Environment/GUI tutorials show lighting via the GUI (Create menu) but I could not confirm a canonical Python snippet.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/introduction/quickstart_isaacsim_robot.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/python_scripting/environment_setup.html (Scene Setup Snippets — confirmed present: rigid object creation, ground-plane-adjacent rigid prim examples, physics scene creation, material creation, world-pose setting; did **not** contain `create_new_stage`/`add_reference_to_stage`/lighting snippets when fetched)

### Physics materials: combine modes
`PhysicsMaterial` (`isaacsim.core.api.materials.PhysicsMaterial`, and its Core-Experimental equivalent) exposes `static_friction`, `dynamic_friction`, and `restitution`. NVIDIA's own constructor example:
```python
from isaacsim.core.api.materials import PhysicsMaterial

material = PhysicsMaterial(
    prim_path="/World/physics_material/aluminum",
    dynamic_friction=0.4,
    static_friction=1.1,
    restitution=0.1,
)
```
Documented defaults for the `PhysicsMaterial` Python wrapper: `static_friction=0.5`, `dynamic_friction=0.5`, `restitution=0.8` when not otherwise specified. The material is then bound to a prim via `apply_physics_material()`.

**Combine modes.** Isaac Sim's Physics Simulation Fundamentals page states the resolution rule explicitly: contact behavior between two colliders requires combining each side's material property, and NVIDIA documents a **priority ordering among modes**, quoted directly: `"average < min < multiply < max"`, with the worked example: "If Collider A has friction combine mode average while Collider B has min, their interaction resolves as the minimum friction between the two" (i.e., when the two touching materials specify different combine modes, the higher-priority mode in that ordering wins, not a per-mode arithmetic average of the modes themselves).

The four valid values for both `frictionCombineMode` and `restitutionCombineMode` (PhysX/USD Physics schema, `PhysxSchema.PhysxMaterialAPI`) are **average, min, multiply, max**. Per the PhysX API reference (`PxCombineMode`) and Omniverse's PhysX schema documentation, **the default combine mode for both friction and restitution is `average`.** NVIDIA's own Isaac Sim docs did not, in the pages retrieved, restate this default number explicitly in prose (it is documented at the PhysX SDK / USD schema level, which Isaac Sim consumes as-is) — so this default is sourced to the PhysX/Omniverse schema docs rather than an Isaac-Sim-specific page.

**Recommendation for ground contact:** the general Isaac Sim docs do not give a specific "use `max` (or `min`) for ground contact" recommendation. The `skills/physics-simulation/SKILL.md` reference (isaac-sim GitHub org) gives a table of representative static/dynamic friction and restitution values by material pairing (e.g. rubber–rubber: static 0.8, dynamic 0.7, restitution 0.5; concrete–concrete: 0.6/0.5/0.05) intended as starting points for contact-material authoring, and separately notes that for stiff, low-bounce contact chains one should set `restitutionCombineMode=max` on `PhysxMaterialAPI` "so the highest restitution value wins at each contact" — but this is a general physics-authoring tip, not a ground-plane-specific recommendation, and not from the public docs site.

**Not documented:** an official, Isaac-Sim-docs-site statement of "here is what to set frictionCombineMode/restitutionCombineMode to for a robot-on-ground-plane scenario." This is a real gap — the thesis should flag that NVIDIA leaves ground-contact combine-mode tuning to the user/PhysX defaults.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html
- https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/_api_build/struct_px_combine_mode.html (PhysX SDK reference, official NVIDIA PhysX docs)
- https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/106.1/physx_material_a_p_i_8h_source.html (Omniverse PhysX USD schema header docs)
- https://raw.githubusercontent.com/isaac-sim/IsaacSim/main/skills/physics-simulation/SKILL.md (isaac-sim GitHub org)

### Collision approximation types (documented options)
From Isaac Sim's Physics Simulation Fundamentals and the URDF importer page, the documented collider approximation choices are: **Convex Hull** (the default approximation for arbitrary mesh geometry; "efficient"), **Convex Decomposition** (splits a mesh into multiple convex pieces; "moderate cost; fewer hulls = better performance"; recommended for shapes like a torus with a hole), **Bounding Cube** and **Bounding Sphere** (cheapest, crudest), **Sphere Approximation** (an alternative to convex decomposition that fits a mesh with a group of spheres rather than convex hulls), and **SDF Mesh** (signed-distance-field collision, supported directly on triangle-mesh geometry for rigid bodies — this is the one case where an actual triangle mesh can be used as a rigid-body collider without falling back to convex hull). NVIDIA states plainly that outside the SDF-mesh case, "triangle mesh and mesh simplification are not supported by rigid bodies and fall back to convex hull." Convex Hull approximation can be "incompatible with GPU simulation if the input mesh has a high aspect ratio, triggering a CPU fallback that can significantly impact performance."

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html

### Stock environments and asset resolution chain
`get_assets_root_path()` (module `isaacsim.storage.native`, formerly under `omni.isaac.core` utils) "tries to find the root path to the Isaac Sim assets on a Nucleus server," with an optional `skip_check` to bypass existence verification; it raises `RuntimeError` if the root setting is unset or the path can't be found. An async twin, `get_assets_root_path_async()`, is documented for extension/async contexts. Resolution is controlled by the persistent setting `persistent.isaac.asset_root.default`; NVIDIA's documented current default value for that setting is an **S3 bucket URL** — `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0` in the version examined — i.e. Isaac Sim ships configured to resolve assets from NVIDIA's public S3-hosted content by default, with a Nucleus server as the alternative/overridable target if one is configured (e.g. `omniverse://localhost/NVIDIA/Assets/Isaac/...` in a local/enterprise Nucleus deployment). Stock environment paths follow the convention `<assets_root_path>/Isaac/Environments/...` (e.g. `.../Isaac/Environments/Simple_Warehouse/warehouse.usd`), and stock robots follow `<assets_root_path>/Isaac/Robots/<Vendor>/<Model>/....usd` (e.g. the Franka path shown in the quickstart example above). In-app verification is via Isaac Sim Assets Browser → gear icon → "Check Default Assets Root Path."

**Not documented in the pages retrieved:** an explicit, single NVIDIA page walking through the full fallback chain logic (i.e., "if Nucleus X fails, then try S3, then fail") as a flowchart/algorithm. What's documented is the *setting* that determines the root (`persistent.isaac.asset_root.default`) and the *function* that resolves/validates it, not a multi-step fallback narrative.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.storage.native/docs/index.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.storage.native/docs/api.html
- https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_faq.html

### Waiting for asset load
NVIDIA documents `is_stage_loading()` as "a convenience function to check if any files are being loaded, returning `True` if loading and `False` otherwise," intended to be polled in a loop alongside `simulation_app.update()` until it returns `False`, before proceeding to play/step. Separately, `SimulationApp`'s config dict accepts a `sync_loads` option which, when enabled, "will pause rendering until all assets are loaded" as an alternative/complementary mechanism for standalone scripts. **Caveat:** I was only able to confirm the `is_stage_loading()` description via a downstream project (Isaac Lab's `isaaclab.sim.utils.stage`, which wraps the Isaac Sim utility) and via forum/search-snippet evidence for `sync_loads`; I could not load the exact `isaacsim.core.utils.stage` (or `isaacsim.core.experimental.utils.stage`) API reference page content directly in this session to quote it verbatim from the primary docs site. Treat the function's existence and one-line behavior as reasonably confirmed, but its exact primary-source docstring is **not independently quoted here**.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.utils/docs/index.html (index only; did not yield inline docstring in this session)
- https://forums.developer.nvidia.com/t/using-isaac-sim-assets-causes-a-2-minute-delay-in-standalone-scripts/341137 (forum, corroborating `sync_loads` behavior — not primary docs)

---

## 3. Robot import & articulation

### URDF Importer output: single file vs. "Asset Structure" directory
This is a genuine, documented **4.x/5.x → 6.x change**. In 6.0.x, NVIDIA's URDF Importer Extension page and dedicated Asset Structure page describe the importer's output as following the **"Isaac Sim Asset Structure" convention** — a directory of multiple, cross-referenced USD files, not one monolithic `.usd`. The stated rationale: separate USD components into multiple files for review, isolate attributes per physics engine to avoid clashes, and use layers/payloads/variants for different robot configurations. The documented layout (paraphrased from the Asset Structure page) is approximately:

```
asset_root/
├── asset.usd            # final composed interface (what you reference)
├── base.usda             # simulation-ready kinematic/transform structure
├── geometries.usd        # mesh topology/vertex data only
├── materials.usda        # shader/material bindings
├── instances.usda        # visual+collision+collider-approximation assembly
├── robot.usda             # robot schema/metadata/joint relationships
├── physics.usda           # cross-engine (USD/Newton) rigid bodies, joints, articulation
├── mujoco.usda            # MuJoCo-specific physics tuning
├── physx.usda             # PhysX-specific physics tuning
└── payloads/
    ├── base.usda
    └── Physics/
        ├── physics.usda
        ├── mujoco.usda
        └── physx.usda
```
References are used to assemble geometry (`prepend references = @geometries.usd@</Geometries/...>`), while optional features — end-effectors (`gripper.usda`), control graphs (`asset_control.usda`) — are **dynamically added as payloads**, and physics-engine choice is exposed as a **USD variant set** (`variantSet "Physics" = { "physx" (prepend payload = @payloads/Physics/physx.usda@) {} }`), letting the same asset carry PhysX-only, MuJoCo-only, or Newton-only physics data without duplicating geometry. NVIDIA frames this explicitly as aligned with "USD Asset Structure 3.0" guidance. The URDF Importer extension page itself (still current wording as of 6.0.0) says: "The Imported model follows the Isaac Sim Asset Structure convention, and the meshes are already instantiable to optimize performance" — confirming the multi-file/instanceable structure is the current (6.0.x) importer output, a change from the older single-`.usd`-file mental model common in 4.x/early docs and tutorials.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup/asset_structure.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html

### `ArticulationRootAPI` placement
The 6.0 URDF importer "applies the standard `UsdPhysics.ArticulationRootAPI` and the `NewtonArticulationRootAPI` on the root link of the URDF file" — i.e., **both** the USD-Physics-standard schema and a new Newton-specific articulation-root marker are stamped on the same prim, so the same asset works whichever backend is active. Per the 6.0 migration notes surfaced during research: "Newton `ArticulationRootAPI` is disabled in the PhysX layer, to avoid conflicts with the `PhysxArticulationRootAPI`" and `enable_self_collision` now marks roots with `UsdPhysics.ArticulationRootAPI` plus `NewtonArticulationRootAPI` (`newton:selfCollisionEnabled`) instead of writing `physxArticulation:enabledSelfCollisions`/`PhysxArticulationAPI` directly — a concrete attribute-level rename to watch for when diffing 5.x vs 6.x-imported robots.

Separate, general (not 6.0-specific) NVIDIA guidance on *where* to put the articulation root on a mobile robot (from the Rig a Mobile Robot tutorial): **"It is recommended that you place the articulation root on the base or the chassis of a mobile robot"**, because doing so "automatically assigns the articulation root to a rigid body in the robot, which minimizes the depth of the articulation tree" — i.e. shallow articulation trees are preferred for solver performance, and the chassis/base link is the recommended anchor point for a wheeled robot's articulation root. Separately, generic USD-Physics documentation (surfaced via Isaac Lab's schema docs, which quote the same convention) states: for a floating-base articulation, `ArticulationRootAPI` "should be on the root body"; for a fixed-base articulation it "can be on a direct or indirect parent of the root joint which is fixed to the world."

**On stability of the importer's placement choice:** NVIDIA documents *where the importer puts it* (root link of the URDF) and *where NVIDIA recommends you put it* (chassis/base for mobile robots) as consistent guidance, but I found no explicit statement of the form "this location is a stable/guaranteed API contract across versions" — the safest reading is that the *behavior* (root link) is documented and current, but no forward-compatibility guarantee is stated in the docs.

Sources:
- (URDF importer 6.0 ArticulationRootAPI/NewtonArticulationRootAPI behavior) — surfaced from https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html and Isaac Sim 6.0 release-note/migration material at https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_mobile_robot.html (chassis/base placement recommendation, direct quote)
- https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab/sim/schemas/schemas.html (floating vs fixed-base ArticulationRootAPI placement convention, Isaac Lab docs quoting the underlying USD Physics convention)

### Drive setup for differential-drive wheels: velocity vs. position
The URDF importer exposes joint drives through `UsdPhysics.DriveAPI`, with two documented control modes:
- **Position control:** set `stiffness` (drive strength / "spring" gain) and `targetPosition`; keep `damping` at zero or minimal.
- **Velocity control:** set `damping` (drive strength for velocity tracking) and `targetVelocity`; set `stiffness` to **zero**.

This zero-stiffness / nonzero-damping pattern is exactly PhysX's standard "velocity drive" idiom (a pure damper against the target velocity, no positional spring term), and it's the documented mode for **wheel joints**, which are continuously-rotating and have no meaningful target angle. The URDF importer docs' own worked wheel example uses `left_wheel_drive.GetDampingAttr().Set(15000)` with stiffness left at 0.

The Rig a Mobile Robot tutorial gives a second, slightly different concrete numeric example for a driven back wheel: **damping 10,000, stiffness 100** with a nonzero `targetVelocity` used to test rolling — a low-but-nonzero stiffness alongside high damping (rather than strictly stiffness=0), suggesting NVIDIA's own tutorials are not perfectly uniform on whether stiffness must be exactly zero for velocity-driven wheels, only that damping should dominate. The same tutorial contrasts this against a **steering/swivel joint**, which is position-controlled with the opposite ratio — damping 100, stiffness 100,000 — confirming the general pattern "propulsion joints: damping-dominated; position/steering joints: stiffness-dominated."

**Not documented:** a single official numeric table of "recommended stiffness/damping for wheel velocity drives" applicable across robot scales — the two examples found (15000 damping/0 stiffness in the importer docs; 10000/100 in the rigging tutorial) are both robot-specific worked examples, not general recommendations, and neither is TurtleBot3-scale-calibrated.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_mobile_robot.html

### Wheel collider geometry: what does NVIDIA recommend?
This directly answers the "sphere workaround" question in the prompt, and the documented answer is **no** — NVIDIA's official guidance for wheels is the **opposite** of a sphere workaround. The Rig a Mobile Robot tutorial states plainly: **"any collision approximation that is not smooth and captures the exact shape and curvature of the wheel causes bumpy motion when attempting to drive the wheel."** Its concrete recommendation is a **cylinder** collider matching the wheel's actual radius/width (worked example: front roller wheels scaled X=0.16, Y=0.16, Z=0.08 with a 90° Y rotation; back wheels scaled up to X=0.3, Y=0.3, Z=0.1), explicitly **against** using Convex Decomposition for wheels, because "while convex decomposition provides tighter mesh approximation for general collision, it's unsuitable for wheels because the resulting geometry produces uneven rolling behavior."

Separately, general PhysX/Isaac Sim collision-approximation docs note that **cylinder (and cone) collision geometries get special-cased smooth-contact handling against triangle meshes specifically to improve wheeled-vehicle rolling behavior**: "Cylinder and cone collision geometries have special support for smooth collisions with triangle meshes for better wheeled simulation behavior. However, this comes at a cost of performance and may not always be desired," and this can be disabled via the stage settings `--/physics/collisionApproximateCylinders` and `--/physics/collisionApproximateCones` (both default to providing the smooth approximation; setting them `true`... — note: exact boolean polarity of these two settings was not independently re-verified from a primary page in this session, only from a search-derived summary, so treat the *existence* of these two settings and their purpose as confirmed, but double-check the literal on/off polarity against `isaacsim_sensors`/physics settings docs before citing the boolean value in the thesis).

Regarding **spheres specifically**: the only documented use of spheres in Isaac Sim's collision-approximation vocabulary is the generic, shape-agnostic **"Bounding Sphere"** (crudest bounding-volume option) and **"Sphere Approximation"** (an alternative to convex decomposition that fits *arbitrary* meshes with a cluster of spheres) — neither is presented anywhere in the fetched docs as a wheel-specific recommendation or as a documented workaround for cylindrical-wheel rolling problems.

**Conclusion for the thesis comparison:** *Not documented*: NVIDIA does not document spheres as a wheel-rolling workaround. *Documented, and the opposite*: NVIDIA's official position is that wheels should use **shape-matched cylinder colliders** (or, implicitly, convex hulls tight enough to match the wheel's curvature) precisely *because* generic/decomposed approximations roll badly — the officially sanctioned fix for "convex/cylindrical wheels roll poorly" is a better cylinder fit and PhysX's built-in cylinder/triangle-mesh smoothing, not a switch to spherical colliders.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_mobile_robot.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/simulation_fundamentals.html (general collision-approximation catalogue, Sphere Approximation / Bounding Sphere definitions)

---

## 4. OmniGraph + ROS 2 bridge

### The canonical differential-drive ActionGraph
NVIDIA's "Driving TurtleBot using ROS 2 Messages" tutorial (present across 4.2.0 → latest, content stable across versions with only the extension-namespace change noted below) documents this exact graph for driving a differential robot from `/cmd_vel`:

| Node (exact type) | Role |
|---|---|
| `omni.graph.action.OnPlaybackTick` | Emits an execution pulse "for every frame, but only while the simulation is playing" — the graph's clock. |
| `isaacsim.ros2.bridge.ROS2Context` | Establishes the ROS 2 DDS context/Domain ID (default 0; toggle `useDomainIDEnvVar`). |
| `isaacsim.ros2.bridge.ROS2SubscribeTwist` | Subscribes `geometry_msgs/Twist` on the configured topic (`/cmd_vel`); outputs `linearVelocity` (`vectord[3]`, m/s) and `angularVelocity` (`vectord[3]`, rad/s), plus `execOut` fired on message receipt. |
| `Break 3-Vector` (×2, generic OmniGraph node, not ROS-specific) | Extracts the scalar x-component of linear velocity and z-component of angular velocity, "because the input of the differential controller node only takes a forward velocity and rotation velocity in z-axis." |
| `isaacsim.robot.wheeled_robots.DifferentialController` | Converts (forward speed, rotation speed) → (left wheel speed, right wheel speed) using the unicycle model. Documented inputs: `wheelRadius`, `wheelDistance`, `maxLinearSpeed`, `maxAngularSpeed`, `maxWheelSpeed`, `maxAcceleration`, `maxDeceleration`. |
| `Constant Token` (×2) + `Make Array` | Build the ordered joint-name token array (e.g. `wheel_left_joint`, `wheel_right_joint`) fed to the articulation controller. |
| `isaacsim.core.nodes.IsaacArticulationController` | Applies the computed per-joint commands to the robot's drives. Documented inputs: `execIn`, `targetPrim` (or `robotPath`), `jointNames`, `jointIndices`, `positionCommand`, `velocityCommand`, `effortCommand`. NVIDIA notes angular units are radians even though USD angle attributes are in degrees — the node handles the conversion. |

Wiring (as documented): `OnPlaybackTick.outputs:tick → DifferentialController.inputs:execIn` and `→ IsaacArticulationController.inputs:execIn`; `ROS2SubscribeTwist` linear/angular outputs → `Break 3-Vector` nodes → `DifferentialController` `inputs:linearVelocity`/`inputs:angularVelocity`; `DifferentialController.outputs:velocityCommand` (wheel speeds) → `IsaacArticulationController.inputs:velocityCommand`; the joint-name `Make Array` output → `IsaacArticulationController.inputs:jointNames`. NVIDIA explicitly notes the *persistence* behavior of this wiring: **"The Articulation Controller node is ticked by On Playback Tick. So that if no new Twist message arrives, it will continue to execute whatever command it had received before"** — i.e. there is no built-in command timeout/watchdog in the stock graph; a `/cmd_vel` publisher that stops publishing does **not** stop the robot.

A worked parameter set NVIDIA uses for its own TurtleBot3-class example: `maxLinearSpeed=0.22`, `maxAngularSpeed=1.0`, `wheelRadius=0.025`, `wheelDistance=0.16` (values specific to the TurtleBot3 Burger geometry used in that tutorial — cross-check against your own robot's actual wheel radius/base).

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_drive_turtlebot.html
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_simulation/mobile_robot_controllers.html (DifferentialController math/inputs, plus 6.0-era import path `isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController` for the Python-class equivalent, distinct from the OmniGraph node type)

### `DifferentialController` Python class (non-graph use)
```python
from isaacsim.robot.wheeled_robots.controllers import DifferentialController
controller = DifferentialController(name="simple_control", wheel_radius=0.035, wheel_base=0.1)
```
Model: unicycle-style differential drive; converts a (linear V, angular ω) command into wheel angular velocities via ω_R = (1/2r)(2V + ωb), ω_L = (1/2r)(2V − ωb), where r = wheel radius, b = wheel-to-wheel distance. NVIDIA's 6.0 docs additionally show a renamed import path for the newer wrapper: `isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController` with `.forward([linear_speed, angular_speed])`. Both the classic and "experimental" namespaces coexist in 6.0.x docs; **not documented**: an explicit deprecation timeline for the non-experimental path.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.robot.wheeled_robots/docs/index.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_simulation/mobile_robot_controllers.html

### Building the graph via `og.Controller.edit()` — NVIDIA's own scripting API
Yes — `og.Controller.edit()` with `og.Controller.Keys.CREATE_NODES` / `CONNECT` / `SET_VALUES` (and `CREATE_ATTRIBUTES`) is the documented, official way to build OmniGraphs from Python, both in general OmniGraph docs and in the ROS 2-standalone-workflow tutorial. NVIDIA's own minimal example (OmniGraph via Python Scripting Tutorial):
```python
import omni.graph.core as og

keys = og.Controller.Keys
graph_handle, list_of_nodes, _, _ = og.Controller.edit(
    {"graph_path": "/action_graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnTick"),
            ("print", "omni.graph.ui_nodes.PrintText"),
        ],
        keys.SET_VALUES: [
            ("print.inputs:text", "Hello World"),
            ("print.inputs:logLevel", "Warning"),
        ],
        keys.CONNECT: [("tick.outputs:tick", "print.inputs:execIn")],
    },
)
```
And NVIDIA's ROS2-specific standalone-workflow example (building a clock-publishing graph):
```python
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("Context.inputs:domain_id", 1),
            ("Context.inputs:useDomainIDEnvVar", False),
        ],
    },
)
```
`og.Controller.edit()` takes a graph-descriptor dict (`graph_path`, `evaluator_name`) and an operations dict keyed by `og.Controller.Keys.*`; it returns `(graph_handle, list_of_nodes, ..., ...)`.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/omnigraph/omnigraph_scripting.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_python.html

### Graph evaluators: execution vs. push vs. dirty_push
NVIDIA documents three evaluator/graph types (OmniGraph settings expose them as `execution`, `push`, `dirty_push`, corresponding to the user-facing names Action Graph, Push Graph, and Lazy Graph respectively):

- **Action Graph ("Execution Graph", evaluator `execution`)** — quoted directly: **"An Action graph, or sometimes called an 'Execution Graph', is executed whenever an execution node gets triggered."** In Isaac Sim, the trigger is almost always `OnPlaybackTick`, "set to be triggered every simulation frame tick, so that the graph doesn't do anything when simulation isn't running, and ticks at every rendering frame once simulation starts." Without an execution/trigger node, "the graph will not run upon pressing 'play.'"
- **Push Graph ("Generic Graph", evaluator `push`)** — quoted directly: **"A Push graph...will execute automatically on every rendering frame, without needing an 'execution' node."** If node connections imply a procedural order it computes them in that order; otherwise "it'll execute all of the nodes in no guaranteed order."
- **Lazy/Dirty-Push Graph (evaluator `dirty_push`)** — a pull-style graph: modified nodes propagate "dirty" bits downstream and dirtied nodes are scheduled for re-evaluation (unlike a classic pull graph such as Maya/Houdini's DG, *all* dirtied nodes get scheduled rather than only the subset whose output is explicitly requested).

NVIDIA's explicit recommendation: **"For majority of cases in Isaac Sim, you will be using the Action Graph"** — and every ROS 2 bridge example found (drive-TurtleBot, clock-publishing, RTX-lidar-to-ROS) uses `"evaluator_name": "execution"` with an `OnPlaybackTick` (or `OnImpulseEvent`) driver. The reason, stated implicitly throughout: ROS publishing/control graphs need frame-synchronized, play-state-gated execution (nothing should publish while the sim is stopped/paused, and exactly one Twist→wheel-command cycle should happen per simulation tick) — properties the Action Graph's trigger-driven model guarantees and the always-on Push Graph does not.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/4.2.0/gui_tutorials/tutorial_gui_omnigraph.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/omnigraph/omnigraph_tutorial.html (confirms `OnPlaybackTick` semantics quoted above)

### `IsaacComputeOdometry` semantics
Documented inputs/outputs (`isaacsim.core.nodes.IsaacComputeOdometry`, C++ node): input **`chassisPrim`** (`target`) — "Usd prim reference to the articulation root or rigid body prim"; outputs `position` (`vectord[3]`, m), `orientation` (`quatd[4]`), `linearVelocity`/`angularVelocity` (`vectord[3]`, m/s / rad/s), and `linearAcceleration`/`angularAcceleration`. NVIDIA's own one-line summary: **"Holds values related to odometry, this node is not a replacement for the IMU sensor."**

On the ground-truth-vs-encoder-integration question: the accompanying ROS2 Transform Trees and Odometry tutorial describes the node's behavior in prose as reading the chassis prim's actual simulated motion and computing **"the position of the robot relative to its start location"** — i.e., this is a **ground-truth kinematic readout of the physics-simulated chassis pose** (via the prim referenced by `chassisPrim`, typically the robot's base_link/articulation root), **not** an integration of individual wheel-joint encoder velocities. Its output is documented as relative to the robot's own start pose (accumulated from where the chassis prim was when the node/graph started), not the world/stage origin as an independent frame — though because the chassis prim typically starts at a known placed pose in the stage, "relative to start location" and "relative to world origin" often coincide numerically unless the robot is spawned away from `(0,0,0)`. **Not documented**: an explicit sentence stating "this is ground truth, not wheel-encoder-integrated" in exactly those terms — that characterization is inferred from (a) the `chassisPrim` input being the *only* data source described, with no wheel-joint-state input on the node, and (b) the node living in `isaacsim.core.nodes` (physics/core-simulation utilities) rather than being wired downstream of any joint-state-reading node in the documented graphs. The node's OGN reference page itself, as retrieved, does **not** contain an explicit "ground truth vs. encoder" statement — this is the clearest gap in Area 4 and should be flagged as such in the thesis: *NVIDIA does not explicitly document whether `IsaacComputeOdometry` is meant to model "perfect" localization (bypassing wheel-slip error that a real encoder-based odometry would accumulate) — it is inferable from the node's inputs (only a chassis prim, no joint states) but not stated outright.*

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.nodes/docs/ogn/OgnIsaacComputeOdometry.html
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_tf.html

### TF ownership: `ROS2PublishTransformTree` vs. `robot_state_publisher`, and `ROS2PublishRawTransformTree`
**`ROS2PublishTransformTree`** (moved to `isaacsim.ros2.nodes` in 6.0, see migration note below) "publishes the pose of prims as a ROS2 Transform Tree." Documented inputs include `targetPrims` ("Prims to publish poses for, if prim is an articulation, the entire articulation tree will be published"), optional `parentPrim` (defaults to World if blank), `topicName` (default `"tf"`), `nodeNamespace`, `context`, QoS/queue-size settings, and a `staticPublisher` flag ("Overrides QoS settings to publish static transform trees"). In 6.0, this node's `targetPrims`-driven internal USD resolution was **deprecated in favor of an explicit upstream node**, `isaacsim.core.nodes.IsaacComputeTransformTree` (or the joint-state equivalent, `IsaacReadJointState`, for `ROS2PublishJointState`) — the publisher now takes pre-computed `childFrames`/`parentFrames`/`orientations`/`translations` array inputs rather than resolving prims itself. NVIDIA frames this 6.0 change as a **"separation of concerns"**: "data source nodes (compute/read operations) are now decoupled from data publishing nodes," allowing more flexible graph composition.

**`ROS2PublishRawTransformTree`** publishes a **single, user-defined, manually-specified transform** between two named frames — not a resolved robot kinematic tree. Documented inputs: `childFrameId` (default `"base_link"`), `parentFrameId` (default `"odom"`), `translation` (meters), `rotation` (quaternion, IJKR), plus the standard `context`/`nodeNamespace`/`topicName`/`qosProfile`/`queueSize`/`timeStamp`/`staticPublisher` inputs. Its documented purpose is exactly the canonical `odom → base_link` (or `world → odom`) link that a robot's odometry source (not URDF kinematics) is responsible for — i.e., the piece of the TF tree that `IsaacComputeOdometry`'s output logically feeds, as distinct from the URDF-driven joint-to-joint tree that `ROS2PublishTransformTree` handles for the articulated links.

**On the "fighting over frames" question with `robot_state_publisher`:** the official ROS2 Transform Trees and Odometry tutorial, as retrieved, contains **no explicit warning** about `ROS2PublishTransformTree` and a separately-run `robot_state_publisher` conflicting over the same frames. **This is a genuine documentation silence** — NVIDIA's Isaac Sim ROS 2 tutorials build the *entire* TF tree (both the static/kinematic robot-link transforms and the dynamic odom→base_link transform) from inside Isaac Sim's own OmniGraph nodes, and simply do not discuss the alternative architecture (Isaac Sim publishing only `/joint_states`, with an external `robot_state_publisher` computing the link TF from URDF + those joint states) as a documented option or anti-pattern. No official text says "don't run both" or "here's how to split responsibility" — the thesis should record this explicitly as **Not documented: NVIDIA gives no guidance on TF-ownership conflicts between its own OmniGraph TF publishers and a ROS-side `robot_state_publisher`.**

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.ros2.nodes/docs/ogn/OgnROS2PublishTransformTree.html
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishRawTransformTree.html
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_tf.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html (6.0 TF/joint-state node decoupling change)

### `/clock` and `use_sim_time`
`isaacsim.ros2.bridge.ROS2PublishClock` "publishes the given time as a ROS2 Clock message." The documented pattern feeds it from `isaacsim.core.nodes.IsaacReadSimulationTime` (see the `og.Controller.edit()` example above: `ReadSimTime.outputs:simulationTime → PublishClock.inputs:timeStamp`), with `topicName` set to `"/clock"`. `IsaacReadSimulationTime` "retrieves the current simulation time," and by default this "increases monotonically...regardless of whether simulation is stopped and re-played, the time will continue incrementing, mainly to prevent issues that can arise with the time jumping back when simulation resets"; setting its `resetOnStop` input to `True` makes the clock restart from 0 on every simulation reset instead.

On the ROS side, NVIDIA notes the standard ROS 2 mechanism: "Many ROS2 nodes such as RViz2 use the parameter `use_sim_time` which, if set to True, will indicate to the node to begin subscribing to the `/clock` topic and synchronizing to the published simulation time" — set via a launch file or the command line on each ROS node that should track sim time; Isaac Sim's role is only to publish `/clock`, not to set `use_sim_time` on external nodes.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_clock.html
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishClock.html

### Namespacing: `nodeNamespace` and automatic namespace generation
Two documented mechanisms:
1. **Manual**: every ROS 2 OmniGraph node (publishers, subscribers, service nodes) exposes a **`nodeNamespace`** input field; setting it prepends that string to the node's topic/frame names.
2. **Automatic (NVIDIA's recommended approach for multi-robot scenes)**: an **`isaac:namespace`** USD attribute (added via Property panel → Add → Isaac → Namespace) can be set on prims in the stage hierarchy; the bridge "appends each `isaac:namespace` attribute value that has been set from the top of the prim hierarchy down to each ROS publisher" to build the effective namespace automatically — e.g. a robot prim `/mock_robot` with `isaac:namespace = "mock_robot"` and a nested camera prim with `isaac:namespace = "camera_link"` yields the topic `/mock_robot/camera_link/rgb`.

Node-type-specific nuances documented: **TF nodes** use only the top-level (robot) prim's namespace, so all of a robot's transforms publish under one namespace like `/robot_name/tf` rather than being split per-link; **camera/lidar helper nodes** derive their namespace from the *sensor prim's* location (i.e., the render-product source), not the helper node's own stage location; **general OmniGraph nodes** use the node's own path in the stage hierarchy. This automatic mechanism is what lets a duplicated robot (e.g. `/mock_robot_01`) automatically get non-colliding topics (`/mock_robot_01/lidar_link/laser_scan`) without manually editing every `nodeNamespace` field.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_auto_namespace.html
- (per-node `nodeNamespace` field, corroborated across the OGN reference pages cited elsewhere in this section, e.g. `OgnROS2PublishRawTransformTree`, `OgnROS2PublishOdometry`, `OgnROS2PublishTransformTree`)

### 6.0 ROS2 OmniGraph extension/node renames (5.x → 6.x)
A dedicated migration page documents the ROS2-OmniGraph-relevant 6.0 changes:
- Several ROS2 OGN node *types* moved from the `isaacsim.ros2.bridge` extension namespace to a new **`isaacsim.ros2.nodes`** extension namespace (confirmed directly for `ROS2PublishTransformTree`: its 6.0.0 doc page metadata shows `Extension: isaacsim.ros2.nodes`, whereas the same node in 4.5.0/5.1.0 lived under `isaacsim.ros2.bridge`). Some RTX-lidar-related nodes similarly moved to an `isaacsim.sensors.rtx.nodes` extension (see Area 5).
- `ROS2PublishTransformTree` no longer resolves `targetPrims` internally by default in the documented 6.0 pattern; instead it's paired with a new upstream node, `isaacsim.core.nodes.IsaacComputeTransformTree`, which does the USD prim traversal and hands the publisher pre-computed `childFrames`/`parentFrames`/orientation/translation arrays.
- `ROS2PublishJointState` is similarly paired with a new upstream `isaacsim.sensors.physics.nodes.IsaacReadJointState`(-equivalent) node rather than resolving joints internally.
- General architectural framing given for the change: "separation of concerns" — data acquisition (compute/read) nodes are now distinct from data publishing (ROS2-format-marshalling) nodes.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.ros2.nodes/docs/ogn/OgnROS2PublishTransformTree.html

---

## 5. RTX Lidar

This is the section with the most 6.0-era churn and the most direct evidence for the "is the RTX lidar outside the ActionGraph?" question.

### Two publishing paths, both documented

**(a) The OmniGraph "helper" node path — `isaacsim.ros2.bridge.ROS2RtxLidarHelper`** (name/extension may show as `isaacsim.ros2.nodes` or `isaacsim.sensors.rtx.nodes.ROS2RtxLidarHelper` depending on version — see renames below). NVIDIA's documented canonical graph (RTX Lidar Sensors ROS2 tutorial):

| Node | Role |
|---|---|
| `omni.graph.action.OnPlaybackTick` | Triggers downstream nodes when Play is pressed. |
| `isaacsim.ros2.bridge.ROS2Context` | Sets Domain ID. |
| `isaacsim.core.nodes.IsaacRunOneSimulationFrame` | "Runs the create render product pipeline once at the start to improve performance." |
| `isaacsim.core.nodes.IsaacCreateRenderProduct` | Input `cameraPrim` = the lidar sensor prim (an `OmniLidar` prim); creates/attaches a render product to it and outputs a render-product path. |
| `isaacsim.ros2.bridge.ROS2RtxLidarHelper` | Consumes the render product; publishes ROS2 LaserScan or PointCloud2. |

Helper-node configuration, documented explicitly:
- For **LaserScan** (2D — what a TurtleBot3's LDS-01 needs): set `type` = `laser_scan`, `topicName` (e.g. `scan`), `frameId` (e.g. `base_scan`).
- For **PointCloud2**: `type` = `point_cloud`, `topicName`, `frameId`, plus a "Publish Full Scan" checkbox.
- **Full-revolution requirement, quoted directly**: **"When type is set to laser_scan, the LaserScan message will only be published when the RTX Lidar generates a full scan. For a rotary Lidar this is a full 360-degree rotation."** Solid-state lidars complete their "full scan" in a single frame instead.
- Scan-geometry parameters (**`horizontalFov`, `rotationRate`, `azimuthRange`**, etc.) are **read automatically from the sensor prim** — they are not manually re-entered on the helper node; the helper only carries ROS-side configuration (topic name, frame id, message type, QoS).
- Rate control: the older `frameSkipCount` input (documented for 4.x/5.x: "Setting `frameSkipCount` will skip frames between publishing and automatically set the `step` attribute of the Isaac Simulation Gate node connected within the SDG Pipeline" — e.g. skipping 11 frames ≡ publishing every 12th frame) is **deprecated in 6.0+** in favor of setting **`omni:sensor:tickRate`** directly on the `OmniLidar` prim (see Multi-Tick Rendering below); NVIDIA states the old parameter "still works for backward compatibility, but a deprecation warning is logged."

**(b) The Python writer path — attaching a writer to the lidar's render product.** Documented pattern (both the older `omni.replicator.core` idiom and the newer `isaacsim.sensors.experimental.rtx` wrapper):
```python
# Lower-level Replicator writer idiom (documented usage pattern)
writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
writer.initialize(topicName="scan", frameId="base_scan")
writer.attach([hydra_texture_2D])   # hydra_texture_2D = the lidar's render product
```
```python
# 6.0 experimental high-level wrapper (NVIDIA's own migration-guide example)
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

sensor = LidarSensor(
    Lidar.create("/World/Lidar", config="Example_Rotary",
                 orientations=np.array([[1.0, 0.0, 0.0, 0.0]])),
    annotators=["generic-model-output"],
)
sensor.attach_writer("draw-point-cloud", size=0.05, color=[0, 1, 0.5, 1.0])   # visualization writer
# For ROS2: attach a "...ROS2PublishLaserScan" / "...ROS2PublishPointCloud" writer by name via the same mechanism.
```
The `RtxLidarROS2PublishLaserScan` writer's documented parameters are **`topicName`**, **`frameId`**, **`horizontalFov`**, **`horizontalResolution`**, **`depthRange`**, **`rotationRate`**, **`azimuthRange`** — mirroring the OG helper node's ROS-facing fields, confirming both paths ultimately configure the same underlying LaserScan-marshalling logic; NVIDIA's writer usage examples pass `topicName`/`frameId` explicitly at `writer.initialize(...)` while the scan-geometry fields are populated automatically from the sensor prim in the same way as path (a), unless explicitly overridden via `attributes={...}`.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html
- https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/migration_guides/isaac_sim_6_0/sensors_rtx_to_experimental_rtx.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_lidar.html

### CRITICAL QUESTION — is the RTX lidar "outside the OmniGraph"? Confirmed, with nuance.

**Short answer: your understanding is essentially correct, and it is confirmable from official sources, though NVIDIA never uses the exact phrase "outside the ActionGraph."** Here is the documented chain of evidence:

1. **The RTX lidar render/data pipeline is itself an OmniGraph.** NVIDIA's own instructions for inspecting it say: go to "Window > Graph Editors > Action Graph, choose Edit Action Graph and open the graph named **`/Render/PostProcess/SDGPipeline`**." This is a *second, separate* OmniGraph instance, auto-managed by the renderer/Replicator machinery, living under `/Render/PostProcess/` in the stage — distinct in both **path** and **lifecycle** from the user-authored `/ActionGraph` that hosts `OnPlaybackTick → DifferentialController → IsaacArticulationController`.
2. **What's inside it**: the SDGPipeline graph's key node is documented as **`IsaacRenderVarToCPUPointer`** ("Isaac RenderVar To CPU Pointer Node"), which "pulls the `RtxSensorCpu` buffer from the frame's render product" — i.e. it reads out of the RTX renderer's per-render-product output buffer (an AOV, "Arbitrary Output Variable," on that render product), not out of the physics/articulation state. Downstream of that pointer node sit lidar-specific processing nodes (e.g. "Compute RTX Lidar Point Cloud," "Compute RTX Lidar Flat Scan") and finally "a writer or publisher node of some kind, like ROS, or ROS2." This confirms: **annotators/writers are the mechanism by which ROS2 LaserScan publishing happens *inside* this render graph — no separate, user-visible ROS node needs to be manually wired for the writer path**, matching your hypothesis. The `ROS2RtxLidarHelper` "helper" OmniGraph node (path (a) above) is a *convenience wrapper* placed in the user's own ActionGraph that, under the hood, still drives/attaches to this same render-product/SDGPipeline machinery via `IsaacCreateRenderProduct` + the writer/annotator system — it is not a second, independent data path.
3. **Why render rate, not physics rate — this is explicitly documented, via the Multi-Tick Rendering page (new in 6.0, but describing an architecture that predates it in substance):** Isaac Sim's per-app-update execution order is documented as: (i) the run-loop advances the timeline by `loop_dt`; (ii) physics executes N≥0 substeps of `physics_dt`, writing cumulative simulated time to a stage attribute (`/ExternalSimulationTime`); (iii) **"Hydra reads `/ExternalSimulationTime` once and compares against each sensor's last render time and `tickRate`"** to decide which sensors render *this* frame; (iv) only then do OmniGraph nodes read that same simulation time for consistent timestamps. In other words: **rendering (and therefore RTX-sensor output) is gated by the Hydra renderer's own per-sensor tick scheduler, which runs at a documented, independently-configurable rate (`omni:sensor:tickRate`) that is explicitly decoupled from the physics step rate** — "before Isaac Sim 6.0, every camera and RTX sensor rendered at the simulation frame rate" (a real limitation NVIDIA states was fixed by multi-tick rendering), whereas physics can run at, e.g., 200+ Hz for accurate wheel/contact dynamics while the lidar renders at its own native scan rate (e.g. 20 Hz for LDS-01-class sensors). A differential-drive controller node, by contrast, is documented to run on `OnPlaybackTick` in the *physics/pre-render* execution slot, once per app update (i.e., effectively at the physics-adjacent tick), which is a fundamentally different, faster and differently-scheduled cadence than "once every N renders, only after a full GPU ray-traced sweep of the lidar's rotation completes."
4. **Render product lifetime**: the documented `IsaacRunOneSimulationFrame → IsaacCreateRenderProduct` pattern exists specifically because a render product is an expensive-to-create, renderer-owned resource that is meant to be **created once and reused every frame**, not recreated per tick — "runs the create render product pipeline once at the start to improve performance." This is architecturally incompatible with a naive same-ActionGraph, same-tick node that would (if it tried to emulate a lidar inline) need to either hold a persistent render-product handle across ticks anyway (defeating the point of being "just a node in the physics graph") or recreate one every physics tick (prohibitively expensive, and semantically wrong since a render product's contents are only valid once the GPU has actually rendered that frame — which happens on the *render* pass, not the *physics* pass).

**Direct answer to "why can't the lidar just be a node in the same ActionGraph as the differential controller":** it *can* have a node placed in that same user ActionGraph (`ROS2RtxLidarHelper` — path (a) — is exactly that), but that node is a thin trigger/consumer wrapper; the actual sensing computation cannot execute inline with physics-rate nodes because (i) it depends on a render product's GPU-rendered AOV output, which is only valid after Hydra's render pass for that sensor's tick has completed (physics nodes execute before/independent of that render pass in the documented per-update ordering), (ii) a full LaserScan additionally requires accumulation across an entire rotation (multiple render ticks for a rotary lidar) before the message is even emitted, which is fundamentally a multi-tick, render-cadence-gated operation, not a single-tick physics computation, and (iii) the render product and its SDG/annotator/writer pipeline are lifecycle-managed as renderer resources (create-once, `/Render/PostProcess/SDGPipeline`-scoped) rather than as ordinary per-tick ActionGraph state.

**What is and isn't explicitly stated by NVIDIA, precisely:**
- **Confirmed/quoted**: the SDGPipeline is a distinct, separately-editable OmniGraph at `/Render/PostProcess/SDGPipeline`; it pulls from a render product's `RtxSensorCpu`/AOV buffer; multi-tick rendering explicitly decouples per-sensor render cadence from physics cadence via `omni:sensor:tickRate`; LaserScan publishing requires a full rotation of accumulated data; render products are documented as create-once resources.
- **Not verbatim-stated by NVIDIA**: the specific sentence "the RTX lidar is outside the ActionGraph" or an explicit side-by-side architecture diagram contrasting "the ActionGraph" vs "the SDGPipeline graph" as two named, permanent categories. The synthesis above is built from combining the SDGPipeline page, the Multi-Tick Rendering page, and the RTX Lidar Action Graph overview page — each individually documented, not combined into one official diagram/paragraph anywhere I could find.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_rtx_lidar/node_overview.html (SDGPipeline path, IsaacRenderVarToCPUPointer, node chain, "writer or publisher node of some kind, like ROS, or ROS2")
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html (three-clock model, `/ExternalSimulationTime`, Hydra per-sensor tick scheduling, pre-6.0 every-frame-render limitation, `frameSkipCount` deprecation)
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx.html (OmniLidar/render-product/RTX-Sensor-SDK relationship, annotators)
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_annotators.html (annotators vs writers distinction: "Annotators generate data buffers, while Writers consume those buffers for export or visualization")
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html (full-revolution LaserScan requirement, IsaacRunOneSimulationFrame/IsaacCreateRenderProduct pattern)

### API in 6.x: `isaacsim.sensors.rtx` (deprecated) → `isaacsim.sensors.experimental.rtx` (current)
NVIDIA's dedicated **RTX Sensors migration guide** documents this as a full API replacement, not an additive change:

| | Old (`isaacsim.sensors.rtx`) | New (`isaacsim.sensors.experimental.rtx`) |
|---|---|---|
| Creation | `LidarRtx(prim_path, position=, orientation=, config_file_name=)` via `omni.kit.commands.execute()` | `Lidar.create(path=, config=, translations=[[...]], orientations=[[...]], attributes={...}, tick_rate=, accumulate_outputs=, variant=, aux_output_level=)` — direct instantiation, no command layer |
| Runtime data class | Single `LidarRtx` class does authoring + runtime | Split: `Lidar` (authoring/prim wrapper) + `LidarSensor(lidar, annotators=[...])` (runtime data/writer wrapper) |
| Transform params | Singular `position=`, `orientation=` | Plural, array-shaped `translations=`/`orientations=` with shape `(N,3)`/`(N,4)` (only `N=1` currently supported per sensor) |
| Get one frame | `sensor.get_current_frame()` | `sensor.get_data("generic-model-output")` → `(data, info)` tuple |
| Attach annotator | `attach_annotator(...)` | Constructor arg `annotators=["generic-model-output", ...]` |
| Visualization | `enable_visualization()` / `disable_visualization()` | `sensor.attach_writer("draw-point-cloud", ...)` / `detach_writer(...)` |
| Lifecycle | `initialize()` / `pause()` / `resume()` | Removed — driven by `omni.timeline`'s play/pause/stop directly |
| GMO decode | `from isaacsim.sensors.rtx import get_gmo_data` (took a raw OG-node pointer) | `from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data` (module-level function operating on the sensor's own buffer) |

NVIDIA's own before/after code (migration guide):
```python
# Old
from isaacsim.sensors.rtx import LidarRtx
sensor = LidarRtx(prim_path="/World/Lidar", config_file_name="Example_Rotary",
                   orientation=np.array([1.0, 0.0, 0.0, 0.0]))
frame = sensor.get_current_frame()

# New
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
sensor = LidarSensor(
    Lidar.create("/World/Lidar", config="Example_Rotary",
                 orientations=np.array([[1.0, 0.0, 0.0, 0.0]])),
    annotators=["generic-model-output"],
)
data, info = sensor.get_data("generic-model-output")
```

**`tick_rate` and `accumulate_outputs` — documented meaning and defaults.**
- **`tick_rate`** (float, Hz): the sensor's independent render frequency; **0 is documented as the default/"autotrigger" value, meaning "render every frame"** (i.e., no multi-tick decoupling unless a nonzero value is set). NVIDIA's explicit, load-bearing constraint: **"For `OmniLidar` prims, `tick_rate` (i.e. `omni:sensor:tickRate`) must equal `omni:sensor:Core:scanRateBaseHz` for scan accumulation and multi-tick rendering to behave correctly."** Mismatching the two is documented to cause the sensor to silently "fall back to producing partial scans every frame" instead of full rotations — described as a **silent correctness bug** (no error is logged).
- **`accumulate_outputs`** (bool): controls the underlying `omni:sensor:Core:accumulateOutputs` USD attribute. **Not documented in the pages retrieved**: an explicit statement of accumulate_outputs's own default boolean value or a plain-English description of exactly what "accumulating" means frame-to-frame beyond its name (the closest documented adjacent fact is the tick_rate/scanRateBaseHz equality requirement "for scan accumulation...to behave correctly," implying accumulate_outputs is the mechanism by which multiple per-tick partial-rotation renders are assembled into one full-rotation `GenericModelOutput` buffer, consistent with the LaserScan full-revolution requirement documented in the ROS2 tutorial — but I could not find NVIDIA's literal default value or one-sentence definition for this specific flag).
- **`aux_output_level`**: documented as one of `"NONE"` (default), `"BASIC"`, `"EXTRA"`, `"FULL"`, controlling how much auxiliary per-point metadata (beyond XYZ) the `GenericModelOutput` buffer carries; `"FULL"` is noted as slower.
- **Polling vs. writer gotcha, explicitly documented**: "RTX sensors emit `GenericModelOutput` asynchronously under multitick. Polling `sensor.get_data(...)` drops or duplicates frames" — the documented fix is to **use a Writer** instead of polling, since "the Replicator scheduler invokes `Writer.write()` on every rendered frame, guaranteeing no gaps." This is a second, independent piece of evidence for why lidar output is fundamentally an event/writer-driven, render-cadence-gated pipeline rather than a simple per-tick value you can read like a physics-graph attribute.

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/migration_guides/isaac_sim_6_0/sensors_rtx_to_experimental_rtx.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_lidar.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html
- https://raw.githubusercontent.com/isaac-sim/IsaacSim/main/skills/isaac-sim-sensor/SKILL.md (isaac-sim GitHub org — supplementary; used for the polling/writer gotcha and `SUPPORTED_LIDAR_CONFIGS` registry structure, cross-checked against the official migration guide above)

### Custom lidar profiles: what changed in 6.0
This is a clear, confirmed **regression in customization surface area from 5.x → 6.0**, corroborated by an official maintainer statement on the `isaac-sim/IsaacSim` GitHub repo:

- **Pre-6.0 mental model** (per the prompt, and consistent with how older `config_file_name=` usage reads): a named JSON lidar profile could be registered/dropped into the sensors extension and then referenced by name.
- **In Isaac Sim 5.0, this already stopped working as a simple file-drop.** A GitHub discussion ("How can custom RTX lidar configs be added in 5.0?") from a user migrating a working 4.5 Livox MID360 config reports that "simply copying configs to `isaacsim.sensors.rtx` no longer works" and causes segfaults. **An official maintainer (`vick-yu`) responded with the documented-recommended approach**: **"instantiate a supported config and override its sensor attributes programmatically"** rather than registering a new named config — i.e., start from one of the shipped `SUPPORTED_LIDAR_CONFIGS` entries (a stock USD asset) and author/override its `OmniSensorGenericLidarCoreAPI` USD attributes (emitter azimuth/elevation/fire-time arrays, scan rate, range, etc.) directly, via `attributes={...}` on `Lidar.create(...)` or by editing the resulting prim. The maintainer explicitly called this "customization via attribute overrides only."
- **In 6.0**, `config=` on `Lidar`/`Lidar.create()` is documented as accepting either **a registered configuration name from `SUPPORTED_LIDAR_CONFIGS`** (a fixed registry of vendor USD assets — NVIDIA/Ouster/HESAI/Velodyne/Robosense/SICK/Zvision families, each potentially with a `variant=` selecting sub-models, e.g. `Ouster/OS1` + `variant="OS1_REV6_32ch20hz512res"`) **or a direct USD path**. This confirms the user's hypothesis: **there is no separate, extensible JSON-profile *registry* API in the current Python surface — `config=` is effectively restricted to (a) the fixed stock list, or (b) pointing at your own already-authored USD file/prim.** The dedicated "Creating Custom RTX Sensor Profiles" doc page in 6.0.0, as retrieved, is explicitly marked **"under development. Additional content will be added in a future update"** and, beyond confirming the `OmniSensorGenericLidarCoreAPI` (lidar) / `OmniSensorGenericRadarWpmDmatAPI` (radar) schema names, defers to an external, non-Isaac-Sim-docs page ("Setting Lidar Attributes" in "the Omniverse Lidar Extension documentation") for the actual attribute list.
- **NVIDIA's documented replacement workflow for shipping a custom sensor model**, synthesized from the above: (1) start from/reference an existing stock lidar USD asset (or author a bare `OmniLidar` prim with `OmniSensorGenericLidarCoreAPI` applied from scratch), (2) set/override its `omni:sensor:Core:*` attributes — either by hand-editing the USD, through the Property panel, or via `attributes={...}` passed to `Lidar.create()` — and (3) ship the result as **your own USD file/asset** (referenced by path in `config=`), rather than as a named registry entry. This is a materially different distribution story than a drop-in JSON file: a custom sensor model is now itself a USD asset artifact.

**USD schema attributes under `omni:sensor:Core:`** — the following names were confirmed present in the schema by cross-referencing the `isaac-sim-sensor` skill reference (isaac-sim GitHub org) against the official schema-name callouts on the (under-construction) Custom RTX Sensor Profiles doc page, which independently names `OmniSensorGenericLidarCoreAPI` and confirms attributes exist "under development" without enumerating them itself:
- `omni:sensor:Core:scanRateBaseHz` (also referenced directly, unprefixed form, in the Multi-Tick Rendering doc's `tick_rate` constraint sentence quoted above — that specific attribute name **is** independently confirmed by an official docs-site page, not just the community skill file)
- `omni:sensor:Core:accumulateOutputs` (independently confirmed as the attribute backing the `accumulate_outputs` Python parameter, per the 6.0 migration guide's parameter-mapping table)
- `omni:sensor:Core:emitterState:s001:azimuthDeg`, `omni:sensor:Core:emitterState:s001:elevationDeg`, `omni:sensor:Core:emitterState:s001:fireTimeNs` — per-beam arrays for solid-state/multi-emitter lidar patterns
- `patternFiringRateHz`, `nearRangeM`, `farRangeM`, `scanType`, `intensityProcessing`, `rotationDirection`, `rayType`, `highLod`, `drawPoints` — named in search-derived summaries of the schema; **not independently re-confirmed by directly fetching and quoting a primary schema-reference page in this session**, so these specific names should be treated as probable-but-not-doc-verbatim-quoted, and cross-checked against the installed 6.1.0 build's own `OmniSensorGenericLidarCoreAPI` schema (e.g. via `usdview`/Property panel) before being asserted as stable API in the thesis text.

**Bottom line for the thesis:** *Confirmed*: the JSON-profile-registry style of adding a fully custom named lidar model is gone / unsupported as of 5.0+, official guidance is attribute-overrides-on-a-stock-config, and 6.0's `config=` is scoped to `SUPPORTED_LIDAR_CONFIGS` names or literal USD paths. *Not documented*: a complete, official, single-page enumeration of every `omni:sensor:Core:*` attribute with types/defaults — NVIDIA explicitly defers this to an external, not-yet-linked-in-detail "Omniverse Lidar Extension" reference and marks its own 6.0 "Creating Custom RTX Sensor Profiles" page as unfinished.

Sources:
- https://github.com/isaac-sim/IsaacSim/discussions/183 (official maintainer guidance: "instantiate a supported config and override its sensor attributes programmatically"; "customization via attribute overrides only")
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_custom.html (page explicitly marked under development; confirms `OmniSensorGenericLidarCoreAPI`/`OmniSensorGenericRadarWpmDmatAPI` schema names)
- https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html (`omni:sensor:tickRate` / `omni:sensor:Core:scanRateBaseHz` equality requirement — the one schema attribute name independently confirmed on an official, finished docs page)
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/migration_guides/isaac_sim_6_0/sensors_rtx_to_experimental_rtx.html (`accumulateOutputs` attribute/parameter mapping)
- https://raw.githubusercontent.com/isaac-sim/IsaacSim/main/skills/isaac-sim-sensor/SKILL.md (isaac-sim GitHub org — supplementary source for the broader attribute list and `SUPPORTED_LIDAR_CONFIGS` vendor registry; flagged as not independently re-verified against a primary schema doc page in this session)

### Mounting a lidar on a robot: transform convention and `frameId` vs. the robot's TF tree
Documented mounting pattern is USD reference + variant selection at a child Xform under the robot prim:
```usda
over "Robot" {
    def Xform "sensor_mount" {
        double3 xformOp:translate = (0.0, 0.0, 0.35)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def "lidar" (
            prepend references = @${assetsRoot}/Isaac/Sensors/Ouster/OS1/OS1.usd@
            variants = { string sensor = "OS1_REV6_32ch20hz512res" }
        ) {}
    }
}
```
i.e., the lidar's physical offset from the robot is authored as an ordinary USD `xformOp:translate`/rotate on a mount Xform prim parented under the robot, exactly like any other rigidly-attached child link — there is no lidar-specific offset API distinct from normal USD parenting/transform authoring. The Python equivalent is `Lidar.create(path="<mount_prim_path>/lidar", config=..., translations=[[...]], orientations=[[...]])`, with `translations`/`orientations` expressing the sensor's pose **relative to its parent prim** at creation, consistent with standard USD prim transform semantics (not documented as world-relative).

On **`frameId` vs. the robot's TF tree**: the ROS2RtxLidarHelper/writer's `frameId` input (e.g. `"base_scan"` in NVIDIA's own TurtleBot-adjacent examples) is a **plain string set independently by the user on the publishing node/writer** — it is not automatically derived from the sensor prim's name or stage path. This means it is the integrator's responsibility to make sure the `frameId` string passed to the lidar publisher (path a or b) **matches** the corresponding frame name that appears in the robot's published TF tree (from `ROS2PublishTransformTree`/`IsaacComputeTransformTree`, or from a URDF-driven `robot_state_publisher` if that's the chosen architecture) — e.g. if the URDF names the lidar link `base_scan`, the lidar publisher's `frameId` must also be the literal string `"base_scan"` for `tf2` to resolve the LaserScan into the rest of the robot's frames. **Not documented**: any automatic reconciliation, validation, or warning if these two strings (the TF link name and the sensor `frameId`) diverge — NVIDIA relies entirely on the integrator keeping them consistent by convention.

Sources:
- https://raw.githubusercontent.com/isaac-sim/IsaacSim/main/skills/isaac-sim-sensor/SKILL.md (isaac-sim GitHub org — USD mounting example)
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html (`frameId="base_scan"` usage convention)
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/migration_guides/isaac_sim_6_0/sensors_rtx_to_experimental_rtx.html (`translations=`/`orientations=` array semantics)

---

## Summary of confirmed 4.x/5.x → 6.x API renames and architectural changes found during this research

- **Physics backend**: PhysX remains default; **Newton** (`isaacsim.physics.newton`) added as an experimental, switchable alternative via `SimulationManager.switch_physics_engine()`. URDF/MJCF importers rebuilt on the USD Exchange SDK to support both.
- **Core API**: "Core API" (`isaacsim.core.api`, e.g. classic `World`) is being superseded for new work by **"Core Experimental API"** (`isaacsim.core.experimental.*`, Warp-based, engine-agnostic prim wrappers `XformPrim`/`RigidPrim`/`Articulation`/`GroundPlane`), used by Isaac Sim 6.0's own flagship "Getting Started" tutorial in place of `World`.
- **Time/device setup**: `SimulationManager.setup_simulation(dt=, device=)` is the new unified entry point; `SimulationManager.set_physics_dt()`/`set_physics_sim_device()` are deprecated since 1.8.0.
- **URDF importer output**: single-`.usd`-file mental model → **multi-file "Asset Structure" directory** (`geometries.usd`, `materials.usda`, `instances.usda`, `robot.usda`, `physics.usda`/`mujoco.usda`/`physx.usda`, `payloads/`) using references/payloads/variants, aligned to "USD Asset Structure 3.0."
- **Articulation root schema**: importer now stamps both `UsdPhysics.ArticulationRootAPI` **and** `NewtonArticulationRootAPI` on the root link; self-collision now expressed via `newton:selfCollisionEnabled` rather than `physxArticulation:enabledSelfCollisions` directly.
- **ROS2 OmniGraph nodes**: several moved from `isaacsim.ros2.bridge` → **`isaacsim.ros2.nodes`** (confirmed for `ROS2PublishTransformTree`); TF and joint-state publishers decoupled from USD-prim resolution into separate `IsaacComputeTransformTree`/`IsaacReadJointState`-style upstream nodes ("separation of concerns").
- **RTX sensors**: `isaacsim.sensors.rtx` (`LidarRtx`, command-based creation) → **`isaacsim.sensors.experimental.rtx`** (`Lidar`/`LidarSensor` split, direct instantiation, `annotators=`/`attach_writer()`), with `isaacsim.sensors.physics` → `isaacsim.sensors.experimental.physics` for IMU/contact/effort sensors.
- **Multi-tick rendering** (new in 6.0): per-sensor `omni:sensor:tickRate` decouples RTX sensor render cadence from physics/timeline cadence; deprecates the old ROS2-helper-node `frameSkipCount` parameter.
- **Custom RTX lidar profiles**: named JSON-profile registration is gone; only stock `SUPPORTED_LIDAR_CONFIGS` names or direct USD-path `config=` are supported, with attribute-override-on-a-stock-config as the only documented customization path (confirmed via official maintainer statement on GitHub, not the polished docs site, whose "Creating Custom RTX Sensor Profiles" page is marked unfinished).

## Notable gaps flagged as "Not documented" throughout this reference
1. Wheeled-robot-specific PhysX sub-step-rate/Hz recommendation (Area 1).
2. A ground-plane/robot-contact-specific `frictionCombineMode`/`restitutionCombineMode` recommendation (Area 2).
3. A canonical standalone-script lighting-setup code snippet (Area 2).
4. A full narrated Nucleus→S3 asset-root fallback-chain algorithm, as opposed to the resolving function/setting individually (Area 2).
5. A primary-source, verbatim `is_stage_loading()` docstring page (only corroborated via a downstream project and forum evidence) (Area 2).
6. A single authoritative numeric table of wheel-drive stiffness/damping values generalizable beyond the two specific worked examples found (Area 3).
7. An explicit NVIDIA statement on whether the URDF importer's root-link `ArticulationRootAPI` placement is a stable, version-guaranteed contract (Area 3).
8. An explicit "ground truth vs. wheel-encoder-integration" statement for `IsaacComputeOdometry` (inferred from its single `chassisPrim` input, not stated outright) (Area 4).
9. Any NVIDIA guidance/warning on `ROS2PublishTransformTree` vs. a separately-run `robot_state_publisher` fighting over the same TF frames — this is a real, notable silence (Area 4).
10. A default boolean value and precise frame-to-frame semantics for `accumulate_outputs`/`omni:sensor:Core:accumulateOutputs` (Area 5).
11. A single finished, complete, official enumeration of all `omni:sensor:Core:*` schema attributes with types/defaults (the 6.0 "Creating Custom RTX Sensor Profiles" page is explicitly marked under development) (Area 5).
12. An explicit, single official sentence stating "the RTX lidar pipeline is a separate OmniGraph from the ActionGraph" — this is a well-supported synthesis from multiple official pages (SDGPipeline location, multi-tick rendering, annotator/writer docs) but not one quotable sentence (Area 5).
