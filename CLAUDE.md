# turtlebot3_isaacsim

Isaac Sim simulation package for the TurtleBot3, mirroring `turtlebot3_gazebo`'s
ROS 2 interface. Runs on Isaac Sim 6.1.0 + ROS 2 Humble inside the container in
`docker/`.

## Where the answers already are

`DESIGN.md` (24 KB) and `UPSTREAM.md` (37 KB) are long. **Grep them, don't read
them whole** — every section has a `##`/`###` heading:

| question | where |
|---|---|
| what is broken / what is verified | `DESIGN.md`, `## Status` and the `###` above it |
| why a piece is built this way | `DESIGN.md`, the `##` named after the directory |
| what NVIDIA documents, and where we differ | `UPSTREAM.md` |
| how to run it | `README.md`, `## Run` |

`sed -n '/^## Status/,$p' DESIGN.md` answers "what's left to do" on its own.

## Recently fixed

The generated warehouse map was mirrored in x — fixed 2026-09-14, `DESIGN.md`,
"The occupancy map was flipped". Any map built before that is wrong; rebuild
with `scripts/build_map.py`. A live nav2 goal against the corrected map has not
been run yet, so warehouse navigation is still not "verified" end to end.

## Generated, not committed

`models/turtlebot3_*/`, `worlds/*.usd`, `maps/*` are built by
`scripts/build_models.sh` and `scripts/build_map.py`. Absent files are normal,
not a broken checkout.

## Conventions

- `scripts/` is what a person runs by hand: `build_images.sh`,
  `build_models.sh`, `build_map.py`, `import_turtlebot3.py`. `runtime/` is what
  a launch file runs: `isaacsim.launch.py` invokes `turtlebot3_isaacsim.py`
  directly, which imports `assets.py`. Everything in both directories runs on
  Isaac Sim's Python (3.12 in the container), not the system Python, and none
  of it is importable from a normal ROS 2 node.
- Apache-2.0 header on every new file, matching the existing ones.
