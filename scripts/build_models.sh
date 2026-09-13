#!/usr/bin/env bash
#
# Build the robot asset from turtlebot3_description.
#
#     scripts/build_models.sh [burger|waffle|waffle_pi]
#
# Run from a ROS 2 shell: the xacro expansion and the package lookup need the
# ROS environment, the URDF import needs Isaac Sim's. See DESIGN.md.
set -euo pipefail

MODEL="${1:-${TURTLEBOT3_MODEL:-burger}}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(dirname "$HERE")"

case "$MODEL" in
  burger|waffle|waffle_pi) ;;
  *) echo "error: unknown model '$MODEL'" >&2; exit 1 ;;
esac

if [ -n "${ISAACSIM_PYTHON:-}" ]; then
  PYTHON="$ISAACSIM_PYTHON"
elif command -v isaacsim-python >/dev/null 2>&1; then
  PYTHON="isaacsim-python"
elif [ -x "${ISAACSIM_PATH:-/isaac-sim}/python.sh" ]; then
  PYTHON="${ISAACSIM_PATH:-/isaac-sim}/python.sh"
else
  echo "error: no Isaac Sim python.sh. Set ISAACSIM_PYTHON or ISAACSIM_PATH." >&2
  exit 1
fi

DESCRIPTION="$(ros2 pkg prefix --share turtlebot3_description)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# namespace:= expands the template's ${namespace} to nothing.
xacro "$DESCRIPTION/urdf/turtlebot3_${MODEL}.urdf" namespace:= \
  > "$WORK/turtlebot3_${MODEL}.urdf"

"$PYTHON" "$HERE/import_turtlebot3.py" \
  --model "$MODEL" \
  --urdf "$WORK/turtlebot3_${MODEL}.urdf" \
  --description-share "$DESCRIPTION" \
  --output "$PKG/models/turtlebot3_${MODEL}/turtlebot3_${MODEL}.usd"
