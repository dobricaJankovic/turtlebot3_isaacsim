#!/usr/bin/env bash
#
# Smoke test for the Isaac Sim + ROS 2 base image.
#
#   ./verify.sh isaacsim61-humble:ngc
#
# The last two checks need a GPU and are skipped without one.
set -uo pipefail

IMAGE="${1:?usage: verify.sh IMAGE}"
DOCKER_RUN=(docker run --rm --network host --ipc host
            -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DOMAIN_ID=99)

pass=0
fail=0
skip=0

report() {
    if [ "$1" -eq 0 ]; then
        printf '  \033[32mPASS\033[0m  %s\n' "$2"; pass=$((pass + 1))
    else
        printf '  \033[31mFAIL\033[0m  %s\n' "$2"; fail=$((fail + 1))
    fi
}

skipped() {
    printf '  \033[33mSKIP\033[0m  %s\n' "$1"; skip=$((skip + 1))
}

echo "verifying ${IMAGE}"

# --- 1. the ROS 2 side ------------------------------------------------------
"${DOCKER_RUN[@]}" "$IMAGE" bash -c '
    set -e
    test -n "$ROS_DISTRO"
    ros2 pkg list > /dev/null
    for pkg in rviz2 nav2_bringup pointcloud_to_laserscan xacro tf2_tools; do
        ros2 pkg prefix "$pkg" > /dev/null
    done' > /dev/null 2>&1
report $? "ROS 2 environment, RViz, Nav2 and the shared tooling"

# --- 2. simulators ----------------------------------------------------------
"${DOCKER_RUN[@]}" "$IMAGE" bash -c '
    set -e
    if [ "$ROS_DISTRO" = humble ]; then
        command -v gzserver > /dev/null
        ros2 pkg prefix gazebo_ros > /dev/null
    fi
    command -v ign > /dev/null || command -v gz > /dev/null
    ros2 pkg prefix ros_gz_bridge > /dev/null' > /dev/null 2>&1
report $? "Gazebo, and the ros_gz bridge"

# --- 3. environment isolation ----------------------------------------------
"${DOCKER_RUN[@]}" "$IMAGE" bash -c '
    set -e
    export AMENT_PREFIX_PATH="/overlay/install:${AMENT_PREFIX_PATH}"
    export PYTHONPATH="/overlay/install/lib/python3/dist-packages:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="/overlay/install/lib:${LD_LIBRARY_PATH:-}"

    ros-isolate bash -c "
        set -e
        case \"\${PATH}:\${PYTHONPATH:-}:\${LD_LIBRARY_PATH:-}\" in
            */opt/ros/*) echo \"system ROS 2 survived isolation\" >&2; exit 1 ;;
            */overlay/*) echo \"overlay workspace survived isolation\" >&2; exit 1 ;;
        esac
        [ -z \"\${AMENT_PREFIX_PATH:-}\" ] || { echo \"AMENT_PREFIX_PATH survived\" >&2; exit 1; }
        case \"\${LD_LIBRARY_PATH:-}\" in
            *isaacsim.ros2.core/*/lib*) ;;
            *) echo \"bundled ROS 2 libraries missing from LD_LIBRARY_PATH\" >&2; exit 1 ;;
        esac
        [ \"\${ROS_DOMAIN_ID}\" = 99 ] || { echo \"ROS_DOMAIN_ID was lost\" >&2; exit 1; }
        [ \"\${ROS_DISTRO}\" = \"${ISAACSIM_ROS_DISTRO:-}\" ] || {
            echo \"ROS_DISTRO not switched to the bundled distribution\" >&2; exit 1; }
        command -v bash > /dev/null
    "' > /dev/null 2>&1
report $? "ros-isolate strips ROS 2 and keeps the DDS settings"

# --- GPU-only checks --------------------------------------------------------
if ! docker run --rm --gpus all "$IMAGE" true > /dev/null 2>&1; then
    skipped "Kit starts on this base image (no GPU available)"
    skipped "ROS 2 bridge publishes to the system ROS 2 installation (no GPU available)"
else
    # --- 4. Kit starts ------------------------------------------------------
    "${DOCKER_RUN[@]}" --gpus all "$IMAGE" \
        isaacsim-python -c '
import os, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.kit.app
print(f"KIT_VERSION={omni.kit.app.get_app().get_build_version()}", flush=True)
sys.stdout.flush()
os._exit(0)' 2>&1 | grep -q "KIT_VERSION="
    report $? "Kit starts and reports its version"

    # --- 5. the bridge reaches the system ROS 2 installation ----------------
    "${DOCKER_RUN[@]}" --gpus all "$IMAGE" bash -c '
cat > /tmp/clock_only.py <<"PY"
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils
import omni.graph.core as og
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROS2Clock", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [("PubClock.inputs:topicName", "/clock")],
    },
)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()
app_utils.play()
for _ in range(9000):
    simulation_app.update()
PY
        isaacsim-python /tmp/clock_only.py > /tmp/sim.log 2>&1 &
        for _ in $(seq 1 90); do
            ros2 topic list 2>/dev/null | grep -qx /clock && break
            sleep 2
        done
        timeout 60 ros2 topic echo /clock --once > /tmp/clock.txt 2>/dev/null
        status=$?
        [ $status -eq 0 ] && grep -q "sec:" /tmp/clock.txt || {
            echo "--- simulator log (tail) ---" >&2; tail -40 /tmp/sim.log >&2; exit 1; }' > /dev/null 2>&1
    report $? "ROS 2 bridge publishes /clock to the system ROS 2 installation"
fi

echo
echo "${pass} passed, ${fail} failed, ${skip} skipped"
[ "$fail" -eq 0 ]
