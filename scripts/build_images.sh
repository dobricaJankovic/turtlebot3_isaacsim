#!/usr/bin/env bash
#
# Build the base image, then the package image on top of it.
#
#   scripts/build_images.sh              # both
#   scripts/build_images.sh --base-only  # just the base
#
# The Isaac Sim base is pulled from NGC: `docker login nvcr.io` first.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

BASE_IMAGE="${TB3_BASE_IMAGE:-isaacsim61-humble:ngc}"
ISAACSIM_IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.1.0}"

echo "==> base: ${BASE_IMAGE}  (Isaac Sim from ${ISAACSIM_IMAGE})"
docker build \
    -f docker/isaacsim-ros2/Dockerfile.humble \
    --build-arg "ISAACSIM_IMAGE=${ISAACSIM_IMAGE}" \
    -t "${BASE_IMAGE}" \
    docker/isaacsim-ros2/

if [ "${1-}" = "--base-only" ]; then
    echo "==> base built. Verify with: docker/isaacsim-ros2/verify.sh ${BASE_IMAGE}"
    exit 0
fi

echo "==> app: turtlebot3_isaacsim:humble"
TB3_BASE_IMAGE="${BASE_IMAGE}" docker compose build

echo "==> done. ./docker/x11-auth.sh, then docker compose up -d"
