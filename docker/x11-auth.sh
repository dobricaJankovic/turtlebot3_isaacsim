#!/usr/bin/env bash
#
# Write an Xauthority cookie the container can present to the host X server,
# so GUI windows open without `xhost +`. Run once per X session, before
# `docker compose up`; re-run if the X session restarts.
#
#   ./docker/x11-auth.sh
#   docker compose up -d

set -euo pipefail

XAUTH="${TB3_XAUTH:-/tmp/turtlebot3_isaacsim.docker.xauth}"

if [ -d "$XAUTH" ] || { [ -e "$XAUTH" ] && [ ! -w "$XAUTH" ]; }; then
  what=$([ -d "$XAUTH" ] && echo "a directory" || echo "a file you cannot write")
  cat >&2 <<EOF
error: $XAUTH is $what.

  $(ls -ld "$XAUTH")

Clear it, re-key, and restart the container (in that order):

  docker compose down
  sudo rm -rf $XAUTH
  $0
  docker compose up -d
EOF
  exit 1
fi

touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
chmod 644 "$XAUTH"

echo "Wrote $XAUTH for DISPLAY=$DISPLAY (restart any running container to pick it up)"
