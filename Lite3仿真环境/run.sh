#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
WS="$ROOT/ws"

xhost +local:docker >/dev/null 2>&1 || true
xhost +local:root >/dev/null 2>&1 || true

sg docker -c "docker rm -f lite3_sim >/dev/null 2>&1 || true"
sg docker -c "docker run --rm -it \
  --name lite3_sim \
  --net=host \
  -e DISPLAY=${DISPLAY:-:0} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${WS}:/root/sim_ws \
  lite3_sim:foxy \
  bash -lc 'source /opt/ros/foxy/setup.bash && source /root/sim_ws/install/setup.bash && exec bash'"
