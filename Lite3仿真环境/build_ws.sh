#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$ROOT/.."
WS="$ROOT/ws"

if [ -f "$REPO_ROOT/.gitmodules" ]; then
  git -C "$REPO_ROOT" submodule update --init --recursive || true
fi

"$ROOT/setup_repo_ws.sh"

sg docker -c "docker run --rm --network=host \
  -v ${WS}:/root/sim_ws \
  lite3_sim:foxy \
  bash -lc 'source /opt/ros/foxy/setup.bash && cd /root/sim_ws && colcon build --symlink-install'"
