#!/usr/bin/env bash
# Start fleet_manager (ground station coordinator).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "ERROR: workspace is not built. Run: bash scripts/build.sh fleet" >&2
  exit 1
fi
if [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: do not run as root." >&2
  exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

echo "============================================"
echo "  fleet_manager – Ground Station Coordinator"
echo "  Monitors: lite3_01 + lite3_02"
echo "============================================"

exec ros2 launch fleet_manager fleet.launch.py \
  inspector_id:=lite3_01 \
  operator_id:=lite3_02 \
  "$@"
