#!/usr/bin/env bash
# Legacy single-dog inspection stack (see run_dog1.sh for the dual-dog Dog 1).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "ERROR: workspace is not built; run scripts/build.sh first." >&2
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

exec ros2 launch lite3_inspection inspection.launch.py "$@"
