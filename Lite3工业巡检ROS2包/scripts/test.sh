#!/usr/bin/env bash
# Offline domain tests (no ROS needed).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

cd "$ROOT/src/lite3_inspection"
PYTHONPATH="$PWD:$PYTHONPATH" python3 -m pytest test/ -v "$@"
