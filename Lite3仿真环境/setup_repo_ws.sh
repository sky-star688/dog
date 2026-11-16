#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
SRC="$ROOT/../Lite3工业巡检ROS2包/src/lite3_inspection"
DST="$ROOT/ws/src/lite3_inspection"

if [ ! -d "$SRC" ]; then
  echo "Missing repo package: $SRC" >&2
  exit 1
fi

rm -rf "$DST"
cp -r "$SRC" "$DST"
echo "Synced $SRC -> $DST"
