#!/usr/bin/env bash
# Verify map files and print hashes so a changed copy is obvious.
set -euo pipefail

MAP_DIR="${1:-}"
if [ -z "$MAP_DIR" ] || [ ! -d "$MAP_DIR" ]; then
  echo "usage: bash scripts/verify_map.sh <map_dir>" >&2
  exit 1
fi

for file in "$MAP_DIR"/*.yaml "$MAP_DIR"/*.pgm "$MAP_DIR"/*.pcd; do
  [ -f "$file" ] || continue
  echo "$(sha256sum "$file" | cut -d' ' -f1)  $(basename "$file")"
done

echo
echo "If a copy differs from the baseline hash, re-survey waypoints and"
echo "re-validate with map_server before driving. Do not force a pass."
