#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$ROOT/.."
DEST="$ROOT/third_party"

# deep_robotics_model is registered as a git submodule under ws/src.
if [ -f "$REPO_ROOT/.gitmodules" ]; then
  git -C "$REPO_ROOT" submodule update --init --recursive
fi

mkdir -p "$DEST"

if [ ! -d "$DEST/Lite3_rl_deploy/.git" ]; then
  git clone --depth 1 \
    https://github.com/DeepRoboticsLab/Lite3_rl_deploy.git \
    "$DEST/Lite3_rl_deploy"
else
  echo "Already exists: $DEST/Lite3_rl_deploy"
fi
