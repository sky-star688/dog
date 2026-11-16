#!/usr/bin/env bash
set -euo pipefail

sg docker -c "docker rm -f lite3_sim >/dev/null 2>&1 || true"
