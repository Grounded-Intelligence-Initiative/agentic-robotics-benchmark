#!/usr/bin/env bash
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$(dirname "$0")/solver.py" "$SUB/solver.py"
cp "$(dirname "$0")/launch.sh" "$SUB/run.sh"
chmod +x "$SUB/run.sh"
echo '[oracle] simulator artifact staged' >&2
