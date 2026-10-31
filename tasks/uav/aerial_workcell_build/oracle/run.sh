#!/usr/bin/env bash
# The reference build: the cell authored as a USD stage by reference/build_scene.py (runs Isaac Sim headless on the
# sandbox's GPU, as the agent's account, within the agent budget).
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
/opt/venv/bin/python3 "$(dirname "$0")/reference/build_scene.py" --out "$SUB/scene.usda"
echo "[oracle] scene ready at $SUB/scene.usda" >&2
