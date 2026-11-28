#!/usr/bin/env bash
# The reference design: robot.xml beside this script, copied into the submission folder and checked with the agent's
# own design checker. It was found by the task author with a random search over legal designs followed by a local
# search, both run in the author's development copy of the walkway simulator (not available to the agent); the
# agent has the same catalog, gait and limits, plus 4 practice runs, to reason its way to a design.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$HERE/robot.xml" "$SUB/robot.xml"
/opt/venv/bin/python3 "${ALE_HOME:-/home/user}/qm24/check_design.py" "$SUB/robot.xml"
