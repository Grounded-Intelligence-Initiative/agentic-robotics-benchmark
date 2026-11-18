#!/usr/bin/env bash
# Oracle: the privileged reference program plays the graded episode, run in place of the agent by `ale validate` as
# the same unprivileged account, through the agent's own client. graded.py connects to the graded service, starts the
# graded episode with the 256-bit reference token (the service compares its sha256 with its root-only digest and, on a
# match, gives the episode's cell the privileged calls `get_state_privileged` and `reference_run`), runs
# reference/packer.py (one privileged call: the scripted reference controller inside the cell) and finishes the
# episode; the service's record of it is what tests/verify.py scores.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
HERE="$(cd "$(dirname "$0")" && pwd)"
export ARB_GIVEN_DIR="$HOME_DIR/given"
exec /opt/venv/bin/python3 -B "$HERE/graded.py"
