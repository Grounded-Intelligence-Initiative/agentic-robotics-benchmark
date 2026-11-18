#!/usr/bin/env bash
# Oracle (online variant): the privileged reference program, run in place of the agent by `ale validate` as the same
# unprivileged account. It plays the graded episode itself, through the agent-side client (skidcell.client.connect_graded):
# reference/play_graded.py starts the episode with the 256-bit oracle token (the graded episode's service compares its
# sha256 with the one it holds and, on a match, gives the episode's cell the privileged calls `get_state_privileged` and
# `reference_run`), runs the reference submission reference/operate.py on it (describe, two camera frames, then
# reference_run: the scripted reference controller inside the cell) and finishes it. Nothing else is graded: the
# submission folder stays empty.
set -euo pipefail
A="$(cd "$(dirname "$0")" && pwd -P)/reference"
/opt/venv/bin/python3 -B "$A/play_graded.py"
echo "[oracle] the graded episode is played and finished" >&2
