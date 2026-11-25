#!/usr/bin/env bash
# Oracle (dual mode): the privileged reference program, run in place of the agent by `ale validate` as the same
# unprivileged account, for both items, in this order (the program must be in place before the graded start freezes it):
#   1. main's oracle: it installs operate.py (one privileged call) and the 256-bit oracle token as .oracle_token in the
#      submission; the grader compares the token's sha256 with tests/hidden/oracle_token.sha256 and, on a match, gives
#      this submission's cells the privileged calls `get_state_privileged` and `reference_run` (the scripted reference
#      controller inside the cell);
#   2. the graded episode: reference/play_graded.py starts it with the same token (the graded episode's service compares
#      its sha256 with the one it holds; at that start it freezes the submission folder, which then holds step 1's
#      program), runs the reference submission on it (describe, two camera frames, then reference_run) and finishes it.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd -P)/reference"
mkdir -p "$SUB"
cp "$A/operate.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
/opt/venv/bin/python3 -B "$A/play_graded.py"
echo "[oracle] the graded episode is played and finished" >&2
