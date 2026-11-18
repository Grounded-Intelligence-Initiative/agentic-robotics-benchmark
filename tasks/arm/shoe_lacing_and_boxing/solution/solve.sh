#!/usr/bin/env bash
# Oracle (dual mode): run in place of the agent by `ale validate` as the same unprivileged account.
#  1. What main's solve.sh does: the privileged reference program into the submission folder: packer.py (one privileged
#     call) and the 256-bit oracle token as .oracle_token. The grader compares the token's sha256 with
#     tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells the privileged calls
#     `get_state_privileged` and `reference_run` (the scripted reference controller inside the cell).
#  2. Then the graded episode with the same reference (graded.py: connect_graded, start with the token, reference/packer.py,
#     finish). The order matters: the service freezes the submission folder when the episode starts, so the frozen copy
#     the program item grades holds the reference program.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
A="$HERE/reference"
mkdir -p "$SUB"
cp "$A/packer.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
export ARB_GIVEN_DIR="$HOME_DIR/given"
exec /opt/venv/bin/python3 -B "$HERE/graded.py"
