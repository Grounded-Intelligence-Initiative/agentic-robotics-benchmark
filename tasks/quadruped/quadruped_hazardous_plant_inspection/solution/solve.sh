#!/usr/bin/env bash
# Oracle (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this order:
# 1. Main's oracle: the privileged reference program into the submission. It installs patrol.py (one privileged call)
#    and the 256-bit oracle token as .oracle_token; the grader compares the token's sha256 with
#    tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells the privileged calls
#    `get_state_privileged` and `reference_run` (the reference program inside the cell, with the true pose and the
#    site's true layout; it reads every value from its own frames).
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    program item grades holds the reference program. reference/online_oracle.py connects to the graded episode's
#    service (plantcell.graded.connect_graded), starts THE graded mission with the same oracle token (the service
#    compares its sha256 with the digest it holds and, on a match, gives this mission the privileged calls, as
#    offline), runs reference/patrol.py's solve(robot) on it (one frame of every sensor, then reference_run: the same
#    calls in the same order as the offline oracle's mission) and finishes the mission.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd)/reference"
mkdir -p "$SUB"
cp "$A/patrol.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -I -B "$A/online_oracle.py" "$HOME_DIR/given/lib" "$A"
