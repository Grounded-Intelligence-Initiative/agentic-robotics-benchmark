#!/usr/bin/env bash
# Oracle (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this order:
# 1. Main's oracle: the privileged reference program into the submission. It installs rescue.py (one privileged call)
#    and the 256-bit oracle token as .oracle_token; the grader compares the token's sha256 with
#    tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells the privileged calls
#    `get_state_privileged` and `reference_run` (the reference program inside the cell, which reads the site's plan: the
#    route, the obstacles, the survivor's position).
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    program item grades holds the reference program. reference/online_oracle.py connects to the graded episode's service
#    (rubblecell.client.connect_graded), starts THE graded episode with the same oracle token (the service compares its
#    sha256 with the digest it holds and, on a match, gives this episode the privileged calls), runs rescue.py's
#    solve(robot) on it (a camera frame, a lidar sweep and a thermal frame, then the reference program inside one call)
#    and finishes the episode.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd)/reference"
mkdir -p "$SUB"
cp "$A/rescue.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -I -B "$A/online_oracle.py" "$HOME_DIR/given/lib" "$A/oracle_token"
