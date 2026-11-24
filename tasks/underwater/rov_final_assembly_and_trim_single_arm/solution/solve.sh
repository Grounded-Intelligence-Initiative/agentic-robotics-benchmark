#!/usr/bin/env bash
# Reference solution (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this
# order:
# 1. Main's oracle: the privileged reference program into the submission. It installs assemble.py (one frame of each
#    camera, then one privileged call) and the 256-bit token as .oracle_token; the grader compares the token's sha256
#    with tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells the privileged call
#    `reference_assemble` (the reference program inside the cell: it weighs, trims, turns and fits every unit) on the
#    same four fixed hidden episodes as every other submission.
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    program item grades holds the reference program. online_oracle.py connects to the graded episode's cell service
#    (arm.assembly_rov.graded.connect_graded), starts THE graded episode with the same token (the service compares its
#    sha256 with the digest it holds and, on a match, gives this episode the privileged calls), runs assemble.py's
#    solve(cell) on it (the public calls, then `reference_assemble` inside the cell) and finishes the episode.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd -P)"
TOKEN=6bedaf3de93b1ebe76e7da91ed113d75833b5dd57c73417a959b83aec4f79fe1
mkdir -p "$SUB"
cp "$HERE/assemble.py" "$SUB/assemble.py"
printf '%s\n' "$TOKEN" > "$SUB/.oracle_token"
echo "[reference] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -I -B "$HERE/online_oracle.py" "$HOME_DIR/given/lib" "$SUB/.oracle_token"
