#!/usr/bin/env bash
# Oracle (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this order:
# 1. Main's oracle: the privileged program into the submission. It installs assemble.py (one privileged call) and the
#    256-bit oracle token as .oracle_token; the grader compares the token's sha256 with tests/hidden/oracle_token.sha256
#    and, on a match, gives this submission's cells the bench plan's nominal layout and the privileged call
#    `reference_assemble` (the privileged program inside the cell).
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    offline item grades holds the privileged program. reference/online_oracle.py connects to the graded episode's cell
#    service (arm.assembly_openarm.graded.connect_graded), starts THE graded episode with the same oracle token (the
#    service compares its sha256 with the digest it holds and, on a match, builds the episode's cell again from its
#    seed in the privileged mode, as offline: the bench plan's nominal layout and the privileged calls), runs
#    reference/assemble.py's solve(cell) on it (one frame of each camera, then the privileged program inside one call)
#    and finishes the episode.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd)/reference"
mkdir -p "$SUB"
cp "$A/assemble.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -I -B "$A/online_oracle.py" "$HOME_DIR/given/lib" "$A/oracle_token"
