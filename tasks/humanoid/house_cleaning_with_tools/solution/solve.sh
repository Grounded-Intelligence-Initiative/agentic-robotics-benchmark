#!/usr/bin/env bash
# Oracle (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this order:
# 1. Main's oracle: the privileged reference program into the submission. It installs clean.py (camera frames, then one
#    privileged call) and the 256-bit oracle token as .oracle_token; the grader compares the token's sha256 with
#    tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells (the same fixed hidden scenes the
#    agents get) the privileged call `reference_clean` (the reference cleaner inside the cell: true object poses and
#    process states, the G1 driven through the cell's own controller, grip rule, safety watch and cleaning processes).
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    offline item grades holds the reference program. reference/online_oracle.py connects to the graded episode's cell
#    service (house_client.connect_graded), starts THE graded episode with the same oracle token (the service compares
#    its sha256 with the digest it holds and, on a match, gives this episode the privileged calls `reference_clean` and
#    `truth`), runs reference/clean.py's solve(cell) on it (one frame of each public camera, then the reference cleaner
#    inside one call) and finishes the episode.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd)/reference"
mkdir -p "$SUB"
cp "$A/clean.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -I -B "$A/online_oracle.py" "$HOME_DIR/given/lib" "$A/oracle_token"
