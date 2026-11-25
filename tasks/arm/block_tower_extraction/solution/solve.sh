#!/usr/bin/env bash
# The reference submission: the reference program (play.py) and the 256-bit oracle token. With the token the graded
# cell answers the privileged calls (every block's true pose, the load on it from above, its gap to the level above,
# the contact between a pulled block and the level above): the reference uses them to choose loose blocks, to aim its
# motions and to keep a pulled block clear of the level above. Runs as the agent's account, offline, in seconds.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(dirname "$0")/reference"
mkdir -p "$SUB"
cp "$A/play.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
