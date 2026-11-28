#!/usr/bin/env bash
# Oracle: the privileged reference controller, run in place of the agent by `ale validate` as the same unprivileged
# account. It installs controller/controller.py (an echo of the grader's reference commands) and the 256-bit oracle
# token as controller/oracle_token; the grader compares the token's sha256 with tests/hidden/oracle_token.sha256 and,
# on a match, runs its reference script and sends that script's setup reply and step replies to the controller, which
# returns them.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission/controller"
A="$(dirname "$0")/reference"
mkdir -p "$SUB"
cp "$A/controller.py" "$SUB/controller.py"
cp "$A/oracle_token" "$SUB/oracle_token"
echo "[oracle] submission ready at $SUB" >&2
