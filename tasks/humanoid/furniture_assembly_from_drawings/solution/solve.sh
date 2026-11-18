#!/usr/bin/env bash
# Oracle (dual mode): both items of the task, in this order.
#   1. Main's oracle, unchanged: the privileged reference assembler's program goes into the submission, assemble.py
#      (camera frames, then one privileged call) with the 256-bit oracle token as .oracle_token. The offline grader
#      compares the token's sha256 with tests/hidden/oracle_token.sha256 and, on a match, gives this program's cells the
#      nominal layout and the privileged call `reference_assemble`.
#   2. The live item: reference/graded_oracle.py connects to the graded episode's service through the agent-side
#      client (~/given/lib/furniture_client: connect_graded), starts the graded episode with the same token (the
#      service compares its sha256 with environment/setup/online/oracle_token.sha256 and, on a match, allows the cell's
#      privileged calls on the graded layout), runs reference/assemble.py's solve(cell) on it and finishes it.
# The order matters: the program is frozen when the graded episode starts, so it must be in place first.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd -P)/reference"
mkdir -p "$SUB"
cp "$A/assemble.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -B "$A/graded_oracle.py" --token "$A/oracle_token" --program "$A/assemble.py"
