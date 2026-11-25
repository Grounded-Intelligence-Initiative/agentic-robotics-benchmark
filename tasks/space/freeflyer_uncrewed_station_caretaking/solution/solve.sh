#!/usr/bin/env bash
# Oracle (dual mode): both items of the task, in this order.
#   1. Main's oracle, unchanged: the privileged reference program goes into the submission, run in place of the agent
#      by `ale validate` as the same unprivileged account. It installs caretake.py (one privileged call, two lines) and
#      the 256-bit oracle token as .oracle_token; the verifier compares the token's sha256 with
#      tests/hidden/oracle_token.sha256 and, on a match, gives this submission's cells the privileged call
#      `reference_run` (tests/reference/reference.py inside the cell: the five privileged reads the reference may make,
#      every move through the public calls; it takes one frame of every camera).
#   2. The live item: reference/graded_oracle.py connects to the graded episode's service through the agent-side
#      client (~/given/lib/cabinkit/graded.py: connect_graded), starts the graded episode with the same token (the
#      service compares its sha256 with its own digest, environment/setup/online/oracle_token.sha256, and on a match
#      allows the cell's privileged call `reference_run` on the graded site), runs it and finishes the episode.
# The order matters: the program is frozen when the graded episode starts, so it must be in place first.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
A="$(cd "$(dirname "$0")" && pwd -P)/reference"
mkdir -p "$SUB"
cp "$A/caretake.py" "$SUB/"
cp "$A/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
exec /opt/venv/bin/python3 -B "$A/graded_oracle.py" --token "$A/oracle_token"
