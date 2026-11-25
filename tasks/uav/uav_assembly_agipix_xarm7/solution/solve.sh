#!/usr/bin/env bash
# Reference solution (dual mode), run in place of the agent by `ale validate` as the same unprivileged account, in this
# order:
# 1. Main's oracle: the reference program (assemble.py: one frame of each camera, then the privileged call) and the
#    256-bit oracle token (.oracle_token) into the submission; the grader compares the token's sha256 with
#    tests/oracle_token.sha256 and, on a match, runs the privileged physical reference inside each workcell.
# 2. The graded episode, after the program is in place: the start freezes the submission folder, so the frozen copy the
#    program item grades holds the reference program. online_oracle.py connects to the graded episode's cell service
#    (~/graded_client.py, connect_graded), starts THE graded episode with the same token (the service compares its
#    sha256 with the digest it holds and, on a match, builds the workcell as the offline grader builds a privileged
#    program's and enables reference_assemble), runs assemble.py's solve(cell) on it and finishes the episode.
set -euo pipefail
TASK_HOME="${ALE_HOME:-/home/user}"
HERE="$(cd "$(dirname "$0")" && pwd -P)"
mkdir -p "$TASK_HOME/submission"
cp "$HERE/assemble.py" "$TASK_HOME/submission/assemble.py"
cp "$HERE/token.txt" "$TASK_HOME/submission/.oracle_token"
exec /opt/venv/bin/python3 -I -B "$HERE/online_oracle.py" "$TASK_HOME" "$TASK_HOME/submission/.oracle_token"
