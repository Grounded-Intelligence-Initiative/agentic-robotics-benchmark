#!/usr/bin/env bash
# The task's reference solution, run in place of the agent by `ale validate`, as the same unprivileged
# account with the same timeout. Online variant: it plays the graded episode. drive_oracle.py starts the
# episode through the agent's own client (~/graded_episode.py) with the reference token, and the graded
# episode service then hands the simulator's ground truth to every observation (`privileged`: every
# object's identity, pose and state, the containers' poses, the open request's object, the true
# proprioception), exactly as the offline grader does for a submission carrying the token. The PRIVILEGED
# reference controller (solution/privileged/oracle.py, unchanged) plans from it; the driver steps the
# episode with its actions until the horizon and finishes it. The anchor is its own measured episode score.
# An agent gets no such block and works from the cameras.
#
# The non-privileged reference that works from the observation alone (oracle v17, the evidence that the
# agent's perception suffices) is solution/reference_v17/. This script does not use it.
set -euo pipefail

HOME_DIR="${ALE_HOME:-/home/user}"
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="$HOME_DIR/oracle_run"
mkdir -p "$WORK"
cp "$HERE/privileged/oracle.py" "$HERE/privileged/geometry.json" "$WORK/"

export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MESSY_DESK_ORACLE_LOG=0
echo "[oracle] privileged controller ready at $WORK; playing the graded episode" >&2
exec /opt/venv/bin/python3 -B "$HERE/drive_oracle.py" --controller "$WORK/oracle.py" \
  --token-file "$HERE/privileged/oracle_token" --client "$HOME_DIR/graded_episode.py"
