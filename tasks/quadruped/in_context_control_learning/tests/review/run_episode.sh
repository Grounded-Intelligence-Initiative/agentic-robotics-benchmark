#!/usr/bin/env bash
# run_episode.sh <seed> <out dir>: one episode the way the engine runs it, inside the task image, as root in a
# throwaway container whose /task is this task folder (read-only): setup_runtime.py with a one-scene hidden set, the
# oracle (solution/) as the agent account through the robot socket, then the grader on a copy of tests/ whose hidden
# set is that scene, reading only the ledger and an oracle trajectory (what the separate verifier gets).
# Writes <out>/<seed>.ledger.json, .verdict.json, .oracle.txt, .grader.log and .time.
set -euo pipefail
SEED=$1; OUT=$2
STAGE=$(mktemp -d); cp -r /task/environment/setup/. "$STAGE/"
printf '{"seeds": [%s]}\n' "$SEED" > "$STAGE/scenes.json"
/opt/venv/bin/python3 "$STAGE/setup_runtime.py"
SOL=$(mktemp -d); cp -r /task/solution/. "$SOL/"; chown -R user:user "$SOL"
START=$(date +%s.%N)
runuser -u user -- /opt/venv/bin/python3 "$SOL/oracle_run.py" > "$OUT/$SEED.oracle.txt" 2> "$OUT/$SEED.oracle.err" || true
END=$(date +%s.%N)
cp /var/lib/visual_robotics/state.json "$OUT/$SEED.ledger.json"
TESTS=$(mktemp -d); cp -r /task/tests/. "$TESTS/"; printf '{"seeds": [%s]}\n' "$SEED" > "$TESTS/hidden_scenes.json"
echo '{"schema_version": "ATIF-v1.8", "agent": {"name": "oracle"}, "steps": []}' > "$TESTS/trajectory.json"
ALE_TRAJECTORY_PATH="$TESTS/trajectory.json" ROBOTICS_GRADER_VERDICT_PATH="$OUT/$SEED.verdict.json" \
    /opt/venv/bin/python3 "$TESTS/grader.py" > "$OUT/$SEED.grader.log" 2>&1 || echo "grader exit $?" >> "$OUT/$SEED.grader.log"
python3 -c "import sys; print(round(float(sys.argv[2])-float(sys.argv[1]), 1))" "$START" "$END" > "$OUT/$SEED.time"
