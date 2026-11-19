#!/usr/bin/env bash
# The task's reference solution, run in place of the agent by `ale validate`, as the same unprivileged account with
# the same timeout. Dual mode: both items, in this order.
#
# 1. The program (main's solve.sh, unchanged): the PRIVILEGED reference controller (solution/privileged/oracle.py)
#    and the reference token go into the submission folder. The offline item grades this folder as the graded start
#    froze it: the grader hands a submission whose .oracle_token matches the digest in tests/ the simulator's ground
#    truth in every observation, and the controller plans from it.
# 2. The graded episode (the online variant's path): drive_oracle.py starts the episode through the agent's own
#    client (~/graded_episode.py) with the reference token, and the service hands the same ground truth to every
#    observation; the same controller, from a working copy, plays the episode to its horizon and finishes it. The
#    start freezes the submission folder, so step 1 must come first.
# The anchors are the reference's own measured scores on the two items. An agent gets no ground truth and works
# from the cameras. The non-privileged reference that works from the observation alone (oracle v17) is
# solution/reference_v17/; this script does not use it.
set -euo pipefail

HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"

# -- 1. the program (main's solve.sh)
mkdir -p "$SUB"

cp "$HERE/privileged/oracle.py" "$HERE/privileged/geometry.json" "$SUB/"
cp "$HERE/privileged/oracle_token" "$SUB/.oracle_token"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MESSY_DESK_ORACLE_LOG=0
exec /opt/venv/bin/python3 -B oracle.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] privileged submission ready at $SUB" >&2

# -- 2. the graded episode (the online variant's solve.sh)
WORK="$HOME_DIR/oracle_run"
mkdir -p "$WORK"
cp "$HERE/privileged/oracle.py" "$HERE/privileged/geometry.json" "$WORK/"

export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MESSY_DESK_ORACLE_LOG=0
echo "[oracle] privileged controller ready at $WORK; playing the graded episode" >&2
exec /opt/venv/bin/python3 -B "$HERE/drive_oracle.py" --controller "$WORK/oracle.py" \
  --token-file "$HERE/privileged/oracle_token" --client "$HOME_DIR/graded_episode.py"
