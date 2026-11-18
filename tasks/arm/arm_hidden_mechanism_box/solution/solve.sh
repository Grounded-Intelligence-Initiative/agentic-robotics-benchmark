#!/usr/bin/env bash
# Privileged reference solution (SPEC 14.1 / 17), dual mode: a scripted controller that knows every hidden box's
# mechanism (solution/ref/boxes.json, protected reference data: part types, positions, directions and the unlock
# order). At each episode start it estimates the box placement from the two fixed cameras (top-face segmentation + fit
# of the known outline), then drives the stages in order with scripted primitives.
# 1. The program item, as main's solve.sh: the controller goes into the submission folder (run.sh and its files).
# 2. The graded episode: the same controller plays it through the agent-side client (helpers/graded_bridge.py and
#    labclient.py in the agent home): graded_start (which freezes the submission folder just filled), one graded_step
#    per controller reply until the episode ends (all of the box's stages done, or 2500 steps), then graded_finish if
#    it is still running. The order matters: the program frozen at the start must be the reference program.
# Runs as the agent account with the agent's timeouts and network policy; spends no lab trials.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE"/solver/*.py "$SUB/"
cp "$HERE/ref/boxes.json" "$SUB/boxes.json"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /opt/venv/bin/python3 -I oracle_main.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] controller ready at $SUB; playing the graded episode" >&2
rc=0
/opt/venv/bin/python3 "$HOME_DIR/helpers/graded_bridge.py" --submission "$SUB" \
  --log "$HOME_DIR/oracle_graded.jsonl" --finish >&2 || rc=$?
/opt/venv/bin/python3 "$HOME_DIR/labclient.py" graded_status >&2 || true
exit "$rc"
