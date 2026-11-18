#!/usr/bin/env bash
# Privileged reference solution: a scripted controller that knows the hidden units (solution/oracle/units.json and
# their models) and is told their placement and control states by the oracle token's truth. It builds its own copy of
# the scene each episode (reference.Shadow) and runs one scripted primitive per request.
# Online graded episode: the controller plays the session's graded episode itself, through the agent-side client
# (labclient.py in the agent home), driven by solution/graded_run.py: graded_start with the oracle token (the lab then
# adds `truth` to every graded observation, as the offline grader does for a tokened submission), one graded_step per
# controller reply until the episode ends (its third request closed), then graded_finish if it is still running.
# Runs as the agent account with the agent's timeouts and network policy; spends no lab trials.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp -r "$HERE"/oracle/. "$SUB/"
cp "$HERE/oracle_token" "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /opt/venv/bin/python3 -I oracle_main.py --units units.json
SH
chmod +x "$SUB/run.sh"
echo "[oracle] controller ready at $SUB; playing the graded episode" >&2
rc=0
/opt/venv/bin/python3 "$HERE/graded_run.py" --submission "$SUB" --token-file "$HERE/oracle_token" \
  --log "$HOME_DIR/oracle_graded.jsonl" >&2 || rc=$?
exit "$rc"
