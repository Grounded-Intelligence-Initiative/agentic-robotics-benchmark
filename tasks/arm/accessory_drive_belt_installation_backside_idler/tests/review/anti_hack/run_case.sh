#!/usr/bin/env bash
# Local verify of one submission in the task image (review tool):
#   run_case.sh IMAGE TESTS_DIR SUBMISSION_DIR OUT_DIR CPUS [noise|-] [VAR=VALUE ...]
# The container has the default (bridge) network, so blocked attempts are real attempts. With "noise", a root process
# opens a TCP connection every 2 s during the verify (the kind of namespace traffic that is not the program's).
# VAR=VALUE arguments go into the verify container's environment, i.e. the grader's process (for instance the
# renderer's LP_NATIVE_VECTOR_WIDTH; amendment 9 of hidden_set.md). A submission that holds a pre.sh stays owned by the
# agent account, and pre.sh starts in the background as that account just before the verify, as a process an agent
# session left behind would (the late_write probe); otherwise the submission is root-owned, as the verify locks it.
set -euo pipefail
IMAGE=$1; TESTS=$(cd "$2" && pwd -P); SUB=$(cd "$3" && pwd -P); OUT=$(mkdir -p "$4" && cd "$4" && pwd -P); CPUS=$5
NOISE=${6:-}; [ "$NOISE" = "-" ] && NOISE=""
shift $(( $# < 6 ? $# : 6 ))
EXTRA=(); for kv in "$@"; do EXTRA+=(-e "$kv"); done
docker run --rm --memory 12g --memory-swap 12g --pids-limit 8192 -v "$TESTS:/opt/src-tests:ro" -v "$SUB:/opt/src-sub:ro" -v "$OUT:/out" -e NOISE="$NOISE" ${EXTRA[@]+"${EXTRA[@]}"} \
  "$IMAGE" taskset -c "$CPUS" bash -c '
set -e
mkdir -p /opt/ale && cp -r /opt/src-tests /opt/ale/tests
rm -rf /home/user/submission && mkdir -p /home/user/submission && cp -r /opt/src-sub/. /home/user/submission/
if [ -f /home/user/submission/pre.sh ]; then
  # the submission as the agent phase left it: owned by the agent account and written an hour before the verify
  find /home/user/submission -exec touch -h -d "@$(( $(date +%s) - 3600 ))" {} +
  chown -R user:user /home/user/submission
  (runuser -u user -- bash /home/user/submission/pre.sh > /out/pre.log 2>&1 &)
  sleep 1
else
  chown -R root:root /home/user/submission
fi
if [ -n "$NOISE" ]; then (while true; do /opt/venv/bin/python3 -c "import socket
try: socket.create_connection((\"1.1.1.1\", 80), timeout=1).close()
except Exception: pass" ; sleep 2; done) & fi
cd /opt/ale/tests && (bash test.sh > /out/verify.log 2>&1 || true)
cp /opt/ale/tests/verdict.local.json /out/ 2>/dev/null || true
cp -r /opt/arb-verify/logs /out/ 2>/dev/null || true
ls -la /home/user/submission > /out/submission_after.txt 2>&1 || true
'
python3 - "$OUT/verdict.local.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
m = d.get("metrics", {})
print("reward", d.get("reward"), json.dumps({k: m.get(k) for k in (
    "contract_ok", "belt_acceptance", "mean_sim_time_s",
    "camera_frames_mean", "program_time_s_mean",
    "mean_episode_wall_s", "cell_internal_errors", "render_errors", "grader_exception", "remote_sockets_seen",
    "net_outbound_events", "nonet_selftest_ok", "network_imports", "runner_processes_killed", "runner_processes_left",
    "stray_files_removed", "ipc_swept_unused", "submission_late_writes", "late_writes_quarantined",
    "illegal_call_count", "bad_param_calls", "step_calls", "ipc_swept") if k in m}))
PY
