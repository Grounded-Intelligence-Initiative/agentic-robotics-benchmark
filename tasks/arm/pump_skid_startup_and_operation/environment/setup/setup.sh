#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase. Stages the givens (the cell documents, the
# client library, the instrument catalog, the practice script and the example program) into the agent home, owned by
# the agent, and starts the practice service (below). The grader never reads
# these copies: verification is separate (task.toml), and tests/test.sh builds the grader's own ROOT-OWNED read-only
# copy of the same givens in the fresh verifier sandbox, from tests/setup_payload/ (a byte copy of payload/ here: change
# both together, then tests/given_hashes.json) and the image's robot model (/opt/task-assets/panda).
# Dynamic-only: packages, the robot model and the practice service's program live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r payload/. "$HOME_DIR/"
if [ -d /opt/task-assets/panda ]; then
  mkdir -p "$HOME_DIR/given"
  cp -r /opt/task-assets/panda "$HOME_DIR/given/panda"
fi
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# The practice service, a training simulator of the skid series: the image installs the cell library and the service
# program under /opt/skidcell-practice for the account `skidcell` only (the agent account can neither read nor list
# them); setup starts it now, to run through the agent's session, under a loop that restarts it should it ever exit.
# The agent reaches it through the socket /run/skidcell/practice.sock (skidcell.client.connect_practice,
# ~/practice/grade.py): the cell's public calls, reset(seed) and a debrief per shift; no ground truth.
PRACTICE=/opt/skidcell-practice
[ -f "$PRACTICE/practiced.py" ] || { echo "[setup] the image lacks the practice service ($PRACTICE)" >&2; exit 1; }
install -d -o skidcell -g skidcell -m 755 /run/skidcell
rm -f /run/skidcell/practice.sock
setsid runuser -u skidcell -- env HOME="$PRACTICE" MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=2 \
    OPENBLAS_NUM_THREADS=2 /bin/bash -c 'while :; do /opt/venv/bin/python3 -s /opt/skidcell-practice/practiced.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/skidcell/practice.sock ] && break; sleep 0.5; done
[ -S /run/skidcell/practice.sock ] || { echo "[setup] the practice service did not come up" >&2; exit 1; }
echo "[setup] practice service listening on /run/skidcell/practice.sock" >&2
