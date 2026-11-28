#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup), before the agent's session. Stages the agent-visible
# payload (example cars with their numbers and manuals, the practice runner) into the agent home and starts the
# practice cell.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP

HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"

# cp -r, NOT -a: with -a the payload directory's own root ownership would land on the agent home.
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"

# The practice cell (environment/practice_service/practiced.py): the cell on its practice seeds, behind a unix socket,
# for a fixed number of practice episodes. It runs as `toycell` under a loop that restarts it should it ever exit.
# A separate verifier never runs this script, so no practice cell runs where the work is graded.
SERVICE=/opt/toycell
[ -f "$SERVICE/practiced.py" ] || { echo "[setup] the image lacks the practice cell ($SERVICE)" >&2; exit 1; }
install -d -o toycell -g toycell -m 755 /run/toycell
rm -f /run/toycell/practice.sock
setsid runuser -u toycell -- env -i HOME="$SERVICE/state" TMPDIR="$SERVICE/state" \
    PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 \
    LP_NUM_THREADS=2 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
    /bin/bash -c 'cd /opt/toycell/state && while :; do /opt/venv/bin/python3 -B -s /opt/toycell/practiced.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/toycell/practice.sock ] && break; sleep 0.5; done
[ -S /run/toycell/practice.sock ] || { echo "[setup] the practice cell did not come up" >&2; exit 1; }
echo "[setup] practice cell listening on /run/toycell/practice.sock" >&2
