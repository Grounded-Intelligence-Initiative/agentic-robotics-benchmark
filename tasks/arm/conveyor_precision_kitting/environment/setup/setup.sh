#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent's session. Stages the agent-visible payload (the
# cell's documents and the practice tools) into the agent home and starts the practice cell.
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

# The practice cell (see environment/practice_service/practiced.py): runs as `kitcell` under a loop that restarts it
# should it ever exit. A separate verifier never runs this script, so no practice cell runs where the work is graded.
SERVICE=/opt/kitcell
[ -f "$SERVICE/practiced.py" ] || { echo "[setup] the image lacks the practice cell ($SERVICE)" >&2; exit 1; }
install -d -o kitcell -g kitcell -m 755 /run/kitcell
rm -f /run/kitcell/practice.sock
setsid runuser -u kitcell -- env -i HOME="$SERVICE/state" TMPDIR="$SERVICE/state" \
    PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 \
    LP_NUM_THREADS=1 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
    /bin/bash -c 'cd /opt/kitcell/state && while :; do /opt/venv/bin/python3 -B -s /opt/kitcell/practiced.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/kitcell/practice.sock ] && break; sleep 0.5; done
[ -S /run/kitcell/practice.sock ] || { echo "[setup] the practice cell did not come up" >&2; exit 1; }
echo "[setup] practice cell listening on /run/kitcell/practice.sock" >&2
