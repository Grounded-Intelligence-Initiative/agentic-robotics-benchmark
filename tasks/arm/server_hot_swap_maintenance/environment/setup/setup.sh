#!/usr/bin/env bash
# Setup (trusted root, before the agent's session, cwd /opt/ale/setup): stage the givens into the agent home (the
# manual, the robot model, the commissioning client, the wire skeleton, the session-time script), write the session's
# time stamp, and start the commissioning service under its own account. Nothing hidden is in this folder.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
PANDA=/opt/task-assets/panda
[ -f "$PANDA/panda.xml" ] || { echo "[setup] $PANDA missing: the image was built without its robot model" >&2; exit 1; }
# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership onto the agent
# home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"
mkdir -p "$HOME_DIR/given/robot"
rm -rf "$HOME_DIR/given/robot/panda"
cp -r "$PANDA" "$HOME_DIR/given/robot/panda"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/given"
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"

install -d -o root -g root -m 755 /run/session
date -u +%s > /run/session/start
chown root:root /run/session/start && chmod 0444 /run/session/start

SERVICE=/opt/srvcell-commission
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
install -d -o srvcell -g srvcell -m 755 /run/srvcell
rm -f /run/srvcell/commission.sock
setsid runuser -u srvcell -- env HOME="$SERVICE" MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 SRV_PANDA_DIR=/opt/task-assets/panda \
    OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" OPENCV_IPP=avx2 \
    /bin/bash -c 'cd /opt/srvcell-commission/state; exec >> /opt/srvcell-commission/state/commissiond.out 2>&1; while :; do /opt/venv/bin/python3 -B -s /opt/srvcell-commission/commissiond.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 120); do [ -S /run/srvcell/commission.sock ] && break; sleep 0.5; done
[ -S /run/srvcell/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }
echo "[setup] givens staged in $HOME_DIR; commissioning service on /run/srvcell/commission.sock" >&2
