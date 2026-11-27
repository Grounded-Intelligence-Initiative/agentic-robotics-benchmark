#!/usr/bin/env bash
# Setup (trusted root, before the agent's session, cwd /opt/ale/setup): stage the givens into the agent home, record
# the session's start for ~/bin/session_time.sh, and start the commissioning service under its own account.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
PANDA=/opt/task-assets/panda
[ -f "$PANDA/panda.xml" ] || { echo "[setup] $PANDA missing: the image was built without its robot model" >&2; exit 1; }
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
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

install -d -o root -g root -m 755 /run/towercell
date +%s > /run/towercell/session_start
chmod 644 /run/towercell/session_start

SERVICE=/opt/towercell-commission
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
chown towercell:towercell /run/towercell
chmod 755 /run/towercell
rm -f /run/towercell/commission.sock
setsid runuser -u towercell -- env HOME="$SERVICE" MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=4 BT_PANDA_DIR=/opt/task-assets/panda \
    OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" OPENCV_IPP=avx2 \
    /bin/bash -c 'cd /opt/towercell-commission/state; exec >> /opt/towercell-commission/state/commissiond.out 2>&1; while :; do /opt/venv/bin/python3 -B -s /opt/towercell-commission/commissiond.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 120); do [ -S /run/towercell/commission.sock ] && break; sleep 0.5; done
[ -S /run/towercell/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }
echo "[setup] commissioning service listening on /run/towercell/commission.sock" >&2
