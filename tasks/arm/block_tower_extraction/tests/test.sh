#!/usr/bin/env bash
# Verify stage: runs as root, cwd = this staged folder, in a SEPARATE verifier sandbox (task.toml) holding only the
# declared artifact (/home/user/submission); environment/setup/setup.sh never runs here. The engine uploads this folder
# with modes 0644 / 0755, so the FIRST thing done here is to close it and its parent to every other account (verify.py
# restores the modes at the end).
# Runs as root; the hidden towers run in parallel, each program under its own account (btrun1..4).
# The grader's root-owned, read-only copy of the givens (/opt/arb): setup_payload/ (this folder's byte copy of
# environment/setup/payload/) plus the robot model from the image's task assets (/opt/task-assets/panda, the same bytes
# setup.sh stages into the agent home); verify.py checks it file by file against given_hashes.json and hands its client
# library to the program. Every thread count and the numerics pin are exported here, so grading does not depend on the
# image's variables.
set -euo pipefail
STAGE="$(pwd -P)"
export ARB_STAGE_MODE="$(stat -c %a "$STAGE")"
chmod 700 "$STAGE"
PARENT="$(dirname "$STAGE")"
if [ "$PARENT" != "/" ] && [ "$PARENT" != "${ALE_HOME:-/home/user}" ]; then
  export ARB_STAGE_PARENT_MODE="$(stat -c %a "$PARENT")"
  chmod 700 "$PARENT"
fi
ROOT_PAYLOAD="${ARB_ROOT_PAYLOAD:-/opt/arb}"
PANDA=/opt/task-assets/panda
[ -f "$PANDA/panda.xml" ] || { echo "[verify] $PANDA missing: the image was built without its robot model" >&2; exit 1; }
rm -rf "$ROOT_PAYLOAD"
mkdir -p "$ROOT_PAYLOAD/given/robot"
cp -r "$STAGE/setup_payload/." "$ROOT_PAYLOAD/"
cp -r "$PANDA" "$ROOT_PAYLOAD/given/robot/panda"
chown -R root:root "$ROOT_PAYLOAD"
chmod -R u+rwX,go+rX,go-w "$ROOT_PAYLOAD"
export HOME=/root PYTHONNOUSERSITE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache MPLCONFIGDIR=/root/.config/matplotlib
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa LP_NUM_THREADS=4 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" OPENCV_IPP=avx2
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
