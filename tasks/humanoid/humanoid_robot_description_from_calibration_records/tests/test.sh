#!/usr/bin/env bash
# Verify stage: root, cwd = this staged folder, after the agent phase, in a SEPARATE verifier sandbox (task.toml): a
# fresh sandbox from the task image into which the engine restored only /home/user/submission. setup.sh has not run
# there and nothing the agent left in its own sandbox is present, so every program this script runs (stat, chmod,
# dirname, the verify Python) is the image's own. The engine uploads the folder world-readable, so the FIRST thing done
# here, before the interpreter starts, is to close the folder and its parent to every other account (verify.py records
# the original modes from the two variables, freezes and kills the agent account's leftover processes, snapshots the
# submission, gates late writes, locks every file and restores the modes at the end). The grader executes nothing it
# is given: unit.xml is parsed and compiled as data, calibration.yaml is token-scanned and read with a safe loader.
set -euo pipefail
STAGE="$(pwd -P)"
export ARB_STAGE_MODE="$(stat -c %a "$STAGE")"
chmod 700 "$STAGE"
PARENT="$(dirname "$STAGE")"
if [ "$PARENT" != "/" ] && [ "$PARENT" != "${ALE_HOME:-/home/user}" ]; then
  export ARB_STAGE_PARENT_MODE="$(stat -c %a "$PARENT")"
  chmod 700 "$PARENT"
fi
# One thread and the image's numerics pin (environment/Dockerfile): the grade is the same on every x86-64 CPU.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"
export PYTHONDONTWRITEBYTECODE=1 PYTHONHASHSEED=0
exec "${ALE_VERIFY_PYTHON:-python3}" -B verify.py
