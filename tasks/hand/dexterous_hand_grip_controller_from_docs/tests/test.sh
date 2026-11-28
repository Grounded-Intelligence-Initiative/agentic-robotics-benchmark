#!/usr/bin/env bash
# Verify stage: runs as root, cwd = this staged folder, after the agent phase, in a SEPARATE verifier sandbox (task.toml
# [verifier] environment_mode = "separate"): a fresh sandbox from the task image that the agent never touched, holding
# only the declared artifact (/home/user/submission, restored root-owned); environment/setup/setup.sh never runs here.
# The engine uploads this folder world-readable, so the FIRST thing done here -- before the interpreter starts -- is to
# close the folder and its parent to every other account (verify.py records the original modes from the two variables
# and restores them at the end, kills the agent account's leftover processes, snapshots the submission, gates late
# writes and locks every file).
set -euo pipefail
STAGE="$(pwd -P)"
export ARB_STAGE_MODE="$(stat -c %a "$STAGE")"
chmod 700 "$STAGE"
PARENT="$(dirname "$STAGE")"
if [ "$PARENT" != "/" ] && [ "$PARENT" != "${ALE_HOME:-/home/user}" ]; then
  export ARB_STAGE_PARENT_MODE="$(stat -c %a "$PARENT")"
  chmod 700 "$PARENT"
fi
# The grader's root-owned, read-only copy of the givens (/opt/arb): verify.py checks it file by file against
# given_hashes.json. It holds what environment/setup/setup.sh stages into the agent home: setup_payload/, this folder's
# byte copy of environment/setup/payload/ (which never reaches the verifier; change both together, then
# given_hashes.json). The grader's own model of the hand is under grip/ (never shown to the agent).
ROOT_PAYLOAD="${ARB_ROOT_PAYLOAD:-/opt/arb}"
rm -rf "$ROOT_PAYLOAD"
mkdir -p "$ROOT_PAYLOAD"
cp -r "$STAGE/setup_payload/." "$ROOT_PAYLOAD/"
chown -R root:root "$ROOT_PAYLOAD"
chmod -R u+rwX,go+rX,go-w "$ROOT_PAYLOAD"
# Only the grader reads the givens: the controller's author is told the documents are not there at the acceptance, so
# the copy is closed to every other account (its root folder stays traversable: the run folders lie under it).
chmod 0700 "$ROOT_PAYLOAD/given"
export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
# The numerics environment of every grader process (always the pair CORETYPE + L2_SIZE, one BLAS thread, and numpy's
# loops on one code path): the bundled OpenBLAS picks its kernels by CPU type and numpy its loops, and these settings
# give the same numerics on every host (the image's ENV carries the same three values).
export OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"
# The memory killer spares the grader before the controller: every process of the controller starts with oom_score_adj
# 1000 (choom, innermost) and is kept there (the root sampler kills one found below 1000, sparing the launch chain's own
# helpers runuser / prlimit / choom); the grader lowers its own value to -500 where the container allows it
# (CAP_SYS_RESOURCE; a default container does not, and the grader then keeps the value it was started with).
{ echo -500 > /proc/self/oom_score_adj; } 2>/dev/null || true
exec "${ALE_VERIFY_PYTHON:-python3}" verify.py
