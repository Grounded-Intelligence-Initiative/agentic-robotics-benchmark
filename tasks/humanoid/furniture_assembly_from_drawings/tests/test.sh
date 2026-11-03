#!/usr/bin/env bash
# Verify stage: runs as root, cwd = this staged folder, after the agent phase. The folder was absent while the agent
# worked; the engine uploads it world-readable, so the FIRST thing done here -- before the interpreter starts -- is to
# close the folder and its parent to every other account (one chmod each; verify.py records the original modes from the
# two variables and restores them at the end, then kills the agent account's and the practice lab's leftover processes,
# snapshots the submission, gates late writes and locks every file).
#
# The grader runs as root with nothing of the agent home on its import or configuration paths: HOME and the XDG /
# matplotlib folders point at root's own, PYTHONNOUSERSITE=1 and `-s` keep any user site-packages out, and the cell
# renders through OSMesa (the agents' cameras use the same path in the practice lab).
set -euo pipefail
STAGE="$(pwd -P)"
export ARB_STAGE_MODE="$(stat -c %a "$STAGE")"
chmod 700 "$STAGE"
PARENT="$(dirname "$STAGE")"
if [ "$PARENT" != "/" ] && [ "$PARENT" != "${ALE_HOME:-/home/user}" ]; then
  export ARB_STAGE_PARENT_MODE="$(stat -c %a "$PARENT")"
  chmod 700 "$PARENT"
fi
export HOME=/root PYTHONNOUSERSITE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache \
  MPLCONFIGDIR=/root/.config/matplotlib
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
