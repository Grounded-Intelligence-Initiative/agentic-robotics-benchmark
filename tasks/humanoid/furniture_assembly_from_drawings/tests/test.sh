#!/usr/bin/env bash
# Verify stage (online variant): runs as root, cwd = this staged folder, after the agent phase, in a SEPARATE verifier
# sandbox (task.toml [verifier] environment_mode = "separate"): a fresh sandbox from the task image that the agent never
# touched, holding only the declared artifacts, restored root-owned: /home/user/submission (not read) and
# /var/lib/arb-online, the record directory of the graded episode's service. environment/setup/setup.sh never runs here
# and no program runs: verify.py reads the record. The engine uploads this folder world-readable, so the FIRST thing
# done here -- before the interpreter starts -- is to close the folder and its parent to every other account (one chmod
# each; verify.py records the original modes from the two variables and restores them at the end, after it has frozen
# the agent account's processes, which in the separate verifier are the image's own desktop session).
#
# The grader runs as root with nothing of the agent home on its import or configuration paths: HOME and the XDG
# folders point at root's own, PYTHONNOUSERSITE=1 and `-s` keep any user site-packages out.
set -euo pipefail
STAGE="$(pwd -P)"
export ARB_STAGE_MODE="$(stat -c %a "$STAGE")"
chmod 700 "$STAGE"
PARENT="$(dirname "$STAGE")"
if [ "$PARENT" != "/" ] && [ "$PARENT" != "${ALE_HOME:-/home/user}" ]; then
  export ARB_STAGE_PARENT_MODE="$(stat -c %a "$PARENT")"
  chmod 700 "$PARENT"
fi
export HOME=/root PYTHONNOUSERSITE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
