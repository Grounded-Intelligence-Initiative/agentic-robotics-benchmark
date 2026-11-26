#!/usr/bin/env bash
# The reference submission (runs in place of the agent, as the agent's account, cwd ~/.ale-solution).
#
# It leaves in /home/user/submission: run.sh and controller.py (the reference controller), the published holder model
# it computes with (a copy of ~/given/holder_model.py, which setup staged), and the 256-bit reference token as
# .oracle_token. The grader compares the token's sha256 with tests/hidden/oracle_token.sha256 and, on a match, adds
# `privileged` to every observation: what a perfect perception system would have read from the same delayed frame
# (the scope fitted, the camera head's angle, the video latency, whether the lens is smeared, each instrument's tip in
# the camera frame). The reference controller
# reads that in place of the picture and moves the holder only by the rate commands every submission uses.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
mkdir -p "$SUB"
cp "$HERE/reference/controller.py" "$HERE/reference/run.sh" "$SUB/"
cp "$HOME_DIR/given/holder_model.py" "$SUB/holder_model.py"
cp "$HERE/reference/oracle_token" "$SUB/.oracle_token"
echo "[solution] reference controller and token ready in $SUB" >&2
