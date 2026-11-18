#!/usr/bin/env bash
# Verify stage: runs as root, cwd = this staged folder, after the agent phase, in a SEPARATE verifier sandbox (task.toml
# [verifier] environment_mode = "separate"): a fresh sandbox from the task image that the agent never touched, holding
# only the declared artifacts (/home/user/submission and the graded service's record directory /var/lib/arb-online,
# both restored root-owned); environment/setup/setup.sh never runs here. No program runs: verify.py scores the graded
# episode's record. The engine uploads this folder with modes 0644 / 0755, so it is closed to every other account
# before the interpreter starts.
set -euo pipefail
chmod 700 "$(pwd -P)"
export HOME=/root PYTHONNOUSERSITE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
