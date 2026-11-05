#!/usr/bin/env bash
# The verify stage's entry point: root, cwd = this staged folder (/opt/ale/tests), after the agent phase.
#
# `$ALE_VERIFY_PYTHON`: the engine stages its `ale_verify` package, through which the kit's write_verdict writes
# /logs/verifier/reward.json, into the first 3.12+ interpreter it finds (`python3`, then `python3.12`) and names it
# here. In this image that `python3` is /opt/venv/bin/python3 (the image's PATH, and /usr/local/bin/python3 wraps it),
# the interpreter this stage named explicitly before.
set -euo pipefail
chmod 700 .
unset DISPLAY WAYLAND_DISPLAY
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LP_NUM_THREADS=8
exec "${ALE_VERIFY_PYTHON:-python3}" -u verify.py
