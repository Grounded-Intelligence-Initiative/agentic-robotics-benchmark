#!/usr/bin/env bash
set -euo pipefail
chmod 700 .
unset DISPLAY WAYLAND_DISPLAY
export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LP_NUM_THREADS=8
exec /opt/venv/bin/python3 -u verify.py
