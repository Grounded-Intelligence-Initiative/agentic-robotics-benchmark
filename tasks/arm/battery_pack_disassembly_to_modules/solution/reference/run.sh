#!/usr/bin/env bash
# The reference controller behind the grader's wire. The grader starts this with PATH, HOME and LANG only.
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/opt/venv/bin/python3
[ -x "$PY" ] || PY=python3
exec "$PY" -I main.py
