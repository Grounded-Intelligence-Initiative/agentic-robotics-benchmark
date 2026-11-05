#!/usr/bin/env bash
set -euo pipefail
chmod 700 . ..
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "${ALE_VERIFY_PYTHON:-python3}" -I -B verify.py
