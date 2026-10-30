#!/usr/bin/env bash
# Verify stage entry point (root, separate verification sandbox): hardened environment, then grade.
set -euo pipefail
umask 077
export HOME=/root PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export SUPERDEX_PRECISION=double SUPERDEX_ASSETS_PATH=/opt/superdex_assets SORTER_DATA=/opt/sorter_data
unset PYTHONPATH PYTHONSTARTUP PYTHONHOME
TMPROOT="$(mktemp -d)"
export TMPDIR="$TMPROOT" XDG_CACHE_HOME="$TMPROOT/cache" XDG_CONFIG_HOME="$TMPROOT/config"
exec python3 -B verify.py
