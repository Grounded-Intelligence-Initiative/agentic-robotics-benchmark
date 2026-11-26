#!/bin/bash
set -euo pipefail
umask 077
export HOME=/root PYTHONNOUSERSITE=1 MESA_SHADER_CACHE_DISABLE=true
ROOT_TMP=$(mktemp -d)
export TMPDIR="$ROOT_TMP" XDG_CACHE_HOME="$ROOT_TMP/cache" XDG_CONFIG_HOME="$ROOT_TMP/config"
unset PYTHONPATH PYTHONSTARTUP
cd -- "$(dirname -- "$0")"
exec "${ALE_VERIFY_PYTHON:-/opt/venv/bin/python3}" -I -B verify.py
