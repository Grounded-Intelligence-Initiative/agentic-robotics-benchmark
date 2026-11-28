#!/usr/bin/env bash
set -euo pipefail

exec "$ALE_VERIFY_PYTHON" -B verify.py
