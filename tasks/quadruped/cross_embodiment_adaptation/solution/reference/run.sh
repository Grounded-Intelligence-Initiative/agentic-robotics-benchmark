#!/usr/bin/env bash
# The reference mission program from the robots' own observations (no token, no truth).
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -B reference.py
