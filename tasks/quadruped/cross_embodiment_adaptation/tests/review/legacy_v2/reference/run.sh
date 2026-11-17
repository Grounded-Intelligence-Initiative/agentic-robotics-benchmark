#!/usr/bin/env bash
# The image-only reference mission program (no token, no truth): scout, map, switch, deliver.
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -B reference.py
