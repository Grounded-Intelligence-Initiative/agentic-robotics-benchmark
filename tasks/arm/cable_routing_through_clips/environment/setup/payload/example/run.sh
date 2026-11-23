#!/usr/bin/env bash
# The shape of a submission: run.sh starts a controller that speaks the wire (see the instruction).
set -euo pipefail
cd "$(dirname "$0")"
exec python3 hover.py
