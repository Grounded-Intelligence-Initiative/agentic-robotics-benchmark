#!/usr/bin/env bash
# The reference controller (controller.py): speaks the grader's stdio wire.
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1
exec /opt/venv/bin/python3 -u controller.py
