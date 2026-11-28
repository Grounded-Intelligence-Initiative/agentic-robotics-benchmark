#!/usr/bin/env bash
# The reference controller on the grading wire (main.py).
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1
exec /opt/venv/bin/python3 -u main.py
