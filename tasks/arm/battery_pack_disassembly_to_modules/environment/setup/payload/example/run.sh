#!/usr/bin/env bash
# The grader starts this with `bash run.sh` in the submission folder, with PATH, HOME and LANG only.
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -I controller.py
