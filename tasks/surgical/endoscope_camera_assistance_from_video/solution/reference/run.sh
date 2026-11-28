#!/usr/bin/env bash
# The reference controller: the grader launches this as `bash run.sh` in the submission folder and speaks JSON lines
# to it. (ARB_PYTHON is only set by a maintainer's run outside the task image.)
exec "${ARB_PYTHON:-/opt/venv/bin/python3}" -B -s controller.py
