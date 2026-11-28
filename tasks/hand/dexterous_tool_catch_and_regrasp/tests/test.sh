#!/usr/bin/env bash
# The verify stage's entry point. After the agent phase the engine uploads tests/ to /opt/ale/tests (root-only) and
# runs `bash test.sh` there as root, cwd = that folder. `$ALE_VERIFY_PYTHON` is the interpreter the engine staged its
# `ale_verify` package into (the image's /opt/venv python3); `python3` stands in when run by hand.
set -euo pipefail

exec "${ALE_VERIFY_PYTHON:-python3}" verify.py
