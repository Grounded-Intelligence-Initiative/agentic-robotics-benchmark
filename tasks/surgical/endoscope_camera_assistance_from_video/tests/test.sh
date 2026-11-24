#!/usr/bin/env bash
# The verify stage's entry point: root, cwd = this staged folder (/opt/ale/tests), in a fresh verifier sandbox that
# holds only the agent's /home/user/submission (task.toml: separate verification). While the agent worked this
# folder did not exist in its sandbox.
#
# "$ALE_VERIFY_PYTHON" and nothing else: the engine stages `ale_verify` into the image's python3 (the py3.12
# /opt/venv, which also holds numpy for the grader and Isaac Sim for the renderer it starts) and names it here.
# The grader keeps the agent's home off its import and configuration paths.
set -euo pipefail
export HOME=/root PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
