#!/usr/bin/env bash
# Verify stage (online variant): runs as root, cwd = this staged folder, after the agent phase, in a SEPARATE verifier
# sandbox (task.toml [verifier] environment_mode = "separate"): a fresh sandbox from the task image holding only the
# declared artifacts, restored root-owned (the submission folder, which nothing reads, and /var/lib/arb-online, the
# graded episode's record). verify.py runs no program: it reads /var/lib/arb-online/record.json, cross-checks it with
# online.json and scores the episode it records against anchor.json. Nothing of the agent home is on its import or
# configuration paths, and the stage sits in the root-only /opt/ale.
set -euo pipefail
export HOME=/root PYTHONNOUSERSITE=1 XDG_CONFIG_HOME=/root/.config XDG_CACHE_HOME=/root/.cache
exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py
