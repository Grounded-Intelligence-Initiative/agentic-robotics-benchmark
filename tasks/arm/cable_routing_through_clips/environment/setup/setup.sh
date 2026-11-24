#!/usr/bin/env bash
# Setup: the engine uploads all of environment/setup/ to /opt/ale/setup once the sandbox is up and runs
# `bash setup.sh` there as trusted root (network open), before the agent's session. setup_runtime.py starts the
# practice cell from a root-only copy of runtime/ and stages payload/ into the agent's home.
set -euo pipefail
cd "$(dirname "$0")"
exec /usr/bin/env -i PATH=/opt/venv/bin:/usr/bin:/bin HOME=/root LANG=C.UTF-8 /opt/venv/bin/python3 -E -s setup_runtime.py
