#!/usr/bin/env bash
set -euo pipefail
mkdir -p "${ALE_HOME:-/home/user}/submission"
cp "$(dirname "$0")/controller.py" "${ALE_HOME:-/home/user}/submission/controller.py"
cp "$(dirname "$0")/submission_run.sh" "${ALE_HOME:-/home/user}/submission/run.sh"
chown -R "$(basename "${ALE_HOME:-/home/user}")":"$(basename "${ALE_HOME:-/home/user}")" "${ALE_HOME:-/home/user}/submission"
