#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 independent.py --oracle "${ALE_HOME:-/home/user}/submission/submission.json"
