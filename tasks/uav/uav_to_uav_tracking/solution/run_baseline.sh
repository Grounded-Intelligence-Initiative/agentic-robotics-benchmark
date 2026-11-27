#!/usr/bin/env bash
set -euo pipefail
TASK_SOLUTION="$(cd "$(dirname "$0")" && pwd)"
TASK_OUTPUT="${ALE_HOME:-/home/user}/submission"
mkdir -p "$TASK_OUTPUT"
cp "$TASK_SOLUTION/model.py" "$TASK_SOLUTION/train.py" "$TASK_SOLUTION/train.sh" "$TASK_SOLUTION/export_checkpoint.py" "$TASK_OUTPUT/"
cd "$TASK_OUTPUT"
bash train.sh
