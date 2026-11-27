#!/usr/bin/env bash
set -euo pipefail
# Backward-compatible author entry point; solve.sh is the learned reference.
TASK_SOLUTION="$(cd "$(dirname "$0")" && pwd)"
exec bash "$TASK_SOLUTION/solve.sh"
