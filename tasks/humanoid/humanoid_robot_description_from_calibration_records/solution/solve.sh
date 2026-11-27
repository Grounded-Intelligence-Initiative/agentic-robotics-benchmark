#!/usr/bin/env bash
# Reference: the crated unit's own files (the model file corrected to the unit and its true calibration), the
# privileged reference of the task, run in place of the agent by `ale validate` as the same unprivileged account. It
# scores 1.0 by construction: every band of the grader is met far inside its full end by the unit's own description.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
SRC="$(dirname "$0")/answer"
mkdir -p "$SUB"
cp "$SRC/unit.xml" "$SRC/calibration.yaml" "$SUB/"
echo "[reference] the unit's description installed at $SUB" >&2
