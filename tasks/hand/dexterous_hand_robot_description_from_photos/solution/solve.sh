#!/usr/bin/env bash
# Reference solution: the photographed unit's own description (robot.xml, its assets and config.yaml), copied into the
# submission folder by the same unprivileged account the agent uses. It scores 1.0 on every item.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
SRC="$(dirname "$0")/answer"
mkdir -p "$SUB"
cp -r "$SRC"/. "$SUB/"
echo "[solution] reference description installed at $SUB ($(ls "$SUB/assets" | wc -l) assets)" >&2
