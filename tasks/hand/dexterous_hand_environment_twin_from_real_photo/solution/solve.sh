#!/usr/bin/env bash
# Reference solution: the privileged reference twin of the photographed board as a submission (scene.xml with the
# support box and the ten things' convex collision pieces at their true poses, objects.json), run in place of the
# agent by `ale validate` as the same unprivileged account. Collision geometry only: the grader ignores appearance.
# The task's request classes are computed on this twin, so it scores 1.0 by construction.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
mkdir -p "$SUB"
cp -r "$(dirname "$0")/answer/." "$SUB/"
echo "[solution] submission ready at $SUB" >&2
