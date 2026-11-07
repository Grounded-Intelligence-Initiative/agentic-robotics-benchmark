#!/usr/bin/env bash
# The reference detector, run in place of the agent by `ale validate` as the same unprivileged
# account. It is grasplib's candidate generator and a gradient-boosted scorer (scorer.pkl,
# scikit-learn 1.6.1, fitted offline on evaluator labels from the practice-scene meshes plus
# labels transferred from the annotated objects). Because it also carries the oracle token, the
# grader hands it the ground truth (object poses and meshes) of the hidden frames, and it
# re-ranks its best candidates by the true force-closure label. It shows that the task can be
# done; it is not a detector an agent could build from what it observes.
set -euo pipefail

SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE/solver.py" "$HERE/grasplib.py" "$HERE/scorer.pkl" "$HERE/oracle_token" "$SUB/"
echo "[oracle] submission ready at $SUB" >&2
