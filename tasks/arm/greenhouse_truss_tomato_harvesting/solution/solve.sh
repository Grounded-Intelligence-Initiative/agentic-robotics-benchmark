#!/usr/bin/env bash
# The reference solution: a scripted program (reference/controller.py) that harvests every ready truss of a row
# section through the cell's public actions, behind the grading wire (reference/main.py, reference/run.sh). It is a
# privileged reference: it installs the 256-bit token as .oracle_token; the grader compares the token's sha256 with
# tests/hidden/oracle_token.sha256 and only on a match adds `truth` (where every stem, leaf, peduncle and fruit is,
# which trusses are ready, each peduncle's cutting force, the cart's true place) to every observation. With that the
# reference skips the perception and identification a submission has to do; its motions, cuts and the trusses it lays
# in the crates go through the same actions, rules and physics as any program's.
# Runs as the agent account with the agent's timeouts and network policy; reads nothing outside solution/.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE"/reference/controller.py "$HERE"/reference/main.py "$HERE"/reference/site.json "$HERE"/reference/run.sh "$SUB/"
cp "$HERE/reference/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
