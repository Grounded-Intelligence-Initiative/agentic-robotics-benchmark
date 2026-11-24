#!/usr/bin/env bash
# The reference solution: a scripted program (reference/controller.py) that takes each pack apart to module level
# through the cell's public actions, behind the grading wire (reference/main.py, reference/run.sh). It is a
# privileged reference: it installs the 256-bit token as .oracle_token; the grader compares the token's sha256 with
# tests/hidden/oracle_token.sha256 and only on a match adds `truth` (where every part and fastener is, which head a
# fastener has) to every observation. With that the reference skips the perception a submission has to do, and
# nothing else: its motions, tool and bit exchanges, fasteners and parts go through the same actions, rules and
# physics as any program's, and what it decides while a fastener turns it decides from the nutrunner's readings.
# Runs as the agent account with the agent's timeouts and network policy; reads nothing outside solution/.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE"/reference/controller.py "$HERE"/reference/main.py "$HERE"/reference/cell.json "$HERE"/reference/run.sh "$SUB/"
cp "$HERE/reference/oracle_token" "$SUB/.oracle_token"
echo "[oracle] submission ready at $SUB" >&2
