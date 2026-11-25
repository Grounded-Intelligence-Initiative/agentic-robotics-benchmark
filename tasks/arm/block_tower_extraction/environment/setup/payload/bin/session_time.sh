#!/usr/bin/env bash
# The agent session's clock: elapsed and remaining wall seconds of your 4 h session (14 400 s), from the timestamp the
# task's setup wrote (root-owned) when your session began.
start="$(cat /run/towercell/session_start 2>/dev/null || true)"
[ -n "$start" ] || { echo "session start unknown (/run/towercell/session_start missing)"; exit 1; }
now="$(date +%s)"
el=$((now - start)); left=$((14400 - el))
echo "session elapsed ${el} s, remaining ${left} s (of 14400 s)"
