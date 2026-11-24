#!/usr/bin/env bash
# How much of the 7-hour session is used and left, and of its first 4 hours (the start window: the graded episode
# must be started within it, and the program in ~/submission is frozen at the graded start or when the window closes).
# The stamp was written when this sandbox was set up, moments before the session started and before the graded
# episode's service started its start window, so the time left is slightly more than printed, never less.
#   bash /home/user/session_time.sh
set -u
LIMIT_MIN=420
START_WINDOW_MIN=240
STAMP=/run/session/start
if ! start=$(cat "$STAMP" 2>/dev/null) || ! [ "$start" -gt 0 ] 2>/dev/null; then
  echo "session_time: no stamp at $STAMP; note the time with 'date' yourself" >&2
  exit 1
fi
now=$(date -u +%s)
used=$(( (now - start) / 60 ))
left=$(( LIMIT_MIN - used ))
[ "$left" -lt 0 ] && left=0
wleft=$(( START_WINDOW_MIN - used ))
[ "$wleft" -lt 0 ] && wleft=0
echo "session: ${used} min used, ${left} min left of ${LIMIT_MIN} (set up $(date -u -d "@$start" +%H:%M:%S) UTC, now $(date -u -d "@$now" +%H:%M:%S) UTC)"
echo "start window: ${wleft} min left of ${START_WINDOW_MIN} (start the graded episode within it; your program freezes at the start, or when it closes)"
