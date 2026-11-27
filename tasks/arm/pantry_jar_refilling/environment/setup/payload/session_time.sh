#!/usr/bin/env bash
# How much of the session is used and left: the 2-hour preparation window (start the graded episode within it; your
# controller is frozen at the start, or when the window closes) and the 5-hour session. The stamp was written when this
# sandbox was set up, a few minutes before the session started and just before the graded episode's service started,
# so the time left is slightly more than printed, never less. Once the graded episode has started, its own time left is
# in ~/graded/client.py's status() (graded_window_left_s).
#   bash /home/user/session_time.sh
set -u
PREP_MIN=120
LIMIT_MIN=300
STAMP=/run/session/start
if ! start=$(cat "$STAMP" 2>/dev/null) || ! [ "$start" -gt 0 ] 2>/dev/null; then
  echo "session_time: no stamp at $STAMP; note the time with 'date' yourself" >&2
  exit 1
fi
now=$(date -u +%s)
used=$(( (now - start) / 60 ))
prep_left=$(( PREP_MIN - used ))
[ "$prep_left" -lt 0 ] && prep_left=0
left=$(( LIMIT_MIN - used ))
[ "$left" -lt 0 ] && left=0
echo "session: ${used} min used; preparation window: ${prep_left} min left of ${PREP_MIN}; session: ${left} min left of ${LIMIT_MIN} (set up $(date -u -d "@$start" +%H:%M:%S) UTC, now $(date -u -d "@$now" +%H:%M:%S) UTC)"
