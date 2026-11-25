#!/usr/bin/env bash
# How much of the 5-hour session is used and left, and of its 2-hour start window (your controller and your
# preparation: the graded episode must be started within it, and your controller is frozen at that start or when the
# window closes). The stamp was written when this sandbox was set up, moments before the session started and before
# the graded episode's service started, so the time left is slightly more than printed, never less. The graded
# episode's own window (3 hours from its start) is in its status (graded/client.py: graded_window_left_s).
#   bash /home/user/session_time.sh
set -u
LIMIT_MIN=300
PREP_MIN=120
STAMP=/run/session/start
if ! start=$(cat "$STAMP" 2>/dev/null) || ! [ "$start" -gt 0 ] 2>/dev/null; then
  echo "session_time: no stamp at $STAMP; note the time with 'date' yourself" >&2
  exit 1
fi
now=$(date -u +%s)
used=$(( (now - start) / 60 ))
left=$(( LIMIT_MIN - used ))
[ "$left" -lt 0 ] && left=0
prep_left=$(( PREP_MIN - used ))
[ "$prep_left" -lt 0 ] && prep_left=0
echo "session: ${used} min used, ${left} min left of ${LIMIT_MIN}; start window: ${prep_left} min left of ${PREP_MIN} (set up $(date -u -d "@$start" +%H:%M:%S) UTC, now $(date -u -d "@$now" +%H:%M:%S) UTC)"
