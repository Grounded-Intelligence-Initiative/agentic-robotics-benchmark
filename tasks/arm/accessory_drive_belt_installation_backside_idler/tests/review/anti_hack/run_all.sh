#!/usr/bin/env bash
# run every probe folder here through run_case.sh in the task image, one after the other (review tool)
#   run_all.sh IMAGE OUT_ROOT CPUS [probe ...]
set -u
IMAGE=$1; OUT=$2; CPUS=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd -P); TESTS=$(cd "$HERE/../.." && pwd -P)
# before the anchor exists (it is set from the oracle), probes run on a copy of tests/ with a provisional anchor
# (value 1.0, full_at 0.98): rewards are then relative to 1.0; the raw belt_acceptance is what the probes record
if [ ! -f "$TESTS/anchor.json" ]; then
  TMPT=$(mktemp -d); cp -r "$TESTS/." "$TMPT/"
  printf '{"direction": "higher", "metric": "belt_acceptance", "value": 1.0, "full_at": 0.98, "note": "provisional, probes only"}\n' > "$TMPT/anchor.json"
  TESTS=$TMPT
fi
probes=("$@"); [ ${#probes[@]} -eq 0 ] && probes=($(cd "$HERE" && ls -d */ | tr -d /))
for p in "${probes[@]}"; do
  echo "=== $p $(date -u +%FT%TZ)"
  t0=$(date +%s)
  SUBD="$HERE/$p"
  if [ -f "$HERE/$p/uses_oracle" ]; then
    # a probe built on the oracle: solution/honest/ with the probe's own files on top
    SUBD=$(mktemp -d); cp "$HERE/../../../solution/honest/"*.py "$SUBD/"; cp "$HERE/$p/"*.py "$SUBD/"
  fi
  bash "$HERE/run_case.sh" "$IMAGE" "$TESTS" "$SUBD" "$OUT/$p" "$CPUS" 2>&1 | tail -3
  echo "--- $p $(( $(date +%s) - t0 )) s"
done
