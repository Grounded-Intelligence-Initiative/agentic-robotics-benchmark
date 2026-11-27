#!/usr/bin/env bash
# The reference method, run in place of the agent (same account, budget and network):
# self-calibrate the swarm's hardware from the three flights (no ground truth), then leave the
# localizer and its calibration in /home/user/submission/.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUB=/home/user/submission
mkdir -p "$SUB"
cp "$HERE/oracle/common.py" "$HERE/oracle/localize.py" "$SUB/"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
(cd "$HERE/oracle" && python3 selfcal.py "$SUB/calib.json" /home/user/flights/flight_01 \
    /home/user/flights/flight_02 /home/user/flights/flight_03)
cat > "$SUB/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec python3 localize.py "$1" "$2"
RUN
