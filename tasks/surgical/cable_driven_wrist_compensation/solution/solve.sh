#!/usr/bin/env bash
# The reference method, run in place of the agent (same account, budget and network): leave a
# program that, for each freshly mounted instrument, calibrates itself from the instrument's
# calibration recording (vision, then identification) and compensates the request. Nothing is
# fitted ahead of time: every hidden instrument is different.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUB=/home/user/submission
mkdir -p "$SUB"
cp "$HERE/oracle/"*.py "$SUB/"
# the instrument's meshes, from the datasheet's model folder in the agent home
cp -r /home/user/instrument_model "$SUB/instrument_model"
cat > "$SUB/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 run.py "$1" "$2"
RUN
