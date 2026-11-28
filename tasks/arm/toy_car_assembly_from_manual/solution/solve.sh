#!/usr/bin/env bash
# Reference solution: a controller that reads each episode's picture manual when it arrives with the observation at
# t = 0 (read_manual.py: the ruler gives the scale, the orthographic views the sizes), finds and localises every part
# and the stand with the wrist camera, and builds that episode's car. It reads nothing hidden: no part pose, no car
# numbers, nothing from the grader. Runs as the agent account; leaves a self-contained submission.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE/controller.py" "$HERE/read_manual.py" "$SUB/"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /opt/venv/bin/python3 -I controller.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
