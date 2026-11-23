#!/usr/bin/env bash
# Reference solution: installs the reference calibrator (calibrate.py) as the submission.
# `ale validate` runs this as the agent account in place of the agent; the verify stage then runs
# /home/user/submission/run.sh on the hidden rigs. Nothing here reads a rig: the calibrator works
# out every parameter from the logs it is given at grading time.
set -euo pipefail

SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$(dirname "$0")/calibrate.py" "$SUB/calibrate.py"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 calibrate.py "$@"
SH
chmod +x "$SUB/run.sh"

echo "[oracle] submission ready at $SUB" >&2
