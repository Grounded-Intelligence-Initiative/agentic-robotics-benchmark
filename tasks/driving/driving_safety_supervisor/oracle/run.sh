#!/usr/bin/env bash
# The reference supervisor, run in place of the agent by `ale validate`, as the same
# unprivileged account under the same limits. Nothing trains: it is the image's own
# detectors (COCO Faster R-CNN, OWLv2) with flat-road geometry, tracking and a passing
# manoeuvre. It leaves a self-contained
# /home/user/submission/ that the verify stage drives.
set -euo pipefail

SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$(dirname "$0")/solver.py" "$SUB/solver.py"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 solver.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] submission ready at $SUB" >&2
