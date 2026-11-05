#!/usr/bin/env bash
# install.sh [submission dir]: put the non-privileged reference controller (the task author's identifying
# controller, solver.py: it works from the 16-number observation alone, numpy from /opt/venv) into a submission
# folder, for a re-measurement on the hidden episodes. solve.sh does not use it; its run.sh is the one solve.sh wrote
# for it before the privileged reference existed.
set -euo pipefail

SUB="${1:-${ALE_HOME:-/home/user}/submission}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE/solver.py" "$SUB/solver.py"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 solver.py
SH
chmod +x "$SUB/run.sh"
echo "[reference] submission ready at $SUB" >&2
