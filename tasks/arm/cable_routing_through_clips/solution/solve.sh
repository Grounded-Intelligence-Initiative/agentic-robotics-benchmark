#!/usr/bin/env bash
# Reference solution: installs the reference router (router.py) as the submission's controller.
# `ale validate` runs this as the agent account in place of the agent; the grader then drives
# /home/user/submission/run.sh over the closed-loop wire on hidden episodes.
set -euo pipefail

SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$(dirname "$0")/router.py" "$SUB/router.py"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 router.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
