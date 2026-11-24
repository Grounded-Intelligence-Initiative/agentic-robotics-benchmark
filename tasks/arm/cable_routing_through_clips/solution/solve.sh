#!/usr/bin/env bash
# Privileged reference solution. It is NOT a perception system: tests/verify.py adds the cell's true state
# (`obs["privileged"]`) to the observations of a submission whose .oracle_token hashes to
# tests/hidden/oracle_token.sha256, and of no other; router.py routes the cable and mates the plug from that state.
# It shows that the hidden episodes can be completed through the wire, and its measured credit is the anchor.
# Runs as the agent account with the agent's limits.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE/router.py" "$SUB/router.py"
cp "$HERE/oracle_token" "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 router.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
