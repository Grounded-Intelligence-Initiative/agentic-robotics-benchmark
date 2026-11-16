#!/usr/bin/env bash
# The oracle: copies a program that answers every request with the key supplied by the grader when the token is present.
# It exists to prove that the wire and the grader work end to end; it is not a method for the task.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$HERE/reference/oracle.py" "$SUB/oracle.py"
cp "$HERE/reference/oracle_token" "$SUB/.oracle_token"
chmod 600 "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 oracle.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready" >&2
