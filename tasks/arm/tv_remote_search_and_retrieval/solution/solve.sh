#!/usr/bin/env bash
# Privileged reference solution: a hand-written controller (solver/) driven by the grader's truth.
# tests/verify.py adds `truth` to every observation only when the submission's .oracle_token hashes to
# tests/hidden/oracle_token.sha256. From it the reference takes where the furniture and the basket stand and
# which place the remote is in (solver/oracle.py); it then does what that place takes with the skills of
# solver/search_skills.py (lift the seat pad, fetch the hanger and sweep under the settee, reach in under the
# chest, open the drawer and take the hat out, pour the basket out) and finds the remote itself in the camera
# images it asks for. Nothing is trained. `ale validate` runs this in place of the agent, as the agent's
# account; it reads nothing outside solution/ and the agent's home.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
mkdir -p "$SUB"

cp "$HERE"/solver/*.py "$SUB/"
cp "$HERE/oracle_token" "$SUB/.oracle_token"
# The controller plans with the room's model: the environment file the agent was given.
cp "$HOME_DIR/remote_env.py" "$SUB/remote_env.py"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -B main.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] submission ready at $SUB" >&2
