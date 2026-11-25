#!/usr/bin/env bash
# The reference solution: a hand-written controller (solver/) that looks under the sofa with the
# wrist camera, opens the drawers, lifts the seat pads and the hat and, when the remote is under
# the sofa, fetches the hanger and sweeps it out; it finds the remote in the camera images it
# asks for. Nothing is trained. It is the task author's controller: it knows the room (it
# carries the simulator, remote_env.py, and the room's meshes, which the agent is not given) but
# reads nothing of an episode except the observations on the wire.
# `ale validate` runs this in place of the agent, as the agent's account.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
mkdir -p "$SUB"

cp "$HERE"/solver/*.py "$SUB/"
# The controller plans with the room's model: its own copy of the simulator and the room's files.
cp "$HERE/remote_env.py" "$SUB/remote_env.py"
cp -r "$HERE/room" "$SUB/room"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 main.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] submission ready at $SUB" >&2
