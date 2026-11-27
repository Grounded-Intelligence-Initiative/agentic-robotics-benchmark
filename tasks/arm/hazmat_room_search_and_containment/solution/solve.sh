#!/usr/bin/env bash
# The reference controller, run in place of the agent (same account, budget and network): it
# is copied into /home/user/submission/ beside the run.sh that starts it. Nothing is tuned to
# a lab or a package ahead of time; the controller finds out about each one over the wire.
# (The reference was developed against the live range, tests/lab_sim.py, which the agent
# never has: it is the measure of what a controller can do there, not of what can be written
# blind.)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUB="${ALE_HOME:-/home/user}/submission"
mkdir -p "$SUB"
cp "$HERE/controller.py" "$SUB/controller.py"
# the controller does its kinematics on the range's own model of the arm: the agent's copy
# of that file goes with it
cp "${ALE_HOME:-/home/user}/arm_model.py" "$SUB/arm_model.py"
cat > "$SUB/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec python3 controller.py
RUN
echo "[oracle] submission ready at $SUB" >&2
