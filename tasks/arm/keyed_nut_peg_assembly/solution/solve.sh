#!/usr/bin/env bash
# Privileged reference solution: a scripted controller that reads the ground-truth scene state
# (nut pose and keyway angle, peg poses). The grader attaches that state to the observations only
# for a submission carrying solution/oracle_token, whose SHA-256 alone is staged with the grader;
# the agent never sees the token. The solution only shows that every hidden episode can be done
# with the public robot, controller and step budget. Runs as the agent account; reads nothing
# outside solution/.
#
# Dual mode (experiment, never merged into main), for both items in this order (the program must be in place before
# the graded start freezes it): 1. main's oracle above writes the reference controller and its token into the
# submission; 2. solver/play_graded.py starts the graded episode with the same token (the graded episode's service
# compares its sha256 with the one it holds; at that start it freezes the submission folder, which then holds step 1's
# controller), plays it with the same reference controller through ~/commission/client.py and finishes it.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE"/solver/oracle_ctrl.py "$HERE"/solver/oracle_main.py "$SUB/"
cp "$HERE/oracle_token" "$SUB/oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /opt/venv/bin/python3 -I oracle_main.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
/opt/venv/bin/python3 -B "$HERE/solver/play_graded.py"
echo "[oracle] the graded episode is played and finished" >&2
