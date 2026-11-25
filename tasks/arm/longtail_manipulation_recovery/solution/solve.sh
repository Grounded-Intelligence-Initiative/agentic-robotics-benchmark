#!/usr/bin/env bash
# The reference controller, run in place of the agent by `ale validate`, as the same unprivileged account. It installs
# oracle_main.py, the recorded rescue action sequences and the 256-bit oracle token as .oracle_token; the grader compares
# the token's sha256 with tests/oracle_token.sha256 and, on a match, tells this controller which hidden state each
# episode is. Each sequence is the executed action list of a successful rescue rollout recorded on the authoring machine.
# Dual mode (experiment): both items, in this order (the controller must be in place before the graded start freezes
# it):
#   1. main's oracle, unchanged: the reference submission (oracle_main.py, the rescues and the token as .oracle_token)
#      in /home/user/submission;
#   2. the graded episode: play_graded.py starts it with the same token (the graded episode's service compares its
#      sha256 with the one it holds; at that start its root supervisor freezes the submission folder, which then holds
#      step 1's submission) and replays the state's recorded rescue on it, as oracle_main.py does.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
A="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB/rescues"
cp "$A/oracle_main.py" "$SUB/oracle_main.py"
cp "$A"/rescues/*.npz "$SUB/rescues/"
cp "$A/oracle_token" "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_CORETYPE=Haswell NUMBA_CPU_NAME=haswell
exec /opt/venv/bin/python3 -I oracle_main.py
RUN
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/venv/bin/python3 -I -B "$A/play_graded.py"
echo "[oracle] the graded episode is played and finished" >&2
