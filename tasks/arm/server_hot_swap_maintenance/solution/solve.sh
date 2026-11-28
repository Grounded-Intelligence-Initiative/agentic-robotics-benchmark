#!/usr/bin/env bash
# Privileged reference solution: a scripted two-arm controller (oracle.py) that reads the token truth (the rack's
# pose, the failed parts, every caddy's, lever's and fan's state) and drives both arms with Cartesian steps computed
# from the observed tool poses over the grading wire (oracle_main.py). tests/verify.py adds `truth` to every
# observation only when the submission's .oracle_token hashes to tests/hidden/oracle_token.sha256. Per failed drive:
# the left arm presses the release button, grips the standing lever and draws the caddy up its rails; the right arm
# takes the caddy's body from the side before it clears the rails, the left lets go, the right lifts it out and drops
# it in the failed-parts bin; the left arm takes a spare from the holder, lowers it into the same slot, swings the
# lever down and presses it home until the latch catches. Then the fan: the left arm holds the spare fan beside the
# bay while the right pulls the failed one and clears; the left lowers the spare in and presses it home; the right
# bins the old fan. Both arms park and the reference says done. Runs as the agent account; reads nothing outside
# solution/.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp -r "$HERE"/oracle/. "$SUB/"
cp "$HERE/oracle_token" "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /opt/venv/bin/python3 -I oracle_main.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] submission ready at $SUB" >&2
