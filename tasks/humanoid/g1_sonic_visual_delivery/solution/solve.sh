#!/usr/bin/env bash
set -euo pipefail
src=$(cd "$(dirname "$0")" && pwd)
dest="${ALE_HOME:-/home/user}/submission"
mkdir -p "$dest"
cp "$src"/{solver.py,camera_input.py,visual_body.py,delivery_policy.py,stand.py,approach_codes.npy,cartesian_fsq.npz,optical_scene.py,optical_servo.py,direct_token_control.py} "$dest/"
cat > "$dest/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "${SONIC_PYTHON:-/opt/venv/bin/python3}" -B -u solver.py
RUN
chmod 755 "$dest/run.sh"
