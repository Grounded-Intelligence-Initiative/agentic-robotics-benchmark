#!/usr/bin/env bash
set -euo pipefail
source_dir=$(cd "$(dirname "$0")" && pwd)
submission="${ALE_HOME:-/home/user}/submission"
mkdir -p "$submission"
cp "$source_dir/mobile_policy.py" "$submission/"
cp "$source_dir"/../{v4.py,v4_actor.onnx,solver.py} "$submission/"
cat > "$submission/run.sh" <<'INNER'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "${SONIC_PYTHON:-/opt/venv/bin/python3}" -B solver.py
INNER
chmod 755 "$submission/run.sh"
