#!/usr/bin/env bash
set -euo pipefail
source_dir=$(cd "$(dirname "$0")" && pwd)
submission="${ALE_HOME:-/home/user}/submission"
mkdir -p "$submission"
cp "$source_dir"/{solver.py,route.py,temporal.py,gait.py,vision.py,perception.py} "$submission/"
cat > "$submission/run.sh" <<'LAUNCH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="/opt/sonic${PYTHONPATH:+:$PYTHONPATH}"
exec "${SONIC_PYTHON:-/opt/venv/bin/python3}" -B solver.py
LAUNCH
chmod 755 "$submission/run.sh"
