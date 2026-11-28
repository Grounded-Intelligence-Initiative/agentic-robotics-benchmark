#!/usr/bin/env bash
# The reference: a privileged controller. It never looks at a camera; it asks the cell for ground truth with the oracle
# token, which exists only in solution/ (absent while an agent works) and is checked by hash on the grader's side.
set -euo pipefail
SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"
cp "$HERE/privileged/oracle.py" "$HERE/privileged/kin.py" "$HERE/privileged/reference_main.py" "$HERE/privileged/reference_params.json" "$HERE/privileged/r1pro.urdf" "$SUB/"
cp "$HERE/privileged/oracle_token" "$SUB/.oracle_token"
cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "${REFERENCE_PYTHON:-/opt/venv/bin/python3}" -B reference_main.py
SH
chmod +x "$SUB/run.sh"
echo "[oracle] privileged submission ready at $SUB" >&2
