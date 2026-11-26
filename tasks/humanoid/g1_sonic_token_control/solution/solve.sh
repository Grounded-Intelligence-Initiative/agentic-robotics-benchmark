#!/usr/bin/env bash
set -euo pipefail
source_dir=$(cd "$(dirname "$0")" && pwd)
submission="${ALE_HOME:-/home/user}/submission"
policy="${SONIC_REFERENCE_POLICY:-v4}"
case "$policy" in
    v4|p1) ;;
    *) echo "Expected SONIC_REFERENCE_POLICY=v4 or p1" >&2; exit 2 ;;
esac
mkdir -p "$submission"
cp "$source_dir/solver.py" "$submission/solver.py"
if [[ "$policy" == v4 ]]; then
    cp "$source_dir/v4.py" "$source_dir/v4_actor.onnx" "$submission/"
else
    mkdir -p "$submission/p1"
    cp "$source_dir"/p1/*.py "$submission/p1/"
fi
cat > "$submission/run.sh" <<SH_INNER
#!/usr/bin/env bash
set -euo pipefail
cd "\$(dirname "\$0")"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="/opt/sonic\${PYTHONPATH:+:\$PYTHONPATH}"
exec "\${SONIC_PYTHON:-/opt/venv/bin/python3}" -B solver.py --policy "$policy"
SH_INNER
chmod 755 "$submission/run.sh"
