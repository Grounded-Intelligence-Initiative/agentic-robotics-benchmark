# block_tower_extraction's environment image. `environment/` (less its `setup/`) is the sole build context and
# the final stage derives DIRECTLY from the official ALE base, at an exact version. Nothing hidden is here: the givens
# are staged by environment/setup/setup.sh; the grader, the anchor, the hidden towers and the reference program live
# in tests/ and solution/ (absent from the image). The image does hold the commissioning service (the cell on its
# practice towers), readable by its own account only.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

# OSMesa for the cameras (MuJoCo's classic renderer on llvmpipe: no GPU in the task's resources) and the GL client
# libraries MuJoCo links; git for the robot model's pinned fetch.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libosmesa6 libgl1 libglu1-mesa libglew2.2 libxrender1 libxext6 libsm6 libxi6 libgomp1 git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# World-readable venv at /opt/venv (setup as root, the agent `user`, the oracle `user`, the commissioning service
# `towercell`, the programs under test `btrun1`..`btrun4`, verify as root). UV_PYTHON_INSTALL_DIR must be traversable by every one.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN export HOME=/root UV_CACHE_DIR=/tmp/uv-cache \
    && uv venv /opt/venv --python 3.12 \
    && uv pip install --no-cache --python /opt/venv -r /tmp/requirements.txt \
    && find /opt/venv -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; } \
    && rm -rf /tmp/uv-cache /tmp/requirements.txt
RUN [ ! -e /opt/venv/.lock ] || chmod 0644 /opt/venv/.lock

# Numerics (contributing-a-task.md section 5; the pantry_jar_refilling pin): OpenBLAS's Haswell kernels with the L2
# guard of numpy 2.5.2-2.5.5, one BLAS / OpenMP thread, NumPy's AVX-512 loops off, OpenCV's IPP on AVX2. A .pth sets
# each as a default for every /opt/venv Python, the program under test (started with a minimal environment) included;
# the build fails if NumPy does not know a disabled feature's name. tests/numerics_check.py proves it at verify.
ENV OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256
RUN printf '%s\n' \
      'import os' \
      'os.environ.setdefault("OPENBLAS_CORETYPE", "Haswell")' \
      'os.environ.setdefault("OPENBLAS_L2_SIZE", "256")' \
      'os.environ.setdefault("NPY_DISABLE_CPU_FEATURES", "X86_V4 AVX512_ICL AVX512_SPR")' \
      'os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")' \
      'os.environ.setdefault("OMP_NUM_THREADS", "1")' \
      'os.environ.setdefault("OPENCV_IPP", "avx2")' \
      > /opt/venv/lib/python3.12/site-packages/bt_numerics_pin.py \
    && printf '%s\n' 'import bt_numerics_pin' > /opt/venv/lib/python3.12/site-packages/bt_numerics_pin.pth \
    && chmod 0644 /opt/venv/lib/python3.12/site-packages/bt_numerics_pin.py /opt/venv/lib/python3.12/site-packages/bt_numerics_pin.pth \
    && env -i PATH=/opt/venv/bin:/usr/bin:/bin HOME=/tmp /opt/venv/bin/python3 -I -W error::ImportWarning -c "import os; e = os.environ; assert (e.get('OPENBLAS_CORETYPE'), e.get('OPENBLAS_L2_SIZE'), e.get('OPENBLAS_NUM_THREADS'), e.get('OMP_NUM_THREADS'), e.get('OPENCV_IPP')) == ('Haswell', '256', '1', '1', 'avx2'), dict(e); import numpy; from numpy._core._multiarray_umath import __cpu_dispatch__ as D, __cpu_features__ as F; assert set(e['NPY_DISABLE_CPU_FEATURES'].split()) == set(D) - {'X86_V3'}, D; print('[image] numerics pin OK', D)"

# The robot model: MuJoCo Menagerie's Franka Emika Panda (folder franka_emika_panda, Apache-2.0, LICENSE inside),
# fetched at the commit pinned in panda_assets.sha256 and checked file by file; obj2stl.py turns the visual OBJ meshes
# into binary STL and the result is checked against panda_converted.sha256 (the pump_skid_startup_and_operation
# procedure, byte-identical lists). setup.sh stages it into the agent home (~/given/robot/panda), tests/test.sh into
# the grader's root-owned copy; the commissioning service reads it here.
COPY panda_assets.sha256 panda_converted.sha256 obj2stl.py /opt/task-assets/build/
RUN set -eu; \
    build=/opt/task-assets/build; \
    list="$build/panda_assets.sha256"; \
    repo="$(sed -n 's/^# repo: *//p' "$list")"; \
    commit="$(sed -n 's/^# commit: *//p' "$list")"; \
    folder="$(sed -n 's/^# folder: *//p' "$list")"; \
    test -n "$repo"; test -n "$commit"; test -n "$folder"; \
    work="$(mktemp -d)"; \
    git -C "$work" init -q; \
    git -C "$work" remote add origin "$repo"; \
    git -C "$work" config core.sparseCheckout true; \
    echo "/$folder/" > "$work/.git/info/sparse-checkout"; \
    for try in 1 2 3; do \
        if git -C "$work" fetch -q --depth 1 --filter=blob:none origin "$commit" \
           && git -C "$work" -c advice.detachedHead=false checkout -q FETCH_HEAD; then break; fi; \
        if [ "$try" = 3 ]; then echo "[image] fetching $repo@$commit failed" >&2; exit 1; fi; \
        sleep 15; \
    done; \
    test "$(git -C "$work" rev-parse HEAD)" = "$commit"; \
    cd "$work/$folder"; \
    grep -v '^#' "$list" | sha256sum --check --strict --quiet; \
    test "$(find . -type f | wc -l)" -eq "$(grep -vc '^#' "$list")"; \
    cd /; \
    rm -rf /opt/task-assets/panda; \
    HOME=/root /opt/venv/bin/python3 -I -B "$build/obj2stl.py" "$work/$folder" /opt/task-assets/panda; \
    rm -rf "$work"; \
    cd /opt/task-assets/panda; \
    grep -v '^#' "$build/panda_converted.sha256" | sha256sum --check --strict --quiet; \
    test "$(find . -type f | wc -l)" -eq "$(grep -vc '^#' "$build/panda_converted.sha256")"; \
    cd /; \
    rm -rf "$build"; \
    chown -R root:root /opt/task-assets; \
    chmod -R u+rwX,go+rX,go-w /opt/task-assets; \
    echo "[image] $folder @ $commit: converted, $(find /opt/task-assets/panda -type f | wc -l) files verified"

# The declared artifact path must exist even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# The accounts the grader runs the hidden towers' programs under (one per tower, so the towers run in parallel and
# stopping, killing or sweeping one run never touches another): no home, no shell, no password.
RUN for i in 1 2 3 4; do useradd --system --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin btrun$i; done

# The commissioning service (the cell on its practice towers: commission_service/, its towercell/ and cellkit/ byte
# copies of the grader's), for its own account only: mode 700, so the agent account can neither read nor list it.
RUN useradd --system --no-create-home --home-dir /opt/towercell-commission --shell /usr/sbin/nologin towercell
COPY commission_service /opt/towercell-commission
RUN find /opt/towercell-commission -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && rm -rf /opt/towercell-commission/state && mkdir -p /opt/towercell-commission/state \
    && chown -R towercell:towercell /opt/towercell-commission \
    && chmod -R u+rwX,go-rwx /opt/towercell-commission

ENV MPLBACKEND=Agg MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LP_NUM_THREADS=4
WORKDIR /

# Proofs: the toolchain; an OSMesa render of the robot model; the AGENT account can use the venv; the service's
# account builds a development cell and renders a wrist frame; the agent account can neither list nor read the
# service; nothing in the agent home is root-owned; no world-writable regular file.
RUN export HOME=/root \
    && cd /tmp \
    && python3 -c "import sys; assert sys.version_info >= (3, 12), sys.version; import mujoco, numpy, scipy, PIL, cv2; print('[bt] import OK mujoco', mujoco.__version__, 'numpy', numpy.__version__)" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('/opt/task-assets/panda/panda.xml'); r = mujoco.Renderer(m, 64, 64); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d); px = r.render(); print('[bt] OSMesa render OK', px.shape, int(px.max()))" \
    && runuser -u user -- python3 -c "import mujoco, numpy, scipy, PIL, cv2; m = mujoco.MjModel.from_xml_path('/opt/task-assets/panda/panda_nohand.xml'); print('[bt] agent-UID import OK, panda_nohand nbody', m.nbody)" \
    && runuser -u btrun1 -- python3 -c "import mujoco, numpy; print('[bt] runner-UID import OK')" \
    && runuser -u towercell -- env HOME=/opt/towercell-commission BT_PANDA_DIR=/opt/task-assets/panda /opt/venv/bin/python3 -B -s -c "import sys, json; sys.path.insert(0, '/opt/towercell-commission'); from towercell.server import CellServer; from towercell.tower import draw_tower; d = json.load(open('/opt/towercell-commission/sites.json')); s = CellServer(draw_tower(d['sites'][0]['seed']), duration_s=60.0); st = s.handle('step', {'n': 5}); f = s.handle('get_camera', {'name': 'wrist', 'depth': True}); assert 'error' not in f, f; print('[bt] commissioning cell OK for its account: t', st['t'], 'frame', f['intrinsics']['width'])" \
    && ! runuser -u user -- ls /opt/towercell-commission > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/towercell-commission/sites.json > /dev/null 2>&1 \
    && echo "[bt] the agent account can neither list nor read the commissioning service" \
    && test -z "$(find /home/user -user root -print -quit)" \
    && ! runuser -u user -- test -w /opt/venv/.lock \
    && test -z "$(find / -xdev -type f -perm -0002 -print -quit)" \
    && echo "[bt] no root-owned entry in the agent home, no world-writable regular file in the image"
