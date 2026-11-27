# humanoid_robot_description_from_calibration_records's environment image (robot-description stage: a lab engineer
# writes the model file and the calibration file of one humanoid unit as crated from the unit's calibration records,
# the maker's design files and documents and the lab's notes).
#
# `environment/` (less its `setup/`) is the sole build context and the final stage derives DIRECTLY from the official
# ALE base, at an exact version. Layers: the X / GL client libraries the CAD kernel links even when headless (the line
# of the benchmark's other robot images, so it comes from their layer cache), a world-readable venv at /opt/venv from
# requirements.lock.txt (every wheel pinned by sha256, binary wheels only, no dependency resolution at build time),
# the numerics pin, the build-time checks, the idle-timeout block and the task assets. The engineer's tools: NumPy,
# SciPy, the physics engine (headless: no display and no GL context is needed to compile, step or query a model),
# trimesh with its spatial index (distances to meshes), FCL (collision and distance queries), the CAD kernel that reads
# STEP files, PyYAML, jsonschema, matplotlib and Pillow. No network at run time (task.toml). No hidden data: the
# givens are staged by environment/setup/setup.sh, the grader lives in tests/ (absent from the image), and the unit's
# ground truth never enters it.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# The X / GL client libraries the CAD kernel and the visualisation toolkit it requires link even when headless; OSMesa
# for an offscreen render the engineer may choose (MUJOCO_GL=osmesa; nothing in this task needs one); libgomp.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libosmesa6 libgl1 libglu1-mesa libglew2.2 libxrender1 libxext6 libsm6 libxi6 \
        libxcursor1 libxinerama1 libxrandr2 libfontconfig1 libfreetype6 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# World-readable venv at /opt/venv on the base's CPython 3.12: setup (root), the agent (`user`), the reference
# (`user`) and the grader (root) all use the same interpreter. The engine stages its verification package into the
# `python3` it finds and refuses anything below 3.12. pip installs exactly the locked wheels (--require-hashes,
# --no-deps, binary only) and `pip check` proves the set complete.
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.lock.txt /tmp/requirements.lock.txt
RUN python3 -m venv /opt/venv \
    && PIP_CONFIG_FILE=/dev/null /opt/venv/bin/python -m pip install --no-cache-dir --disable-pip-version-check \
           --no-deps --only-binary=:all: --require-hashes -r /tmp/requirements.lock.txt \
    && PIP_CONFIG_FILE=/dev/null /opt/venv/bin/python -m pip check --disable-pip-version-check \
    && find /opt/venv -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && chmod -R a+rX /opt/venv \
    && rm -f /tmp/requirements.lock.txt

# The declared artifact path must exist even when nobody works (the engine captures declared artifacts after every
# episode, untouched validation passes included).
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# The same numerics on every x86-64 CPU: OpenBLAS otherwise picks its kernels per CPU (SkylakeX on a host with 512-bit
# vector units, Haswell on others) and NumPy its SIMD paths, so a matrix product differs in the last bits between
# hosts. OPENBLAS_L2_SIZE=256 keeps the pinned kernels' own block sizes: without it NumPy's bundled OpenBLAS enlarges
# them on AMD Zen 4 / 5 hosts and its Haswell float64 kernel overruns a fixed buffer. tests/test.sh exports the same
# three before the grader's interpreter starts. MPLBACKEND: plots go to files (no display).
ENV OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"
ENV MPLBACKEND=Agg

# Fail the build early if the toolchain is broken and prove the AGENT account can use it: every package imports, a
# model compiles and steps with no display and no GL context, the CAD kernel's STEP reader and a trimesh / FCL distance
# query answer. One import line per -c: a continuation line starting with that keyword would read as a stage line to
# the final-stage lint.
RUN python3 -c "import numpy, scipy, mujoco, trimesh, rtree, fcl, yaml, jsonschema, OCP, vtk, matplotlib, PIL; print('[task-image] import OK numpy', numpy.__version__, 'scipy', scipy.__version__, 'mujoco', mujoco.__version__, 'trimesh', trimesh.__version__)" \
    && runuser -u user -- python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type=\"box\" size=\"0.1 0.1 0.1\"/></body></worldbody></mujoco>'); d = mujoco.MjData(m); mujoco.mj_step(m, d); print('[task-image] agent-UID mujoco step OK', m.nbody)" \
    && runuser -u user -- python3 -c "import trimesh, fcl, numpy; a = trimesh.creation.box((1, 1, 1)); p = trimesh.proximity.closest_point(a, numpy.array([[2.0, 0.0, 0.0]]))[1]; r = fcl.distance(fcl.CollisionObject(fcl.Box(1, 1, 1)), fcl.CollisionObject(fcl.Box(1, 1, 1), fcl.Transform(numpy.array([3.0, 0.0, 0.0]))), fcl.DistanceRequest(), fcl.DistanceResult()); print('[task-image] agent-UID trimesh', float(p[0]), 'fcl', r)" \
    && runuser -u user -- python3 -c "import OCP.STEPControl as S; print('[task-image] agent-UID STEP reader OK', S.STEPControl_Reader.__name__)"

# The claude-code harness aborts a model stream after 300 s without a byte; both timers are raised to the gateway's
# 1800 s upstream ceiling. ENV covers processes the guest service starts, profile.d the CLI's login shell. Nothing else
# in this image reads them.
ENV BUN_CONFIG_HTTP_IDLE_TIMEOUT=1800 CLAUDE_STREAM_IDLE_TIMEOUT_MS=1800000
RUN printf '%s\n' 'export BUN_CONFIG_HTTP_IDLE_TIMEOUT=1800' 'export CLAUDE_STREAM_IDLE_TIMEOUT_MS=1800000' \
        > /etc/profile.d/99-ale-idle-timeouts.sh \
    && chmod 0644 /etc/profile.d/99-ale-idle-timeouts.sh

# ---- task assets ---------------------------------------------------------------------------------------------------
# The large givens (the maker's design files, the CAD export, the procedure's figures). assets.sha256 pins every file
# by sha256. Two sources, one check:
#   * the build context's assets/payload/ when it exists (the task's local asset folder, used while the task is
#     developed): exactly the listed files are copied, an unlisted or missing file fails the build, and every copied
#     file is checked against its sha256;
#   * otherwise the benchmark's Hugging Face asset repository at the commit pinned in assets.sha256 (fetch_assets.py,
#     the benchmark's canonical copy in shared/task_assets/; it checks every file against its sha256 as it downloads).
# environment/setup/setup.sh stages them into the agent home next to environment/setup/payload/; the scoring stage
# reads the given model's file and meshes from this image's copy, checked against its own pinned hashes.
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN --mount=type=bind,target=/tmp/build-context \
    set -eu; \
    list=/opt/task-assets/assets.sha256; dest=/opt/task-assets/payload; src=/tmp/build-context/assets/payload; \
    if [ -d "$src" ]; then \
        grep -v -e '^#' -e '^[[:space:]]*$' "$list" > /tmp/assets.list; \
        sed -e 's/^[0-9a-fA-F]\{64\}  \*\{0,1\}//' /tmp/assets.list | LC_ALL=C sort > /tmp/assets.want; \
        (cd "$src" && find . -type f | sed -e 's#^\./##' | LC_ALL=C sort) > /tmp/assets.have; \
        extra="$(LC_ALL=C comm -13 /tmp/assets.want /tmp/assets.have)"; \
        missing="$(LC_ALL=C comm -23 /tmp/assets.want /tmp/assets.have)"; \
        if [ -n "$extra" ]; then echo "[assets] files in assets/payload/ that assets.sha256 does not list:" $extra >&2; exit 1; fi; \
        if [ -n "$missing" ]; then echo "[assets] files assets.sha256 lists that assets/payload/ lacks:" $missing >&2; exit 1; fi; \
        mkdir -p "$dest"; \
        (cd "$src" && tar -cf - -T /tmp/assets.want) | (cd "$dest" && tar -xf - --no-same-owner); \
        (cd "$dest" && sha256sum --check --strict --quiet /tmp/assets.list); \
        echo "[assets] $(wc -l < /tmp/assets.list) file(s) from the build context's assets/payload/ verified into $dest"; \
        rm -f /tmp/assets.list /tmp/assets.want /tmp/assets.have; \
    else \
        python3 /opt/task-assets/fetch_assets.py "$list" "$dest"; \
    fi; \
    chmod -R a+rX,go-w /opt/task-assets

# Nothing a program writes may reach the next scored episode: the build proves that no regular file of the image is
# world-writable. Appended last so the layers above stay cached.
RUN ww="$(find / -xdev -type f -perm -0002 -print)"; if [ -n "$ww" ]; then echo "[image] world-writable regular files:" $ww >&2; exit 1; fi; echo "[image] no world-writable regular file in the image"
