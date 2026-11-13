# Verification image of longtail_manipulation_recovery (context tests/). The same stack as the agent's image plus what
# the agent's image never holds: the graded kitchens of RoboCasa's held-out split and their object instances.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1@sha256:f891988f27075f9b19b10c1630956513554f3b5de290ae99ee4a69db59262c63

COPY --from=ghcr.io/astral-sh/uv:0.11.33@sha256:77280f2f771df71f90786c314fe1bbc1e023feac652969bbf139c280babf2eb7 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1

COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12.14 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# RoboCasa 1.0.1 and robosuite 1.5.2, the pinned commits the recordings were made with (MIT).
RUN mkdir -p /opt/sim /tmp/src \
    && curl -fsSL https://github.com/ARISE-Initiative/robosuite/archive/5ce6643f3092639d08f7b0f90ed1c6a84f50552c.tar.gz | tar -xz -C /tmp/src \
    && curl -fsSL https://github.com/robocasa/robocasa/archive/456174f62b89b8fca99eaaf33949c29fec9cfc2a.tar.gz | tar -xz -C /tmp/src \
    && mv /tmp/src/robosuite-* /opt/sim/robosuite && mv /tmp/src/robocasa-* /opt/sim/robocasa && rm -rf /tmp/src \
    && uv pip install --python /opt/venv --no-deps -e /opt/sim/robosuite -e /opt/sim/robocasa \
    && chmod -R a+rX /opt/sim

# Software rendering (Mesa OSMesa, llvmpipe) and libraries MuJoCo and OpenCV need.
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 libgl1 libegl1 libgl1-mesa-dri libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    OPENBLAS_CORETYPE=Haswell NUMBA_CPU_NAME=haswell NUMBA_CPU_FEATURES=+avx2,+fma,+sse4.2 LP_NUM_THREADS=4 PYTHONHASHSEED=0 \
    NPY_DISABLE_CPU_FEATURES="AVX512F AVX512CD AVX512_KNL AVX512_KNM AVX512_SKX AVX512_CLX AVX512_CNL AVX512_ICL AVX512_SPR AVX512FP16"

# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md):
# the fixtures, textures and object models of the graded kitchens.
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && cp -r /opt/task-assets/payload/verifier_assets/. /opt/sim/robocasa/robocasa/models/assets/ \
    && rm -rf /opt/task-assets/payload/verifier_assets

# Stub catalogue entries for every object instance not shipped (RoboCasa lists instance folders to split them into
# pretrain and target sets; the listing must keep its length or the seeded choice of object changes).
COPY object_instances.txt make_stubs.py /opt/task-assets/
# The simulator runs as root here, so the asset tree (held-out layouts, fixtures and object models) is root-only: the
# controller, which runs in this sandbox at the agent's UID, can neither read nor change what later states load.
# robosuite writes texture patches to /tmp/robosuite_temp_tex: created root-only too (the grader re-checks both).
RUN python3 /opt/task-assets/make_stubs.py /opt/task-assets/object_instances.txt /opt/sim/robocasa/robocasa/models/assets/objects \
    && chmod -R a+rX /opt/task-assets /opt/sim \
    && chown -R root:root /opt/sim/robocasa/robocasa/models/assets \
    && chmod -R go-rwx /opt/sim/robocasa/robocasa/models/assets /opt/task-assets \
    && mkdir -p /tmp/robosuite_temp_tex && chown root:root /tmp/robosuite_temp_tex && chmod 0700 /tmp/robosuite_temp_tex

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

RUN python3 -c "import numpy, scipy, cv2, mujoco, robosuite, robocasa; print('[image] mujoco', mujoco.__version__)" 2>/dev/null \
    && runuser -u user -- env HOME=/tmp /opt/venv/bin/python3 -c "import numpy, mujoco, cv2; print('[image] agent-UID import OK')"
