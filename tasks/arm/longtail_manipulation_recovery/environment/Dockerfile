# Agent image of longtail_manipulation_recovery. Holds the RoboCasa stack and the PRACTICE kitchens only: the graded
# kitchens (RoboCasa's held-out split), their object instances, tests/ and solution/ are never copied here.
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

# ---- task assets: fetched at the dataset commit pinned in assets.sha256 (docs/task-assets.md): the fixtures, textures
# and object models of the practice kitchens only. The 1,971 files travel as one archive, pinned by its sha256 (one
# request per file would run into the Hub's rate limit); after unpacking, every file is checked against the per-file
# list robocasa_assets.files.sha256, and a file the list does not name fails the build.
COPY fetch_assets.py assets.sha256 robocasa_assets.files.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && tar -xzf /opt/task-assets/payload/robocasa_assets.tar.gz -C /opt/task-assets/payload --no-same-owner \
    && rm /opt/task-assets/payload/robocasa_assets.tar.gz \
    && (cd /opt/task-assets/payload && sha256sum -c --quiet /opt/task-assets/robocasa_assets.files.sha256 \
        && [ "$(find robocasa_assets -type f | wc -l)" = "$(wc -l < /opt/task-assets/robocasa_assets.files.sha256)" ]) \
    && cp -r /opt/task-assets/payload/robocasa_assets/. /opt/sim/robocasa/robocasa/models/assets/ \
    && rm -rf /opt/task-assets/payload/robocasa_assets

# Stub catalogue entries for every object instance not shipped (RoboCasa lists instance folders to split them into
# pretrain and target sets; the listing must keep its length or the seeded choice of object changes).
COPY object_instances.txt make_stubs.py /opt/task-assets/
RUN python3 /opt/task-assets/make_stubs.py /opt/task-assets/object_instances.txt /opt/sim/robocasa/robocasa/models/assets/objects \
    && rm -rf /opt/sim/robocasa/robocasa/models/assets/scenes/kitchen_layouts/test \
              /opt/sim/robocasa/robocasa/models/assets/scenes/kitchen_styles/test \
    && find /opt/sim/robocasa/robocasa/models/assets/objects -type d -exec chmod a+rwx {} + \
    && chmod -R a+rX /opt/task-assets /opt/sim

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

RUN python3 -c "import numpy, scipy, cv2, mujoco, robosuite, robocasa; print('[image] mujoco', mujoco.__version__)" 2>/dev/null \
    && runuser -u user -- env HOME=/tmp /opt/venv/bin/python3 -c "import numpy, mujoco, cv2; print('[image] agent-UID import OK')"

# ---- the dual mode's graded episode (experiment, branch exp/dual-mode-scoring): environment/setup/online/graded_service.py
# runs ONE hidden failure state, live, for the agent, in RoboCasa's held-out kitchens, with the grader's own kitchen
# process (tests/sim_worker.py). It needs those kitchens: a second RoboCasa tree (the same pinned commit) with the graded
# kitchens' assets installed exactly as tests/Dockerfile installs them for the grader (graded_assets.sha256 and
# verifier_assets.files.sha256 are byte copies of tests/assets.sha256 and tests/verifier_assets.files.sha256), owned by
# the service's own account `longgrade` (never root, never the agent; RoboCasa writes its temporary model files into the
# tree it builds from) and closed to every other account. The agent's practice tree under /opt/sim keeps the practice
# kitchens only, and its account can neither list nor read this one. The kitchen process imports RoboCasa from this
# tree (PYTHONPATH), robosuite and everything else from /opt/venv and /opt/sim as the grader does.
RUN useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin longgrade
RUN mkdir -p /opt/longtail-graded /tmp/src \
    && curl -fsSL https://github.com/robocasa/robocasa/archive/456174f62b89b8fca99eaaf33949c29fec9cfc2a.tar.gz | tar -xz -C /tmp/src \
    && mv /tmp/src/robocasa-* /opt/longtail-graded/robocasa && rm -rf /tmp/src
COPY graded_assets.sha256 verifier_assets.files.sha256 /opt/longtail-graded/
RUN python3 /opt/task-assets/fetch_assets.py /opt/longtail-graded/graded_assets.sha256 /opt/longtail-graded/payload \
    && tar -xzf /opt/longtail-graded/payload/verifier_assets.tar.gz -C /opt/longtail-graded/payload --no-same-owner \
    && rm /opt/longtail-graded/payload/verifier_assets.tar.gz \
    && (cd /opt/longtail-graded/payload && sha256sum -c --quiet /opt/longtail-graded/verifier_assets.files.sha256 \
        && [ "$(find verifier_assets -type f | wc -l)" = "$(wc -l < /opt/longtail-graded/verifier_assets.files.sha256)" ]) \
    && cp -r /opt/longtail-graded/payload/verifier_assets/. /opt/longtail-graded/robocasa/robocasa/models/assets/ \
    && rm -rf /opt/longtail-graded/payload \
    && python3 /opt/task-assets/make_stubs.py /opt/task-assets/object_instances.txt /opt/longtail-graded/robocasa/robocasa/models/assets/objects \
    && chown -R longgrade:longgrade /opt/longtail-graded && chmod -R u+rwX,go-rwx /opt/longtail-graded \
    && ! runuser -u user -- ls /opt/longtail-graded \
    && runuser -u longgrade -- env -i HOME=/tmp PATH=/opt/venv/bin:/usr/bin:/bin PYTHONPATH=/opt/longtail-graded/robocasa \
         /opt/venv/bin/python3 -s -c "import robocasa; assert robocasa.__file__.startswith('/opt/longtail-graded/robocasa/robocasa/'), robocasa.__file__; print('[image] the graded kitchens tree imports as robocasa', robocasa.__file__)"
