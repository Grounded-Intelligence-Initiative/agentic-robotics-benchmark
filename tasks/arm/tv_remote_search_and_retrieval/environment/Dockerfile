# The image of tv_remote_search_and_retrieval: MuJoCo with a CPU renderer, the Panda's model
# (robosuite's asset files) and the living room's meshes and textures. The build context is
# environment/ without setup/. Nothing of tests/ or solution/ is in here.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

# One venv for every identity (setup and verify run as root, the agent and the oracle
# unprivileged): a world-traversable interpreter behind it, Python 3.12 for the engine's verify
# package.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && uv pip install --python /opt/venv --no-deps robosuite==1.5.2 \
    && rm -f /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# Numerics. The simulator and its CPU renderer compute the same on every machine (measured on two CPU families), but
# a controller's own arithmetic does not: NumPy's OpenBLAS picks its kernels and its block size by CPU (a controller's
# inverse kinematics through np.linalg.solve gave other joint targets on Zen 3 and Zen 5 from its 34th step, and the
# same controller then lifted the remote on one machine and not on the other). It differs in other ways too (NumPy's AVX-512 loops round arctan2, exp, log, power and others differently; a threaded
# product is split by the host's core count; OpenCV's IPP picks its code by CPU). So that a development run here
# and the graded run compute the same on every machine, a .pth sets these for every /opt/venv Python, also one that
# is started without them (each is a default: a variable that is already set is kept):
#   * OPENBLAS_CORETYPE=Haswell, and OPENBLAS_L2_SIZE=256 (the OpenBLAS bundled with numpy 2.5.2 to 2.5.5 raises its
#     block size on AMD Zen 4 / Zen 5 hosts even for a forced kernel; this keeps the block at 256 on every host);
#   * NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR": the names of numpy 2.5's dispatch groups (the names
#     of numpy 1.x / 2.0-2.2, AVX512F, AVX512_SKX, ..., are rejected by this numpy and disable nothing). The build
#     fails below if this numpy does not know a name;
#   * OPENBLAS_NUM_THREADS=1 and OMP_NUM_THREADS=1;
#   * OPENCV_IPP=avx2, and OPENCV_CPU_DISABLE=AVX512-SKX on a CPU that has it.
# The grader's launcher (tests/nonet_exec.py) and the practice grader pass the same variables to run.sh, and the
# grader refuses to grade on a machine that computes differently (tests/numerics_check.py).
ENV OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256
RUN printf '%s\n' \
      'import os' \
      'os.environ.setdefault("OPENBLAS_CORETYPE", "Haswell")' \
      'os.environ.setdefault("OPENBLAS_L2_SIZE", "256")' \
      'os.environ.setdefault("NPY_DISABLE_CPU_FEATURES", "X86_V4 AVX512_ICL AVX512_SPR")' \
      'os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")' \
      'os.environ.setdefault("OMP_NUM_THREADS", "1")' \
      'os.environ.setdefault("OPENCV_IPP", "avx2")' \
      'try:' \
      '    with open("/proc/cpuinfo") as _f:' \
      '        _flags = set(_f.read().split())' \
      'except OSError:' \
      '    _flags = set()' \
      'if "avx512f" in _flags:' \
      '    os.environ.setdefault("OPENCV_CPU_DISABLE", "AVX512-SKX")' \
      > /opt/venv/lib/python3.12/site-packages/remote_numerics_pin.py \
    && printf '%s\n' 'import remote_numerics_pin' > /opt/venv/lib/python3.12/site-packages/remote_numerics_pin.pth \
    && chmod 0644 /opt/venv/lib/python3.12/site-packages/remote_numerics_pin.py /opt/venv/lib/python3.12/site-packages/remote_numerics_pin.pth \
    && env -i PATH=/opt/venv/bin:/usr/bin:/bin HOME=/tmp /opt/venv/bin/python3 -I -W error::ImportWarning -c "import os; e = os.environ; assert (e.get('OPENBLAS_CORETYPE'), e.get('OPENBLAS_L2_SIZE'), e.get('OPENBLAS_NUM_THREADS'), e.get('OMP_NUM_THREADS'), e.get('OPENCV_IPP')) == ('Haswell', '256', '1', '1', 'avx2'), dict(e); import numpy; from numpy._core._multiarray_umath import __cpu_dispatch__ as D, __cpu_features__ as F; assert set(e['NPY_DISABLE_CPU_FEATURES'].split()) == set(D) - {'X86_V3'}, D; assert [k for k in D if F[k]] in ([], ['X86_V3']), [k for k in D if F[k]]; print('[image] numerics pin OK', D)"

# Offscreen rendering on the CPU: EGL through Mesa's llvmpipe.
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=2

# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md):
# room/, the meshes and textures of the settee, the chest of drawers, the two remotes, the other small things, the
# hanger, the floor and the wall (their sources and licences: /opt/room/SOURCES.md). The simulator reads them in
# place, as every account: they are not staged into the agent home.
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && chown -R root:root /opt/task-assets && chmod -R u=rwX,go=rX /opt/task-assets \
    && ln -s /opt/task-assets/payload/room /opt/room
ENV REMOTE_ROOM_DIR=/opt/room

# The declared artifact path, owned by the agent.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# Fail the build early if the venv, the Panda's files, the room or the renderer is broken, and
# prove that the agent account can use them.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && test -f /opt/room/sofa_frame.obj && test -f /opt/room/remote_b.png \
    && runuser -u user -- env HOME=/tmp MESA_SHADER_CACHE_DISABLE=true python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 1; print('[image] agent-UID render OK')"
WORKDIR /
