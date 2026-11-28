# Environment image for spindle_ring_sorting.
#
# Two stages. `py` builds the py3.12 venv (MuJoCo, NumPy, SciPy, OpenCV, Pillow; robosuite 1.5.2 only for its Panda
# assets). The final stage derives from the official ALE base, takes the venv and installs the practice cell
# (environment/cell/ring_env.py, a byte copy of tests/ring_env.py, and environment/practice_service/practiced.py) for
# its own account, readable by that account only. The image never holds the grader, the reference solution or the
# anchor (the build context is environment/ less setup/).

FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1 AS py
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && uv pip install --python /opt/venv --no-deps robosuite==1.5.2 \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# ---- the final image
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=py /opt/venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1

# Numerics. The cell's operational-space controller calls LAPACK every physics step, and NumPy's OpenBLAS picks its
# kernels by CPU; a controller's own arithmetic (NumPy loops, OpenCV) can differ in the last bit between hosts too.
# The pins below make a development run here and the graded run compute the same on every machine
# (docs/contributing-a-task.md section 5): OpenBLAS kernels and block size, one thread, NumPy's AVX-512 loops off
# (numpy 2.5's dispatch-group names; the build fails if this numpy does not know one), OpenCV's IPP at AVX2. The .pth
# sets each as a default for a /opt/venv Python started without them (the grader starts run.sh with a small
# environment).
ENV OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256
RUN printf '%s\n' \
      'import os' \
      'os.environ.setdefault("OPENBLAS_CORETYPE", "Haswell")' \
      'os.environ.setdefault("OPENBLAS_L2_SIZE", "256")' \
      'os.environ.setdefault("NPY_DISABLE_CPU_FEATURES", "X86_V4 AVX512_ICL AVX512_SPR")' \
      'os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")' \
      'os.environ.setdefault("OMP_NUM_THREADS", "1")' \
      'os.environ.setdefault("OPENCV_IPP", "avx2")' \
      > /opt/venv/lib/python3.12/site-packages/ring_numerics_pin.py \
    && printf '%s\n' 'import ring_numerics_pin' > /opt/venv/lib/python3.12/site-packages/ring_numerics_pin.pth \
    && chmod 0644 /opt/venv/lib/python3.12/site-packages/ring_numerics_pin.py /opt/venv/lib/python3.12/site-packages/ring_numerics_pin.pth \
    && env -i PATH=/opt/venv/bin:/usr/bin:/bin HOME=/tmp /opt/venv/bin/python3 -I -W error::ImportWarning -c "import os; e = os.environ; assert (e.get('OPENBLAS_CORETYPE'), e.get('OPENBLAS_L2_SIZE'), e.get('OPENBLAS_NUM_THREADS'), e.get('OMP_NUM_THREADS'), e.get('OPENCV_IPP')) == ('Haswell', '256', '1', '1', 'avx2'), dict(e); import numpy; from numpy._core._multiarray_umath import __cpu_dispatch__ as D, __cpu_features__ as F; assert set(e['NPY_DISABLE_CPU_FEATURES'].split()) == set(D) - {'X86_V3'}, D; print('[image] numerics pin OK', D)"

# The cell's simulator stack is not the agent's: MuJoCo and robosuite's robot assets are readable by root (the grader)
# and the practice cell's account `ringcell` only. The agent account imports numpy, scipy, opencv and pillow.
RUN useradd --system --no-create-home --home-dir /opt/ringcell --shell /usr/sbin/nologin ringcell \
    && SP=/opt/venv/lib/python3.12/site-packages \
    && for p in "$SP"/mujoco "$SP"/mujoco-*.dist-info "$SP"/mujoco.libs "$SP"/robosuite "$SP"/robosuite-*.dist-info; do \
         [ -e "$p" ] || continue; chown -R root:ringcell "$p"; chmod -R u+rwX,g+rX,g-w,o-rwx "$p"; done \
    && ! runuser -u user -- /opt/venv/bin/python3 -c "from mujoco import MjModel" 2>/dev/null \
    && ! runuser -u user -- ls "$SP"/robosuite > /dev/null 2>&1 \
    && [ -z "$(runuser -u user -- find / -xdev \( -name 'libmujoco*' -o -name 'mujoco*.whl' -o -name 'panda_gripper.xml' \) -readable 2>/dev/null | head -1)" ] \
    && /opt/venv/bin/python3 -c "import mujoco; print('[image] root imports mujoco', mujoco.__version__)"

# Software rendering for MuJoCo's OpenGL renderer (Mesa llvmpipe through EGL, CPU only)
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=1

# The practice cell: the grader's cell (a byte copy of tests/ring_env.py) behind a unix socket, for a few practice
# episodes during the agent's session (practice_service/practiced.py). Installed for its own account `ringcell` only
# (mode 700: the agent account can neither read nor list it); environment/setup/setup.sh starts it.
COPY practice_service/practiced.py /opt/ringcell/practiced.py
COPY cell/ring_env.py /opt/ringcell/ring_env.py
RUN mkdir -p /opt/ringcell/state && chown -R ringcell:ringcell /opt/ringcell && chmod -R u+rwX,go-rwx /opt/ringcell \
    && runuser -u ringcell -- env -i HOME=/opt/ringcell/state PATH=/opt/venv/bin:/usr/bin:/bin MUJOCO_GL=egl \
        PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 LP_NUM_THREADS=1 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
        /opt/venv/bin/python3 -B -s -c "import sys; sys.path.insert(0, '/opt/ringcell'); import ring_env as K, practiced; e = K.RingEnv(); o = e.reset(1); assert o['overview'].shape == (180, 240, 3) and o['overview'].std() > 10 and o['wrist'].std() > 10; o, d, i = e.step([0, 0, 0, 0, -1, 3]); e.close(); print('[image] practice cell loads and renders for its account')" \
    && rm -rf /opt/ringcell/state && mkdir -p /opt/ringcell/state && chown ringcell:ringcell /opt/ringcell/state && chmod 700 /opt/ringcell/state \
    && find /opt/ringcell -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && ! runuser -u user -- ls /opt/ringcell > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/ringcell/ring_env.py > /dev/null 2>&1

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

# Build-time checks: the venv works for root and for the agent account, robosuite's Panda assets are present, MuJoCo
# renders offscreen.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] offscreen render OK')" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, PIL; print('[image] agent-UID import OK')"

# The engine's root helpers run in the image's working directory; the base image's is /home/user, which the agent can
# write. A root-owned working directory keeps a module the agent leaves there from running as root.
WORKDIR /
