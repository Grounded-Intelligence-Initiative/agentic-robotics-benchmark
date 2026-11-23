FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# Numerics pin (docs/contributing-a-task.md section 5): the controller's arithmetic repeats on every
# grading host. OpenBLAS on its Haswell kernels with numpy 2.5's block size, one thread, NumPy's
# AVX-512 loops off, OpenCV's IPP on AVX2. A .pth sets each as a default, so a Python started with a
# scrubbed environment (the graded controller) gets them too.
RUN printf '%s\n' \
      'import os' \
      'os.environ.setdefault("OPENBLAS_CORETYPE", "Haswell")' \
      'os.environ.setdefault("OPENBLAS_L2_SIZE", "256")' \
      'os.environ.setdefault("NPY_DISABLE_CPU_FEATURES", "X86_V4 AVX512_ICL AVX512_SPR")' \
      'os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")' \
      'os.environ.setdefault("OMP_NUM_THREADS", "1")' \
      'os.environ.setdefault("OPENCV_IPP", "avx2")' \
      'os.environ.setdefault("MUJOCO_GL", "osmesa")' \
      'os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")' \
      'try:' \
      '    with open("/proc/cpuinfo") as _f:' \
      '        _flags = set(_f.read().split())' \
      'except OSError:' \
      '    _flags = set()' \
      'if "avx512f" in _flags:' \
      '    os.environ.setdefault("OPENCV_CPU_DISABLE", "AVX512-SKX")' \
      > /opt/venv/lib/python3.12/site-packages/cable_numerics_pin.py \
    && printf '%s\n' 'import cable_numerics_pin' > /opt/venv/lib/python3.12/site-packages/cable_numerics_pin.pth \
    && chmod 0644 /opt/venv/lib/python3.12/site-packages/cable_numerics_pin.py /opt/venv/lib/python3.12/site-packages/cable_numerics_pin.pth \
    && env -i PATH=/opt/venv/bin:/usr/bin:/bin HOME=/tmp /opt/venv/bin/python3 -I -W error::ImportWarning -c "import os; e = os.environ; assert (e.get('OPENBLAS_CORETYPE'), e.get('OPENBLAS_L2_SIZE'), e.get('OPENBLAS_NUM_THREADS'), e.get('OMP_NUM_THREADS'), e.get('OPENCV_IPP')) == ('Haswell', '256', '1', '1', 'avx2'), dict(e); import numpy; from numpy._core._multiarray_umath import __cpu_dispatch__ as D, __cpu_features__ as F; assert set(e['NPY_DISABLE_CPU_FEATURES'].split()) == set(D) - {'X86_V3'}, D; assert [k for k in D if F[k]] in ([], ['X86_V3']), [k for k in D if F[k]]; print('[image] numerics pin OK', D)"

# CPU rendering for the agent's own debugging views (MuJoCo through OSMesa).
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 libgl1 \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=1 MPLBACKEND=Agg

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No scheduler may start agent code after the agent phase.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

RUN python3 -c "import mujoco, numpy; print('[image] mujoco', mujoco.__version__)" \
    && runuser -u user -- env HOME=/tmp /opt/venv/bin/python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><extension><plugin plugin=\"mujoco.elasticity.cable\"/></extension><worldbody><light pos=\"0 0 1\"/><composite type=\"cable\" curve=\"s\" count=\"5 1 1\" size=\"0.2\" initial=\"ball\"><plugin plugin=\"mujoco.elasticity.cable\"><config key=\"bend\" value=\"1e7\"/><config key=\"twist\" value=\"1e7\"/></plugin><geom type=\"capsule\" size=\"0.004\"/></composite></worldbody></mujoco>'); d = mujoco.MjData(m); mujoco.mj_step(m, d); r = mujoco.Renderer(m, 48, 64); r.update_scene(d); im = r.render(); print('[image] agent-UID MuJoCo cable + OSMesa render OK', im.shape)"
