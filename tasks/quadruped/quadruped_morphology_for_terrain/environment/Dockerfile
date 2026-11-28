# Environment image for quadruped_morphology_for_terrain.
#
# One py3.12 venv (/opt/venv): numpy, scipy, opencv, pillow for everybody; MuJoCo for the grader (root) and the
# practice walkway's account `qmcell` only. The agent account can neither import nor read MuJoCo: the agent designs
# without a simulator. The practice walkway (practice_service/ + cell/: the walkway simulator, a byte copy of the
# grader's) is installed for `qmcell` only. The final image never holds the grader, the reference design or the
# anchor (the build context is environment/ less setup/).

FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_NO_CACHE=1
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt /usr/local/bin/uv \
    && rm -rf /root/.cache \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }
# Software rendering for the practice walkway's videos (Mesa llvmpipe through EGL, CPU only)
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=1

# MuJoCo for root and `qmcell` only: its package and its metadata, group qmcell, nothing for others. (For the agent
# account the unreadable package directory imports as an empty namespace package: nothing of MuJoCo is usable.)
RUN useradd --system --no-create-home --home-dir /opt/qmcell --shell /usr/sbin/nologin qmcell \
    && SP=$(/opt/venv/bin/python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])") \
    && for p in "$SP"/mujoco "$SP"/mujoco-*.dist-info "$SP"/mujoco.libs; do \
         [ -e "$p" ] || continue; chown -R root:qmcell "$p"; chmod -R u+rwX,g+rX,g-w,o-rwx "$p"; done \
    && ! runuser -u user -- /opt/venv/bin/python3 -c "from mujoco import MjModel" 2>/dev/null \
    && [ -z "$(runuser -u user -- find / -xdev \( -name 'libmujoco*' -o -name 'mujoco*.whl' -o -name 'mjx' \) -readable 2>/dev/null | head -1)" ] \
    && /opt/venv/bin/python3 -c "import mujoco; print('[image] root imports mujoco', mujoco.__version__)"

# The practice walkway: the grader's simulator behind a unix socket, for a few practice runs during the agent's
# session (practice_service/practiced.py). Mode 700 for `qmcell`; environment/setup/setup.sh starts it.
COPY practice_service/practiced.py /opt/qmcell/practiced.py
COPY cell/ /opt/qmcell/
RUN mkdir -p /opt/qmcell/state && chown -R qmcell:qmcell /opt/qmcell && chmod -R u+rwX,go-rwx /opt/qmcell \
    && runuser -u qmcell -- env -i HOME=/opt/qmcell/state PATH=/opt/venv/bin:/usr/bin:/bin MUJOCO_GL=egl \
        PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 LP_NUM_THREADS=1 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
        /opt/venv/bin/python3 -B -s -c "import sys; sys.path.insert(0, '/opt/qmcell'); import numpy as np, mujoco, qm_design as D, qm_sim as S, practiced; c = D.load_catalog('/opt/qmcell/catalog.json'); d, _ = D.check(open('/opt/qmcell/selftest_robot.xml', 'rb').read(), c); e = S.Episode(d, c, S.make_terrain(np.random.default_rng(1))); r = mujoco.Renderer(e.m, 48, 64); r.update_scene(e.d); assert r.render().std() > 1; print('[image] practice walkway loads, steps and renders for its account')" \
    && rm -rf /opt/qmcell/state && mkdir -p /opt/qmcell/state && chown qmcell:qmcell /opt/qmcell/state && chmod 700 /opt/qmcell/state \
    && find /opt/qmcell -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && ! runuser -u user -- ls /opt/qmcell > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/qmcell/qm_sim.py > /dev/null 2>&1

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

# Build-time checks: the venv works for root and for the agent account (without MuJoCo), MuJoCo renders offscreen.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] root: numpy', numpy.__version__, 'mujoco', mujoco.__version__)" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] offscreen render OK')" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, PIL; print('[image] agent-UID import OK')" \
    && ! runuser -u user -- /opt/venv/bin/python3 -c "from mujoco import MjModel" 2>/dev/null

# The engine's root helpers run in the image's working directory; a root-owned one keeps an agent's module from
# running as root.
WORKDIR /
