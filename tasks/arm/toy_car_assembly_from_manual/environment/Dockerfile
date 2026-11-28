# Environment image for toy_car_assembly_from_manual. The final stage derives from the official ALE base.
# It holds the simulator stack (MuJoCo, rendered on the CPU through Mesa's llvmpipe), the public robot model files
# (robosuite 1.5.2's Panda) and the practice cell (the cell on the practice kit, readable by its own account only).
# The example manuals and the practice runner reach the agent's home through environment/setup/; the grader and
# the reference solution never enter the image (the build context is environment/ less
# setup/).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# Python 3.12 venv owned by root, readable and traversable by every account, first on PATH for every stage.
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --upgrade pip \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --no-deps robosuite==1.5.2 \
    && rm -f /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv

# Software rendering for MuJoCo's OpenGL renderer: Mesa llvmpipe through EGL, CPU only (the task has no GPU).
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=2

# The practice cell: the grader's cell (cell/toycar_env.py and cell/toycar_manual.py, byte copies of tests/) on its
# own practice seeds behind a unix socket, for a fixed number of practice episodes during the agent's session
# (practice_service/practiced.py). Installed for its own account
# `toycell` only (mode 700: the agent account can neither read nor list it); environment/setup/setup.sh starts it.
RUN useradd --system --no-create-home --home-dir /opt/toycell --shell /usr/sbin/nologin toycell
COPY practice_service/practiced.py /opt/toycell/practiced.py
COPY cell/toycar_env.py cell/toycar_manual.py /opt/toycell/
RUN mkdir -p /opt/toycell/state && chown -R toycell:toycell /opt/toycell && chmod -R u+rwX,go-rwx /opt/toycell \
    && runuser -u toycell -- env -i HOME=/opt/toycell/state PATH=/opt/venv/bin:/usr/bin:/bin MUJOCO_GL=egl \
        PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 LP_NUM_THREADS=2 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
        /opt/venv/bin/python3 -B -s -c "import sys; sys.path.insert(0, '/opt/toycell'); import toycar_env as E, practiced; e = E.ToyCarEnv(); o = e.reset(practiced.PRACTICE_SEEDS[0]); assert o['overview'].shape == (224, 224, 3) and o['overview'].std() > 10 and o['wrist'].std() > 5 and len(o['manual']) == 6; e.close(); print('[image] practice cell loads and renders for its account')" \
    && rm -rf /opt/toycell/state && mkdir -p /opt/toycell/state && chown toycell:toycell /opt/toycell/state && chmod 700 /opt/toycell/state \
    && find /opt/toycell -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && ! runuser -u user -- ls /opt/toycell > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/toycell/toycar_env.py > /dev/null 2>&1

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at \
    && ! command -v crond && ! command -v cron && ! command -v atd

# Build-time checks: the venv works for root and for the agent account, robosuite's Panda files are present
# (robosuite itself is never imported), and MuJoCo renders offscreen for both accounts.
RUN python3 -c "import numpy, scipy, PIL, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] offscreen render OK')" \
    && runuser -u user -- env HOME=/tmp /opt/venv/bin/python3 -c "import numpy, scipy, PIL, mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] agent-UID import and render OK')"

# The engine's root helpers run in the image's working directory, and `python3 -c` puts that directory first on
# sys.path. The base image's is /home/user, which the agent can write: a root-owned working directory closes that.
WORKDIR /
