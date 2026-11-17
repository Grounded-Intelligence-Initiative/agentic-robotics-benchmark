# Environment image for keyed_nut_peg_assembly. Final stage derives from the official ALE base.
# Contains the simulator stack, the public robot assets and the commissioning service (the cell's
# simulator on its development kits, readable by its own account only). The grader, the reference
# solution, the hidden episodes and the reference value are never copied into the image (the build
# context is environment/ less setup/).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# Python 3.12 venv owned by root, readable and traversable by every account, first on PATH for every
# stage (root stages then run a venv interpreter with user site disabled).
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

# Software rendering for MuJoCo's OpenGL renderer (Mesa llvmpipe through EGL, CPU only).
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=2

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at \
    && ! command -v crond && ! command -v cron && ! command -v atd

# Build-time checks: the venv works for root and for the agent account, robosuite's Panda assets are
# present (robosuite itself is never imported), and MuJoCo renders offscreen.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] offscreen render OK')" \
    && python3 -c "import site, sys; assert sys.prefix != sys.base_prefix" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, mujoco; print('[image] agent-UID import OK')"

# The commissioning service: the cell on its development kits, for the agent's commissioning window
# (commission_service/: the service program, the cell's simulator keyed_env.py, a byte copy of the
# grader's, and the development kits with the window's size), installed for its own account `keyedcell`
# only: mode 700, so the agent account can neither read nor list it. environment/setup/setup.sh starts
# it before the agent's session (and restarts it should it ever exit); the agent reaches it through a
# unix socket and gets the public wire's observations and how an episode ended: no ground truth, no
# score, no completion verdict.
RUN useradd --system --no-create-home --home-dir /opt/keyedcell-commission --shell /usr/sbin/nologin keyedcell
COPY commission_service /opt/keyedcell-commission
RUN find /opt/keyedcell-commission -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && rm -rf /opt/keyedcell-commission/state \
    && mkdir -p /opt/keyedcell-commission/state \
    && chown -R keyedcell:keyedcell /opt/keyedcell-commission \
    && chmod -R u+rwX,go-rwx /opt/keyedcell-commission

# The commissioning service works for its own account (the cell loads, lays a development kit out and
# renders its cameras offscreen), and the agent account can neither list nor read it. One import line
# per -c: a continuation line starting with `from` reads as a Dockerfile FROM to the final-stage lint.
RUN runuser -u keyedcell -- env -i HOME=/opt/keyedcell-commission/state TMPDIR=/opt/keyedcell-commission/state \
        PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 \
        LP_NUM_THREADS=2 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
        /opt/venv/bin/python3 -B -s -c "import json, sys; sys.path.insert(0, '/opt/keyedcell-commission'); import commissiond, keyed_env as E; k = commissiond.KITS['kits'][0]; env = E.NutEnv(k['kit']['nuts']); obs = env.reset(int(k['layout_seed']), k['kit']['order']); assert obs['wrist'].shape == (192, 256, 3) and obs['left'].mean() > 10; assert env.max_steps == 850 and env.unit is not None; env.close(); print('[image] commissioning service: the cell loads and renders for its account,', len(commissiond.KITS['kits']), 'development kits')" \
    && rm -rf /opt/keyedcell-commission/state && mkdir -p /opt/keyedcell-commission/state \
    && chown keyedcell:keyedcell /opt/keyedcell-commission/state && chmod 700 /opt/keyedcell-commission/state \
    && ! runuser -u user -- ls /opt/keyedcell-commission > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/keyedcell-commission/keyed_env.py > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/keyedcell-commission/kits.json > /dev/null 2>&1 \
    && test -z "$(find /opt/keyedcell-commission ! -user keyedcell -print -quit)" \
    && test -z "$(find /opt/keyedcell-commission -perm /077 -print -quit)"

# The engine's root helpers (the guest service that copies the declared artifact out of this sandbox when
# the agent stops) run in the image's working directory, and `python3 -c` puts that directory first on
# sys.path. The base image's is /home/user, which the agent can write: a module the agent left there
# would run as root and could read the commissioning service's files. A root-owned working directory
# closes that.
WORKDIR /
