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

# Offscreen rendering on the CPU: EGL through Mesa's llvmpipe.
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1

# The room: meshes and textures of the sofa, the chest of drawers, the remote, the hanger, the
# floor and the wall (their sources and licences: /opt/room/SOURCES.md).
COPY room /opt/room
RUN chmod -R a+rX /opt/room
ENV REMOTE_ROOM_DIR=/opt/room

# The declared artifact path, owned by the agent.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# Fail the build early if the venv, the Panda's files, the room or the renderer is broken, and
# prove that the agent account can use them.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && test -f /opt/room/sofa_frame.obj \
    && runuser -u user -- env HOME=/tmp MESA_SHADER_CACHE_DISABLE=true python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 1; print('[image] agent-UID render OK')"
WORKDIR /
