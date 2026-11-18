# Environment image for mixed_box_palletizing. Final stage derives from the official ALE base.
# It holds only the simulator stack (MuJoCo and its Python packages): the cell's own simulator file, tests/,
# solution/, the cartons and orders and the anchor are never copied into the image (the build context is
# environment/ less setup/).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# Python 3.12 venv owned by root, readable and traversable by every account, first on PATH for every stage.
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --upgrade pip \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv

# Software rendering for MuJoCo's offscreen renderer (Mesa llvmpipe through EGL): the task has no GPU.
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libglx-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=2

# The lab service's dedicated account: the lab never runs as root or as the agent while it serves.
RUN useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin alelab

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# Build-time checks: the venv works for root and for the agent account, and MuJoCo renders colour and depth
# offscreen on the CPU.
RUN python3 -c "import numpy, scipy, cv2, PIL, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import site, sys; assert sys.prefix != sys.base_prefix" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 3\"/><geom type=\"box\" size=\".2 .2 .2\" rgba=\"1 0 0 1\"/><camera name=\"c\" pos=\"0 0 2\"/></worldbody></mujoco>'); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r = mujoco.Renderer(m, 96, 128); r.update_scene(d, 'c'); a = r.render(); assert a[48, 64, 0] > 200 and a[48, 64, 1] < 150, a[48, 64]; r.enable_depth_rendering(); r.update_scene(d, 'c'); z = r.render(); assert abs(float(z[48, 64]) - 1.8) < 0.01, z[48, 64]; print('[image] offscreen colour and depth OK')" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, PIL, mujoco; print('[image] agent-UID import OK')"
