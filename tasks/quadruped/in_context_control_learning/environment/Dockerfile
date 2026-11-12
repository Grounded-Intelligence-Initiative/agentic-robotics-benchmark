FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
USER root
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 mesa-vulkan-drivers libvulkan1 \
    && rm -rf /var/lib/apt/lists/*
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV MUJOCO_GL=osmesa LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv
RUN mkdir -p /home/user/submission && chown user:user /home/user/submission
RUN runuser -u user -- python3 -c "import mujoco; m=mujoco.MjModel.from_xml_string('<mujoco/>'); r=mujoco.Renderer(m,64,64); r.close()"
# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md).
# visual/ holds the camera's render assets (Poly Haven CC0 textures and props, the MuJoCo Menagerie Go1 meshes);
# fetch_visual_assets.py checks each against visual_assets.json again, installs them root-owned in
# /opt/visual_robotics/assets (world readable, nothing in them is hidden) and bakes the props for MuJoCo.
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && chmod -R a+rX /opt/task-assets
COPY visual_assets.json fetch_visual_assets.py /tmp/visual_assets/
RUN /opt/venv/bin/python3 /tmp/visual_assets/fetch_visual_assets.py /tmp/visual_assets/visual_assets.json \
        /opt/visual_robotics/assets --source /opt/task-assets/payload/visual \
    && chmod -R a+rX /opt/visual_robotics/assets
# The engine's root helpers (the guest service, which copies the declared artifacts out of this sandbox when the
# agent stops) run in the image's working directory, and `python3 -c` puts that directory first on sys.path. The
# base image's is /home/user, which the agent can write: a module the agent left there would run as root and could
# rewrite the robot service's ledger before it is copied. A root-owned working directory closes that.
WORKDIR /
