FROM ghcr.io/agentslastexam/container-ubuntu22-base:latest
USER root
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 \
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
# The onboard camera's renderer (world.py, fil_render.py): MuJoCo's Filament module (in the mujoco wheel) on Vulkan;
# Mesa's lavapipe is the CPU Vulkan driver (the task has no GPU). Proved at build: a Filament context on lavapipe.
RUN apt-get update && apt-get install -y --no-install-recommends mesa-vulkan-drivers libvulkan1 \
    && rm -rf /var/lib/apt/lists/*
RUN env -u DISPLAY /opt/venv/bin/python3 -c "import mujoco._render_filament as F, mujoco.experimental.studio.window; F.Context(F.ContextConfig(graphics_api=F.GraphicsApi.GRAPHICS_API_VULKAN)); print('[image] Filament on Vulkan OK')"
# LOCAL TEST ONLY (replaced by the task-assets fetch block before the push): the hall's scenery and image light.
COPY render/ /opt/visual_robotics/render/
RUN chmod -R a+rX /opt/visual_robotics/render
ENV VISUAL_ROBOTICS_RENDER=/opt/visual_robotics/render
COPY visual_assets.json fetch_visual_assets.py /tmp/visual_assets/
RUN /opt/venv/bin/python3 /tmp/visual_assets/fetch_visual_assets.py /tmp/visual_assets/visual_assets.json /opt/visual_robotics/assets \
    && chmod -R a+rX /opt/visual_robotics/assets
