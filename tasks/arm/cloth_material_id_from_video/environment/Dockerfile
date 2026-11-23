# Task image. Nothing here depends on tests/, solution/ or the anchor: an image layer is readable by anyone who can
# run the image. The recordings are rendered while the image builds by gen/render_clips.py from a read-only bind
# mount of gen/ (which holds the true materials); no file of gen/ is left in any layer, only the rendered videos and
# the commanded clamp and arm trajectories.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && uv pip install --python /opt/venv --no-deps robosuite==1.5.2 \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# Software rendering for MuJoCo (Mesa OSMesa, no display or GPU) and ffmpeg (H.264 encoding of the recordings; also
# available to the agent).
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 libgl1 ffmpeg \
    && rm -rf /var/lib/apt/lists/*

ENV MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa LP_NUM_THREADS=1

# ---- the recordings: 4 fabrics x 7 clips x 3 cameras, rendered from the true materials (deterministic programs)
RUN --mount=type=bind,source=gen,target=/tmp/gen \
    python3 -I -B /tmp/gen/render_clips.py /opt/task-assets/payload --workers "$(nproc)" \
    && chmod -R a+rX /opt/task-assets

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV LP_NUM_THREADS=4
RUN python3 -c "import numpy, scipy, mujoco, cv2, skimage; print('[image] mujoco', mujoco.__version__, 'cv2', cv2.__version__)" \
    && python3 -c "import site, sys; assert not site.ENABLE_USER_SITE or sys.prefix != sys.base_prefix" \
    && runuser -u user -- python3 -c "import numpy, scipy, mujoco, cv2; print('[image] agent-UID import OK')" \
    && runuser -u user -- env HOME=/tmp python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom type=\"sphere\" size=\".1\"/></worldbody></mujoco>'); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r = mujoco.Renderer(m, 48, 64); r.update_scene(d); im = r.render(); assert im.max() > 0; print('[image] agent-UID MuJoCo OSMesa render OK', im.shape)" \
    && test -s /opt/task-assets/payload/data/fabrics/C/clip06/cam2.mp4
