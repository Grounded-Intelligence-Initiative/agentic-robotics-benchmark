# Task image. Nothing here depends on tests/, solution/ or the anchor: an image layer is readable by anyone who can
# run the image. The joint angles of the test recordings are NOT in the image.
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

# Software rendering for MuJoCo: Mesa OSMesa (llvmpipe, no display or device needed), checked below as the agent user.
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 libgl1 \
    && rm -rf /var/lib/apt/lists/*

# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md):
# the robot model, the 6 labelled recordings (the video and the joint angles) and the 24 test recordings (the video)
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && chmod -R a+rX /opt/task-assets

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa

RUN python3 -c "import numpy, scipy, mujoco, cv2, PIL, matplotlib, skimage, sklearn, trimesh; print('[image] mujoco', mujoco.__version__, 'cv2', cv2.__version__)" \
    && python3 -c "import site, sys; assert not site.ENABLE_USER_SITE or sys.prefix != sys.base_prefix" \
    && runuser -u user -- python3 -c "import numpy, scipy, mujoco, cv2; print('[image] agent-UID import OK')" \
    && runuser -u user -- env HOME=/tmp python3 -c "import cv2; c = cv2.VideoCapture('/opt/task-assets/payload/test/T01/video.mp4'); n = sum(1 for _ in range(48) if c.read()[0]); assert n == 48, n; print('[image] agent-UID MPEG-4 decoding OK')" \
    && runuser -u user -- env HOME=/tmp python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('/opt/task-assets/payload/robot/g1.xml'); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r = mujoco.Renderer(m, 48, 64); r.update_scene(d); im = r.render(); assert im.max() > 0; print('[image] agent-UID MuJoCo OSMesa render OK', im.shape)"
