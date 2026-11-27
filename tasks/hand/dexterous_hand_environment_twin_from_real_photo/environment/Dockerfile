# dexterous_hand_environment_twin_from_real_photo's environment image. MuJoCo and the geometry and image tools in
# /opt/venv (the task's Python: setup, the agent's tools and the grader). CPU only: MuJoCo renders through OSMesa.
# `environment/` without its `setup/` folder is the build context. No task material and no hidden data are in here: the
# givens are staged by environment/setup/setup.sh, the grader arrives only at verify.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

# MuJoCo's OSMesa path and the GL / X client libraries MuJoCo and OpenCV link even when headless; libgomp for
# manifold3d / CoACD.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libosmesa6 libgl1 libglu1-mesa libglew2.2 libegl1 libglvnd0 libopengl0 libglx0 \
        libx11-6 libxext6 libxrender1 libxi6 libxrandr2 libxcursor1 libxinerama1 libsm6 libice6 libxkbcommon0 \
        libfontconfig1 libfreetype6 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# A world-readable venv: setup and verify run as root, the agent and the reference solution as `user`. Python 3.12 (the
# engine stages ale_verify into the python3 it finds and refuses anything older); the base image's CPython 3.12 is used.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --no-cache --python /opt/venv -r /tmp/requirements.txt \
    && find /opt/venv -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; } \
    && rm -f /tmp/requirements.txt

# The declared artifact path must exist even when nobody works.
RUN install -d -o user -g user /home/user/submission

ENV MPLBACKEND=Agg
ENV MUJOCO_GL=osmesa
ENV PYOPENGL_PLATFORM=osmesa

# Fail the build early if a toolchain is broken, and prove the agent account can use the venv. One import statement
# per -c.
RUN python3 -c "import mujoco, numpy, scipy, cv2, skimage, PIL, trimesh, manifold3d, rtree, coacd, shapely; print('[image] mujoco', mujoco.__version__, 'numpy', numpy.__version__)" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom type=\"box\" size=\"0.1 0.1 0.1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 64, 64); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d); px = r.render(); print('[image] OSMesa render OK', px.shape, int(px.max()))" \
    && runuser -u user -- python3 -c "import mujoco, cv2, skimage, trimesh, coacd, scipy; print('[image] agent-UID import OK')"

# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md)
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && chmod -R a+rX /opt/task-assets
