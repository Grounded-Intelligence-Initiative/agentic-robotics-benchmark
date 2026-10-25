# Environment image for arm_hidden_mechanism_box. Final stage derives from the official ALE base.
# Contains only the simulator stack and public scene/robot assets; verify/, oracle/, the hidden boxes
# and the anchor are never copied into the image.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:latest

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

# Public LIBERO scene assets (Hugging Face dataset lerobot/libero-assets, pinned revision): only the
# kitchen tabletop scene and the files it references. Downloaded with a throwaway venv over plain HTTPS
# (the Xet transfer backend is disabled: its CAS host is not reachable through the build proxy).
RUN python3 -m venv /tmp/hfvenv \
    && /tmp/hfvenv/bin/python3 -m pip install --no-cache-dir huggingface_hub==2.2.0 \
    && HF_HUB_DISABLE_XET=1 /tmp/hfvenv/bin/python3 -c "from huggingface_hub import snapshot_download; snapshot_download(repo_id='lerobot/libero-assets', repo_type='dataset', revision='0b3ea86be5fe169d0fd036ae63d1070ec09e90f6', local_dir='/opt/libero-assets', allow_patterns=['scenes/libero_kitchen_tabletop_base_style.xml', 'scenes/kitchen_background/**', 'scenes/kitchen_background_pot/**', 'scenes/kitchen_background_hot_pot/**', 'scenes/kitchen_background_stove/**', 'scenes/fridge/**', 'textures/tile_grigia_caldera_porcelain_floor.png', 'textures/martin_novak_wood_table.png', 'textures/smooth_light_gray_plaster.png'])" \
    && rm -rf /tmp/hfvenv /opt/libero-assets/.cache /root/.cache \
    && chmod -R a+rX /opt/libero-assets
ENV LIBERO_ASSETS=/opt/libero-assets

# Software rendering for MuJoCo (Mesa llvmpipe through EGL).
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=2

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# The lab service's dedicated account (rev-leak R1): the lab never runs as root or as the agent while
# it serves the hidden boxes.
RUN useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin alelab

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at \
    && ! command -v crond && ! command -v cron && ! command -v atd

# Build-time checks: the venv works for root and for the agent account, robosuite's Panda assets and
# the LIBERO scene are present (robosuite itself is never imported), and MuJoCo renders offscreen.
RUN python3 -c "import numpy, scipy, cv2, mujoco; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import importlib.util, os; s = importlib.util.find_spec('robosuite'); p = os.path.join(list(s.submodule_search_locations)[0], 'models', 'assets', 'robots', 'panda', 'robot.xml'); assert os.path.isfile(p), p" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('/opt/libero-assets/scenes/libero_kitchen_tabletop_base_style.xml'); print('[image] LIBERO scene ngeom', m.ngeom)" \
    && python3 -c "import site, sys; assert sys.prefix != sys.base_prefix" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, mujoco; print('[image] agent-UID import OK')"
