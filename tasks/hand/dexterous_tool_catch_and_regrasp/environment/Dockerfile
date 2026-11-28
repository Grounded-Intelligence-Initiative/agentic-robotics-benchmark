# Environment image for dexterous_tool_catch_and_regrasp.
#
# Two stages. `py` builds the py3.12 venv (simulator stack). The final stage derives from the official ALE base, takes
# the venv, installs the catching cell (environment/cell/: the cell, its wire format and the Wonik Allegro hand model
# from MuJoCo Menagerie) for the practice cell's account only, and renders the cameras' calibration shots, which
# setup.sh hands to the agent. The build context is environment/ less setup/; it never holds the grader, the reference
# solution or the anchor.

FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1 AS py
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN mkdir -p /opt/uv-python && uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && chmod -R a+rX /opt/uv-python

# ---- the final image
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=py /opt/venv /opt/venv
COPY --from=py /opt/uv-python /opt/uv-python
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
# Software rendering for MuJoCo's OpenGL renderer (Mesa llvmpipe through EGL, CPU only)
RUN apt-get update && apt-get install -y --no-install-recommends libegl1 libgl1 libgles2 libegl-mesa0 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg OMP_NUM_THREADS=1 LP_NUM_THREADS=1

# The practice cell: the grader's cell behind a unix socket, for a few practice runs during the agent's session
# (practice_service/practiced.py). Installed for its own account `dexcell` only (mode 700: the agent account can
# neither read nor list it); environment/setup/setup.sh starts it. At build time the account also renders the cameras'
# calibration shots (the empty cell, no robot) and writes the camera parameters into /opt/dexcell_public, readable
# by everybody.
RUN useradd --system --no-create-home --home-dir /opt/dexcell --shell /usr/sbin/nologin dexcell
COPY cell/ /opt/dexcell/
COPY practice_service/practiced.py /opt/dexcell/practiced.py
RUN mkdir -p /opt/dexcell/state /opt/dexcell_public/calibration && chown -R dexcell:dexcell /opt/dexcell /opt/dexcell_public \
    && chmod -R u+rwX,go-rwx /opt/dexcell \
    && runuser -u dexcell -- env -i HOME=/opt/dexcell/state PATH=/opt/venv/bin:/usr/bin:/bin MUJOCO_GL=egl \
        PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 LP_NUM_THREADS=1 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
        ALLEGRO_XML=/opt/dexcell/wonik_allegro/right_hand.xml \
        /opt/venv/bin/python3 -B -s -c "import sys, json; sys.path.insert(0, '/opt/dexcell'); import cell as C, practiced; from PIL import Image; bg = C.render_backgrounds(); [Image.fromarray(v).save('/opt/dexcell_public/calibration/background_%s.png' % k) for k, v in bg.items()]; json.dump({'image_width': C.IMG_W, 'image_height': C.IMG_H, 'cameras': {k: {'pos': list(p), 'look_at': list(t), 'up': [0, 0, 1], 'fovy_deg': f} for k, (p, t, f) in C.CAMS.items()}}, open('/opt/dexcell_public/calibration/cameras.json', 'w'), indent=1); e = C.Episode(1, n_throws=1); o = e.reset(); assert o['img_side'].shape == (C.IMG_H, C.IMG_W, 3) and o['img_side'].std() > 5 and o['img_head'].std() > 5; e.close(); print('[image] practice cell loads and renders for its account')" \
    && chown -R root:root /opt/dexcell_public && chmod -R a+rX,go-w /opt/dexcell_public \
    && rm -rf /opt/dexcell/state && mkdir -p /opt/dexcell/state && chown dexcell:dexcell /opt/dexcell/state && chmod 700 /opt/dexcell/state \
    && find /opt/dexcell -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && ! runuser -u user -- ls /opt/dexcell > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/dexcell/cell.py > /dev/null 2>&1 \
    && runuser -u user -- ls /opt/dexcell_public/calibration/background_head.png > /dev/null

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# No job scheduler may run agent-scheduled work outside the graded solver tree.
RUN (apt-get purge -y cron anacron at >/dev/null 2>&1 || true) \
    && rm -rf /etc/cron.d /etc/cron.daily /etc/cron.hourly /var/spool/cron /var/spool/at

# Build-time checks: the venv works for root and for the agent account, and MuJoCo renders offscreen.
RUN python3 -c "import numpy, scipy, cv2, mujoco, PIL; print('[image] mujoco', mujoco.__version__)" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom size=\"1 1 .1\" type=\"plane\"/><camera name=\"c\" pos=\"0 0 1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 32, 32); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d, 'c'); assert r.render().mean() > 10; print('[image] offscreen render OK')" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy, cv2, mujoco, PIL; print('[image] agent-UID import OK')"

# The engine's root helpers run in the image's working directory; the base image's is /home/user, which the agent can
# write. A root-owned working directory keeps a module the agent leaves there from running as root.
WORKDIR /
