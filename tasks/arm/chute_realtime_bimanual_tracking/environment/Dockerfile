# Environment image of chute_realtime_bimanual_tracking. Build context: environment/ less setup/.
# Nothing from tests/ or solution/ is here.
#
# Two interpreters:
#   /opt/venv  py3.12  answers `python3`: verify stage, practice client, the agent's tooling.
#   /opt/sim   py3.12  runs the simulator process (Isaac Sim 6.1 + OmniGibson). Never on PATH.
#
# Third-party material and licences (see NOTICE.md beside this file):
#   OmniGibson, BEHAVIOR-1K repository code         MIT
#   Galaxea R1Pro model (omnigibson-robot-assets)    MIT
#   YCB objects and Simple_Room, NVIDIA Isaac assets NVIDIA Omniverse asset licence (same source as the merged Isaac task)
# No BEHAVIOR-1K scene or object data is used or downloaded: its licence forbids redistribution.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_NO_CACHE=1
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH

RUN apt-get update && apt-get install -y --no-install-recommends \
        git patch unzip libglu1-mesa libxt6 libxrandr2 libxinerama1 libxcursor1 libxi6 libgl1 libvulkan1 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Simulator interpreter. Permissions are fixed in the same layer (a later chmod would double it). OmniGibson writes
# its Kit app file into isaacsim/apps at every start, so that folder is writable by the agent account too.
COPY requirements-sim.txt /tmp/requirements-sim.txt
RUN uv venv /opt/sim --python 3.12 \
    && UV_HTTP_TIMEOUT=900 UV_CONCURRENT_DOWNLOADS=4 uv pip install --python /opt/sim \
         -r /tmp/requirements-sim.txt --extra-index-url https://pypi.nvidia.com \
            --extra-index-url https://download.pytorch.org/whl/cu128 --index-strategy unsafe-best-match \
    && mkdir -p /opt/sim/lib/python3.12/site-packages/omni/cache \
    && chmod -R a+rX /opt/sim && chmod -R a+rwX /opt/sim/lib/python3.12/site-packages/omni/cache \
         /opt/sim/lib/python3.12/site-packages/isaacsim/apps \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# OmniGibson at a fixed commit, with the small patch that ports it to Isaac Sim 6.1.
COPY omnigibson-isaac61.patch /tmp/omnigibson-isaac61.patch
RUN git clone https://github.com/StanfordVL/BEHAVIOR-1K.git /opt/BEHAVIOR-1K \
    && cd /opt/BEHAVIOR-1K && git checkout bd049de3119acdcdf2334fe9e1ebe060fa20c108 \
    && git apply /tmp/omnigibson-isaac61.patch && rm -rf .git \
    && uv pip install --python /opt/sim --no-deps -e /opt/BEHAVIOR-1K/bddl3 -e /opt/BEHAVIOR-1K/OmniGibson \
    && chmod -R a+rX /opt/BEHAVIOR-1K

# Robot models only (R1Pro is the one used). OMNIGIBSON_DATA_PATH is where OmniGibson looks for them.
ENV OMNIGIBSON_DATA_PATH=/opt/og_data
ENV OMNIGIBSON_APPDATA_PATH=/tmp/og-appdata
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV ACCEPT_EULA=Y
COPY fetch_robot_assets.py /tmp/fetch_robot_assets.py
RUN mkdir -p /opt/og_data \
    && /usr/bin/python3 -I /tmp/fetch_robot_assets.py /opt/og_data/omnigibson-robot-assets \
    && chmod -R a+rX /opt/og_data

# Stock Isaac assets from NVIDIA's public bucket, fetched at build time so every stage runs without network.
COPY mirror_assets.py nvidia-assets.json /tmp/
RUN python3 -I /tmp/mirror_assets.py /opt/isaac_assets Isaac/Props/YCB/Axis_Aligned/ Isaac/Environments/Simple_Room/ \
    && mkdir -p /opt/isaac_assets/NVIDIA && chmod -R a+rX /opt/isaac_assets

COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && UV_HTTP_TIMEOUT=900 uv pip install --python /opt/venv -r /tmp/requirements.txt \
         --extra-index-url https://download.pytorch.org/whl/cu128 --index-strategy unsafe-best-match \
    && chmod -R a+rX /opt/venv

# The 20 parts: textured visual mesh plus convex collision parts, built on the CPU from the YCB files above
# (plain usd-core in /opt/venv: Kit's own pxr only loads inside a running Kit).
COPY build_items.py /tmp/build_items.py
RUN /opt/venv/bin/python -I /tmp/build_items.py /opt/cell_items /opt/isaac_assets/Isaac/Props/YCB/Axis_Aligned \
    && chmod -R a+rX /opt/cell_items
ENV CELL_ITEMS=/opt/cell_items/items.json
ENV CELL_ROOM=/opt/isaac_assets/Isaac/Environments/Simple_Room/simple_room.usd
ENV CELL_SIM_PYTHON=/opt/sim/bin/python

RUN mkdir -p /home/user/submission && chown -R user:user /home/user
ENV MPLBACKEND=Agg

RUN python3 -c "import sys; assert sys.version_info >= (3, 12); import numpy, PIL; print('[image] python3', sys.version.split()[0])" \
    && test -s /opt/cell_items/items.json && test -s "$CELL_ROOM" && test -d /opt/og_data/omnigibson-robot-assets/models/r1pro \
    && runuser -u user -- /opt/sim/bin/python -c "import isaacsim, numpy, PIL; print('[image] agent-UID isaacsim import OK')"

# The port references this custom experience; a git diff of tracked upstream files
# alone omits it. Keep it in the build context and check the launch selection as user.
COPY omnigibson_6_1_0.kit /opt/BEHAVIOR-1K/OmniGibson/omnigibson/omnigibson_6_1_0.kit
COPY check_sim_assets.py /opt/cell-check-sim-assets.py
RUN runuser -u user -- /opt/sim/bin/python -I /opt/cell-check-sim-assets.py
