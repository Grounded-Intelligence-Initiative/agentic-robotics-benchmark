FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/physics-requirements.txt
RUN uv venv /opt/physics --python /usr/local/bin/python3 && uv pip install --no-cache --python /opt/physics/bin/python -r /tmp/physics-requirements.txt pillow==11.3.0
RUN uv venv /opt/venv --python /usr/local/bin/python3 && uv pip install --no-cache --python /opt/venv/bin/python numpy==2.5.5 scipy==1.16.3 pillow==11.3.0 opencv-python-headless==4.11.0.86
COPY yam_numerics.pth /opt/venv/lib/python3.12/site-packages/yam_numerics.pth
COPY yam_numerics.pth /opt/physics/lib/python3.12/site-packages/yam_numerics.pth
COPY public_kit/ /opt/public-kit/
RUN echo /opt/public-kit > /opt/venv/lib/python3.12/site-packages/yam_rgbd_client.pth
COPY yam/LICENSE yam/SOURCE.md yam/MESHES.sha256 yam/install_meshes.sh yam/yam.xml /opt/station/yam/
RUN sh /opt/station/yam/install_meshes.sh /opt/station
COPY station.py station_precision.py arena.py workcell.py arena_episode.py rgbd_sensor.py finger-hulls.json /opt/station/
RUN chown -R root:root /opt/physics /opt/station /opt/venv /opt/public-kit && chmod 700 /opt/physics /opt/station && chmod -R go-w /opt/venv \
    && mkdir -p /home/user/submission && chown user:user /home/user/submission \
    && PYTHONPATH=/opt/station MUJOCO_GL=egl /opt/physics/bin/python -c "from workcell import ArenaRound; a=ArenaRound(17); assert len(a.observation()['cameras']) == 4; a.close()" \
    && runuser -u user -- /opt/venv/bin/python -c "import importlib.util,numpy,scipy,PIL,cv2; assert importlib.util.find_spec('mujoco') is None"
