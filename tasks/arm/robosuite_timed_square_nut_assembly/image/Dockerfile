FROM ghcr.io/agentslastexam/container-ubuntu22-base:latest
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV MUJOCO_GL=osmesa
ENV PYOPENGL_PLATFORM=osmesa
ENV CC=gcc CXX=g++
COPY requirements.txt /tmp/requirements.txt
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential linux-libc-dev libosmesa6 libgl1 libglfw3 \
    && rm -rf /var/lib/apt/lists/*
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission
RUN python3 -c "import mujoco, robosuite; print('MuJoCo', mujoco.__version__, 'robosuite', robosuite.__version__)" \
    && runuser -u user -- python3 -c "import mujoco, robosuite; print('agent imports OK')"
