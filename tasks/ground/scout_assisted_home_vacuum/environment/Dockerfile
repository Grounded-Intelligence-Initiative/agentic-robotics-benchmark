FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa LP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
COPY requirements.txt /tmp/requirements.txt
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 libgl1 && rm -rf /var/lib/apt/lists/* \
    && uv venv /opt/venv --python 3.12 && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && mkdir -p /home/user/submission && chown user:user /home/user/submission \
    && runuser -u user -- python3 -c "import mujoco, numpy, PIL"
