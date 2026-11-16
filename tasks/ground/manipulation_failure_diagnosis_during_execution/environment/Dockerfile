# The agent's image. Nothing here depends on tests/ or solution/. The practice episodes and CLIP come from the benchmark's
# asset dataset, pinned in assets.sha256 (docs/task-assets.md).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV UV_INDEX_STRATEGY=unsafe-best-match
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# ---- task assets: practice episodes (agent-visible, staged by setup.sh) and CLIP weights (/opt/models)
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && mkdir -p /opt/models && mv /opt/task-assets/payload/models/clip-vit-base-patch16 /opt/models/clip-vit-base-patch16 \
    && rmdir /opt/task-assets/payload/models \
    && chmod -R a+rX /opt/models /opt/task-assets

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV MPLBACKEND=Agg OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1

RUN python3 -c "import numpy, scipy, cv2, torch, transformers, sklearn; print('[image] torch', torch.__version__)" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, cv2, torch, transformers; print('[image] agent-UID import OK')"
