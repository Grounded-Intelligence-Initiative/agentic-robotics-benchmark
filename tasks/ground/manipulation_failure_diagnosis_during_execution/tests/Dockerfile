# The separate verification image. The same Python and CLIP the agent's program uses (it runs here), plus the hidden
# episodes it is graded on, root-only under /opt/hidden_episodes. The answer key stays in tests/hidden/ and is not
# copied into any image layer. The hidden episodes and CLIP come from the benchmark's asset dataset, pinned in
# assets.sha256 (docs/task-assets.md).
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

COPY fetch_assets.py assets.sha256 /tmp/task-assets/
RUN python3 /tmp/task-assets/fetch_assets.py /tmp/task-assets/assets.sha256 /tmp/task-assets/files \
    && mkdir -p /opt/models && mv /tmp/task-assets/files/models/clip-vit-base-patch16 /opt/models/clip-vit-base-patch16 \
    && mv /tmp/task-assets/files/hidden /opt/hidden_episodes \
    && rm -rf /tmp/task-assets \
    && chmod -R a+rX /opt/models \
    && chown -R root:root /opt/hidden_episodes && chmod -R go-rwx /opt/hidden_episodes

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV MPLBACKEND=Agg OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1

RUN python3 -c "import numpy, scipy, cv2, torch, transformers, sklearn; print('[image] torch', torch.__version__)" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, cv2, torch, transformers; print('[image] agent-UID import OK')" \
    && test -d /opt/hidden_episodes
