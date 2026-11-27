FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_NO_CACHE=1
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }
# Only the separate verifier builds these frames. No hidden assets are uploaded to
# the benchmark's public asset dataset or included in any agent image layer.
COPY prepare_hidden.py prepare_release.py raw_annotations.py /opt/uav-build/
RUN python /opt/uav-build/prepare_hidden.py \
    && chown -R root:root /opt/uav-heldout \
    && chmod 0700 /opt/uav-heldout \
    && rm -rf /opt/uav-build
# Preserve the source dataset's redistribution notice in the separate image too.
COPY nps-LICENSE.txt /opt/uav-heldout/LICENSE.txt
RUN mkdir -p /home/user/submission && chown user:user /home/user/submission
RUN runuser -u user -- python -c "import torch, torchvision, PIL, pycocotools; from torchvision.ops import nms; assert nms(torch.tensor([[0.,0.,1.,1.]]),torch.tensor([1.]),0.5).tolist()==[0]" \
    && runuser -u user -- test ! -r /opt/uav-heldout/test/annotations.json
ENV MPLBACKEND=Agg
