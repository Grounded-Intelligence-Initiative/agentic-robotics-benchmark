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
COPY fetch_assets.py assets.sha256 install_public.py unpack_public.py /opt/uav-assets/
RUN python /opt/uav-assets/install_public.py /opt/uav-assets/assets.sha256 /opt/uav-data \
    && chmod -R a+rX /opt/uav-data
# Generic ImageNet initialization, never a UAV detector or reference solution.
RUN curl -fL --retry 5 https://download.pytorch.org/models/resnet18-f37072fd.pth \
         -o /opt/uav-data/resnet18-f37072fd.pth \
    && echo 'f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec  /opt/uav-data/resnet18-f37072fd.pth' | sha256sum -c - \
    && chmod 0644 /opt/uav-data/resnet18-f37072fd.pth
RUN mkdir -p /home/user/submission && chown user:user /home/user/submission
RUN runuser -u user -- python -c "import torch, torchvision, PIL, pycocotools; from torchvision.ops import nms; assert nms(torch.tensor([[0.,0.,1.,1.]]),torch.tensor([1.]),0.5).tolist()==[0]; print('agent imports and detection operators OK')"
ENV MPLBACKEND=Agg
