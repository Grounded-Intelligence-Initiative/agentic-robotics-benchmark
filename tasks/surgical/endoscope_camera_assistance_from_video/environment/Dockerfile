# The task's environment: NVIDIA Isaac Sim (the RTX renderer that draws the endoscope's frames) on the official ALE
# container base, for one NVIDIA RTX GPU, plus the commissioning service the agent practises against.
# `environment/` (less its `setup/`) is the sole build context: nothing of the grader or of the reference is in here.
# The image does hold the commissioning service (the benchmark's simulator on development procedures), readable by
# the service's own account only. Modelled on tasks/swarm/hazard_bay_build (the same Isaac Sim 6.2 / Isaac Lab 3.0
# install, validated there).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

# Isaac Sim's runtime libraries. The GPU driver's own libraries are mounted by the container runtime at start
# (the sandbox needs the `graphics` driver capability for the RTX renderer), but the Vulkan and EGL loaders and
# their ICD manifests have to be in the image, as in NVIDIA's own Isaac Sim images.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libvulkan1 libglu1-mesa libegl1 libgl1 libxrandr2 libxt6 \
       libxkbcommon0 libxi6 libsm6 libice6 libxrender1 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/vulkan/icd.d /etc/vulkan/implicit_layer.d /usr/share/glvnd/egl_vendor.d \
    && printf '{\n  "file_format_version": "1.0.0",\n  "ICD": {"library_path": "libGLX_nvidia.so.0", "api_version": "1.3.194"}\n}\n' \
       > /etc/vulkan/icd.d/nvidia_icd.json \
    && printf '{\n  "file_format_version": "1.0.0",\n  "ICD": {"library_path": "libEGL_nvidia.so.0"}\n}\n' \
       > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Isaac Sim asks to accept its licence (NVIDIA Software License Agreement) on first start.
ENV OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y

# A world-readable venv behind a world-traversable interpreter: setup and the grader run as root, the agent and the
# reference unprivileged, the renderer and the commissioning service as their own account, and all of them use this
# interpreter. Python 3.12: the engine stages `ale_verify` into whatever answers `python3` and refuses anything
# older; Isaac Sim 6 is built for 3.12 as well.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
# The published pair does not resolve as is: isaacsim-kernel 6.2.0.0 pins urllib3==2.7.0, isaaclab 3.0.0 asks for
# urllib3>=2.8.0. The override settles it on the newer release. isaacsim-core pins a pre-release (tinyobjloader rc).
# (The base image's HOME is the agent's home: uv's cache goes to a folder of its own and is removed.)
RUN uv venv /opt/venv --python 3.12 \
    && echo "urllib3==2.8.0" > /tmp/overrides.txt \
    && UV_CACHE_DIR=/tmp/uv-cache UV_HTTP_TIMEOUT=600 uv pip install --python /opt/venv --index-strategy unsafe-best-match \
       --prerelease=if-necessary-or-explicit --override /tmp/overrides.txt -r /tmp/requirements.txt \
    && rm -rf /tmp/uv-cache /root/.cache/uv /home/user/.cache/uv \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# The scene is procedural: no file of NVIDIA's asset library is used. Isaac Sim's default asset root is pointed at an
# empty local folder all the same, so that nothing in the sandbox (which has no network) ever waits for the cloud.
RUN mkdir -p /opt/assets/isaac && chmod -R a+rX /opt/assets \
    && grep -rlZ "Assets/Isaac/6.2" /opt/venv/lib/python3.12/site-packages/isaacsim/apps \
       /opt/venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.storage.native/config \
       /opt/venv/lib/python3.12/site-packages/isaaclab/apps \
       | xargs -0 sed -i -E 's#https://omniverse-content-production\.s3(-us-west-2)?\.amazonaws\.com/Assets/Isaac/6\.2#/opt/assets/isaac#g'

# Kit writes its caches (shaders, derived data), logs and settings next to its own files; the pip install leaves
# those folders absent and an unprivileged account could not create them (RTX then fails to start). Shared and
# sticky here; the grader makes them the renderer account's alone before it starts its renderer.
RUN for d in cache logs data; do install -d -m 1777 /opt/venv/lib/python3.12/site-packages/isaacsim/kit/$d; done

# The commissioning service: the benchmark's simulator (endosim/, a byte copy of the grader's), which records the
# development set and plays commissioning procedures of its own, a device engineer's test bench. Installed for its own
# account `endosim` only (mode 700): the agent's account can neither read nor list it, so the procedure generator, the
# development and commissioning seeds and the commissioning procedures' rigs stay out of its reach.
# environment/setup/setup.sh starts it for the agent's session. The grader's renderer runs as this account too.
RUN useradd --system --no-create-home --home-dir /var/lib/endocam --shell /usr/sbin/nologin endosim
COPY commission_service /opt/endosim-commission
RUN find /opt/endosim-commission -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && chown -R endosim:endosim /opt/endosim-commission \
    && chmod -R u+rwX,go-rwx /opt/endosim-commission

# The declared artifact path must exist even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV MPLBACKEND=Agg

# Fail the build early if the venv is broken; prove that the agent's account can use it and that the service's
# account can import the service, and that the agent's account cannot read the service (then leave the agent's home
# as the agent's: nothing root-owned in it).
RUN HOME=/tmp/selfcheck python3 -c "import numpy, cv2, torch, isaacsim; print('[image] torch', torch.__version__, 'numpy', numpy.__version__)" \
    && runuser -u user -- python3 -c "import numpy, cv2, torch, torchvision, isaacsim; print('[image] agent-UID import OK')" \
    && runuser -u endosim -- env HOME=/tmp/selfcheck /opt/venv/bin/python3 -B -s -c "import sys; sys.path.insert(0, '/opt/endosim-commission'); import endosim.procedure, endosim.session, endosim.recorder, endosim.render_client; print('[image] commissioning service imports OK for its account')" \
    && ! runuser -u user -- ls /opt/endosim-commission > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/endosim-commission/dev_procedures.json > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/endosim-commission/commission_procedures.json > /dev/null 2>&1 \
    && rm -rf /tmp/selfcheck /home/user/.nvidia-omniverse /home/user/.cache \
    && chown -R user:user /home/user
