# The task's environment: NVIDIA Isaac Sim (with Isaac Lab) on the official ALE container base, for one NVIDIA RTX GPU,
# with the robot's model from NVIDIA's asset library and the commissioning service of the disassembly cell.
# `environment/` (less its `setup/`) is the sole build context. Nothing of the grader or of the reference is in here.
# The image is modelled on tasks/swarm/hazard_bay_build's (the same Isaac Sim 6.2 / Isaac Lab 3.0 stack and pins).
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

# A world-readable venv behind a world-traversable interpreter: setup and verify run as root, the agent, the
# reference and the cell's own account unprivileged, and all of them use this interpreter. Python 3.12: the engine
# stages `ale_verify` into whatever answers `python3` and refuses anything older; Isaac Sim 6 is built for 3.12.
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

# The robot: the UR20 of NVIDIA's public Isaac Sim asset library (seven files, pinned one by one; the folder carries
# its own licence), and Isaac Sim pointed at the local copy instead of the cloud (the sandbox has no network).
COPY fetch_isaac_assets.py isaac_assets.sha256 /opt/assets/
RUN python3 /opt/assets/fetch_isaac_assets.py /opt/assets/isaac_assets.sha256 /opt/assets/isaac \
    && chmod -R a+rX /opt/assets \
    && grep -rlZ "Assets/Isaac/6.2" /opt/venv/lib/python3.12/site-packages/isaacsim/apps \
       /opt/venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.storage.native/config \
       /opt/venv/lib/python3.12/site-packages/isaaclab/apps \
       | xargs -0 sed -i -E 's#https://omniverse-content-production\.s3(-us-west-2)?\.amazonaws\.com/Assets/Isaac/6\.2#/opt/assets/isaac#g'

# Kit writes its caches (cooked physics meshes, shaders, derived data), logs and settings next to its own files; the
# pip install leaves those folders absent and the agent account could not create them (an Isaac Sim the agent starts
# itself would then fail). Shared and sticky here; the cell's processes use folders of their own.
RUN for d in cache logs data; do install -d -m 1777 /opt/venv/lib/python3.12/site-packages/isaacsim/kit/$d; done

# The declared artifact path must exist even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# The cell's own account and the commissioning service: the cell (commission_service/packcell, a byte copy of the
# grader's) on the development packs, which an automation engineer commissions a program on before it runs unattended.
# Installed for the account `packcell` only (mode 700): the agent account can neither read nor list it.
# environment/setup/setup.sh starts it before the agent's session; the agent reaches it through a unix socket and
# gets the cell's public actions on a fixed number of sessions: no ground truth, no credit. The grader runs its own
# cell processes as the same account.
RUN useradd --system --create-home --home-dir /var/lib/packcell --shell /usr/sbin/nologin packcell \
    && chmod 700 /var/lib/packcell
COPY commission_service /opt/packcell-commission
RUN find /opt/packcell-commission -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && rm -rf /opt/packcell-commission/state \
    && mkdir -p /opt/packcell-commission/state \
    && chown -R packcell:packcell /opt/packcell-commission \
    && chmod -R u+rwX,go-rwx /opt/packcell-commission

ENV MPLBACKEND=Agg
# One linear-algebra kernel family and one thread, for every process of the image: a program's own arithmetic then
# repeats on every x86-64 host with AVX2 (docs/contributing-a-task.md, section 5).
ENV OPENBLAS_CORETYPE=Haswell OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

# Fail the build early if the venv is broken, prove the agent account can use it and that the cell's account can
# load the cell's code, and that the agent account cannot read the service (then leave the agent's home as the
# agent's: nothing root-owned in it). No GPU at build time: the simulator itself is first started by setup.
RUN HOME=/tmp/selfcheck python3 -c "import isaacsim, cv2, numpy, scipy; print('[image] isaacsim ok, numpy', numpy.__version__, 'cv2', cv2.__version__)" \
    && runuser -u user -- python3 -c "import isaacsim, cv2, numpy, scipy; print('[image] agent-UID import OK')" \
    && runuser -u packcell -- /opt/venv/bin/python3 -I -s -c "import sys; sys.path.insert(0, '/opt/packcell-commission'); import packcell.series, packcell.rules, packcell.wire, packcell.kin, packcell.scene, packcell.sensors; print('[image] the cell imports for its account')" \
    && test -f /opt/assets/isaac/Isaac/Robots/UniversalRobots/ur20/ur20.usd \
    && ! runuser -u user -- ls /opt/packcell-commission > /dev/null 2>&1 \
    && ! runuser -u user -- cat /opt/packcell-commission/dev_packs.json > /dev/null 2>&1 \
    && rm -rf /tmp/selfcheck /home/user/.nvidia-omniverse /home/user/.cache \
    && chown -R user:user /home/user
