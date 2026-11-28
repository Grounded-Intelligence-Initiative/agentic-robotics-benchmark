FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates libvulkan1 libglu1-mesa libegl1 libgl1 \
       libxrandr2 libxt6 libxkbcommon0 libxi6 libsm6 libice6 libxrender1 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/vulkan/icd.d /usr/share/glvnd/egl_vendor.d \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.194"}}\n' \
       > /etc/vulkan/icd.d/nvidia_icd.json \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' \
       > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONPATH=/opt/curtain

COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && printf 'urllib3==2.8.0\n' > /tmp/overrides.txt \
    && UV_CACHE_DIR=/tmp/uv-cache UV_HTTP_TIMEOUT=600 uv pip install --python /opt/venv \
       --index-strategy unsafe-best-match --prerelease=if-necessary-or-explicit \
       --override /tmp/overrides.txt -r /tmp/requirements.txt \
    && rm -rf /tmp/uv-cache \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

COPY curtain_env.py curtain_scene.py curtain_scoring.py practice.py /opt/curtain/
COPY robot_description/ /opt/curtain/robot_description/
COPY simulation_assets/curtain.usda simulation_assets/rod.usda simulation_assets/LICENSE /opt/curtain/simulation_assets/

RUN curl --fail --silent --show-error --location \
      'https://github.com/frankarobotics/franka_simulation/releases/download/v0.1.0/mobile_fr3_duo_with_vision_and_manipulation_kit-isaacsim6.tar.gz' \
      --output /tmp/mobile_fr3_duo.tar.gz \
    && printf '26b3543fe23085304f20188196dcde67d13e90f779adf2b2428f89637e54ffb8  /tmp/mobile_fr3_duo.tar.gz\n' | sha256sum -c - \
    && tar -xzf /tmp/mobile_fr3_duo.tar.gz -C /opt/curtain/simulation_assets \
    && rm /tmp/mobile_fr3_duo.tar.gz \
    && chmod -R a+rX /opt/curtain \
    && for d in cache logs data; do install -d -m 1777 /opt/venv/lib/python3.12/site-packages/isaacsim/kit/$d; done

RUN install -d -o user -g user /home/user/submission \
    && runuser -u user -- python3 -c 'import isaacsim, isaaclab, isaaclab_physx, numpy, scipy, PIL'
