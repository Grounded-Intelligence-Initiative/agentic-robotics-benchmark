FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1
USER root
RUN apt-get update && apt-get install -y --no-install-recommends libosmesa6 mesa-vulkan-drivers libvulkan1 \
    && rm -rf /var/lib/apt/lists/*
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV MUJOCO_GL=osmesa LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
# One numerics path on every host, here and for the graded program (tests/grader.py sets the same two): the OpenBLAS
# kernel and NumPy's dispatch otherwise follow the CPU, and an image-driven program then flies slightly differently on
# an AVX2 and an AVX-512 machine.
ENV OPENBLAS_CORETYPE=HASWELL NPY_DISABLE_CPU_FEATURES=AVX512F,AVX512CD,AVX512_SKX,AVX512_CLX,AVX512_CNL,AVX512_ICL,AVX512_SPR
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv
RUN mkdir -p /home/user/submission && chown user:user /home/user/submission
RUN runuser -u user -- python3 -c "import mujoco; m=mujoco.MjModel.from_xml_string('<mujoco/>'); r=mujoco.Renderer(m,64,64); r.close()"
COPY visual_assets.json fetch_visual_assets.py /tmp/visual_assets/
RUN /opt/venv/bin/python3 /tmp/visual_assets/fetch_visual_assets.py /tmp/visual_assets/visual_assets.json /opt/visual_robotics/assets \
    && chmod -R a+rX /opt/visual_robotics/assets
