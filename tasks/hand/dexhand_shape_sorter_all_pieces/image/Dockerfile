# Environment image for dexhand_shape_sorter_all_pieces. Final stage derives from the official ALE base.
# Contains the public SuperDex 1.0.0 simulator (double precision), the public SuperDex assets this task uses
# (shape_box prefab, Wuji Hand 2 right hand) and a finer collision SDF of the box baked at build time from the
# public mesh. setup/, verify/, oracle/ and the anchor are never copied into the image.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:latest

ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt superdex-requirements.txt /tmp/
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --retries 30 --timeout 60 --upgrade pip \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --retries 30 --timeout 60 -r /tmp/requirements.txt \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --retries 30 --timeout 60 --no-deps -r /tmp/superdex-requirements.txt \
    && rm -f /tmp/requirements.txt /tmp/superdex-requirements.txt \
    && chmod -R a+rX /opt/venv

# Public SuperDex assets (subset). Source: SuperDex asset tree (non-internal), see superdex_assets/*/LICENSE.
COPY superdex_assets /opt/superdex_assets
COPY bake_box_sdf.py /opt/sorter_tools/bake_box_sdf.py
ENV SUPERDEX_ASSETS_PATH=/opt/superdex_assets SUPERDEX_PRECISION=double SORTER_DATA=/opt/sorter_data \
    OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
RUN python3 -B /opt/sorter_tools/bake_box_sdf.py /opt/sorter_data \
    && chmod -R a+rX /opt/superdex_assets /opt/sorter_data /opt/sorter_tools

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission/plans && chown -R user:user /home/user/submission

# Build-time checks: double-precision SuperDex imports for root and for the agent account; no job scheduler.
RUN python3 -c "import os; os.environ['SUPERDEX_PRECISION']='double'; import superdex.physics as sp, superdex.robotics; assert sp.USE_DOUBLE_PRECISION; print('[image] superdex fp64 OK')" \
    && python3 -c "import site, sys; assert sys.prefix != sys.base_prefix" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import superdex.physics, trimesh, h5py, scipy; print('[image] agent-UID import OK')" \
    && ! command -v cron && ! command -v atd && ! command -v crond
