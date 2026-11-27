# Task image. Nothing here depends on tests/, solution/ or the anchor: an image layer is readable by anyone who can
# run the image. The answers of the test events are NOT in the image.
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --python /opt/venv -r /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; }

# ---- IEDD's own code (GitHub egik-von/IEDD, Apache-2.0) at a pinned commit, checked file by file: the definitions
# every answer follows; environment/setup/setup.sh stages it into the agent home as iedd_reference/
ARG IEDD_COMMIT=0a530a0e2e69dcb5db397fa6a42350ce625278bb
RUN set -eu; d=/opt/iedd_reference; mkdir -p "$d/utils"; \
    for f in LICENSE utils/IEDD_compute.py utils/data_models.py utils/para.py; do \
      curl -fsSL --retry 4 "https://raw.githubusercontent.com/egik-von/IEDD/${IEDD_COMMIT}/$f" -o "$d/$f"; \
    done; \
    printf '%s\n' \
      'c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4  LICENSE' \
      '21d318e19fce99191ba2e7b33f65d9cb6c1ab9034ae37643a092230befe1383f  utils/IEDD_compute.py' \
      'c0de685d62c01656fdcd94cfb92c434240bab3d20f3ef39f4c5f5e00a37ace05  utils/data_models.py' \
      '7d6bd53492950954cb8336b5cb38e1f9f5fdfa04ad9a42f3d64b172b0f856c09  utils/para.py' \
      | (cd "$d" && sha256sum -c --quiet -); \
    chmod -R a+rX "$d"

# ---- task assets: fetched at the dataset commit pinned in assets.sha256, checked file by file (docs/task-assets.md):
# the test videos, the labelled videos and the labelled answers
COPY fetch_assets.py assets.sha256 /opt/task-assets/
RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload \
    && chmod -R a+rX /opt/task-assets

RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

ENV MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

# TODO(author): once the assets are uploaded, also check as the agent user that test/T01.mp4 decodes (see
# tasks/arm/arm_joint_angles_from_video/environment/Dockerfile); before the upload the asset list is empty.
RUN python3 -c "import numpy, scipy, cv2, PIL, matplotlib, skimage, sklearn; print('[image] cv2', cv2.__version__)" \
    && python3 -c "import site, sys; assert not site.ENABLE_USER_SITE or sys.prefix != sys.base_prefix" \
    && runuser -u user -- python3 -c "import numpy, scipy, cv2; print('[image] agent-UID import OK')" \
    && runuser -u user -- python3 -c "import sys; sys.path.insert(0, '/opt/iedd_reference'); from utils.IEDD_compute import calculate_q_i, calculate_all_metrics; print('[image] agent-UID IEDD reference import OK')"
