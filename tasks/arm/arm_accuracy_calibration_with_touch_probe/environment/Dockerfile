# Environment image for arm_accuracy_calibration_with_touch_probe. Final stage derives from the official ALE base.
# It holds only the numerical stack. The cell's simulator, the units, tests/ and solution/ are never copied into
# the image (the build context is environment/ less setup/).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# Python 3.12 venv owned by root, readable and traversable by every account, first on PATH for every stage.
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
ENV PYTHONNOUSERSITE=1
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir --upgrade pip \
    && /opt/venv/bin/python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt \
    && chmod -R a+rX /opt/venv
ENV OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

# The lab service's dedicated account: the lab never runs as root or as the agent while it serves.
RUN useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin alelab

# The declared artifact path exists even when nobody works.
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# Build-time checks: the venv works for root, for the agent account and for the lab account.
RUN python3 -c "import numpy, scipy; print('[image] numpy', numpy.__version__, 'scipy', scipy.__version__)" \
    && python3 -c "import sys; assert sys.prefix != sys.base_prefix" \
    && runuser -u user -- /opt/venv/bin/python3 -c "import numpy, scipy.optimize; print('[image] agent-UID import OK')" \
    && runuser -u alelab -- /opt/venv/bin/python3 -c "import numpy; print('[image] lab-UID import OK')"
