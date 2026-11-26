FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

# The official base provides world-traversable CPython 3.12, EGL/GLVND and
# Mesa's swrast DRI driver. Force llvmpipe so cameras work on CPU-only hosts.
ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl \
    EGL_PLATFORM=surfaceless \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    LP_NUM_THREADS=2 \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1

COPY requirements.txt /tmp/sonic-requirements.txt
RUN HOME=/root /usr/local/bin/python3 -m venv /opt/venv \
    && HOME=/root python -m pip install --no-cache-dir --only-binary=:all: -r /tmp/sonic-requirements.txt \
    && chmod -R a+rX /opt/venv

# ALE's interactive agent starts a login shell. Restore the same Python and
# offscreen settings even when login clears the process environment.
RUN printf '%s\n' 'export PATH="/opt/venv/bin:$PATH"' > /etc/profile.d/20-sonic-runtime.sh \
    && env | grep -E '^(MUJOCO_GL|PYOPENGL_PLATFORM|EGL_PLATFORM|__EGL_VENDOR_LIBRARY_FILENAMES|LIBGL_ALWAYS_SOFTWARE|GALLIUM_DRIVER|LP_NUM_THREADS|OPENBLAS_NUM_THREADS|OMP_NUM_THREADS)=' \
       | sed 's/^/export /' >> /etc/profile.d/20-sonic-runtime.sh \
    && chmod 0644 /etc/profile.d/20-sonic-runtime.sh

COPY fetch_public_assets.py public_assets.sha256 NOTICE /opt/sonic/
RUN python /opt/sonic/fetch_public_assets.py /opt/sonic/assets
COPY sonic_sim/*.py /opt/sonic/sonic_sim/
COPY sonic_io.py /opt/sonic/
RUN python -c "import pathlib, sysconfig; (pathlib.Path(sysconfig.get_path('purelib')) / 'sonic_runtime.pth').write_text('/opt/sonic\n')" \
    && chmod -R a+rX /opt/sonic \
    && install -d -o user -g user /home/user/submission \
    && runuser -u user -- /opt/venv/bin/python -c "import sonic_io, sonic_sim; s=sonic_sim.BenchSession('box_to_pad_challenge', 0, '/opt/sonic/assets'); assert s.camera('hand', 64, 48).shape == (48, 64, 3); s.close()"
