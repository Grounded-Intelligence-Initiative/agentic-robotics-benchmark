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
    OPENBLAS_CORETYPE=Haswell \
    OPENBLAS_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 \
    NPY_DISABLE_CPU_FEATURES="AVX512F AVX512CD AVX512_KNL AVX512_KNM AVX512_SKX AVX512_CLX AVX512_CNL AVX512_ICL" \
    OMP_NUM_THREADS=1

COPY requirements.txt /tmp/sonic-requirements.txt
RUN HOME=/root /usr/local/bin/python3 -m venv /opt/venv \
    && HOME=/root python -m pip install --no-cache-dir --only-binary=:all: -r /tmp/sonic-requirements.txt \
    && chmod -R a+rX /opt/venv

COPY fetch_public_assets.py public_assets.sha256 NOTICE /opt/sonic/
RUN python /opt/sonic/fetch_public_assets.py /opt/sonic/assets
# The grader starts submissions with a minimal environment. Save the image's
# selected numeric and EGL values in the venv, before NumPy or MuJoCo imports.
# NumPy 1.26.4's AMD64 wheel dispatches exactly these eight AVX-512 groups.
RUN python -c "import os, pathlib, sysconfig; names='OPENBLAS_CORETYPE OPENBLAS_NUM_THREADS OMP_NUM_THREADS MKL_NUM_THREADS NPY_DISABLE_CPU_FEATURES MUJOCO_GL PYOPENGL_PLATFORM EGL_PLATFORM __EGL_VENDOR_LIBRARY_FILENAMES LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER LP_NUM_THREADS'.split(); pins={k: os.environ[k] for k in names}; (pathlib.Path(sysconfig.get_path('purelib')) / '00_sonic_runtime.pth').write_text('import os; os.environ.update(' + repr(pins) + ')\n')" \
    && printf '%s\n' 'export PATH="/opt/venv/bin:$PATH"' > /etc/profile.d/20-sonic-runtime.sh \
    && chmod 0644 /etc/profile.d/20-sonic-runtime.sh \
    && env -i HOME=/tmp PATH=/opt/venv/bin:/usr/bin:/bin /opt/venv/bin/python3 -I -W error::ImportWarning -c "import os,numpy,scipy; import numpy.core._multiarray_umath as m; names=set(os.environ['NPY_DISABLE_CPU_FEATURES'].split()); assert names == {f for f in m.__cpu_dispatch__ if f.startswith('AVX512')}; assert not any(m.__cpu_features__[f] for f in names); assert m.__cpu_features__['AVX2'] and m.__cpu_features__['FMA3']; assert os.environ['OPENBLAS_CORETYPE']=='Haswell'"

COPY sonic_sim/*.py /opt/sonic/sonic_sim/
COPY sonic_io.py /opt/sonic/
RUN python -c "import pathlib, sysconfig; (pathlib.Path(sysconfig.get_path('purelib')) / 'sonic_runtime.pth').write_text('/opt/sonic\n')" \
    && chmod -R a+rX /opt/sonic \
    && install -d -o user -g user /home/user/submission \
    && runuser -u user -- /opt/venv/bin/python -c "import sonic_io, sonic_sim; s=sonic_sim.BenchSession('box_to_pad_challenge', 0, '/opt/sonic/assets'); assert s.camera('hand', 64, 48).shape == (48, 64, 3); s.close()"
