# dexterous_hand_grip_controller_from_docs's environment image (the control stage of the four-digit robot hand: a grip
# controller written from the documents of the hand, its servos and its tactile modules).
#
# `environment/` without its `setup/` folder is the sole build context and the final stage derives DIRECTLY from the
# official ALE base. The block below is the arm tasks' common base (the same RUN / COPY / ENV lines and the same pins in
# requirements.txt as the other arm tasks). Task layers: none beyond the base (the controls engineer works with numpy,
# scipy, pinocchio, coal, trimesh, matplotlib), then the idle-timeout ENV block, the numerics ENV lines, the image
# check (the last RUN) and the image's working directory (the last instruction). No hidden data: the givens are
# staged by environment/setup/setup.sh, the grader and its model of the hand live in tests/ (absent from the image).
FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1

COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

# OSMesa for software rendering (the image has no GPU, so every MuJoCo render goes through
# MUJOCO_GL=osmesa); the X / GL client libraries OCP and MuJoCo link even when headless; libgomp
# for manifold3d / CoACD.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libosmesa6 libgl1 libglu1-mesa libglew2.2 libxrender1 libxext6 libsm6 libxi6 \
        libxcursor1 libxinerama1 libxrandr2 libfontconfig1 libfreetype6 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# World-readable venv at /opt/venv: setup (root), the agent (`user`), the oracle (`user`) and
# verify (root) all use the same interpreter. UV_PYTHON_INSTALL_DIR must be world-readable too
# (the default /root/.local/share/uv is untraversable for the agent account).
#
# Python 3.12: the engine stages `ale_verify` into the `python3` it finds and refuses anything
# below 3.12. The ALE base ships a standalone CPython 3.12 at /opt/python, so uv links the venv
# against it.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PROJECT_ENVIRONMENT=/opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY requirements.txt /tmp/requirements.txt
# --no-cache: the wheel cache would otherwise stay inside this layer (~1 GB).
# uv leaves its two lock files world-writable (0666: /opt/venv/.lock and its cache's .lock): closed (0644) at the end
# of this RUN, the only one that calls uv (this RUN line differs from the arm tasks' common base by that chmod alone),
# so that no regular file of the image is writable by every account: a controller run after another one in the same
# verifier could otherwise read through them what the first one wrote.
RUN uv venv /opt/venv --python 3.12 \
    && uv pip install --no-cache --python /opt/venv -r /tmp/requirements.txt \
    && find /opt/venv -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && chmod -R a+rX /opt/venv \
    && { [ ! -d /opt/uv-python ] || chmod -R a+rX /opt/uv-python; } \
    && for f in /opt/venv/.lock /home/user/.cache/uv/.lock "${UV_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/uv}/.lock"; do \
           [ ! -e "$f" ] || chmod 0644 "$f"; done

# The declared artifact path must exist even when nobody works (the engine captures declared
# artifacts after every episode, untouched validation passes included).
RUN mkdir -p /home/user/submission && chown -R user:user /home/user/submission

# Headless defaults: MuJoCo renders through OSMesa inside the image; a task may override to egl
# where a GPU exists.
ENV MPLBACKEND=Agg
ENV MUJOCO_GL=osmesa
ENV PYOPENGL_PLATFORM=osmesa

# Fail the build early if the toolchain is broken, prove the AGENT account can use the venv, and
# prove an offscreen OSMesa render works. One import line per -c: a continuation line starting
# with `from` reads as a Dockerfile FROM to the final-stage lint.
RUN python3 -c "import mujoco, mink, pinocchio, coacd, trimesh, manifold3d, gymnasium, numpy, scipy, jsonschema, yaml, PIL, matplotlib, cadquery, OCP; print('[arb-base-mujoco] import OK mujoco', mujoco.__version__, 'cadquery', cadquery.__version__)" \
    && python3 -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos=\"0 0 1\"/><geom type=\"box\" size=\"0.1 0.1 0.1\"/></worldbody></mujoco>'); r = mujoco.Renderer(m, 64, 64); d = mujoco.MjData(m); mujoco.mj_forward(m, d); r.update_scene(d); px = r.render(); print('[arb-base-mujoco] OSMesa render OK', px.shape, int(px.max()))" \
    && runuser -u user -- python3 -c "import mujoco, mink, cadquery, coacd; print('[arb-base-mujoco] agent-UID import OK')"

# ---- task layers -----------------------------------------------------------------------------------------------------
# Stream idle timers: the claude-code harness of earlier engines aborted a model stream after 300 s without a byte
# (Bun's HTTP idle timeout and the CLI's stream watchdog); both are raised to the gateway's own 1800 s upstream ceiling.
# ENV is inherited by every process the guest service starts, and the profile.d file covers the login shell the CLI is
# launched with (`bash -lc`). Nothing else in this image reads them; the block stays as in the other arm tasks.
ENV BUN_CONFIG_HTTP_IDLE_TIMEOUT=1800 CLAUDE_STREAM_IDLE_TIMEOUT_MS=1800000
RUN printf '%s\n' 'export BUN_CONFIG_HTTP_IDLE_TIMEOUT=1800' 'export CLAUDE_STREAM_IDLE_TIMEOUT_MS=1800000' \
        > /etc/profile.d/99-ale-idle-timeouts.sh \
    && chmod 0644 /etc/profile.d/99-ale-idle-timeouts.sh

# The controller's numerics must be the same in the session, in the self-test and at acceptance, on every CPU type: the
# bundled OpenBLAS picks its kernels by the CPU it finds, so every process of this image uses one fixed kernel set
# (the two settings always go together; the acceptance also gives the controller one BLAS thread), and numpy's own
# loops stay on one code path (its AVX-512 loops round differently from the AVX2 ones). The acceptance gives every
# controller process the same three values.
ENV OPENBLAS_CORETYPE=Haswell
ENV OPENBLAS_L2_SIZE=256
ENV NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"

# The image check, the last RUN: the build fails when the controller's account (nobody) or every account can write a
# regular file anywhere in the image, or a directory outside SWEPT, the places the acceptance clears between two
# controllers (what that account made there is removed); the directories inside SWEPT that it can write are printed
# with their mode and owner. SWEPT is the acceptance's own list of those places (a test binds the two).
RUN SWEPT="/tmp /var/tmp /dev/shm /run/lock /dev/mqueue /home/user"; \
    files=$( { runuser -u nobody -- find / -xdev -type f -writable -print 2>/dev/null; \
               find / -xdev -type f -perm -0002 -print 2>/dev/null; } | sort -u ); \
    if [ -n "$files" ]; then echo "image check: regular files writable by nobody or by every account:" >&2; \
        echo "$files" >&2; exit 1; fi; \
    dirs=$( { runuser -u nobody -- find / -xdev -type d -writable -print 2>/dev/null; \
              find / -xdev -type d -perm -0002 -print 2>/dev/null; } | sort -u ); \
    bad=$(printf '%s\n' "$dirs" | while IFS= read -r d; do [ -n "$d" ] || continue; ok=0; \
        for r in $SWEPT; do case "$d" in "$r"|"$r"/*) ok=1;; esac; done; [ "$ok" = 1 ] || echo "$d"; done); \
    if [ -n "$bad" ]; then echo "image check: directories writable by nobody or by every account outside $SWEPT:" >&2; \
        echo "$bad" >&2; exit 1; fi; \
    echo "[image check] no regular file is writable by nobody or by every account; the directories they can write, all inside $SWEPT:"; \
    printf '%s\n' "$dirs" | while IFS= read -r d; do [ -n "$d" ] && stat -c '[image check] %a %U:%G %n' "$d"; done; true

# The image's working directory is not the agent's home: the engine runs commands of its own as root from the image's
# working directory (among them `python3 -c` copying the declared artifact out when the agent stops, which puts that
# directory first on sys.path), and the base image's is /home/user, which the agent can write. The agent still starts
# in its home (the engine starts its harness there), and setup.sh and test.sh in their staged folders.
WORKDIR /
