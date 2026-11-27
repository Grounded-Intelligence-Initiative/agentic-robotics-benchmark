"""Confinement and native action validation for the visual navigation solver."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import Solver, SolverAborted, deprivileged, require
from navigation_io import validate_action
import numpy as np


OFFLINE_SOURCE = (STAGE / "offline_exec.py").read_text()
MAX_REPLY_CHARS = 64 * 1024


def offline_command(entry):
    # The bootstrap imports only the isolated interpreter's standard library.
    return [sys.executable, "-I", "-S", "-c", OFFLINE_SOURCE, entry]


RUNTIME_ENV = (
    "OPENBLAS_CORETYPE", "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
    "NPY_DISABLE_CPU_FEATURES", "MUJOCO_GL", "PYOPENGL_PLATFORM", "EGL_PLATFORM",
    "__EGL_VENDOR_LIBRARY_FILENAMES", "LIBGL_ALWAYS_SOFTWARE", "GALLIUM_DRIVER", "LP_NUM_THREADS",
)


def solver_environment():
    env = dict(HOME="/home/user", PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin",
               LANG="C.UTF-8", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    # The image's root-installed startup file pins these before NumPy or MuJoCo imports.
    # Propagate only the explicit public runtime settings to native and Python solvers.
    env.update({key: os.environ[key] for key in RUNTIME_ENV})
    return env


def check_numerics():
    """Check the packaged float64 BLAS result before allocating any hidden episode."""
    i = np.arange(71 * 97, dtype=np.int64)
    j = np.arange(97 * 53, dtype=np.int64)
    a = ((i * 31337 + 11) % 3299 - 1649).reshape(71, 97) / 1291.0
    b = ((j * 30757 + 17) % 3511 - 1755).reshape(97, 53) / 1303.0
    digest = hashlib.sha256(np.asarray(a @ b, dtype="<f8").tobytes()).hexdigest()
    require(digest == "f83f8230aa5e4c8b0e595efe592952b387120788d96d7aed3696fea03728c0cc",
            "float64 matrix product differs from the pinned CPU runtime")


class OfflineSolver(Solver):
    def _spawn(self):
        return deprivileged(offline_command(self.entry), clear_account=self.clear_account,
                            cwd=self.submission, env=solver_environment(), stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                            bufsize=1, start_new_session=True)

    @staticmethod
    def _pump(proc, lines):
        # Legal replies are under 3 KiB. Bound unsolicited output on the trusted side.
        try:
            while True:
                line = proc.stdout.readline(MAX_REPLY_CHARS + 1)
                if not line or len(line) > MAX_REPLY_CHARS or lines.qsize() >= 32:
                    break
                lines.put(line)
        except (OSError, ValueError):
            pass
        finally:
            lines.put(None)

    def _exchange(self, message):
        reply = super()._exchange(message)
        if message["type"] == "reset":
            if set(reply) != {"ok"} or reply["ok"] is not True:
                raise SolverAborted("reset requires a true acknowledgment")
        elif message["type"] == "act":
            if set(reply) != {"action"}:
                raise SolverAborted("act requires a single action")
            try:
                validate_action(reply["action"], message["t"], message["obs"]["horizon"])
            except ValueError as exc:
                raise SolverAborted(str(exc)) from exc
        return reply
