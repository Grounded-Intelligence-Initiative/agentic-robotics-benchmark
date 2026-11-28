"""Confinement and native action validation for the visual navigation solver."""
from contextlib import contextmanager
import hashlib
import os
import pwd
import shutil
import tempfile
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import Solver, SolverAborted, SolverMissing, agent_user, clear_agent_account, deprivileged, require
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


def solver_environment(scratch=None):
    env = dict(HOME="/home/user", PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin",
               LANG="C.UTF-8", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    # The image's root-installed startup file pins these before NumPy or MuJoCo imports.
    # Propagate only the explicit public runtime settings to native and Python solvers.
    env.update({key: os.environ[key] for key in RUNTIME_ENV})
    if scratch is not None:
        env.update(TMPDIR=scratch, XDG_CACHE_HOME=scratch, XDG_CONFIG_HOME=scratch)
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


@contextmanager
def submission_template(submission):
    """Preserve the uploaded tree in a root-only directory. Give each member new file inodes."""
    submission = Path(submission)
    if submission.is_symlink() or not submission.is_dir() or not (submission / "run.sh").is_file():
        raise SolverMissing("missing submission/run.sh")
    def copy_contents(source, destination):
        # Preserve ownership and permissions of private weights. Archive mode retains symlinks.
        subprocess.run(["/bin/cp", "-a", "--reflink=auto", "--", str(source) + "/.", destination], check=True)

    def clear_contents():
        # Preserve the actual submission directory, which may be a mounted artifact.
        for child in submission.iterdir():
            if child.is_symlink() or not child.is_dir():
                child.unlink()
            else:
                shutil.rmtree(child)

    with tempfile.TemporaryDirectory(prefix=".navigation-template-") as private:
        saved = Path(private) / "submission"
        saved.mkdir()
        copy_contents(submission, saved)
        try:
            @contextmanager
            def installed():
                clear_contents()
                copy_contents(saved, submission)
                yield submission
            yield installed
        finally:
            clear_contents()
            copy_contents(saved, submission)


@contextmanager
def episode_scratch():
    """Provide fresh standard scratch paths and reap participant processes after the episode."""
    require(clear_agent_account(force=True)["cleared"], "could not clear the participant account")
    with tempfile.TemporaryDirectory(prefix="visual-navigation-", dir="/tmp") as scratch:
        user = pwd.getpwnam(agent_user())
        os.chown(scratch, user.pw_uid, user.pw_gid)
        try:
            yield scratch
        finally:
            require(clear_agent_account(force=True)["cleared"], "could not clean up the participant account")


@contextmanager
def participant_episodes(step_timeout):
    """Restore the submitted tree at its normal writable path for each course."""
    home = Path(os.environ.get("ALE_HOME", "/home/user"))
    with submission_template(home / "submission") as install:
        @contextmanager
        def fresh_solver():
            with install() as submission, episode_scratch() as scratch:
                with OfflineSolver(scratch=scratch, submission=submission, step_timeout=step_timeout) as solver:
                    yield solver
        yield fresh_solver


class OfflineSolver(Solver):
    def __init__(self, *, scratch=None, **options):
        self.scratch = scratch
        super().__init__(**options)

    def _spawn(self):
        return deprivileged(offline_command(self.entry), clear_account=self.clear_account,
                            cwd=self.submission, env=solver_environment(self.scratch), stdin=subprocess.PIPE,
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
