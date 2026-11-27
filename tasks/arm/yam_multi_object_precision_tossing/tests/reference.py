"""Host-authenticated, digest-pinned legacy reference controller.

ALE bfb7505 writes its ATIF trajectory after artifact restoration. Only that
root-owned verifier input can select the oracle; submission flags cannot.
"""
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile

from robotics_grader.account import clear_agent_account
from robotics_grader.solver import Solver
from robotics_grader.stage import agent_home, secure_exec_path

HERE = Path(__file__).resolve().parent
TRAJECTORY_PATH = HERE / 'trajectory.json'
RUNTIME_FILES = ('agent.py', 'arcade_moves.py', 'fixed_motions.json', 'grips.py',
                 'learned-games.json', 'learned-grips.json', 'manipulation.py',
                 'wood_reference.py')


def _root_directory(path, private=False):
    info = os.lstat(path)
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != 0
            or info.st_mode & (0o077 if private else 0o022)):
        raise RuntimeError('unsafe trusted directory: ' + str(path))


def is_reference_episode():
    """Accept only the engine's fixed-path, protected OracleHarness trajectory."""
    supplied = os.environ.get('ALE_TRAJECTORY_PATH')
    if not supplied:
        return False
    if supplied != str(TRAJECTORY_PATH):
        raise RuntimeError('unexpected engine trajectory path')
    try:
        _root_directory(TRAJECTORY_PATH.parent)
        fd = os.open(TRAJECTORY_PATH, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise RuntimeError('unsafe engine trajectory') from exc
    with os.fdopen(fd, 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
            raise RuntimeError('unsafe engine trajectory ownership or mode')
        try:
            payload = json.load(stream)
        except (ValueError, UnicodeError) as exc:
            raise RuntimeError('malformed engine trajectory') from exc
    if (not isinstance(payload, dict) or payload.get('schema_version') != 'ATIF-v1.8'
            or not isinstance(payload.get('agent'), dict)
            or not isinstance(payload['agent'].get('name'), str)):
        raise RuntimeError('unsupported engine trajectory provenance')
    return payload['agent']['name'] == 'oracle'


def _verified_sources(submission):
    """Read bytes once using no-follow descriptors; execute only these checked bytes."""
    manifest = json.loads((HERE / 'reference_manifest.json').read_text())['sha256']
    if set(manifest) != set(RUNTIME_FILES) | {'solve.sh'}:
        raise RuntimeError('invalid reference manifest')
    sources = {}
    try:
        directory = os.open(submission, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            for name in RUNTIME_FILES:
                fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
                with os.fdopen(fd, 'rb') as stream:
                    if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                        raise RuntimeError('reference source must be a regular file: ' + name)
                    data = stream.read()
                if hashlib.sha256(data).hexdigest() != manifest[name]:
                    raise RuntimeError('reference source digest mismatch: ' + name)
                sources[name] = data
        finally:
            os.close(directory)
    except OSError as exc:
        raise RuntimeError('unsafe or missing reference source') from exc
    return sources


class TrustedReferenceSolver(Solver):
    """The existing deadline/stdio protocol around a private copy of pinned code."""
    def __init__(self, step_timeout=15.0, submission=None, max_restarts=3):
        if not is_reference_episode():
            raise RuntimeError('trusted reference requires engine oracle provenance')
        if os.geteuid() != 0:
            raise RuntimeError('trusted reference requires root verifier')
        if not clear_agent_account(force=True).get('cleared'):
            raise RuntimeError('agent account could not be cleared before reference launch')
        secure_exec_path()
        for directory in ('/opt/station', '/opt/physics'):
            _root_directory(directory, private=True)
        sources = _verified_sources(submission or os.path.join(agent_home(), 'submission'))
        self._trusted_directory = tempfile.TemporaryDirectory(prefix='yam-reference-')
        try:
            folder = Path(self._trusted_directory.name)
            for name, data in sources.items():
                (folder / name).write_bytes(data)
            # Solver checks existence; _spawn never executes this compatibility-free stub.
            (folder / 'run.sh').write_text('# TrustedReferenceSolver owns process launch.\n')
            super().__init__(step_timeout=step_timeout, submission=str(folder),
                             max_restarts=max_restarts, clear_account=False)
        except BaseException:
            if getattr(self, 'proc', None) is not None:
                self._stop(grace=0)
            self._trusted_directory.cleanup()
            raise

    def _spawn(self):
        launcher = ('import runpy,sys; '
                    'sys.path[:0]=[sys.argv[1],"/opt/station"]; '
                    'runpy.run_path(sys.argv[1]+"/agent.py",run_name="__main__")')
        return subprocess.Popen(
            ['/opt/physics/bin/python', '-I', '-c', launcher, self.submission],
            cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1, start_new_session=True,
            close_fds=True, env={'PATH': '/usr/bin:/bin', 'MUJOCO_GL': 'egl',
                                 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'})

    def close(self):
        try:
            super().close()
        finally:
            self._trusted_directory.cleanup()


def reference_observation(self):
    """Legacy workcell observation copied verbatim from 4ae07a0 (body unchanged)."""
    return {
        "time": float(self.data.time),
        "previous_item": self.actions[-1]["item"] if self.actions else None,
        "arms": {
            name: {
                "joints": self.data.qpos[arm["qpos"][:6]].tolist(),
                "reference": arm["ref"].tolist(),
                "jaw": float(self.data.qpos[arm["qpos"][6]]),
                "base": self.data.body(f"{name}_base").xpos.tolist(),
                "hand": self.data.body(f"{name}_gripper").xpos.tolist(),
            }
            for name, arm in self.arms.items()
        },
        "items": [
            {
                "id": item.id, "arm": item.arm, "mode": item.mode, "variant": item.variant,
                "label": item.label, "position": self.data.xpos[item.body].tolist(),
                "target": self.data.geom(item.target).xpos.tolist(),
                "attempts": item.attempts, "scored": self.scored(item),
            }
            for item in self.items.values()
        ],
    }
