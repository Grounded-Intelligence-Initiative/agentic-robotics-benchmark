"""Task-local confinement for the canonical stdio Solver; kit remains unchanged."""
import os
from pathlib import Path
import pwd
import stat
import subprocess
import sys

from robotics_grader.account import clear_agent_account
from robotics_grader.solver import Solver
from robotics_grader.stage import agent_user, secure_exec_path


class IsolatedSolver(Solver):
    def __init__(self, *args, **kwargs):
        if os.geteuid() != 0:
            raise RuntimeError('isolated solver requires the trusted root verifier')
        report = clear_agent_account(force=True)
        if not report.get('cleared'):
            raise RuntimeError('agent account could not be cleared before confinement')
        for path in ('/opt/station', '/opt/physics'):
            metadata = os.lstat(path)
            if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_mode & 0o077:
                raise RuntimeError('private simulator directory is not root-only: ' + path)
        if kwargs.get('clear_account') is False:
            raise RuntimeError('isolated solver cannot retain the agent account session')
        super().__init__(*args, **kwargs)

    def _spawn(self):
        secure_exec_path()
        account = pwd.getpwnam(agent_user())
        launcher = Path(__file__).resolve().with_name('nonet_exec.py')
        command = [sys.executable, '-I', '-S', str(launcher), '--uid', str(account.pw_uid),
                   '--gid', str(account.pw_gid), '--cwd', self.submission, '--', '/bin/bash', self.entry]
        return subprocess.Popen(command, cwd='/', stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, bufsize=1,
                                start_new_session=True, close_fds=True, env={'PATH': '/usr/bin:/bin'})
