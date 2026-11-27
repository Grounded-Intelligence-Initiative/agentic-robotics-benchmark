"""Run as root in a disposable task image; never on a workstation or shared host."""
import errno
import json
import os
from pathlib import Path
import pwd
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
PROBE = r'''
import ctypes, errno, importlib.util, json, math, os, platform, socket, subprocess, sys
import numpy as np
result = {}
for family in (socket.AF_INET, socket.AF_INET6, socket.AF_UNIX):
    try:
        sock = socket.socket(family, socket.SOCK_STREAM)
    except OSError as exc:
        result['socket_'+str(family)] = exc.errno == errno.EPERM
    else:
        sock.close(); result['socket_'+str(family)] = False
try:
    pair = socket.socketpair()
except OSError as exc:
    result['socketpair'] = exc.errno == errno.EPERM
else:
    [s.close() for s in pair]; result['socketpair'] = False
libc = ctypes.CDLL(None, use_errno=True)
ret = libc.syscall(425, 1, 0)
result['io_uring'] = ret == -1 and ctypes.get_errno() == errno.EPERM
if platform.machine() == 'x86_64':
    ret = libc.syscall(0x40000000 + 41, 2, 1, 0)
    result['x32'] = ret == -1 and ctypes.get_errno() == errno.EPERM
for label, path in json.loads(sys.argv[1]).items():
    try:
        with open(path, 'rb') as stream: stream.read(1)
    except OSError as exc:
        result[label] = exc.errno in (errno.EACCES, errno.EPERM)
    else:
        result[label] = False
for descriptor in json.loads(sys.argv[2]):
    try: os.fstat(descriptor)
    except OSError as exc: result['fd_'+str(descriptor)] = exc.errno == errno.EBADF
    else: result['fd_'+str(descriptor)] = False
result['environment'] = 'YAM_PRIVATE_SENTINEL' not in os.environ and 'PYTHONPATH' not in os.environ
result['math'] = math.isclose(float(np.dot([1., 2.], [3., 4.])), 11.)
result['identity'] = os.getuid() != 0 and os.geteuid() == os.getuid() and os.getgroups() == []
result['no_new_privs'] = 'NoNewPrivs:\t1' in open('/proc/self/status').read()
result['no_public_mujoco'] = importlib.util.find_spec('mujoco') is None
for family in (socket.AF_INET, socket.AF_INET6, socket.AF_UNIX):
    child = subprocess.run([sys.executable, '-c', 'import socket; socket.socket('+str(family)+', socket.SOCK_STREAM)'], capture_output=True)
    result['descendant_'+str(family)] = child.returncode != 0 and b'Operation not permitted' in child.stderr
print(json.dumps(result), flush=True)
'''


class IsolationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.geteuid() != 0 or not Path('/.dockerenv').exists():
            raise unittest.SkipTest('requires root in a disposable Docker task image')
        cls.account = pwd.getpwnam('user')

    def command(self, program):
        if os.environ.get('ISOLATION_TEST_UNCONFINED'):
            return ['/usr/sbin/runuser', '-u', 'user', '--'] + program
        return [sys.executable, '-I', '-S', str(HERE / 'nonet_exec.py'),
                '--uid', str(self.account.pw_uid), '--gid', str(self.account.pw_gid),
                '--cwd', '/home/user', '--'] + program

    def test_kernel_boundary_and_private_files(self):
        # These are actual task source and private-runtime paths, not fake mocks.
        libraries = list(Path('/opt/physics').glob('**/libmujoco.so.*'))
        library = libraries[0] if libraries else next(Path('/opt/venv').glob('**/libmujoco.so.*'))
        paths = {'source': '/opt/station/workcell.py', 'model': '/opt/station/yam/yam.xml',
                 'physics_library': str(library), 'root_memory': f'/proc/{os.getpid()}/mem',
                 'root_environment': f'/proc/{os.getpid()}/environ'}
        for path in paths.values():
            self.assertTrue(Path(path).exists(), path)
        with tempfile.TemporaryFile() as secret, socket.socket() as inherited_socket:
            descriptors = [secret.fileno(), inherited_socket.fileno()]
            command = self.command(['/opt/venv/bin/python3', '-c', PROBE,
                                    json.dumps(paths), json.dumps(descriptors)])
            result = subprocess.run(command, text=True, capture_output=True, timeout=20,
                                    pass_fds=descriptors,
                                    env={**os.environ, 'YAM_PRIVATE_SENTINEL': 'secret', 'PYTHONPATH': '/opt/station'})
        self.assertEqual(result.returncode, 0, result.stderr)
        checks = json.loads(result.stdout)
        print(json.dumps(checks, sort_keys=True), flush=True)
        self.assertTrue(all(checks.values()), checks)

    def test_root_identity_is_rejected(self):
        result = subprocess.run([sys.executable, '-I', '-S', str(HERE / 'nonet_exec.py'),
                                 '--uid', '0', '--gid', '0', '--cwd', '/tmp', '--', '/bin/true'],
                                capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('non-root controller UID/GID', result.stderr)

    def test_solver_cleanup_fails_closed(self):
        from isolated_solver import IsolatedSolver
        with patch('isolated_solver.clear_agent_account', return_value={'cleared': False}), \
                patch.object(IsolatedSolver, '_start') as start:
            with self.assertRaisesRegex(RuntimeError, 'account'):
                IsolatedSolver()
            start.assert_not_called()

    def test_solver_stdio_protocol(self):
        from isolated_solver import IsolatedSolver
        with tempfile.TemporaryDirectory() as directory:
            os.chmod(directory, 0o755)
            folder = Path(directory)
            (folder / 'run.sh').write_text("exec /opt/venv/bin/python3 protocol.py\n")
            (folder / 'protocol.py').write_text("import json,sys\nprint('READY',flush=True)\n"
                "for line in sys.stdin:\n msg=json.loads(line)\n"
                " print(json.dumps({'action':{'finish':True}} if msg['type']=='act' else {'ok':True}),flush=True)\n"
                " if msg['type']=='close':break\n")
            with IsolatedSolver(submission=directory, step_timeout=2) as solver:
                solver.reset()
                self.assertEqual(solver.act(0, {'rgbd': [], 'joints': []}), {'finish': True})
            solver.proc.stdout.close()


if __name__ == '__main__':
    unittest.main()
