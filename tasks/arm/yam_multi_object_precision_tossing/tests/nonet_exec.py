"""Trusted root launcher: stdio only, no inherited handles or network sockets.

Adapted from tasks/swarm/hazard_bay_build/tests/nonet_exec.py. Unlike that
launcher, every socket family (including AF_UNIX) is denied. The controller
needs only its inherited stdin/stdout/stderr. Execute with Python -I -S.
"""
import argparse
import ctypes
import os
import platform
import pwd
import resource
import struct

ARCH = {
    'x86_64': (0xC000003E, 41, 53, 425, 0x40000000),
    'aarch64': (0xC00000B7, 198, 199, 425, 0),
}
PUBLIC_PATH = '/opt/venv/bin:/usr/local/bin:/usr/bin:/bin'


def install_filter():
    """Fail closed for unsupported ABIs or unavailable seccomp/no_new_privs."""
    audit, socket, socketpair, io_uring, x32 = ARCH[platform.machine()]
    deny, allow = 0x00050001, 0x7FFF0000  # EPERM, SECCOMP_RET_ALLOW
    instructions = [(0x20, 0, 0, 4), (0x15, 1, 0, audit), (0x06, 0, 0, deny),
                    (0x20, 0, 0, 0)]
    if x32:
        instructions += [(0x35, 0, 1, x32), (0x06, 0, 0, deny)]
    for syscall in (socket, socketpair, io_uring):
        instructions += [(0x15, 0, 1, syscall), (0x06, 0, 0, deny)]
    instructions += [(0x06, 0, 0, allow)]
    program = b''.join(struct.pack('HBBI', *row) for row in instructions)
    buffer = ctypes.create_string_buffer(program, len(program))

    class SockFprog(ctypes.Structure):
        _fields_ = [('length', ctypes.c_ushort), ('filter', ctypes.c_void_p)]

    fprog = SockFprog(len(instructions), ctypes.addressof(buffer))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(38, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'PR_SET_NO_NEW_PRIVS failed')
    if libc.prctl(22, 2, ctypes.byref(fprog), 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'PR_SET_SECCOMP failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uid', type=int, required=True)
    parser.add_argument('--gid', type=int, required=True)
    parser.add_argument('--cwd', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if os.geteuid() != 0 or args.uid <= 0 or args.gid <= 0 or not command:
        raise RuntimeError('requires root launcher and a non-root controller UID/GID')
    account = pwd.getpwuid(args.uid)
    if account.pw_gid != args.gid:
        raise RuntimeError('controller UID/GID do not match the account')
    environment = {'PATH': PUBLIC_PATH, 'HOME': account.pw_dir, 'LANG': 'C.UTF-8',
                   'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    # Enumerate rather than trusting RLIMIT_NOFILE: inherited FDs may exceed a
    # newly lowered limit. No agent code has run in this process yet.
    for entry in os.listdir('/proc/self/fd'):
        descriptor = int(entry)
        if descriptor > 2:
            try:
                os.close(descriptor)
            except OSError as exc:
                if exc.errno != 9:  # the directory enumeration's own closed FD
                    raise
    install_filter()
    os.setgroups([])
    os.setgid(args.gid)
    os.setuid(args.uid)
    if os.getuid() != args.uid or os.geteuid() != args.uid or os.getgroups():
        raise RuntimeError('controller privilege drop failed')
    os.chdir(args.cwd)
    os.execvpe(command[0], command, environment)


if __name__ == '__main__':
    main()
