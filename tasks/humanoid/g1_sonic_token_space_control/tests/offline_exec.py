"""Fail-closed socket sandbox, using the cabinet_door_hanging task's audited native-ABI filter.

This helper runs through robotics_grader.deprivileged at the agent UID with isolated stdlib-only
Python. The grader passes its trusted source directly, so the submission cannot substitute a
helper module and the private verifier directory can remain inaccessible to the agent.
"""
import ctypes
import errno
import os
import platform
import socket
import struct
import sys

PR_SET_NO_NEW_PRIVS = 38
PR_SET_SECCOMP = 22
SECCOMP_MODE_FILTER = 2
SECCOMP_RET_ALLOW = 0x7FFF0000
SECCOMP_RET_ERRNO = 0x00050000
EPERM = 1
AF_UNIX = 1
# arch -> (AUDIT_ARCH, socket, socketpair, io_uring_setup, x32 bit)
ARCH = {
    "x86_64": (0xC000003E, 41, 53, 425, 0x40000000),
    "aarch64": (0xC00000B7, 198, 199, 425, 0),
}
LD_W_ABS, JEQ_K, JGE_K, RET_K = 0x20, 0x15, 0x35, 0x06
OFF_NR, OFF_ARCH, OFF_ARG0 = 0, 4, 16
def _ins(code, jt, jf, k):
    return struct.pack("HBBI", code, jt, jf, k)


def build_filter(machine):
    audit, nr_socket, nr_socketpair, nr_uring, x32 = ARCH[machine]
    deny = SECCOMP_RET_ERRNO | EPERM
    prog = [_ins(LD_W_ABS, 0, 0, OFF_ARCH),
            _ins(JEQ_K, 1, 0, audit),
            _ins(RET_K, 0, 0, deny),
            _ins(LD_W_ABS, 0, 0, OFF_NR)]
    if x32:
        prog += [_ins(JGE_K, 0, 1, x32), _ins(RET_K, 0, 0, deny)]
    prog += [_ins(JEQ_K, 0, 1, nr_uring), _ins(RET_K, 0, 0, deny),
             _ins(JEQ_K, 2, 0, nr_socket),
             _ins(JEQ_K, 1, 0, nr_socketpair),
             _ins(RET_K, 0, 0, SECCOMP_RET_ALLOW),
             _ins(LD_W_ABS, 0, 0, OFF_ARG0),
             _ins(JEQ_K, 0, 1, AF_UNIX),
             _ins(RET_K, 0, 0, SECCOMP_RET_ALLOW),
             _ins(RET_K, 0, 0, deny)]
    return b"".join(prog)


def install_filter():
    machine = platform.machine()
    if machine not in ARCH:
        raise OSError("unsupported architecture " + machine)
    prog = build_filter(machine)
    buf = ctypes.create_string_buffer(prog, len(prog))

    class SockFprog(ctypes.Structure):
        _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.c_void_p)]

    fprog = SockFprog(len(prog) // 8, ctypes.cast(buf, ctypes.c_void_p))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "PR_SET_NO_NEW_PRIVS failed")
    if libc.prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, ctypes.byref(fprog), 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "PR_SET_SECCOMP failed")


def main():
    if os.geteuid() == 0:
        raise RuntimeError("solver network confinement must run after the account drop")
    install_filter()
    if sys.argv[1:] == ["--self-test"]:
        for family in (socket.AF_INET, socket.AF_INET6):
            try:
                sock = socket.socket(family, socket.SOCK_STREAM)
            except OSError as exc:
                if exc.errno != errno.EPERM:
                    raise
            else:
                sock.close()
                raise RuntimeError("solver can open a network socket")
        first, second = socket.socketpair(socket.AF_UNIX)
        first.close()
        second.close()
        print("offline", flush=True)
        return
    os.execv("/bin/bash", ["/bin/bash", sys.argv[1]])


if __name__ == "__main__":
    main()
