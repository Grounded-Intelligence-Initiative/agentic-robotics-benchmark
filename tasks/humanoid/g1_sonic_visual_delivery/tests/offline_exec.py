"""Run the submitted program with offline sockets and one writable episode directory."""
import ctypes
import errno
import os
import platform
import socket
import struct
import sys

PR_SET_NO_NEW_PRIVS = 38
PR_SET_SECCOMP = 22
SECCOMP_RET_ALLOW = 0x7FFF0000
SECCOMP_RET_ERRNO = 0x00050000
ARCH = {"x86_64": (0xC000003E, 53, 0x40000000), "aarch64": (0xC00000B7, 199, 0)}
# Filesystem metadata and kernel IPC can outlive their creator and bypass file-write rules.
DENIED = """
socket io_uring_setup chmod fchmod fchmodat fchmodat2 chown fchown lchown fchownat
utime utimes futimesat utimensat setxattr lsetxattr fsetxattr removexattr lremovexattr
fremovexattr setxattrat removexattrat shmget shmat shmctl shmdt semget semop semctl
semtimedop msgget msgsnd msgrcv msgctl add_key request_key keyctl mq_open mq_unlink
mq_timedsend mq_timedreceive mq_notify mq_getsetattr quotactl
""".split()


def checked(value, operation):
    if value < 0:
        raise OSError(ctypes.get_errno(), operation)
    return value


def restrict_writes(libc, scratch):
    # Landlock ABI 3 includes cross-directory links and O_TRUNC. Fail closed on older hosts.
    version = checked(libc.syscall(444, 0, 0, 1), "query Landlock ABI")
    if version < 3:
        raise RuntimeError("Linux Landlock ABI 3 or newer is required")
    # WRITE_FILE, REMOVE_DIR, REMOVE_FILE, MAKE_*, REFER, TRUNCATE.
    writes = (1 << 1) | sum(1 << bit for bit in range(4, 15))
    rules = ctypes.c_uint64(writes)
    ruleset = checked(libc.syscall(444, ctypes.byref(rules), ctypes.sizeof(rules), 0), "create Landlock ruleset")
    try:
        with_fd = os.open(scratch, os.O_PATH | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            # struct landlock_path_beneath_attr is packed, u64 then s32.
            attr = ctypes.create_string_buffer(struct.pack("=Qi", writes, with_fd))
            checked(libc.syscall(445, ruleset, 1, ctypes.byref(attr), 0), "allow episode scratch")
        finally:
            os.close(with_fd)
        checked(libc.syscall(446, ruleset, 0), "restrict episode writes")
    finally:
        os.close(ruleset)


def restrict_syscalls(libc):
    audit, socketpair_nr, x32 = ARCH[platform.machine()]
    load, eq, ge, ret = 0x20, 0x15, 0x35, 0x06
    deny = SECCOMP_RET_ERRNO | errno.EPERM
    def ins(op, yes, no, value):
        return struct.pack("HBBI", op, yes, no, value)
    prog = [ins(load, 0, 0, 4), ins(eq, 1, 0, audit), ins(ret, 0, 0, deny), ins(load, 0, 0, 0)]
    if x32:
        prog.extend([ins(ge, 0, 1, x32), ins(ret, 0, 0, deny)])
    names = ctypes.CDLL("libseccomp.so.2").seccomp_syscall_resolve_name
    numbers = {names(name.encode()) for name in DENIED}
    # Older libseccomp tables omit these newer generic Linux syscall numbers.
    numbers.update((452, 463, 466))
    for nr in sorted(n for n in numbers if n >= 0):
        prog.extend([ins(eq, 0, 1, nr), ins(ret, 0, 0, deny)])
    # Anonymous local socket pairs support pipes within a participant process tree.
    prog.extend([ins(eq, 1, 0, socketpair_nr), ins(ret, 0, 0, SECCOMP_RET_ALLOW),
                 ins(load, 0, 0, 16), ins(eq, 0, 1, socket.AF_UNIX),
                 ins(ret, 0, 0, SECCOMP_RET_ALLOW), ins(ret, 0, 0, deny)])
    raw = ctypes.create_string_buffer(b"".join(prog))
    class Program(ctypes.Structure):
        _fields_ = [("length", ctypes.c_ushort), ("filter", ctypes.c_void_p)]
    program = Program(len(prog), ctypes.cast(raw, ctypes.c_void_p))
    checked(libc.prctl(PR_SET_SECCOMP, 2, ctypes.byref(program), 0, 0), "install seccomp filter")


def main():
    if os.geteuid() == 0 or platform.machine() not in ARCH:
        raise RuntimeError("unsupported solver sandbox")
    libc = ctypes.CDLL(None, use_errno=True)
    checked(libc.prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0), "no new privileges")
    scratch = os.environ["TMPDIR"]
    restrict_writes(libc, scratch)
    restrict_syscalls(libc)
    if sys.argv[1:] == ["--self-test"]:
        name = "delivery-sandbox-" + str(os.getpid())
        fd = os.open(os.path.join(scratch, name), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        os.close(fd)
        for action in (lambda: socket.socket(socket.AF_INET),
                       lambda: os.open(os.path.join("/tmp", name), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)):
            try:
                action()
            except OSError as exc:
                if exc.errno not in (errno.EPERM, errno.EACCES):
                    raise
            else:
                raise RuntimeError("episode sandbox escaped")
        first, second = socket.socketpair()
        first.close()
        second.close()
        print("offline", flush=True)
        return
    os.execv("/bin/bash", ["/bin/bash", sys.argv[1]])


if __name__ == "__main__":
    main()
