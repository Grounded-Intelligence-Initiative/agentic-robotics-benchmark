#!/usr/bin/env python3
"""Run the submitted detector over a list of hidden frames, as the agent's account, offline.

verify.py copies this file to a root-owned directory and starts it with
`runuser -u <agent> -- /opt/grasp/bin/python -I predict_child.py ...`, so the agent can neither
edit it nor plant code that runs before it (-I: no user site-packages, no PYTHON* variables,
no script directory on sys.path).

Before the submission is imported, the process installs a seccomp filter that refuses
AF_INET/AF_INET6 sockets and io_uring. The filter is inherited by every thread and child
process and cannot be removed, so the detector is graded offline even though the engine
opens egress for the verify stage (the hidden frames' labels are public on the internet).

Protocol, all through files in OUT (owned by the agent's account, read by root afterwards):
  _guard_ok_<tag>       the seccomp filter is in place (written before the submission runs)
  _ready_<tag>          the submission imported
  _import_error_<tag>   it did not (first 2000 characters of the error)
  <frame>.started       predict() is being called for this frame
  <frame>.npy           the returned array (float64), written only on success
  <frame>.err           why the frame produced no array
"""
import ctypes
import os
import platform
import signal
import struct
import sys
import time

SUBMISSION = os.path.join(os.environ.get("HOME", "/home/user"), "submission")


def block_network():
    """Install the seccomp filter. Raises if it cannot, so grading never runs unguarded."""
    machine = platform.machine()
    if machine in ("x86_64", "AMD64"):
        audit_arch, nr_socket = 0xC000003E, 41
    elif machine in ("aarch64", "arm64"):
        audit_arch, nr_socket = 0xC00000B7, 198
    else:
        raise RuntimeError("no network guard for architecture " + machine)
    nr_io_uring_setup = 425          # the same number on every architecture
    LD, JEQ, JGE, RET = 0x20, 0x15, 0x35, 0x06      # BPF_LD|W|ABS, JMP|JEQ|K, JMP|JGE|K, RET|K
    ALLOW, ERRNO_EACCES, KILL = 0x7FFF0000, 0x00050000 | 13, 0x80000000

    def ins(code, k, jt=0, jf=0):
        return struct.pack("=HBBI", code, jt, jf, k)

    # struct seccomp_data: nr at 0, arch at 4, args[0] at 16 (low 32 bits, little-endian)
    program = [
        ins(LD, 4),                              # 0  arch
        ins(JEQ, audit_arch, 1, 0),              # 1  native ABI -> 3
        ins(RET, KILL),                          # 2  any other ABI
        ins(LD, 0),                              # 3  syscall number
        ins(JGE, 0x40000000, 7, 0),              # 4  x32 ABI -> 12
        ins(JEQ, nr_io_uring_setup, 5, 0),       # 5  io_uring -> 11
        ins(JEQ, nr_socket, 0, 3),               # 6  not socket() -> 10
        ins(LD, 16),                             # 7  socket domain
        ins(JEQ, 2, 2, 0),                       # 8  AF_INET -> 11
        ins(JEQ, 10, 1, 0),                      # 9  AF_INET6 -> 11
        ins(RET, ALLOW),                         # 10
        ins(RET, ERRNO_EACCES),                  # 11
        ins(RET, KILL),                          # 12
    ]
    blob = b"".join(program)
    buf = ctypes.create_string_buffer(blob, len(blob))

    class SockFprog(ctypes.Structure):
        _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.c_void_p)]

    fprog = SockFprog(len(program), ctypes.cast(buf, ctypes.c_void_p))
    libc = ctypes.CDLL(None, use_errno=True)
    PR_SET_NO_NEW_PRIVS, PR_SET_SECCOMP, SECCOMP_MODE_FILTER = 38, 22, 2
    if libc.prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_NO_NEW_PRIVS) failed")
    if libc.prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, ctypes.byref(fprog), 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_SECCOMP) failed")

    import socket
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            socket.socket(family, socket.SOCK_STREAM).close()
        except PermissionError:
            continue
        raise RuntimeError("network guard is not effective for family %d" % family)


class FrameTimeout(Exception):
    pass


def _alarm(signum, frame):
    raise FrameTimeout()


def _write(path, text):
    with open(path, "w") as handle:
        handle.write(text)


def main():
    frames_dir, out_dir, listing, tag, frame_timeout = sys.argv[1:6]
    frame_timeout = int(frame_timeout)
    try:
        block_network()
    except Exception as exc:
        print("[child] network guard failed: %s" % exc, file=sys.stderr, flush=True)
        return 3
    _write(os.path.join(out_dir, "_guard_ok_" + tag), "")

    import numpy as np
    from PIL import Image

    sys.path.insert(0, SUBMISSION)
    try:
        import solver                                    # the submitted code
        predict = solver.predict
    except BaseException as exc:                         # noqa: BLE001 — any failure is the agent's
        _write(os.path.join(out_dir, "_import_error_" + tag),
               ("%s: %s" % (type(exc).__name__, exc))[:2000])
        return 4
    _write(os.path.join(out_dir, "_ready_" + tag), "")

    with open(listing) as handle:
        names = [line.strip() for line in handle if line.strip()]
    signal.signal(signal.SIGALRM, _alarm)
    for name in names:
        d = os.path.join(frames_dir, name)
        rgb = np.array(Image.open(os.path.join(d, "rgb.png")))
        depth = np.array(Image.open(os.path.join(d, "depth.png")))
        K = np.load(os.path.join(d, "camK.npy"))
        os.environ["ORACLE_FRAME"] = name                # only read by the reference detector, which alone gets ground truth
        _write(os.path.join(out_dir, name + ".started"), "")
        t0 = time.time()
        signal.alarm(frame_timeout)
        try:
            grasps = np.asarray(predict(rgb, depth, K, 1000.0), dtype=np.float64)
            signal.alarm(0)
            tmp = os.path.join(out_dir, name + ".tmp.npy")
            np.save(tmp, grasps)
            os.replace(tmp, os.path.join(out_dir, name + ".npy"))
        except FrameTimeout:
            _write(os.path.join(out_dir, name + ".err"), "timeout after %ds" % frame_timeout)
        except BaseException as exc:                     # noqa: BLE001 — one bad frame, not the rest
            signal.alarm(0)
            _write(os.path.join(out_dir, name + ".err"),
                   ("%s: %s" % (type(exc).__name__, exc))[:300])
        finally:
            signal.alarm(0)
        print("[child] %s %.1fs" % (name, time.time() - t0), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
