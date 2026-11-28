"""Child side of the controller runner (runs as the unprivileged runner account; stdlib only).

    python3 -B -E -s -c <BOOT> <lib dir> <workdir>      (cwd = the read-only copy of the submission's controller/)

Before the import it makes this process the subreaper of what the controller starts (the grader's CPU accounting sees
every descendant) and installs a seccomp filter that refuses every socket but a local Unix one (`_no_network`: the
controller PC has no network); its first line, {"ok": true, "filter": <installed>}, is written before the controller
is imported, so that no code of the controller can forge it (the ready line repeats it). Imports `controller.py`
from the working directory, builds
`Controller(workdir)` and answers JSON lines on stdin, one
reply line per request on the wire, a duplicate of the original stdout (file descriptor 1); before the import descriptor
1 itself is pointed at stderr and sys.stdout rebound to stderr, so that what the controller prints and what native code
writes to descriptor 1 goes to its log, never onto the wire:

    {"op": "setup", "record": {...}, "cell": {...}}  -> {"ok": true, "reply": <the setup reply>}
    {"op": "step", "obs": {...}}                     -> {"ok": true, "cmd": <the step reply>}
    {"op": "end"}                                    -> {"ok": true} and exit

The exit (after the end request's reply, when the requests end, or after a failed import): the standard library's
shutdown of process pools runs, then the interpreter's exit handlers, the wire and the log are flushed, and the process
ends without waiting for threads that are not daemons; processes the controller started are not ended here (the
grader counts them).
Keys of a setup request beside "op", "record" and "cell" are passed to setup() as keyword arguments.
An exception inside the controller answers {"ok": false, "error": "<type>: <message>"} (the grader decides what it
means); a reply that is not JSON-serialisable answers {"ok": false, "error": "reply: ..."}.
"""

import json
import os
import sys
import traceback


def _subreaper():
    """Orphans of the controller's processes become this process's children (PR_SET_CHILD_SUBREAPER), so every process
    the controller starts stays visible to the grader's CPU accounting until it has been waited for."""
    try:
        import ctypes
        ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0)
    except Exception:                              # noqa: BLE001
        pass


def _no_network():
    """A seccomp filter on this process and everything it starts (threads, children, programs they exec): socket(2)
    only for AF_UNIX, io_uring refused, the other syscall ABIs refused; the controller PC has no network. Returns True
    when installed."""
    try:
        import ctypes
        import platform
        tbl = {"x86_64": (0xC000003E, 41, 425), "aarch64": (0xC00000B7, 198, 425)}   # audit arch, socket, io_uring_setup
        if platform.machine() not in tbl:
            return False
        audit_arch, nr_socket, nr_uring = tbl[platform.machine()]
        LD, JEQ, JGE, RET = 0x20, 0x15, 0x35, 0x06
        ALLOW, DENY = 0x7FFF0000, 0x00050000 | 1      # SECCOMP_RET_ALLOW, SECCOMP_RET_ERRNO | EPERM
        prog = [(LD, 0, 0, 4), (JEQ, 0, 7, audit_arch), (LD, 0, 0, 0), (JGE, 5, 0, 0x40000000), (JEQ, 0, 2, nr_socket),
                (LD, 0, 0, 16), (JEQ, 1, 2, 1), (JEQ, 1, 0, nr_uring), (RET, 0, 0, ALLOW), (RET, 0, 0, DENY)]

        class Filt(ctypes.Structure):
            _fields_ = [("code", ctypes.c_ushort), ("jt", ctypes.c_ubyte), ("jf", ctypes.c_ubyte), ("k", ctypes.c_uint)]

        class Prog(ctypes.Structure):
            _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.POINTER(Filt))]
        arr = (Filt * len(prog))(*[Filt(*f) for f in prog])
        fp = Prog(len(prog), arr)
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(38, 1, 0, 0, 0) != 0:            # PR_SET_NO_NEW_PRIVS
            return False
        return libc.prctl(22, 2, ctypes.byref(fp), 0, 0) == 0     # PR_SET_SECCOMP, SECCOMP_MODE_FILTER
    except Exception:                              # noqa: BLE001
        return False


def _say(out, obj):
    out.write(json.dumps(obj, separators=(",", ":")) + "\n")
    out.flush()


def _end(out, code):
    """The end of the client (after its reply to the end request, when the requests end, or after a failed import):
    the standard library's own shutdown of process pools runs (concurrent.futures' exit hook, when that module is
    loaded: Python's normal exit runs it before the exit handlers; it ends the pools' worker processes), then the
    interpreter's exit handlers, the wire and the log are flushed, and the process ends without waiting for the
    controller's threads that are not daemons (a thread is no process the controller started; a process it started
    is not ended here and is counted by the grader)."""
    try:
        _pp = sys.modules.get("concurrent.futures.process")
        if _pp is not None:
            _pp._python_exit()                     # the standard library's shutdown of its process pools
    except BaseException:                          # noqa: BLE001
        traceback.print_exc()
    try:
        import atexit
        atexit._run_exitfuncs()
    except BaseException:                          # noqa: BLE001
        traceback.print_exc()
    for f in (out, sys.stdout, sys.stderr):
        try:
            f.flush()
        except BaseException:                      # noqa: BLE001
            pass
    os._exit(code)


def main(argv):
    workdir = argv[0] if argv else os.getcwd()
    _subreaper()
    no_net = _no_network()
    out = os.fdopen(os.dup(1), "w", buffering=1)  # the wire: a duplicate of descriptor 1, taken before the import
    os.dup2(2, 1)                                    # descriptor 1 itself now writes to stderr (the log)
    sys.stdout = sys.stderr
    _say(out, {"ok": True, "filter": bool(no_net)})   # before any code of the controller runs
    sys.path.insert(0, os.getcwd())
    try:
        import controller as _c                    # noqa: the submission's module
        ctl = _c.Controller(workdir)
    except BaseException as e:                     # noqa: BLE001
        traceback.print_exc()
        _say(out, {"ok": False, "ready": False, "error": "import: {}: {}".format(type(e).__name__, e)})
        _end(out, 1)
    _say(out, {"ok": True, "ready": True, "no_network": bool(no_net)})
    for line in sys.stdin:
        try:
            req = json.loads(line)
        except ValueError:
            _say(out, {"ok": False, "error": "bad request"})
            continue
        op = req.get("op")
        try:
            if op == "setup":
                extra = {k: v for k, v in req.items() if k not in ("op", "record", "cell")}
                rep = {"ok": True, "reply": ctl.setup(req["record"], req["cell"], **extra)}
            elif op == "step":
                rep = {"ok": True, "cmd": ctl.step(req["obs"])}
            elif op == "end":
                _say(out, {"ok": True})
                _end(out, 0)
            else:
                rep = {"ok": False, "error": "unknown op"}
        except BaseException as e:                 # noqa: BLE001
            traceback.print_exc()
            rep = {"ok": False, "error": "{}: {}".format(type(e).__name__, e)}
        try:
            _say(out, rep)
        except (TypeError, ValueError, RecursionError, MemoryError) as e:   # not JSON-serialisable, nested too deep
            _say(out, {"ok": False, "error": "reply: {}".format(e)})
    _end(out, 0)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
