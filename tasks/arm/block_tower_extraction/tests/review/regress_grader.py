"""Maintainers' regression tests of the grader's wire and cell boundary (nothing runs them at verify):

  1. pause bypass: a program that arms a one-shot timer (timer_create, SIGEV_SIGNAL with SIGCONT) 150 ms after its
     request is continued during the cell's turn; the wire must end the shift ("stalled");
  2. parallel compute: a program that forks a busy child must be billed at least the CPU seconds it used;
  3. a chatty program (get_state as fast as it can) must be stopped by the shift's wall-clock cap ("wall_cap") and
     billed the per-request price;
  4. out-of-range parameters (n = 1e400, 400-digit numbers, NaN, booleans, strings) are the program's bad_params,
     never a cell fault;
  5. a program that crashes mid-episode (exception) and one that writes a non-JSON / over-long request line end their
     own episode ("crashed" / "protocol") without an exception out of the wire.
Run as the authoring account (the pauser follows the program's session): python3 tests/review/regress_grader.py
Exit 0 when every test passes.
"""
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
sys.path.insert(0, TESTS)

from cellkit import wire  # noqa: E402
from cellkit.pauser import Pauser, session_pids  # noqa: E402

TIMER_CHILD = r'''
import ctypes, signal, sys
libc = ctypes.CDLL(None, use_errno=True)
class sigevent(ctypes.Structure):
    _fields_ = [("sigev_value", ctypes.c_void_p), ("sigev_signo", ctypes.c_int), ("sigev_notify", ctypes.c_int),
                ("pad", ctypes.c_byte * 48)]
class timespec(ctypes.Structure):
    _fields_ = [("s", ctypes.c_long), ("ns", ctypes.c_long)]
class itimerspec(ctypes.Structure):
    _fields_ = [("interval", timespec), ("value", timespec)]
signal.signal(signal.SIGCONT, lambda *a: None)
ev = sigevent(); ev.sigev_signo = signal.SIGCONT; ev.sigev_notify = 0
tid = ctypes.c_void_p()
assert libc.timer_create(1, ctypes.byref(ev), ctypes.byref(tid)) == 0
for i in range(1, 100):
    sys.stdout.write('{"id":%d,"method":"step","params":{"n":1}}\n' % i); sys.stdout.flush()
    libc.timer_settime(tid, 0, ctypes.byref(itimerspec(timespec(0, 0), timespec(0, 150000000))), None)
    n = 0
    while n < 30000000:
        n += 1
    if not sys.stdin.readline():
        break
'''

FORK_CHILD = r'''
import os, sys
if os.fork() == 0:
    while True:
        pass
i = 0
while True:
    i += 1
    sys.stdout.write('{"id":%d,"method":"get_state"}\n' % i); sys.stdout.flush()
    if not sys.stdin.readline():
        break
'''

SPAM_CHILD = r'''
import sys
i = 0
while True:
    i += 1
    sys.stdout.write('{"id":%d,"method":"get_state"}\n' % i); sys.stdout.flush()
    if not sys.stdin.readline():
        break
'''


class StubServer(object):
    """The wire's view of a cell: every call takes `work_s` of the grader's time."""

    def __init__(self, work_s, calls_max=None):
        self.work_s = work_s
        self.done = False
        self.done_reason = None
        self.calls = 0
        self.calls_max = calls_max

    def handle(self, method, params):
        self.calls += 1
        t0 = time.time()
        while time.time() - t0 < self.work_s:
            pass
        if self.calls_max and self.calls >= self.calls_max:
            self.done, self.done_reason = True, "shift_end"
        return {"t": 0.0}

    def frames_delivered(self):
        return 0

    def _end(self, reason):
        self.done, self.done_reason = True, reason


def run(child_src, server, cap=100.0, wall_cap=None, call_price=0.0):
    d = tempfile.mkdtemp(prefix="bt_regress_")
    p = os.path.join(d, "child.py")
    with open(p, "w") as f:
        f.write(child_src)
    proc = wire.AgentProcess([sys.executable, "-B", p], cwd=d, env=dict(os.environ))
    pz = Pauser(lambda: session_pids(proc.proc.pid))
    logs = []
    t0 = time.time()
    reason = wire.serve_episode(server, proc, cap, 20.0, log=logs.append, pauser=pz, call_price_s=call_price,
                                wall_cap_s=wall_cap)
    wall = time.time() - t0
    cpu = pz.cpu_seconds()
    pz.thaw()
    proc.terminate()
    return reason, logs, wall, cpu, server


def test_pause_bypass():
    reason, logs, wall, cpu, srv = run(TIMER_CHILD, StubServer(0.6))
    ok = reason == "stalled" and any("ran during the cell's turn" in x for x in logs) and srv.calls <= 2
    return ok, "reason %s after %d call(s); %s" % (reason, srv.calls, logs[-1:] if logs else "")


def test_parallel_compute_billed():
    srv = StubServer(0.002, calls_max=400)
    reason, logs, wall, cpu, srv = run(FORK_CHILD, srv)
    billed = srv.program_used
    ok = billed >= 0.9 * cpu and cpu > 0.2
    return ok, "program CPU %.2f s, billed %.2f s (%d calls, %s)" % (cpu, billed, srv.calls, reason)


def test_wall_cap_and_call_price():
    reason, logs, wall, cpu, srv = run(SPAM_CHILD, StubServer(0.001), cap=1000.0, wall_cap=3.0, call_price=0.002)
    ok = reason == "wall_cap" and wall < 4.5 and srv.program_used >= 0.002 * srv.requests
    return ok, "reason %s, grader wall %.1f s, %d requests billed %.2f s" % (reason, wall, srv.requests, srv.program_used)


def test_bad_params():
    from towercell.server import CellServer
    from towercell.tower import draw_tower
    big = int("1" + "0" * 400)
    q = [0, 1, 0, 0]
    cases = [("step", {"n": 1e400}), ("step", {"n": big}), ("step", {"n": float("nan")}), ("step", {"n": True}),
             ("step", {"n": 1, "speed": big, "target": {"pos": [0.4, 0, 0.5], "quat": q}}),
             ("step", {"n": 1, "target": {"pos": [0.4, 0, float("inf")], "quat": q}}),
             ("step", {"n": 1, "target": {"pos": [0.4, 0, 0.5], "quat": [0, 0, 0, 0]}}),
             ("step", {"n": 1, "grip_open": "x"}), ("step", {"n": 1, "grip_open": 1e300}),
             ("step", {"n": 1, "grip_close": big}), ("step", {"n": 1, "grip_open": 0.04, "grip_close": 10}),
             ("step", {"n": 1, "q_target": [0] * 6}), ("step", {"n": 1, "posture": [big] * 7}),
             ("step", {"n": 1, "turntable": big}), ("step", {"n": 1, "frame": "elbow", "target": {"pos": [0.4, 0, 0.5], "quat": q}}),
             ("ik", {"pos": [0.4, 0, big], "quat": q}), ("ik", {"pos": [0.4, 0], "quat": q}),
             ("get_camera", {"name": ["wrist"]}), ("get_camera", {"name": "left", "depth": True, "x": 1})]
    srv = CellServer(draw_tower(123456789), duration_s=60.0)
    bad = []
    for m, p in cases:
        r = srv.handle(m, p)
        if not (isinstance(r, dict) and r.get("error") == "bad_params"):
            bad.append((m, str(p)[:40], str(r)[:80]))
    forb = srv.handle("get_state_privileged", {})
    ok = not bad and srv.internal_errors == 0 and not srv.done and forb.get("error") == "forbidden"
    return ok, "%d cases, internal errors %d, privileged call without token: %s, not refused: %s" % (
        len(cases), srv.internal_errors, forb.get("error"), bad[:3])


CRASH_CHILD = r'''
import sys
sys.stdout.write('{"id":1,"method":"get_state"}\n'); sys.stdout.flush()
sys.stdin.readline()
raise SystemExit(3)
'''

GARBAGE_CHILD = r'''
import sys
sys.stdout.write('{"id":1,"method":"get_state"}\n'); sys.stdout.flush()
sys.stdin.readline()
sys.stdout.write("x" * (5 * 1024 * 1024) + "\n"); sys.stdout.flush()
sys.stdin.readline()
'''


def test_crash_and_overflow():
    r1 = run(CRASH_CHILD, StubServer(0.0))[0]
    r2 = run(GARBAGE_CHILD, StubServer(0.0))[0]
    return r1 == "crashed" and r2 == "protocol", "crash -> %s, 5 MB request line -> %s" % (r1, r2)


def main():
    allok = True
    for name, fn in (("1 pause bypass", test_pause_bypass), ("2 parallel compute billed", test_parallel_compute_billed),
                     ("3 wall cap + call price", test_wall_cap_and_call_price), ("4 bad params", test_bad_params),
                     ("5 crash + over-long line", test_crash_and_overflow)):
        ok, detail = fn()
        allok &= ok
        print("%s %s: %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
