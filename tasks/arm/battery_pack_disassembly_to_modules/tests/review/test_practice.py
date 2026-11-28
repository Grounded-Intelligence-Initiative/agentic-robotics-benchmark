#!/usr/bin/env python3
"""The practice launcher against the grader, without the simulator (maintainers' tool; the grader never runs it).

    python3 test_practice.py          (NumPy needed: the Isaac Sim venv or any Python 3.10+ with numpy)

- the agent's `packcell_client.canonical`, `is_idle` and `replies_differ` give what the grader's `wire.canonical`,
  `wire.is_idle` and the kit's `replies_differ` give, on valid and invalid actions;
- `~/commission/run_session.py` against a stub session: a program that stops reading its input is given up on at the
  request's deadline (the write does not block), its own time is reported, and its repeated pairs are counted by the
  grader's rule.
"""

import os
import random
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
PAYLOAD = os.path.join(TASK, "environment", "setup", "payload")
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(PAYLOAD, "given", "lib"))
sys.dont_write_bytecode = True

import packcell_client as pc  # noqa: E402
from packcell import wire  # noqa: E402
from robotics_grader import replies_differ  # noqa: E402


def check(name, ok, detail=""):
    print("%s  %s%s" % ("ok  " if ok else "FAIL", name, (": " + detail) if detail else ""))
    return bool(ok)


def random_action(rng):
    a = {}
    if rng.random() < 0.8:
        kind = rng.choice(["tcp", "joint", "hold", "warp"] if rng.random() < 0.05 else ["tcp", "joint", "hold"])
        m = {"type": kind, "ticks": rng.choice([1, 5, 40, 41, 0]) if rng.random() < 0.05 else rng.randint(1, 40)}
        if rng.random() < 0.9:
            m["delta"] = [rng.choice([0.0, rng.uniform(-0.01, 0.01), 1e-5, 3.5]) for _ in range(6)]
        if rng.random() < 0.3:
            m["guard_force"] = rng.uniform(0.5, 50)
        if rng.random() < 0.2:
            m["guard_stroke_mm"] = rng.uniform(0.1, 30)
        a["move"] = m
    if rng.random() < 0.4:
        a["spindle"] = {"speed_rpm": rng.uniform(-300, 300), "torque_limit": rng.uniform(0, 60)}
    if rng.random() < 0.3:
        a["gripper"] = rng.uniform(-0.1, 1.0)
    if rng.random() < 0.3:
        a["cmd"] = rng.choice(list(wire.CMDS) + ["bit:X"])
    if rng.random() < 0.3:
        a["cams"] = rng.choice([["overhead"], ["wrist"], ["overhead", "wrist"], ["front"], ["wrist", "wrist"]])
    if rng.random() < 0.02:
        a["extra"] = 1
    return a


def test_copies():
    rng = random.Random(7)
    actions = [random_action(rng) for _ in range(4000)]
    same, bad = True, 0
    for a in actions:
        try:
            want = wire.canonical(a)
        except ValueError:
            want = None
        try:
            got = pc.canonical(a)
        except ValueError:
            got = None
        bad += want is None
        same &= want == got and wire.is_idle(a) == pc.is_idle(a)
    pairs_same = True
    for a, b in zip(actions[::2], actions[1::2]):
        try:
            want = replies_differ(a, b, 1e-4, wire.canonical)
        except ValueError:
            want = "invalid"
        try:
            got = pc.replies_differ(a, b)
        except ValueError:
            got = "invalid"
        pairs_same &= want == got
    near = {"move": {"type": "tcp", "delta": [0.01, 0, 0, 0, 0, 0], "ticks": 5}}
    off = {"move": {"type": "tcp", "delta": [0.01 + 2e-4, 0, 0, 0, 0, 0], "ticks": 5}}
    return all([check("client copy: canonical and is_idle as the grader's on 4000 actions (%d invalid)" % bad, same),
                check("client copy: replies_differ as the kit's on 2000 pairs", pairs_same),
                check("client copy: 2e-4 differs, 5e-5 does not", pc.replies_differ(near, off)
                      and not pc.replies_differ(near, {"move": dict(near["move"], delta=[0.01 + 5e-5, 0, 0, 0, 0, 0])})),
                check("client copy: the grader's constants", (pc.CMDS, pc.CAMS, pc.MOVES, pc.MAX_TICKS)
                      == (wire.CMDS, wire.CAMS, wire.MOVES, wire.DEFAULTS["max_ticks"]))])


STUB = r'''
import os, runpy, sys
sys.argv = ["run_session.py", "--pack", "1", "--submission", os.environ["TP_SUB"], "--seed", "3"]
sys.path.insert(0, os.path.join(os.environ["HOME"], "given", "lib"))
import packcell_client as pc
import numpy as np

class Session(object):
    def __init__(self, pack, cams=None):
        self.number, self.sessions_left, self.n = 1, 9, 0
        self.q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.obs = self._obs()
    def _obs(self):
        T = pc.flange_T(self.q)
        p = pc.T_pose(T)
        return {"time": 0.05 * self.n, "q": self.q, "q_target": self.q, "tcp": p, "tcp_target": p, "events": [],
                "status": {"protective_stops": 0}, "images": {}, "pad": "x" * int(os.environ.get("TP_PAD", "0")),
                "ended": "done" if self.n >= int(os.environ["TP_STEPS"]) else None}
    def step(self, action):
        pc.check_action(action)
        m = action.get("move") or {}
        if m.get("type") == "joint":
            self.q = [a + d for a, d in zip(self.q, m["delta"])]
        self.n += 1
        return self._obs()
    def __enter__(self):
        return self
    def __exit__(self, *a):
        return False
pc.Session = Session
mod = runpy.run_path(os.environ["TP_LAUNCHER"], run_name="not_main")
mod["STEP_TIMEOUT"] = float(os.environ.get("TP_TIMEOUT", "30"))
mod["main"].__globals__["STEP_TIMEOUT"] = mod["STEP_TIMEOUT"]
mod["main"].__globals__["REPEAT_RATE"] = 0.5
mod["main"]()
'''

CONTROLLERS = {
    # answers from the observation it is sent: its pairs differ
    "follows": r'''
import json, sys
print("READY", flush=True)
for line in sys.stdin:
    m = json.loads(line)
    if m["type"] != "act":
        print(json.dumps({"ok": True}), flush=True); continue
    q = m["obs"]["q_target"]
    print(json.dumps({"action": {"move": {"type": "joint", "delta": [0.01 - 0.5 * q[0], 0, 0, 0, 0, 0], "ticks": 2}}}), flush=True)
''',
    # a replayed stream: its pairs do not differ
    "replays": r'''
import json, sys, time
print("READY", flush=True)
for line in sys.stdin:
    m = json.loads(line)
    if m["type"] != "act":
        print(json.dumps({"ok": True}), flush=True); continue
    time.sleep(0.02)
    print(json.dumps({"action": {"move": {"type": "joint", "delta": [0.01, 0, 0, 0, 0, 0], "ticks": 2}}}), flush=True)
''',
    # answers the reset, then stops reading: the first observation (padded past the pipe's buffer) is never read
    "deaf": r'''
import json, sys, time
print("READY", flush=True)
sys.stdin.readline()
print(json.dumps({"ok": True}), flush=True)
time.sleep(300)
''',
}


def launch(work, name, steps, timeout="30", pad=0):
    sub = os.path.join(work, name)
    os.makedirs(sub, exist_ok=True)
    with open(os.path.join(sub, "controller.py"), "w") as f:
        f.write(CONTROLLERS[name])
    with open(os.path.join(sub, "run.sh"), "w") as f:
        f.write("exec %s -I controller.py\n" % sys.executable)
    home = os.path.join(work, "home")
    if not os.path.exists(home):
        os.makedirs(home)
        os.symlink(os.path.join(PAYLOAD, "given"), os.path.join(home, "given"))
    env = dict(os.environ, HOME=home, TP_SUB=sub, TP_STEPS=str(steps), TP_TIMEOUT=timeout, TP_PAD=str(pad),
               TP_LAUNCHER=os.path.join(PAYLOAD, "commission", "run_session.py"))
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, "-B", "-c", STUB], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True, timeout=180)
    return p.stdout, time.monotonic() - t0


def test_launcher(work):
    out_f, _ = launch(work, "follows", 60)
    out_r, _ = launch(work, "replays", 60)
    res = []

    def counts(out):
        for line in out.splitlines():
            if "pairs compared" in line:
                w = line.split()
                return int(w[w.index("pairs") - 1]), int(w[w.index("differed") - 3])
        return None
    cf, cr = counts(out_f), counts(out_r)
    # (a pair whose hidden shift of the first joint is under 2e-4 rad moves this program's reply by under 1e-4)
    res.append(check("launcher: a program that answers from its observation: nearly every compared pair differs",
                     cf is not None and cf[0] > 5 and cf[1] >= 0.9 * cf[0], str(cf)))
    res.append(check("launcher: a replayed stream: no compared pair differs", cr is not None and cr[0] > 5 and cr[1] == 0,
                     str(cr)))
    own = [ln for ln in out_r.splitlines() if "own time" in ln]
    res.append(check("launcher: the program's own time is reported", own and float(own[0].split("own time ")[1].split()[0]) > 1.0,
                     own[0].strip() if own else out_r[-300:]))
    # a deaf program: the large first request is never read; a blocking write would hang here
    out_d, wall = launch(work, "deaf", 60, timeout="3", pad=4 << 20)
    res.append(check("launcher: a program that does not read is given up on at the deadline, the write does not block",
                     "did not read the request" in out_d and wall < 60, "%.1f s; %s" % (wall, out_d.strip().splitlines()[-4:])))
    return all(res)


def main():
    work = tempfile.mkdtemp(prefix="test-practice-")
    try:
        ok = [test_copies(), test_launcher(work)]
    finally:
        subprocess.run(["rm", "-rf", work])
    print("practice path: %s" % ("all ok" if all(ok) else "FAILED"))
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
