"""Practice check: drive your controller over the real wire on visible practice seeds.

    python3 ~/practice_grader/grade.py                  # all practice seeds
    python3 ~/practice_grader/grade.py --episodes 2     # the first two
    python3 ~/practice_grader/grade.py --seeds 5,9      # chosen practice seeds by index

It starts `bash run.sh` in /home/user/submission exactly as grading does (READY within 40 s of
launch, every reply within 10 s, a fresh start of run.sh after an aborted episode), asks a few
steps twice with the same `t` (once with every horizontal coordinate of the observation shifted),
steps the same cell simulator (~/cell/cable_env.py), and prints, per episode, which fixtures the
settled cable satisfies and the route fraction. Practice seeds only.
"""

import argparse
import json
import os
import queue
import random
import subprocess
import sys
import threading

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "cell"))
import cable_env as E  # noqa: E402

PRACTICE_SEEDS = [1001, 2002, 3003, 4004, 5005, 6006, 7007, 8008, 9009, 10010]
STEP_TIMEOUT_S = 10.0
READY_TIMEOUT_S = 4 * STEP_TIMEOUT_S
REPEAT_STEPS = (5, 60, 180, 400)


class WireError(Exception):
    pass


class Wire(object):
    def __init__(self, submission):
        self.submission = submission
        self.start()

    def start(self):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=self.submission, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, universal_newlines=True, bufsize=1)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        for _ in range(200):
            if self._readline(READY_TIMEOUT_S).strip() == "READY":
                return
        raise WireError("your controller never printed READY")

    def _pump(self):
        for line in self.proc.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _readline(self, timeout):
        try:
            line = self._lines.get(timeout=timeout)
        except queue.Empty:
            raise WireError("no reply within %.0f s (grading aborts the episode here)" % timeout)
        if line is None:
            raise WireError("your controller closed its stdout (crashed? see its stderr)")
        return line

    def request(self, message):
        try:
            self.proc.stdin.write(json.dumps(message) + "\n")
            self.proc.stdin.flush()
        except (IOError, OSError):
            raise WireError("your controller's stdin is closed")
        while True:
            line = self._readline(STEP_TIMEOUT_S).strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except ValueError:
                raise WireError("non-JSON on stdout: %r (stdout is the wire; log to stderr)" % line[:120])
            if not isinstance(reply, dict):
                raise WireError("reply is not a JSON object")
            return reply

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("no action at t=%d: %s" % (t, reply.get("error", reply)))
        return reply["action"]

    def restart(self):
        self.proc.kill()
        self.start()

    def close(self):
        try:
            self.proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def shifted(obs, dx, dy):
    out = json.loads(json.dumps(obs))
    for p in out["cable"]:
        p[0] += dx
        p[1] += dy
    out["gripper"]["pos"][0] += dx
    out["gripper"]["pos"][1] += dy
    out["connector"]["pos"][0] += dx
    out["connector"]["pos"][1] += dy
    for f in out["fixtures"]:
        f["pos"][0] += dx
        f["pos"][1] += dy
    return out


def as_action(reply):
    pos = [float(v) for v in reply["pos"]]
    if len(pos) != 3:
        raise ValueError("pos needs 3 numbers")
    return {"pos": pos, "yaw": float(reply["yaw"]), "grip": float(reply["grip"]),
            "done": reply.get("done", False) is True}


def flat(a):
    return a["pos"] + [a["yaw"], a["grip"], 1.0 if a["done"] else 0.0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default="/home/user/submission")
    ap.add_argument("--episodes", type=int, default=len(PRACTICE_SEEDS))
    ap.add_argument("--seeds", default=None, help="comma-separated indices into the practice seeds")
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.submission, "run.sh")):
        print("[practice] no run.sh in %s" % a.submission)
        return 2
    idx = [int(x) for x in a.seeds.split(",")] if a.seeds else list(range(min(a.episodes, len(PRACTICE_SEEDS))))
    rng = random.Random(7)
    env = E.CableRouting()
    try:
        wire = Wire(a.submission)
    except WireError as exc:
        print("[practice] %s" % exc)
        return 1
    fractions, asked, differed = [], 0, 0
    try:
        for i in idx:
            seed = PRACTICE_SEEDS[i]
            obs = env.reset(seed)
            note = ""
            try:
                r = wire.request({"type": "reset"})
                for t in range(E.MAX_STEPS):
                    if t in REPEAT_STEPS:
                        dx, dy = rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03)
                        if rng.random() < 0.5:
                            other = as_action(wire.act(t, shifted(obs, dx, dy)))
                            act = as_action(wire.act(t, obs))
                        else:
                            act = as_action(wire.act(t, obs))
                            other = as_action(wire.act(t, shifted(obs, dx, dy)))
                        asked += 1
                        if max(abs(x - y) for x, y in zip(flat(act), flat(other))) > 1e-4:
                            differed += 1
                    else:
                        act = as_action(wire.act(t, obs))
                    obs, done = env.step(act)
                    if done:
                        break
            except (WireError, KeyError, TypeError, ValueError) as exc:
                note = "aborted: %s" % exc
                try:
                    wire.restart()
                except WireError as exc2:
                    note += "; restart failed: %s" % exc2
            res = env.finish()
            fractions.append(res["credit"])
            print("practice seed %d: route %d/%d %s fraction %.2f connector peak %.1f N%s steps %d %s" % (
                seed, res["routed"], res["fixtures"],
                " ".join(("clip" if c["type"] == "clip" else "post") + ("+" if c["ok"] else "-") for c in res["checks"]),
                res["credit"], res["peak_connector_force"], " DAMAGED" if res["damaged"] else "", env.steps, note),
                flush=True)
    finally:
        wire.close()
    if asked and not differed:
        print("[practice] WARNING: your replies never changed when the observation was shifted at a "
              "repeated t; grading scores a controller that ignores its input 0.")
    print("mean route fraction over %d practice episodes: %.3f (repeated-t replies that differed: %d/%d)" % (
        len(fractions), sum(fractions) / max(len(fractions), 1), differed, asked))
    return 0


if __name__ == "__main__":
    sys.exit(main())
