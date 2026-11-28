"""Range check: drive your controller over the wire on the EMPTY range (the cart among a few
plain blocks on open ground).

There is no lab, no drawer and no package here, the latch closes on nothing and nothing
can be lost: this checks what can be checked without the live range. It speaks the wire exactly
as the grader does (the same requests, the same deadlines, the same kind of repeated-`t`
queries) against the true cart (cart_model.CartRange), and prints what it saw: whether the
protocol held, how long your replies took, where the cart and the latch point went, how many
pictures you asked for, and how many repeated-`t` pairs your replies differed on. It knows
nothing about how a live episode would end.

    python3 /home/user/range_check/check.py [--seconds 20] [--seed 1] [--log steps.jsonl]

`--log` writes one line per control step: the observation sent (pictures left out), the
action applied, the cart's true pose and the latch point's true position in the world, for
comparing your own model of the cart with the range's.
"""

import argparse
import json
import os
import queue
import random
import subprocess
import sys
import threading
import time

sys.path.insert(0, "/home/user")

import mujoco  # noqa: E402

from arm_model import CONTROL_DT, N_JOINTS  # noqa: E402
from cart_model import CartRange  # noqa: E402

STEP_TIMEOUT_S = 5.0
READY_TIMEOUT_S = 4 * STEP_TIMEOUT_S
MAX_PRE_READY_LINES = 200
REPEAT_RATE = 0.03          # the share of steps asked twice
SHIFT_RAD = 0.02            # how far the copy's joint readings are shifted,
SHIFT_BASE = 0.01           # and its cart pose (m, m, rad)


class WireError(Exception):
    """Your controller broke the protocol (deadline, non-JSON, no action)."""


class Wire(object):
    """Your `run.sh`, spoken to over stdio exactly as the real grader does."""

    def __init__(self, submission):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=submission,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True, bufsize=1)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _readline(self, timeout):
        try:
            line = self._lines.get(timeout=timeout)
        except queue.Empty:
            raise WireError("no reply within {:.0f} s: the real grader aborts the episode "
                            "here".format(timeout))
        if line is None:
            raise WireError("your controller closed its stdout (did it crash? check stderr)")
        return line

    def await_ready(self):
        for _ in range(MAX_PRE_READY_LINES):
            if self._readline(READY_TIMEOUT_S).strip() == "READY":
                return
        raise WireError("your controller never printed READY")

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
                raise WireError("non-JSON on stdout: {!r} (stdout is the wire; send "
                                "diagnostics to stderr)".format(line[:120]))
            if not isinstance(reply, dict):
                raise WireError("reply is not a JSON object: {!r}".format(line[:120]))
            return reply

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("controller error at t={}: {}".format(t, reply.get("error", reply)))
        return reply["action"]

    def close(self):
        try:
            self.proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            self.proc.stdin.flush()
        except (IOError, OSError):
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def _numbers(action):
    q = [float(v) for v in action["q"]]
    base = [float(v) for v in action.get("base", [0.0, 0.0, 0.0])]
    if len(q) != N_JOINTS or len(base) != 3:
        raise WireError("an action needs {} joint targets and, if it has a base command, three "
                        "velocities".format(N_JOINTS))
    latch, look = int(action.get("latch", 0)), int(action.get("look", 0))
    if latch not in (0, 1) or look not in (0, 1):
        raise WireError("latch and look are 0 or 1")
    return q + base + [float(latch), float(look)]


def _differs(a, b, tolerance=1e-4):
    # The grader's rule: two replies with a different count of numbers show nothing.
    return len(a) == len(b) and max([abs(x - y) for x, y in zip(a, b)] or [0.0]) > tolerance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--submission", default=os.environ.get(
        "SUBMISSION_DIR", "/home/user/submission"))
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--log", default="")
    args = parser.parse_args()
    if not os.path.isfile(os.path.join(args.submission, "run.sh")):
        print("[range] no run.sh at {}".format(args.submission))
        return 2
    rng = random.Random(2026)
    env = CartRange()
    obs = env.reset(args.seed)
    latch = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "latch")
    pictures = 1
    start_pose = env.cart.base_pose()
    log = open(args.log, "w") if args.log else None
    pairs = differed = 0
    slowest = 0.0
    start = env.data.site_xpos[latch].copy()
    far = 0.0
    fastest = 0.0
    previous = None
    wire = Wire(args.submission)
    try:
        wire.await_ready()
        wire.request({"type": "reset"})
        for t in range(int(round(args.seconds / CONTROL_DT))):
            t0 = time.time()
            # a step whose observation carries a picture is never asked twice
            if obs["camera"] is None and rng.random() < REPEAT_RATE:
                copy = env.observation(
                    q_offset=[rng.uniform(-SHIFT_RAD, SHIFT_RAD) for _ in range(N_JOINTS)],
                    base_offset=[rng.uniform(-SHIFT_BASE, SHIFT_BASE) for _ in range(3)])
                if rng.random() < 0.5:
                    other = wire.act(t, copy)
                    action = wire.act(t, obs)
                else:
                    action = wire.act(t, obs)
                    other = wire.act(t, copy)
                pairs += 1
                differed += int(_differs(_numbers(action), _numbers(other)))
            else:
                action = wire.act(t, obs)
            slowest = max(slowest, time.time() - t0)
            a = _numbers(action)          # only the reply to the real observation is applied
            if previous is not None:
                fastest = max(fastest, max(abs(x - y) for x, y in zip(a[:N_JOINTS], previous))
                              / CONTROL_DT)
            previous = a[:N_JOINTS]
            sent = obs
            obs, _ = env.step({"q": a[:N_JOINTS], "base": a[N_JOINTS:N_JOINTS + 3],
                               "latch": int(a[N_JOINTS + 3]), "look": int(a[N_JOINTS + 4])})
            pictures += int(obs["camera"] is not None)
            tip = env.data.site_xpos[latch]
            far = max(far, float(sum((tip - start) ** 2) ** 0.5))
            if log:
                log.write(json.dumps({"t": t, "obs": dict(sent, camera=None), "action": action,
                                      "cart_pose": [round(float(v), 6) for v in env.cart.base_pose()],
                                      "latch_point": [round(float(v), 6) for v in tip]}) + "\n")
    except WireError as exc:
        print("[range] {}".format(exc))
        return 1
    except (TypeError, ValueError, KeyError) as exc:
        print("[range] malformed action: {}".format(exc))
        return 1
    finally:
        wire.close()
        if log:
            log.close()
    if pairs and differed * 2 < pairs:
        print("[range] WARNING: in more than half of the repeated-t pairs your action did not "
              "change with the shifted readings. The real grader scores a controller that "
              "ignores its input zero, whatever it contained.", file=sys.stderr)
    print(json.dumps({
        "protocol": "ok",
        "steps": int(round(args.seconds / CONTROL_DT)),
        "slowest_reply_s": round(slowest, 4),
        "cart_pose_start": [round(float(v), 4) for v in start_pose],
        "cart_pose_end": [round(float(v), 4) for v in env.cart.base_pose()],
        "latch_point_start": [round(float(v), 4) for v in start],
        "latch_point_end": [round(float(v), 4) for v in env.data.site_xpos[latch]],
        "latch_point_farthest_from_start_m": round(far, 4),
        "pictures_taken": pictures,
        "largest_target_rate_rad_s": round(fastest, 3),
        "repeated_t_queries": pairs,
        "repeated_t_answers_differed": differed,
        "note": "the empty range only: no package, no score",
    }, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
