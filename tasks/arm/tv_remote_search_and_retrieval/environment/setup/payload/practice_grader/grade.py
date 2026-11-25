"""Practice grader: run your controller over the grading wire on practice episodes.

It speaks to `/home/user/submission/run.sh` the way the real grader does (the same requests, the
same deadlines, a few repeated-`t` queries) on episodes whose seeds you choose, and prints the
raw metric it observes: the mean of the episodes' scores. The graded episodes have other seeds,
and nothing here knows what result full credit takes.

    /opt/venv/bin/python3 /home/user/practice_grader/grade.py [--spots under_sofa,left_cushion] [--seeds 1,2]
                                                              [--max-steps 3600] [--film DIR]

`--film DIR` saves a picture of the room every 50 steps (a view of your own choosing is easy to
add: the simulator is yours to read in /home/user/remote_env.py).
"""

import argparse
import copy
import json
import os
import queue
import random
import subprocess
import sys
import threading

sys.path.insert(0, "/home/user")

import remote_env as E  # noqa: E402

STEP_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 4 * STEP_TIMEOUT_S
REPEAT_RATE = 0.01


class WireError(Exception):
    """Your controller broke the protocol (deadline, non-JSON, no action)."""


class Wire(object):
    def __init__(self, submission):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=submission, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, universal_newlines=True, bufsize=1)
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
            raise WireError("no reply within {:.0f} s: the real grader aborts the episode here".format(timeout))
        if line is None:
            raise WireError("your controller closed its stdout (did it crash? look at its stderr)")
        return line

    def await_ready(self):
        for _ in range(200):
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
                raise WireError("non-JSON on stdout: {!r} (stdout is the wire)".format(line[:120]))
            if not isinstance(reply, dict):
                raise WireError("the reply is not a JSON object: {!r}".format(line[:120]))
            return reply

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("no action at t={}: {}".format(t, reply.get("error", reply)))
        return reply["action"]

    def close(self):
        try:
            self.proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def episode(wire, env, spec, rng, max_steps, film):
    obs = env.reset(spec)
    wire.request({"type": "reset"})
    images = 0
    done = False
    while not done and env.steps < max_steps:
        t = env.steps
        if rng.random() < REPEAT_RATE:
            other = copy.copy(obs)
            other["q"] = [v + rng.uniform(-0.004, 0.004) for v in obs["q"]]
            pair = [obs, other]
            first = rng.random() < 0.5
            replies = [wire.act(t, pair[0 if first else 1]), wire.act(t, pair[1 if first else 0])]
            action = replies[0 if first else 1]
        else:
            action = wire.act(t, obs)
        try:
            _, mask = env.clip_action(action)
        except (ValueError, TypeError) as exc:
            raise WireError("malformed action at t={}: {}".format(t, exc))
        images += bin(mask).count("1")
        obs, done = env.step(action)
        if film and env.steps % 50 == 0:
            from PIL import Image
            rgb, _ = env.render("overview", noise=False)
            Image.fromarray(rgb).save(os.path.join(film, "{}_{}_{:05d}.png".format(spec.hiding_spot, spec.seed, env.steps)))
    return {"spot": spec.hiding_spot, "seed": spec.seed, "result": env.score(), "steps": env.steps,
            "ended": env.stopped or "stopped by --max-steps", "progress": dict(env.progress), "images_asked": images}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--submission", default="/home/user/submission")
    parser.add_argument("--spots", default=",".join(E.HIDING_SPOTS))
    parser.add_argument("--seeds", default="1")
    parser.add_argument("--max-steps", type=int, default=E.MAX_STEPS)
    parser.add_argument("--film", default="")
    args = parser.parse_args()
    if not os.path.isfile(os.path.join(args.submission, "run.sh")):
        print("[practice] no run.sh at {}".format(args.submission))
        return 2
    if args.film:
        os.makedirs(args.film, exist_ok=True)
    rng = random.Random(2026)
    env = E.Env()
    results = []
    wire = Wire(args.submission)
    try:
        wire.await_ready()
        for seed in [int(s) for s in args.seeds.split(",")]:
            for spot in args.spots.split(","):
                results.append(episode(wire, env, E.make_episode(seed, spot), rng, args.max_steps, args.film))
                print("[practice] {}".format(json.dumps(results[-1])), file=sys.stderr, flush=True)
    except WireError as exc:
        print("[practice] {}".format(exc))
        return 1
    finally:
        wire.close()
        env.close()
    mean = sum(r["result"] for r in results) / len(results)
    print(json.dumps({"metric": "mean_retrieval_score", "direction": "higher", "raw_value": round(mean, 4),
                      "episodes": results, "note": "practice episodes only; this is not your result"}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
