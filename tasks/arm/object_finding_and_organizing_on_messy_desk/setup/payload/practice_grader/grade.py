"""Practice grader — a self-test you can run while you work.

Drives your submission over the same wire the real grader uses (same request shape, same
deadlines: READY within 120 s of launch, each reply within 30 s, repeated-`t` queries with
perturbed proprioception) on VISIBLE practice seeds, and prints the raw episode score.
It deliberately tells you less than the real grader: practice seeds, the raw metric only,
no reference value. "My controller works and roughly how well", not "what I will be scored".

    python3 /home/user/practice_grader/grade.py [--episodes 1] [--horizon 2000] [--seed 11] [--live 500] [--score-file PATH]

The simulator takes about two minutes to start; keep --horizon small while iterating.
`--live N` prints the running score (S if cut now, the unscaled 1/2 O + 1/2 R, O, R, closed
requests) every N steps on stderr; `--score-file` also writes it as JSON for your own tooling.
Driving the simulator yourself (`python3 /home/user/messy_desk_env.py`, stdio JSON): every
`step` reply carries the same `score` dict, `{"cmd": "score"}` adds every object's value and
baseline, and `{"cmd": "request", "model": "005_tomato_soup_can"}` (or `"absent": true`)
opens a request on demand so you can rehearse marks.
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

from messy_desk_score import EpisodeScorer  # noqa: E402
from sim_client import SimClient, SimError  # noqa: E402

PRACTICE_SEEDS = [11, 22, 33, 44, 55]
STEP_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 4 * STEP_TIMEOUT_S
MAX_PRE_READY_LINES = 200
PROBE_EVERY = 97          # the real grader draws probed steps at random; here every 97th step


class WireError(Exception):
    """Your solver broke the protocol (deadline, non-JSON, no action)."""


class Wire(object):
    """Your `run.sh`, spoken to over stdio exactly as the real grader does."""

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
            raise WireError("no reply within {:.0f}s; the real grader aborts the episode here".format(timeout))
        if line is None:
            raise WireError("your solver closed its stdout (did it crash? check stderr)")
        return line

    def await_ready(self):
        for _ in range(MAX_PRE_READY_LINES):
            if self._readline(READY_TIMEOUT_S).strip() == "READY":
                return
        raise WireError("your solver never printed READY")

    def request(self, message):
        try:
            self.proc.stdin.write(json.dumps(message) + "\n")
            self.proc.stdin.flush()
        except (IOError, OSError):
            raise WireError("your solver's stdin is closed")
        while True:
            line = self._readline(STEP_TIMEOUT_S).strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except ValueError:
                raise WireError("non-JSON on stdout: {!r}; stdout is the wire, send diagnostics to stderr".format(line[:120]))
            if not isinstance(reply, dict):
                raise WireError("reply is not a JSON object: {!r}".format(line[:120]))
            return reply

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("solver error at t={}: {}".format(t, reply.get("error", reply)))
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


def _differs(a, b, tolerance=1e-4):
    fa = [float(x) for x in a]
    fb = [float(x) for x in b]
    return len(fa) != len(fb) or max([abs(x - y) for x, y in zip(fa, fb)] or [0.0]) > tolerance


def _perturbed(obs, rng):
    p = dict(obs)
    p["joint_pos"] = [v + rng.uniform(-0.15, 0.15) for v in obs["joint_pos"]]
    ee = list(obs["ee_pose"])
    for i in range(3):
        ee[i] += rng.uniform(-0.04, 0.04)
    p["ee_pose"] = ee
    return p


def _live_line(seed, t, scorer):
    d = scorer.live()
    return "[practice] seed={} t={} score_now={:.4f} (running {:.4f})  O={:.4f} R={:.4f} ({}/{} requests)  org {:+.0f}/{:.0f}".format(
        seed, t, d["score"], d["score_running"], d["organization"], d["request_rate"], d["requests_success"], d["requests_closed"], d["org_num"], d["org_den"])


def _episode(wire, sim, seed, horizon, rng, stats, verbose, live=0, score_file=None):
    obs, state = sim.reset(seed, horizon)
    wire.request({"type": "reset"})
    scorer = EpisodeScorer(horizon)
    t0 = time.time()
    for t in range(horizon):
        if t % PROBE_EVERY == PROBE_EVERY - 1:
            if rng.random() < 0.5:
                other = wire.act(t, _perturbed(obs, rng))
                action = wire.act(t, obs)
            else:
                action = wire.act(t, obs)
                other = wire.act(t, _perturbed(obs, rng))
            stats["repeated_t_queries"] += 1
            if _differs(action, other):
                stats["repeated_t_answers_differed"] += 1
        else:
            action = wire.act(t, obs)
        obs, state = sim.step(action)      # only the reply to the real obs is applied
        scorer.add_step(state)
        if verbose:
            for e in state.get("events", []):
                print("[practice] t={} {}".format(e.get("t"), json.dumps(e)), file=sys.stderr, flush=True)
        if live and (t + 1) % live == 0:
            print(_live_line(seed, t + 1, scorer), file=sys.stderr, flush=True)
            if score_file:
                d = scorer.live(); d.update(seed=seed, t=t + 1)
                tmp = score_file + ".tmp"
                with open(tmp, "w") as f:
                    json.dump(d, f)
                os.replace(tmp, score_file)
    stats["wall_s_per_1000_steps"] = round((time.time() - t0) / max(horizon, 1) * 1000.0, 1)
    return scorer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--submission", default=os.environ.get("SUBMISSION_DIR", "/home/user/submission"))
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--horizon", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=None, help="one specific practice seed")
    parser.add_argument("--env", default="/home/user/messy_desk_env.py")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--live", type=int, default=500, metavar="N",
                        help="print the running score every N steps (0 = off); the same numbers the final JSON reports")
    parser.add_argument("--score-file", default=None, metavar="PATH",
                        help="also write the running score as JSON to PATH every --live steps (poll it from your own tooling)")
    args = parser.parse_args()

    entry = os.path.join(args.submission, "run.sh")
    if not os.path.isfile(entry):
        print("[practice] no run.sh at {}".format(args.submission))
        return 2
    seeds = [args.seed] if args.seed is not None else PRACTICE_SEEDS[: args.episodes]

    rng = random.Random(2024)
    stats = {"repeated_t_queries": 0, "repeated_t_answers_differed": 0}
    print("[practice] starting the simulator (about two minutes)...", file=sys.stderr, flush=True)
    try:
        sim = SimClient(args.env, log_path=os.path.expanduser("~/practice_sim.log"))
    except SimError as exc:
        print("[practice] simulator failed to start: {} (see ~/practice_sim.log)".format(exc))
        return 3
    wire = Wire(args.submission)
    scorers = []
    try:
        wire.await_ready()
        for seed in seeds:
            scorers.append(_episode(wire, sim, seed, args.horizon, rng, stats, not args.quiet, args.live, args.score_file))
    except WireError as exc:
        print("[practice] {}".format(exc))
        return 1
    finally:
        wire.close()
        sim.close()

    if stats["repeated_t_queries"] and not stats["repeated_t_answers_differed"]:
        print("[practice] WARNING: your action never changed when the proprioception was perturbed at a "
              "repeated t. The real grader treats a policy that ignores its input as a zero regardless "
              "of its metric.", file=sys.stderr)
    scores = [s.score() for s in scorers]
    print(json.dumps({
        "metric": "mean_episode_score",
        "direction": "higher",
        "raw_value": round(sum(scores) / len(scores), 5) if scores else None,
        "per_episode": [round(s, 5) for s in scores],
        "per_episode_summary": [s.summary() for s in scorers],
        "horizon": args.horizon,
        "seeds": seeds,
        "stats": stats,
        "note": "practice seeds and a short horizon; this is not your score",
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
