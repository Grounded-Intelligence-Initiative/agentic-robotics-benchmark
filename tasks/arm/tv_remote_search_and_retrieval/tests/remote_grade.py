"""TV remote search and retrieval: the grader's episode loop over the closed-loop wire (used by tests/verify.py).

- The wire: READY, then reset / act / close, one JSON object per line; an action is 10 numbers (`remote_env.py`). A
  malformed action, a missed deadline or a closed wire aborts the episode (score 0); the solver restarts the
  controller at the next reset (at most 3 restarts).
- The probe lock: a step is asked twice with the same `t`, the true observation and a second copy
  (`Env.perturbed_observation`) in a hidden order, and only the true reply is applied. Which steps, the order and the
  copy come from one stream per episode (`probe_stream`), a function of the episode's index and its seed alone.
- `truth` (`Env.truth()`) is added to every observation only in a token run (the caller decides).
- Time: every episode has a controller budget (`controller_budget_s`): what the grader waits for the reset and for
  each reply (`Clock`), plus `image_charge_s` for every image an applied action asks for. The simulation and the
  drawing are not counted. The budget is checked after every reply, the last one too: an episode whose budget runs
  out stops there and is scored as it stands.
- An unstable simulation (`Env.stopped == "unstable"`) scores that episode 0, as the simulator's own rule says.
- The log names an episode by its index in the set: never by its seed, and never by where the remote was.
"""

import hashlib
import time

import mujoco
import numpy as np

import episodes as R
import remote_env as E
from robotics_grader import ProbeLock, SolverAborted, seed_rng

PROBE_TAG = b"tv_remote_search_and_retrieval verify probe schedule 1"
CONFIG = {"probe_rate": 0.01, "probe_pass_fraction": 0.1, "step_timeout_s": 30, "controller_budget_s": 900.0,
          "image_charge_s": {"overview": 0.2, "wrist": 0.2}}
MAX_LINE = 1 << 22          # a reply line longer than 4 MiB closes the wire (the episode aborts)


def canonical(action):
    values = [float(v) for v in action]
    if len(values) != E.ACTION_SIZE:
        raise ValueError("an action is {} numbers".format(E.ACTION_SIZE))
    return values


def probe_stream(index, seed):
    """The probe lock's stream for episode `index` of the run, whose seed is `seed` (BLAKE2b over the tag, the index
    and the seed; the simulator draws from the seed through its own keyed streams)."""
    digest = hashlib.blake2b(b"%s|%d|%d" % (PROBE_TAG, int(index), int(seed)), digest_size=32).digest()
    return seed_rng(entropy=int.from_bytes(digest, "little"))


class Clock(object):
    """The controller's own time in an episode (s): what the grader waited for the reset and each reply, plus the
    images' charge."""

    def __init__(self):
        self.s = 0.0
        self.total = 0.0

    def timed(self, fn, *args, **kwargs):
        t0 = time.monotonic()
        try:
            return fn(*args, **kwargs)
        finally:
            dt = time.monotonic() - t0
            self.s += dt
            self.total += dt

    def charge(self, seconds):
        self.s += seconds
        self.total += seconds


def wire_obs(env, obs, truth):
    if not truth:
        return obs
    w = dict(obs)
    w["truth"] = env.truth()
    return w


def ask(lock, solver, env, t, obs, truth, clock):
    w = wire_obs(env, obs, truth)
    if lock.due():
        other = wire_obs(env, env.perturbed_observation(obs, np.random.default_rng(lock.rng.seed(bits=64))), truth)
        return clock.timed(lock.probe, solver, t, w, perturbed=other)
    return clock.timed(solver.act, t, w)


def run_episodes(solver, episodes, truth, log=None, config=CONFIG, on_step=None, deadline=None):
    """Run `episodes` ([(index, seed, place)], in order) over one controller. Returns (records, lock, clock); a record
    per episode: index, score, steps, how it ended, the controller's time. `deadline()` (optional) returns True when
    the grader's own wall cap has passed: the episodes not started by then are not run (score 0)."""
    budget = float(config.get("controller_budget_s", 900.0))
    charges = config.get("image_charge_s", {})
    lock = ProbeLock(None, rate=config["probe_rate"], pass_fraction=config["probe_pass_fraction"], canonical=canonical)
    clock = Clock()
    env = E.Env()
    records = []
    try:
        for n, seed, place in episodes:
            rec = {"index": n, "score": 0.0, "steps": 0, "ended": "not run", "scored": False, "controller_s": 0.0}
            if deadline is not None and deadline():
                rec["ended"] = "not run: the grader's wall cap"
                records.append(rec)
                continue
            lock.rng = probe_stream(n, seed)
            clock.s = 0.0
            try:
                env.place = place
                obs = env.reset(R.make_episode(E, int(seed), place))
                clock.timed(solver.reset)
                done, ended = False, "time"
                while not done:
                    if on_step is not None:
                        on_step()
                    t = env.steps
                    action = ask(lock, solver, env, t, obs, truth, clock)
                    try:
                        _, mask = env.clip_action(action)
                    except (ValueError, TypeError, OverflowError) as exc:
                        raise SolverAborted("malformed action: {}".format(exc)[:160])
                    clock.charge(sum(float(charges.get(name, 0.0)) for name in E.CAMERAS if mask & E.CAMERAS[name][0]))
                    over = clock.s > budget
                    obs, done = env.step(action)
                    if over and not done:
                        ended = "controller budget"
                        break
                if done:
                    ended = env.stopped
                rec.update(score=float(env.score()), scored=True, ended=str(ended), progress=dict(env.progress))
            except mujoco.FatalError as exc:
                rec.update(score=0.0, ended="simulator fatal error: {}".format(str(exc)[:120]))
            except SolverAborted as exc:
                rec.update(score=0.0, ended="aborted: {}".format(str(exc)[:120]))
            rec.update(steps=int(env.steps), controller_s=round(clock.s, 2))
            if log:
                log("episode {}: score={:.1f} steps={} ({})".format(n, rec["score"], rec["steps"], rec["ended"]))
            records.append(rec)
    finally:
        env.close()
    return records, lock, clock
