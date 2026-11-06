"""The page's plant (docplant.py, written from given/model.md alone) beside the grader's (tests/env.py): a maintainer's
tool, which the grader never runs.

What it proves. The agent gets no simulator, only the page (environment/setup/payload/given/model.md), which claims to
state the grader's model completely and exactly. A plant written from the page alone, given an episode's draws and
every observation's raw noise numbers, and stepped with the same commands, equals env.py in every state and
observation field after every step (`open`, `reference`, `act`); and the page's table of draws has the distributions
of env.py's worlds (`draws`). A difference anywhere is a defect of the page (or of docplant.py's reading of it).

How the two are paired. A TwinEnv is env.py's simulator with the page's plant beside it (Twin):
  * The draws. env.py's generator is counter mode (its i-th number depends on the seed and i alone), so the harness
    regenerates every number env.py drew from a generator of its own. docplant.draw_episode asks for the page's draws
    by name; EnvReset answers each with the number env.py's reset consumed for it, turned into the draw as env.py turns
    it. Which number belongs to which draw, and how env.py maps a number to a draw, is harness knowledge (env.py's
    reset), never the page's; the ranges are the page's (draw_episode passes them), so a range that differs between the
    page and env.py shows as a difference, and so does a draw either side makes and the other does not.
  * The noise. Each observation's raw numbers are served to the page's plant by name, in env.py's order (EnvNoise); a
    number env.py drew that the plant does not ask for, or the other way round, is a difference.
  * The comparison, after the reset and after every step: every attribute of env.py's simulator against the page's
    quantity it holds (env_view; an attribute it does not map is a difference, so a new one cannot slip by), all 16
    observation fields, and the step's quality, end, errors and fuel. All with ==, so to the last bit.

The episodes. `open`: dev seeds 2**42 + 104729 j, scripted commands of five kinds (small duties with a full one now and
then, bang-bang, a crude push toward the target, tiny duties, tiny duties with relay triggers and out-of-range
entries), the odd episodes moved inside the plume's field (r 1.0 to 1.45 m) right after the reset, both plants alike.
`reference`: the privileged reference's own episodes through the grader's loop (verify.run_episode, a token run, the
match lock keyed by oracle_batch's dev key and the seed), on dev seeds of random.Random(20261106). `act`: the page's
example request, regenerated from its dev seed. `draws`: env.py's worlds on dev seeds against as many episodes drawn
from the page's table with random.Random (docplant.PageRandom): each draw read off a world's attributes the same way
for both, a two-sample Kolmogorov-Smirnov test for a continuous draw and a chi-square homogeneity test for a discrete
one, at a family-wise level of 1% (Bonferroni). No seed here is in the hidden set or among its selection's candidates.

    python3 -B doc_vs_env.py open [--count 400] [--workers 8]
    python3 -B doc_vs_env.py reference [--count 100] [--rng 20261106] [--workers 8]
    python3 -B doc_vs_env.py draws [--count 50000] [--rng 1]
    python3 -B doc_vs_env.py act

The pins: docplant.PAGE_SHA256 is the page the plant was written from and ENV_SHA256 the env.py these checks last
passed on. When either file changes, re-read the page against docplant.py, run the four checks (the README's figures)
and move the pins; until then the repository test fails. Paths are found from this file, so it runs from any
checkout; it writes nothing. The exit status is 1 on any difference, a failed test or a moved file.
"""

import argparse
import collections
import hashlib
import json
import math
import multiprocessing
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(TASK, "solution", "privileged"))
sys.path.insert(0, TESTS)
sys.path.insert(0, HERE)

import docplant  # noqa: E402
import env as ENV  # noqa: E402
import oracle as O  # noqa: E402
import oracle_batch  # noqa: E402
import verify  # noqa: E402
from robotics_grader import ProbeLock  # noqa: E402

ENV_FILE = os.path.join(TESTS, "env.py")
ENV_SHA256 = "c2900397cb6df20e6bc34e57242da814ba2b53985f28345dd0e22859463c26bd"
OPEN_BASE, OPEN_STRIDE = 1 << 42, 104729    # open-loop episode j: seed OPEN_BASE + OPEN_STRIDE * j
REFERENCE_RNG = 20261106                    # the reference's episodes: oracle_batch.dev_seeds(count, REFERENCE_RNG)
ACT_SEED, ACT_T = 1099511640001, 37         # the page's example request
HARNESS_ATTRIBUTES = {"rng", "twin"}        # (attributes of a TwinEnv that are no plant quantity)
REASON = {None: "", "docked": "docked", "collision": "target_collision", "escape": "escaped_operating_region",
          "empty tank": "fuel_exhausted", "timeout": "timeout"}


def sha256_file(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def moved_pins():
    """The pinned files that changed: [(file, pinned sha256, sha256 now)]."""
    out = []
    for path, pinned in ((docplant.PAGE, docplant.PAGE_SHA256), (ENV_FILE, ENV_SHA256)):
        now = sha256_file(path)
        if now != pinned:
            out.append((os.path.relpath(path, TASK), pinned, now))
    return out


def hidden_seeds():
    """Every seed of the hidden set and every candidate of its selection: no seed of this tool may be one of them."""
    with open(os.path.join(TESTS, "hidden", "episodes.json")) as f:
        seeds = {e["seed"] for e in json.load(f)["episodes"]}
    with open(os.path.join(TESTS, "hidden", "selection.json")) as f:
        seeds |= {c["seed"] for c in json.load(f)["candidates"]}
    return seeds


class Difference(Exception):
    """The page's plant and env.py's differ, or their numbers cannot be paired."""


# ------------------------------------------------------------------------------------------------ the pairing
def _reset_order():
    """env.py's reset: the page's draws in the order env.py consumes its numbers, with how many each takes."""
    order = [("phi0", 1), ("slew1", 1), ("slew1 sign", 1), ("bearing", 1), ("range", 1), ("vx", 1), ("vy", 1),
             ("theta", 1), ("omega", 1), ("G", 5)]
    order += [((name, n), 1) for n in range(6) for name in ("eff", "inst", "arm")]
    order += [("O1", 5), ("O2", 5), ("s1", 1), ("d1", 1), ("bias ax", 1), ("bias ay", 1), ("bias yaw", 1), ("a1", 1),
              ("b1", 1), ("K1", 1), ("S1", 1), ("M1", 1)]
    order += [(("e1", n), 1) for n in range(6)] + [(("m1", i), 1) for i in range(6)]
    order += [("E", 1), ("D", 1), ("rho", 1), ("s2", 1), ("d2", 1), ("a2", 1), ("b2", 1), ("S2", 1), ("M2", 1)]
    order += [(("e2", n), 1) for n in range(6)] + [(("m2", i), 1) for i in range(6)]
    order += [(name, 1) for name in ("slew2", "alpha", "mu", "eta", "Q", "eta2", "d", "c1", "c2", "l1", "l2", "P", "A",
                                     "omega_b", "psi0", "chi1", "chi2")]
    index, at = {}, 0
    for name, count in order:
        index[name] = (at, count)
        at += count
    return index, at


RESET_INDEX, RESET_DRAWS = _reset_order()   # (102 numbers)


class EnvReset(object):
    """draw_episode's requests answered with the numbers env.py's reset drew, each turned into its draw as env.py does
    (U(a, b): a + (b - a) u; an integer a..b: a + int(u (b - a + 1)); a sign: u < 0.5 gives -1, except slew1's, where
    it gives +1; a permutation: env.py's shuffle of 0..5 over five numbers; one of the other five: the int(5 u)-th in
    increasing order; one of the remount pairs: the int(8 u)-th in sorted order), with the page's ranges."""

    def __init__(self, raw):
        self.raw = raw
        self.used = []

    def _take(self, name):
        if name not in RESET_INDEX:
            raise Difference("the page's table draws {!r}, which env.py's reset does not draw".format(name))
        at, count = RESET_INDEX[name]
        if at + count > len(self.raw):
            raise Difference("env.py's reset drew fewer numbers than the page's table needs")
        self.used.extend(range(at, at + count))
        return self.raw[at:at + count]

    def U(self, name, a, b):
        return a + (b - a) * self._take(name)[0]

    def integer(self, name, a, b):
        return a + int(self._take(name)[0] * (b - a + 1))

    def sign(self, name):
        u = self._take(name)[0]
        if name == "slew1 sign":
            return 1 if u < 0.5 else -1
        return -1 if u < 0.5 else 1

    def permutation(self, name):
        values = list(range(6))
        for u, i in zip(self._take(name), range(5, 0, -1)):
            j = int(u * (i + 1))
            values[i], values[j] = values[j], values[i]
        return values

    def other(self, name, n):
        others = [k for k in range(6) if k != n]
        return others[int(self._take(name)[0] * len(others))]

    def one_of(self, name, options):
        options = sorted(options)
        return options[int(self._take(name)[0] * len(options))]

    def check(self):
        if sorted(self.used) != list(range(len(self.raw))):
            missing = sorted(set(range(len(self.raw))) - set(self.used))
            raise Difference("env.py's reset drew numbers {} that no draw of the page's table takes".format(missing))


ENV_CAMERA = ("camera theta", "camera phi", "camera x", "camera y", "camera port x", "camera port y")
ENV_IMU = ("imu x", "imu y", "imu yaw")
ENV_OBSERVATION = {3: ENV_IMU, 4: ("dropout",) + ENV_IMU, 9: ENV_CAMERA + ENV_IMU,
                   10: ("dropout",) + ENV_CAMERA + ENV_IMU}


class EnvNoise(object):
    """One observation's raw numbers from env.py, named in env.py's order (the dropout test, which env.py skips at k = 0
    and in a blackout; for a fresh sample the camera's six, the two yaws first; the IMU's three) and served to the
    page's plant by name."""

    def __init__(self, raw):
        names = ENV_OBSERVATION.get(len(raw))
        if names is None:
            raise Difference("env.py drew {} numbers for one observation".format(len(raw)))
        self.values = dict(zip(names, raw))
        self.asked = []

    def __call__(self, name):
        if name in self.asked or name not in self.values:
            raise Difference("the page's plant asks for {!r}, which env.py {} for this observation".format(
                name, "drew once" if name in self.asked else "did not draw"))
        self.asked.append(name)
        return self.values[name]

    def check(self):
        left = [name for name in self.values if name not in self.asked]
        if left:
            raise Difference("env.py drew {} for this observation, which the page's plant does not use".format(left))


def env_view(doc):
    """The page's plant under env.py's attribute names, each as env.py holds it (harness knowledge: before an event
    env.py keeps the event's drawn values in the attributes the event installs, and the re-key's step holds the
    deadline until the re-key fires; the camera keeps its last 12 records)."""
    d, first = doc.dr, doc.first
    return {
        "phase": doc.phi, "phase_rate": doc.slew, "x": doc.x, "y": doc.y, "vx": doc.vx, "vy": doc.vy,
        "theta": doc.theta, "omega": doc.omega, "columns": doc.c,
        "routing_order": doc.O if first else d["O1"], "rekey_order": d["O2"], "routing_initial_shift": d["s1"],
        "routing_direction": doc.dir if first else d["d1"], "routing_phase": doc.h if first else 0,
        "routing_active": first, "routing_relay_armed": doc.armed, "first_relay_reversals": doc.rev1,
        "rekey_relay_reversals": doc.rev2, "bias": doc.bias,
        "failure_nozzle": doc.a if first else d["a1"], "secondary_failure_nozzle": doc.b if first else d["b1"],
        "failure_step": d["K1"], "failure_scale": doc.S if first else d["S1"],
        "secondary_failure_scale": doc.M if first else d["M1"], "failure_exponents": d["e1"],
        "failure_valve_memory": d["m1"], "rekey_earliest": doc.E, "rekey_deadline": d["D"],
        "rekey_proximity": d["rho"], "rekey_blackout_duration": doc.L,
        "rekey_step": doc.kr if doc.rekeyed else d["D"], "rekey_initial_shift": d["s2"], "rekey_direction": d["d2"],
        "rekey_failure_nozzle": d["a2"], "rekey_secondary_failure_nozzle": d["b2"], "rekey_failure_scale": d["S2"],
        "rekey_secondary_failure_scale": d["M2"], "rekey_exponents": d["e2"], "rekey_valve_memory": d["m2"],
        "nozzle_exponents": doc.e if first else [1.0] * 6, "valve_memory": doc.m if first else [0.0] * 6,
        "slot_valves": doc.z, "rekey_phase_rate": doc.slew2, "rekey_active": doc.rekeyed,
        "imu_angle": d["alpha"], "imu_mirror": d["mu"], "imu_yaw_sign": d["eta"], "rekey_imu_linear": doc.Q,
        "rekey_imu_yaw_sign": d["eta2"], "pose_delay": d["d"], "blackouts": doc.B,
        "plume_onset_gap": docplant.ONSET, "plume_strength": docplant.STRENGTH, "plume_swirl": doc.sw,
        "plume_switch_energy": doc.T, "plume_energy": doc.cell, "port_amplitude": d["A"], "port_omega": d["omega_b"],
        "port_extension_phase": doc.psi, "port_encoder_sign": doc.chi, "rekey_port_encoder_sign": d["chi2"],
        "fuel": doc.fuel, "steps": doc.k, "success": doc.end == "docked", "failure_reason": REASON[doc.end],
        "hold_steps": doc.hold, "last_accel_body": doc.acc_body, "pose_history": doc.records[-12:],
        "last_visible_pose": doc.last_fresh, "initial_error": doc.initial_error, "best_error": doc.best_error,
    }


def _plain(value):
    """Containers as lists (the two keep a tuple or a list for the same quantity); numbers as they are."""
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


class Twin(object):
    """One episode: env.py's simulator (`env`, a TwinEnv) and the page's plant (`doc`), compared after the reset and
    after every step. `difference` is the first difference (None: equal so far); `counts` what the episode covered."""

    def __init__(self, env, seed, obs):
        self.env, self.seed, self.doc = env, seed, None
        self.shadow = ENV._RNG(seed)
        self.difference = None
        self.compared = 0
        self.counts = collections.Counter()
        self.before = None
        raw = self.drawn()
        self._guard("the reset", lambda: self._start(raw, obs))

    def _guard(self, where, fn):
        if self.difference is not None:
            return
        try:
            fn()
        except Difference as exc:
            self.difference = "{}: {}".format(where, exc)
        except Exception as exc:  # noqa: BLE001 - the page's plant failing is a difference, not a crash of the tool
            self.difference = "{}: the page's plant failed: {}: {}".format(where, type(exc).__name__, exc)

    def drawn(self):
        """The numbers env.py's generator gave since the last call, regenerated from the seed."""
        return [self.shadow.random() for _ in range(self.env.rng._counter - self.shadow._counter)]

    def _start(self, raw, obs):
        src = EnvReset(raw[:RESET_DRAWS])
        self.doc = docplant.DocPlant(docplant.draw_episode(src))
        src.check()
        self._compare(obs, self._observe(raw[RESET_DRAWS:]))

    def _observe(self, raw):
        noise = EnvNoise(raw)
        obs = self.doc.observe(noise)
        noise.check()
        return obs

    def _compare(self, obs, doc_obs, outcome=None):
        if obs is not None:
            if len(obs) != len(doc_obs):
                raise Difference("{} observation fields against the page's {}".format(len(obs), len(doc_obs)))
            for i, (a, b) in enumerate(zip(obs, doc_obs)):
                if a != b:
                    raise Difference("observation field {}: env.py {!r}, the page {!r}".format(i, a, b))
        view = env_view(self.doc)
        names = set(vars(self.env)) - HARNESS_ATTRIBUTES
        if names != set(view):
            raise Difference("env.py attributes the harness does not map: {}; mapped but not in env.py: {}".format(
                sorted(names - set(view)), sorted(set(view) - names)))
        for name in sorted(names):
            a, b = _plain(getattr(self.env, name)), _plain(view[name])
            if a != b:
                raise Difference("{}: env.py {!r}, the page {!r}".format(name, a, b))
        if outcome is not None:
            quality, done, info = outcome
            doc = self.doc
            pairs = (("quality", quality, doc.quality()), ("done", done, doc.end is not None),
                     ("success", info["success"], doc.end == "docked"),
                     ("collision", info["collision"], doc.end == "collision"), ("fuel", info["fuel"], doc.fuel),
                     ("errors", [info["position_error"], info["velocity_error"], info["angle_error"],
                                 info["rate_error"]], list(doc.errors)),
                     ("failure_reason", info["failure_reason"], REASON[doc.end]))
            for name, a, b in pairs:
                if a != b:
                    raise Difference("the step's {}: env.py {!r}, the page {!r}".format(name, a, b))

    def move(self, x, y):
        """Both plants moved to (x, y), as they stand (the open-loop episodes that start inside the plume's field)."""
        self.env.x, self.env.y = x, y
        if self.doc is not None:
            self.doc.x, self.doc.y = x, y
        self._guard("the move", lambda: self._compare(None, None))

    def before_step(self):
        env = self.env
        self.before = (list(env.plume_swirl), env.first_relay_reversals, env.rekey_relay_reversals, env.rekey_active,
                       math.hypot(env.x, env.y) - docplant.BODY < docplant.ONSET)

    def after_step(self, action, out):
        obs, quality, done, info = out
        raw = self.drawn()

        def pair():
            self.doc.step(action)
            self._compare(obs, self._observe(raw), (quality, done, info))
            self.compared += 1

        self._guard("step {}".format(self.env.steps - 1), pair)
        env, c = self.env, self.counts
        swirl, rev1, rev2, rekeyed, near = self.before
        c["steps"] += 1
        c["steps in the plume's field"] += int(near)
        c["swirl flips"] += sum(1 for a, b in zip(swirl, env.plume_swirl) if a != b)
        c["reversals of the first rotor"] += env.first_relay_reversals - rev1
        c["reversals of the second rotor"] += env.rekey_relay_reversals - rev2
        if env.routing_active and sum(min(max(float(v), 0.0), 1.0) for v in action) > docplant.STRIDE_ABOVE:
            c["two-slot strides"] += 1
        if env.rekey_active and not rekeyed:
            c["proximity re-keys" if env.rekey_step < env.rekey_deadline else "deadline re-keys"] += 1
        c["docked steps"] += int(env.hold_steps > 0)
        if obs[13] == 0.0:
            c["dropouts" if not env._is_blackout() else "observations in a blackout"] += 1
        if done:
            c["end " + info["failure_reason"]] += 1


class TwinEnv(ENV.FaultAdaptiveDocking):
    """env.py's simulator with the page's plant beside it (its Twin, `twin`)."""

    last = None

    def reset(self, seed):
        obs = super().reset(seed)
        self.twin = Twin(self, seed, obs)
        TwinEnv.last = self
        return obs

    def step(self, action):
        self.twin.before_step()
        out = super().step(action)
        self.twin.after_step(action, out)
        return out


# ------------------------------------------------------------------------------------------------ the episodes
def scripted(seed, t, obs, kind):
    """A command that is not a controller (kinds 0 to 4, the module docstring's order)."""
    h = (seed * 2654435761 + t * 40503) & 0xFFFFFFFF
    if kind == 0:   # small duties, a full one now and then
        return [((h >> (3 * i)) % 7) / 20.0 + (0.9 if (h >> (20 + i)) % 11 == 0 else 0.0) for i in range(6)]
    if kind == 1:   # bang-bang
        return [1.0 if (h >> i) & 1 else 0.0 for i in range(6)]
    if kind == 3:   # tiny duties: long episodes, deadline re-keys, timeouts
        return [((h >> (3 * i)) % 9) / 100.0 for i in range(6)]
    if kind == 4:   # tiny duties, a full one on one slot now and then (relay triggers), out-of-range entries
        u = [((h >> (3 * i)) % 9) / 120.0 for i in range(6)]
        if (h >> 24) % 6 == 0:
            u[(h >> 27) % 6] = 0.95
        if (h >> 30) & 1:
            u[(h >> 5) % 6] = -0.3 if (h >> 9) & 1 else 1.7
        return u
    # kind 2: toward the target along -p in the body frame, the nominal columns read crudely, with dither
    x, y, c, s = obs[0], obs[1], obs[2], obs[3]
    r = math.hypot(x, y) or 1.0
    bx, by = -(c * x + s * y) / r, (s * x - c * y) / r
    base = [max(0.0, bx), max(0.0, bx), max(0.0, -bx), max(0.0, -bx), max(0.0, by), max(0.0, -by)]
    return [min(1.0, 0.25 * b + ((h >> (2 * i)) % 5) / 25.0) for i, b in enumerate(base)]


def open_seed(j):
    return OPEN_BASE + OPEN_STRIDE * j


def plume_start(j):
    """Where odd open-loop episode j is moved right after the reset: inside the plume's field, at r 1.0 to 1.45 m."""
    angle = (j * 0.618) % (2 * math.pi)
    radius = 1.0 + 0.45 * ((j * 0.377) % 1.0)
    return radius * math.cos(angle), radius * math.sin(angle)


def open_episode(j):
    """Open-loop episode j (scripted commands of kind j % 5; moved inside the plume's field if j is odd): its Twin."""
    seed = open_seed(j)
    env = TwinEnv()
    obs = env.reset(seed)
    if j % 2 == 1:
        env.twin.move(*plume_start(j))
    for t in range(env.max_steps):
        obs, _, done, _ = env.step(scripted(seed, t, obs, j % 5))
        if done:
            break
    return env.twin


def reference_episode(seed, key_hex=None):
    """The privileged reference's own episode: the grader's loop (verify.run_episode, a token run, the match lock keyed
    by the dev key and the seed) over a TwinEnv, the reference served in this process (oracle_batch.InProcess).
    Returns (twin, quality, info, steps)."""
    cfg = oracle_batch.config()
    lock = ProbeLock(None, rate=cfg.get("probe_rate", 0.06), pass_fraction=cfg.get("probe_pass_fraction", 0.35),
                     tolerance=1e-4)
    lock.rng = verify.episode_stream(int(key_hex or oracle_batch.DEV_KEY, 16), seed)
    saved, TwinEnv.last = verify.FaultAdaptiveDocking, None
    verify.FaultAdaptiveDocking = TwinEnv
    try:
        quality, info, steps = verify.run_episode(seed, oracle_batch.InProcess(O.Oracle()), lock, privileged=True)
    finally:
        verify.FaultAdaptiveDocking = saved
    twin = TwinEnv.last.twin
    if twin.seed != seed:
        raise RuntimeError("the grader's loop did not run on a TwinEnv")
    if str(info.get("failure_reason", "")).startswith("aborted"):
        twin.difference = twin.difference or "the episode aborted: {}".format(info["failure_reason"])
    return twin, quality, info, steps


def act_line():
    """The page's example request regenerated (its dev seed, the duties 0.10 + 0.05 ((t + i) mod 3) until t = 37), with
    the Twin that produced it."""
    env = TwinEnv()
    obs = env.reset(ACT_SEED)
    for t in range(ACT_T):
        obs = env.step([0.10 + 0.05 * ((t + i) % 3) for i in range(6)])[0]
    return json.dumps({"type": "act", "t": ACT_T, "obs": obs}), env.twin


def _summary(twin):
    return {"seed": twin.seed, "difference": twin.difference, "compared": twin.compared, "counts": dict(twin.counts)}


def _open_job(j):
    return _summary(open_episode(j))


def _reference_job(seed):
    twin, quality, info, steps = reference_episode(seed)
    row = _summary(twin)
    row.update(quality=quality, reason=info.get("failure_reason", ""), steps=steps,
               page=(REASON[twin.doc.end] if twin.doc else None, twin.doc.k if twin.doc else None,
                     twin.doc.quality() if twin.doc else None))
    return row


def _run(job, items, workers):
    if workers > 1:
        with multiprocessing.Pool(workers) as pool:
            return list(pool.imap(job, items, chunksize=1))
    return [job(item) for item in items]


def _report(rows, what):
    bad = [r for r in rows if r["difference"] is not None]
    total = collections.Counter()
    for r in rows:
        total.update(r["counts"])
    print("{}: {} episodes, {} steps compared; every state and observation field equal after every step in {}".format(
        what, len(rows), sum(r["compared"] for r in rows), len(rows) - len(bad)))
    for r in bad[:10]:
        print("  DIFFERS seed {}: {}".format(r["seed"], r["difference"]))
    print("covered:", ", ".join("{} {}".format(k, v) for k, v in sorted(total.items())))
    return not bad


def cmd_open(a):
    if {open_seed(j) for j in range(a.count)} & hidden_seeds():
        raise SystemExit("an open-loop seed is in the hidden set or among its candidates")
    return _report(_run(_open_job, range(a.count), a.workers),
                   "open-loop episodes (scripted commands; the odd ones moved inside the plume's field)")


def cmd_reference(a):
    seeds = oracle_batch.dev_seeds(a.count, a.rng)
    if set(seeds) & hidden_seeds():
        raise SystemExit("a dev seed is in the hidden set or among its candidates")
    rows = _run(_reference_job, seeds, a.workers)
    for r in rows:
        print("{} {} steps {} quality {!r}; the page: {} steps {} quality {!r}".format(
            r["seed"], r["reason"], r["steps"], r["quality"], *r["page"]))
    ends = collections.Counter(r["reason"] for r in rows)
    same = sum(1 for r in rows if r["page"] == (r["reason"], r["steps"], r["quality"]))
    print("the reference's ends: {}; the page's plant ends the same way at the same step with the same quality in "
          "{} of {}".format(dict(ends), same, len(rows)))
    return _report(rows, "the reference's episodes (dev seeds of random.Random({}), the dev lock key)".format(a.rng)) \
        and same == len(rows)


def cmd_act(a):
    line, twin = act_line()
    with open(docplant.PAGE) as f:
        on_page = line in f.read().splitlines()
    dev = ACT_SEED not in hidden_seeds()
    print("the page's example request is env.py's: {}; the page's plant equal after every step: {}; its seed is a dev "
          "seed: {}".format(on_page, twin.difference is None, dev))
    if twin.difference is not None:
        print("  DIFFERS:", twin.difference)
    return on_page and twin.difference is None and dev


# ------------------------------------------------------------------------------------------------ the table of draws
def geometry(cx, cy, ct):
    """The nominal geometry a far-field column comes from (within 0.18 rad of its direction, its yaw of the same
    sign), or None."""
    found = [g for g, (gx, gy, gt) in enumerate(docplant.NOMINAL)
             if abs(docplant.wrap(math.atan2(cy, cx) - math.atan2(gy, gx))) <= 0.18 + 1e-9 and ct * gt > 0]
    return found[0] if len(found) == 1 else None


def world_features(w):
    """(continuous, discrete) features of a world: its draws, read off its attributes under env.py's names, the same
    way for env.py's worlds and the page's (env_view)."""
    cont, disc = {}, {}
    phi0 = w["phase"]
    cont["phi0"] = phi0
    cont["|slew1|"] = abs(w["phase_rate"])
    cont["start range"] = math.hypot(w["x"], w["y"])
    cont["start bearing - phi0"] = docplant.wrap(math.atan2(w["y"], w["x"]) - phi0)
    cont["vx"], cont["vy"] = w["vx"], w["vy"]
    cont["theta - phi0"] = docplant.wrap(w["theta"] - phi0)
    cont["omega"] = w["omega"]
    G = []
    for n, (cx, cy, ct) in enumerate(w["columns"]):
        g = geometry(cx, cy, ct)
        G.append(g)
        if g is not None:
            gx, gy, gt = docplant.NOMINAL[g]
            eff = math.hypot(cx, cy) / math.hypot(gx, gy)
            cont["eff %d" % n] = eff
            cont["inst %d" % n] = docplant.wrap(math.atan2(cy, cx) - math.atan2(gy, gx))
            cont["arm %d" % n] = ct / (eff * gt)
    disc["G"] = tuple(G)
    for name, v in zip(("bias ax", "bias ay", "bias yaw"), w["bias"]):
        cont[name] = v
    disc["K1"] = w["failure_step"]
    disc["O1"] = tuple(w["routing_order"])
    disc["s1"], disc["d1"] = w["routing_initial_shift"], int(w["routing_direction"])
    disc["(a1, b1)"] = (w["failure_nozzle"], w["secondary_failure_nozzle"])
    cont["S1"], cont["M1"] = w["failure_scale"], w["secondary_failure_scale"]
    for n in range(6):
        cont["e1 %d" % n] = w["failure_exponents"][n]
        cont["m1 %d" % n] = w["failure_valve_memory"][n]
    disc["E"] = w["rekey_earliest"]
    disc["E = K1 + 25"] = w["rekey_earliest"] == w["failure_step"] + 25
    disc["D"] = w["rekey_deadline"]
    cont["rho"] = w["rekey_proximity"]
    O1, O2 = list(w["routing_order"]), list(w["rekey_order"])
    disc["O2"] = tuple(O2)
    disc["O2 against O1"] = "equal" if O2 == O1 else ("O1 rotated by one" if O2 == O1[1:] + O1[:1] else "other")
    disc["s2"], disc["d2"] = w["rekey_initial_shift"], int(w["rekey_direction"])
    disc["(a2, b2)"] = (w["rekey_failure_nozzle"], w["rekey_secondary_failure_nozzle"])
    disc["nozzles the two pairs share"] = len({w["failure_nozzle"], w["secondary_failure_nozzle"]}
                                              & {w["rekey_failure_nozzle"], w["rekey_secondary_failure_nozzle"]})
    cont["S2"], cont["M2"] = w["rekey_failure_scale"], w["rekey_secondary_failure_scale"]
    for n in range(6):
        cont["e2 %d" % n] = w["rekey_exponents"][n]
        cont["m2 %d" % n] = w["rekey_valve_memory"][n]
    cont["|slew2|"] = abs(w["rekey_phase_rate"])
    disc["signs of slew1, slew2"] = (w["phase_rate"] > 0, w["rekey_phase_rate"] > 0)
    cont["alpha"] = w["imu_angle"]
    disc["mu"], disc["eta"], disc["eta2"] = int(w["imu_mirror"]), int(w["imu_yaw_sign"]), int(w["rekey_imu_yaw_sign"])
    disc["Q"] = tuple(round(v, 12) for row in w["rekey_imu_linear"] for v in row)
    disc["d"] = w["pose_delay"]
    (c1, end1), (c2, end2) = w["blackouts"]
    disc["c1"], disc["l1"], disc["c2"], disc["l2"] = c1, end1 - c1, c2, end2 - c2
    disc["L"] = w["rekey_blackout_duration"]
    disc["L = 30 + ((K1 + D) mod 13)"] = (w["rekey_blackout_duration"]
                                          == 30 + ((w["failure_step"] + w["rekey_deadline"]) % 13))
    P = 1 if w["plume_swirl"][0] > 0 else -1
    disc["P"] = P
    disc["swirl / P"] = tuple(round(s * P, 12) for s in w["plume_swirl"])
    cont["A"], cont["omega_b"], cont["psi0"] = w["port_amplitude"], w["port_omega"], w["port_extension_phase"]
    disc["chi1"], disc["chi2"] = int(w["port_encoder_sign"]), int(w["rekey_port_encoder_sign"])
    return cont, disc


def ks_distance(a, b):
    """The two-sample Kolmogorov-Smirnov distance."""
    a, b = sorted(a), sorted(b)
    i = j = 0
    d = 0.0
    while i < len(a) and j < len(b):
        v = min(a[i], b[j])
        while i < len(a) and a[i] == v:
            i += 1
        while j < len(b) and b[j] == v:
            j += 1
        d = max(d, abs(i / len(a) - j / len(b)))
    return d


def ks_p(d, n, m):
    """Its asymptotic p-value (Kolmogorov's distribution, with the usual small-sample correction)."""
    ne = n * m / float(n + m)
    lam = (math.sqrt(ne) + 0.12 + 0.11 / math.sqrt(ne)) * d
    if lam < 0.2:
        return 1.0
    total = 0.0
    for k in range(1, 200):
        term = 2.0 * (-1) ** (k - 1) * math.exp(-2.0 * k * k * lam * lam)
        total += term
        if abs(term) < 1e-16:
            break
    return min(1.0, max(0.0, total))


def gamma_q(a, x):
    """The regularized upper incomplete gamma function Q(a, x) (series below a + 1, continued fraction above)."""
    if x <= 0.0:
        return 1.0
    log_front = -x + a * math.log(x) - math.lgamma(a)
    if x < a + 1.0:
        ap, term, total = a, 1.0 / a, 1.0 / a
        for _ in range(100000):
            ap += 1.0
            term *= x / ap
            total += term
            if abs(term) < abs(total) * 1e-16:
                break
        return max(0.0, 1.0 - total * math.exp(log_front))
    tiny = 1e-300
    b = x + 1.0 - a
    c, d = 1.0 / tiny, 1.0 / b
    h = d
    for i in range(1, 100000):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        d = tiny if abs(d) < tiny else d
        c = b + an / c
        c = tiny if abs(c) < tiny else c
        d = 1.0 / d
        step = d * c
        h *= step
        if abs(step - 1.0) < 1e-16:
            break
    return math.exp(log_front) * h


def chi2_homogeneity(ca, cb):
    """(statistic, degrees of freedom, p-value) of the chi-square test that two samples' categories share a law."""
    na, nb = sum(ca.values()), sum(cb.values())
    keys = set(ca) | set(cb)
    stat = sum((ca[k] * nb - cb[k] * na) ** 2 / float(na * nb * (ca[k] + cb[k])) for k in keys)
    df = len(keys) - 1
    return stat, df, (gamma_q(df / 2.0, stat / 2.0) if df > 0 else 1.0)


def cmd_draws(a):
    seeds = oracle_batch.dev_seeds(a.count, a.rng)
    if set(seeds) & hidden_seeds():
        raise SystemExit("a dev seed is in the hidden set or among its candidates")
    worlds = {"env.py": [], "the page": []}
    for seed in seeds:
        e = ENV.FaultAdaptiveDocking()
        e.reset(seed)
        worlds["env.py"].append(world_features(vars(e)))
    src = docplant.PageRandom(random.Random("the page {}".format(a.rng)))
    for _ in range(a.count):
        worlds["the page"].append(world_features(env_view(docplant.DocPlant(docplant.draw_episode(src)))))
    rows = []
    for kind in (0, 1):
        names = sorted(set().union(*(w[kind] for w in worlds["env.py"] + worlds["the page"])))
        for name in names:
            xs = [w[kind].get(name) for w in worlds["env.py"]]
            ys = [w[kind].get(name) for w in worlds["the page"]]
            if kind == 0:
                xs, ys = [v for v in xs if v is not None], [v for v in ys if v is not None]
                if not xs or not ys:
                    rows.append((0.0, name, "KS -", "absent from {}".format("env.py" if not xs else "the page")))
                    continue
                d = ks_distance(xs, ys)
                rows.append((ks_p(d, len(xs), len(ys)), name, "KS {:.4f}".format(d),
                             "env.py [{:+.5f}, {:+.5f}] ({}), the page [{:+.5f}, {:+.5f}] ({})".format(
                                 min(xs), max(xs), len(xs), min(ys), max(ys), len(ys))))
            else:
                ca, cb = collections.Counter(xs), collections.Counter(ys)
                stat, df, p = chi2_homogeneity(ca, cb)
                note = "values: env.py {}, the page {}{}".format(
                    len(ca), len(cb), "" if set(ca) == set(cb) else " (supports differ)")
                rows.append((p, name, "chi2 {:.1f} on {} df".format(stat, df), note))
    tests = sum(1 for r in rows if not r[2].endswith(" 0 df"))
    line = 0.01 / tests
    rows.sort()
    print("{} env.py worlds (dev seeds of random.Random({})) against {} episodes of the page's table".format(
        a.count, a.rng, a.count))
    for p, name, stat, note in rows:
        print("  p {:<9.4g} {:32s} {:24s} {}{}".format(p, name, stat, note, "  BELOW THE LINE" if p < line else ""))
    worst = rows[0]
    print("{} tests ({} continuous: two-sample Kolmogorov-Smirnov; {} discrete: chi-square homogeneity, {} with a "
          "single value on both sides); the smallest p-value {:.4g} ({}); the family-wise 1% line (Bonferroni) "
          "{:.3g}".format(len(rows), sum(1 for r in rows if r[2].startswith("KS")),
                          sum(1 for r in rows if r[2].startswith("chi2")), len(rows) - tests, worst[0], worst[1], line))
    return all(p >= line for p, _, _, _ in rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("open")
    p.add_argument("--count", type=int, default=400)
    p.add_argument("--workers", type=int, default=1)
    p.set_defaults(fn=cmd_open)
    p = sub.add_parser("reference")
    p.add_argument("--count", type=int, default=100)
    p.add_argument("--rng", type=int, default=REFERENCE_RNG)
    p.add_argument("--workers", type=int, default=1)
    p.set_defaults(fn=cmd_reference)
    p = sub.add_parser("draws")
    p.add_argument("--count", type=int, default=50000)
    p.add_argument("--rng", type=int, default=1)
    p.set_defaults(fn=cmd_draws)
    p = sub.add_parser("act")
    p.set_defaults(fn=cmd_act)
    a = ap.parse_args()
    ok = a.fn(a)
    moved = moved_pins()
    for path, pinned, now in moved:
        print("MOVED: {} is no longer the version these checks passed on (pinned {}, now {})".format(path, pinned, now))
    print("OK" if ok and not moved else "FAILED")
    sys.exit(0 if ok and not moved else 1)


if __name__ == "__main__":
    main()
