"""The grader of quadruped_morphology_for_terrain. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

Design artifact: the agent leaves /home/user/submission/robot.xml, a QM-24 robot description (a strict MJCF subset
over the catalog's parts). This process reads it as data only (qm_design.py: size cap, no DTD or entities, only
the dialect's elements and attributes), checks it against the catalog and the client's limits, and REBUILDS its own
MuJoCo model from the catalog (qm_sim.py): masses, inertias, sizes, friction, servo gains, torque limits, contact
and solver settings all come from the catalog and this file, never from the submission. The agent's XML is never
handed to MuJoCo and nothing of the agent's runs here.

The rebuilt robot runs the site's standard gait (qm_gait.py, the agent's own copy is byte-identical) on hidden
instances of the walkway's terrain family (qm_sim.make_terrain, knobs in grader_config.json); the instances come
from OS entropy in this process and never touch disk.

Metric `terrain_progress`: the mean over the instances of min(forward progress of the body / track length, 1).
A fall (body or pod touching anything, a leg part hitting an overhead pipe, tilt over 50 degrees, body underside
lower than 2.5 cm over the ground) ends that run and keeps the best progress made at least 0.5 s before it; a
simulation fault (divergence, runaway speed, flight, solver warnings) scores that run 0. A missing, unreadable or
unbuildable design scores 0 (zero_verdict). Higher is better.
"""

import os
import stat
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import qm_design as D  # noqa: E402
from robotics_grader import ratio_rewards, read_anchor, seed_rng, stage_config, write_verdict, zero_verdict  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CATALOG = D.load_catalog(os.path.join(HERE, "catalog.json"))


class Deadline(object):
    """The verify phase's own deadline (ALE_VERIFY_DEADLINE_SECONDS, when the engine sets one) less a margin."""

    def __init__(self, margin_s):
        secs = os.environ.get("ALE_VERIFY_DEADLINE_SECONDS")
        self.end = time.monotonic() + float(secs) - margin_s if secs else None

    def left(self):
        return None if self.end is None else self.end - time.monotonic()


def read_design(path):
    """The submission's bytes: a regular file, not followed through a link, at most MAX_BYTES."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | getattr(os, "O_NONBLOCK", 0))
    except OSError as exc:
        raise D.DesignError("no readable robot.xml in the submission (%s)" % exc.strerror)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode):
            raise D.DesignError("robot.xml is not a regular file")
        if st.st_size > D.MAX_BYTES:
            raise D.DesignError("robot.xml is larger than %d bytes" % D.MAX_BYTES)
        with os.fdopen(fd, "rb") as f:
            fd = None
            return f.read(D.MAX_BYTES + 1)
    finally:
        if fd is not None:
            os.close(fd)


def run_instance(args):
    data, seed, knobs = args
    import qm_sim as S
    design, _ = D.check(data, CATALOG)
    terrain = S.make_terrain(np.random.default_rng(seed), knobs)
    r = S.Episode(design, CATALOG, terrain).run()
    return r["progress"], r["outcome"], r["reason"]


def main():
    t0 = time.monotonic()
    config = stage_config()
    anchor, meta = read_anchor()
    rng = seed_rng()
    deadline = Deadline(float(config.get("deadline_margin_s", 120)))
    home = os.environ.get("ALE_HOME", "/home/user")
    path = os.path.join(home, "submission", "robot.xml")
    try:
        data = read_design(path)
        design, summ = D.check(data, CATALOG)
    except Exception as exc:  # noqa: BLE001 (DesignError, or anything else an unusable file causes: an honest 0)
        write_verdict(zero_verdict(reason="design rejected: %s" % exc, anchored=anchor is not None))
        print("[grader] design rejected: %s" % exc, file=sys.stderr, flush=True)
        return
    n = int(config.get("instances", 100))
    knobs = config.get("knobs", {})
    seeds = [rng.seed(bits=31) for _ in range(n)]
    workers = max(1, min(int(config.get("workers", 4)), os.cpu_count() or 1))
    results = [None] * n
    with Pool(workers) as pool:
        pending = {i: pool.apply_async(run_instance, ((data, sd, knobs),)) for i, sd in enumerate(seeds)}
        for i, res in pending.items():
            left = deadline.left()
            try:
                results[i] = res.get(timeout=None if left is None else max(left, 1.0))
            except Exception as exc:  # noqa: BLE001 (a run past the deadline or a crash scores 0 for that run)
                results[i] = (0.0, "fault", "not finished: %s" % type(exc).__name__)
    prog = [float(r[0]) for r in results]
    measured = float(np.mean(prog))
    goals = sum(r[1] == "goal" for r in results)
    falls = sum(r[1] == "fall" for r in results)
    faults = sum(r[1] == "fault" for r in results)
    verdict = ratio_rewards(
        measured, anchor, cap=config.get("scoring_cap", 1.5),
        higher_is_better=meta.get("direction", "higher") == "higher", metric_name="terrain_progress",
        extra=dict(instances=float(n), goals=float(goals), falls=float(falls), faults=float(faults),
                   price_usd=float(summ["price_usd"]), mass_kg=float(summ["mass_kg"])),
        full_at=meta.get("full_at", 1.0))
    text = "terrain_progress %.4f over %d instances: %d reached the goal, %d fell, %d faulted; per instance %s; " \
           "design %s, %.0f USD, %.3f kg; %.0fs" % (
               measured, n, goals, falls, faults, [round(p, 3) for p in prog], design["body"], summ["price_usd"],
               summ["mass_kg"], time.monotonic() - t0)
    write_verdict(verdict, diagnostic=text)
    print("[grader] " + text, file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
