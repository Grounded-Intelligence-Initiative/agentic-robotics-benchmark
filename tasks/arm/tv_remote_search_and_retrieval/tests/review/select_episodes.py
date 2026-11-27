"""Selection of the fixed hidden episode set (a maintainer's tool; the grader never runs it).

The rule, fixed before any candidate was drawn (it is also in the pull request's pre-registration):

1. Freeze. The grader's episode loop and world (tests/remote_grade.py, tests/episodes.py, tests/remote_env.py,
   tests/grader_config.json), the launcher and the numerics fingerprint (tests/nonet_exec.py,
   tests/numerics_check.py, tests/numerics.json), the room's files' pin (environment/assets.sha256) and the reference
   (solution/solver/*.py, solution/solve.sh) are committed first: `draw` refuses to start while any of them differs
   from the checkout's HEAD and records their sha256 and the commit; `run` and `make` refuse to go on when any of
   them differs from the recorded hashes.
2. Draw. `draw` takes the set's key from os.urandom(32) (hex) and writes it to the record before any candidate runs.
   Place p's candidate i (i = 0, 1, 2, ...) has the 64-bit seed
   int.from_bytes(blake2b(b"candidate|<p>|<i>", key=key, digest_size=8).digest(), "little").
3. Composition. 12 episodes: two per place for under the settee, under the chest, under the hat, in the basket and
   loose in a drawer; one under each seat pad (the two pads are one place).
4. Run. Candidates i = 0..BATCH-1 of every place, then the next BATCH of a place that is still short, up to
   MAX_CANDIDATES. A candidate in which more than HIDDEN_PIXELS pixels of the TV remote show in either camera at the
   start is rejected unplayed. Otherwise the privileged reference plays it three times over the real wire
   (remote_grade.run_episodes with that one episode and the oracle submission as solution/solve.sh lays it out, given
   the token truth; the probe lock at the grader's rate with a fresh OS-entropy stream per run), stopping at the first
   run that is not kept.
5. Keep. A candidate is kept when each of the three wire runs scores 1.0 within STEP_MARGIN of the step cap (so no
   graded episode needs the reference to use more than 60 % of its steps). Per place the first kept candidates in
   candidate order are the place's episodes. Every candidate tried is recorded: kept, or rejected with its runs.
6. Void. If a place is short after MAX_CANDIDATES, the draw is void: `draw --void REASON` records why and draws a
   new key; the record keeps every draw. A frozen file that changes after a draw voids it too
   (`draw --void REASON --refreeze`).
7. Order. `make` shuffles the 12 (places in the order of PLAN, a place's episodes in candidate order; a Fisher-Yates
   in which position i swaps with int.from_bytes(blake2b(b"order|<i>", key=key, digest_size=8).digest(), "little")
   mod (i + 1), i from 11 down to 1) and writes tests/hidden/episodes.json in that order: the graded order.

    python3 -B tests/review/select_episodes.py draw
    python3 -B tests/review/select_episodes.py run --work DIR [--workers 12]
    python3 -B tests/review/select_episodes.py make

`run` needs the task image's Python and the numerics pin of environment/Dockerfile (set here from
nonet_exec.NUMERICS_ENV; tests/numerics_check.py must reproduce tests/numerics.json). Outside the grader the oracle
submission's run.sh names this interpreter and the controller is started directly rather than through nonet_exec.py
(which needs root to change user); everything else is remote_grade.run_episodes as the grader runs it.
"""

import argparse
import datetime
import hashlib
import json
import multiprocessing
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
SOLVER = os.path.join(TASK, "solution", "solver")
RECORD = os.path.join(TESTS, "hidden", "selection.json")
OUT = os.path.join(TESTS, "hidden", "episodes.json")
sys.dont_write_bytecode = True

FORMAT = "tv_remote_search_and_retrieval episode selection 2"
SET_FORMAT = "tv_remote_search_and_retrieval fixed set 2"
PLAN = (("left_cushion", 1), ("right_cushion", 1), ("under_sofa", 2), ("under_chest", 2), ("under_hat", 2),
        ("in_box", 2), ("in_drawer", 2))
SOLVER_MODULES = ("main.py", "oracle.py", "search_skills.py", "skills.py", "kin.py")
FROZEN = (("tests/remote_grade.py", "tests/episodes.py", "tests/remote_env.py", "tests/grader_config.json",
           "tests/nonet_exec.py", "tests/numerics_check.py", "tests/numerics.json", "environment/assets.sha256")
          + tuple("solution/solver/" + f for f in SOLVER_MODULES) + ("solution/solve.sh",))
WIRE_RUNS = 3
BATCH = 4
MAX_CANDIDATES = 16
HIDDEN_PIXELS = 8
STEP_MARGIN = 0.6
ENV = {"MUJOCO_GL": "egl", "PYOPENGL_PLATFORM": "egl", "MESA_SHADER_CACHE_DISABLE": "true"}


def sha256_file(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def frozen_hashes():
    return {p: sha256_file(os.path.join(TASK, p)) for p in FROZEN}


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git(*args):
    return subprocess.run(["git", "-C", TASK] + list(args), capture_output=True, text=True, timeout=60, check=False)


def frozen_committed():
    tracked = git("ls-files", "--error-unmatch", "--", *FROZEN)
    clean = git("diff", "--quiet", "HEAD", "--", *FROZEN)
    return tracked.returncode == 0 and clean.returncode == 0


def load(path):
    with open(path) as f:
        return json.load(f)


def save(path, rec):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def keyed(key_hex, msg):
    return int.from_bytes(hashlib.blake2b(msg.encode(), key=bytes.fromhex(key_hex), digest_size=8).digest(), "little")


def candidate_seed(key_hex, place, i):
    return keyed(key_hex, "candidate|%s|%d" % (place, i))


def check_frozen(rec):
    now = frozen_hashes()
    bad = [p for p in FROZEN if rec["frozen"].get(p) != now[p]]
    if bad:
        raise SystemExit("frozen files changed since the draw: " + ", ".join(bad))


def open_draw(rec):
    d = rec["draws"][-1]
    if d["status"] != "open":
        raise SystemExit("the latest draw is %s: draw again" % d["status"])
    return d


def _numerics_env():
    sys.path.insert(0, TESTS)
    import nonet_exec  # noqa: E402
    env = dict(ENV)
    env.update(nonet_exec.NUMERICS_ENV)
    for k in ("REMOTE_ROOM_DIR", "OPENCV_CPU_DISABLE", "LP_NUM_THREADS"):
        if os.environ.get(k):
            env[k] = os.environ[k]
    return env


def _modules():
    os.environ.update(_numerics_env())
    if TESTS not in sys.path:
        sys.path.insert(0, TESTS)
    import episodes as R  # noqa: E402
    import remote_env as E  # noqa: E402
    import remote_grade as G  # noqa: E402
    return E, R, G


def submission(work):
    """The oracle submission as solution/solve.sh lays it out, with run.sh naming this interpreter."""
    if os.path.isdir(work):
        shutil.rmtree(work)
    os.makedirs(work)
    for f in SOLVER_MODULES:
        shutil.copy(os.path.join(SOLVER, f), work)
    shutil.copy(os.path.join(TESTS, "remote_env.py"), work)          # byte-identical to the agent's copy
    shutil.copy(os.path.join(TASK, "solution", "oracle_token"), os.path.join(work, ".oracle_token"))
    with open(os.path.join(work, "run.sh"), "w") as f:
        f.write("#!/usr/bin/env bash\nset -euo pipefail\ncd \"$(dirname \"$0\")\"\n"
                "export %s\n"
                "exec %s -B main.py\n" % (" ".join("%s='%s'" % kv for kv in sorted(_numerics_env().items())),
                                             sys.executable))
    os.chmod(os.path.join(work, "run.sh"), 0o755)
    return work


def over_wire(G, place, seed, work):
    from robotics_grader import Solver, seed_rng

    class HostSolver(Solver):
        def _spawn(self):
            return subprocess.Popen(["bash", self.entry], cwd=self.submission, stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True,
                                    bufsize=1, start_new_session=True)

    # each wire run with a fresh OS-entropy stream, not the grader's per-episode stream (probe_stream keys on the
    # episode's index in the graded order, which `make` decides later)
    G.probe_stream = lambda _index, _seed: seed_rng()
    config = dict(G.CONFIG, **{k: v for k, v in load(os.path.join(TESTS, "grader_config.json")).items()
                               if k in G.CONFIG})
    t0 = time.monotonic()
    solver = HostSolver(step_timeout=config["step_timeout_s"], submission=work)
    try:
        records, lock, clock = G.run_episodes(solver, [(0, seed, place)], True, config=config)
    finally:
        solver.close()
    r = records[0]
    return {"score": float(r["score"]), "ended": r["ended"], "steps": int(r["steps"]), "probes": int(lock.total),
            "probes_passed": int(lock.passed), "wall_s": round(time.monotonic() - t0, 1),
            "controller_s": round(clock.total, 2)}


def run_candidate(args):
    place, i, key_hex, work_root = args
    E, R, G = _modules()
    seed = candidate_seed(key_hex, place, i)
    c = {"place": place, "index": i, "seed": seed, "wire": []}
    env = E.Env()
    try:
        env.reset(R.make_episode(E, seed, place))
        shows = int(max(env._remote_pixels(name) for name in E.CAMERAS))
    finally:
        env.close()
    c["shows_at_start"] = shows
    ok = shows <= HIDDEN_PIXELS
    if ok:
        for k in range(WIRE_RUNS):
            w = over_wire(G, place, seed, submission(os.path.join(work_root, "%s_%d_%d" % (place, i, k))))
            c["wire"].append(w)
            if w["score"] != 1.0 or w["steps"] > STEP_MARGIN * E.MAX_STEPS:
                ok = False
                break
    c["status"] = "kept" if ok else "rejected"
    print("[select] candidate %d of a place: %s (%s)" % (i, c["status"], [(w["score"], w["steps"]) for w in c["wire"]]),
          flush=True)
    return c


def cmd_draw(args):
    rec = load(RECORD) if os.path.exists(RECORD) and load(RECORD).get("format") == FORMAT else None
    if not frozen_committed():
        raise SystemExit("commit the frozen files first: " + ", ".join(FROZEN))
    head = git("rev-parse", "HEAD").stdout.strip()
    if rec is None:
        rec = {"format": FORMAT, "rule": __doc__.split("\n\n    python3")[0], "plan": [list(p) for p in PLAN],
               "wire_runs": WIRE_RUNS, "batch": BATCH, "max_candidates": MAX_CANDIDATES, "hidden_pixels": HIDDEN_PIXELS,
               "step_margin": STEP_MARGIN, "frozen": frozen_hashes(), "frozen_commit": head, "draws": []}
    else:
        last = rec["draws"][-1]
        if last["status"] == "open":
            if not args.void:
                raise SystemExit("a draw is open: void it with --void REASON")
            last.update(status="void", void_reason=args.void, voided=utc_now())
            if args.refreeze:
                rec.setdefault("refrozen", []).append({"frozen": rec["frozen"], "frozen_commit": rec["frozen_commit"],
                                                       "until_draw": len(rec["draws"])})
                rec["frozen"], rec["frozen_commit"] = frozen_hashes(), head
        check_frozen(rec)
    rec["draws"].append({"key": os.urandom(32).hex(), "drawn": utc_now(), "status": "open", "candidates": []})
    save(RECORD, rec)
    print("draw %d open" % len(rec["draws"]))


def cmd_run(args):
    rec = load(RECORD)
    check_frozen(rec)
    d = open_draw(rec)
    while True:
        todo = []
        for place, need in PLAN:
            mine = [c for c in d["candidates"] if c["place"] == place]
            if sum(c["status"] == "kept" for c in mine) >= need:
                continue
            if len(mine) >= MAX_CANDIDATES:
                raise SystemExit("a place is short after %d candidates: the draw is void" % MAX_CANDIDATES)
            todo += [(place, i, d["key"], args.work) for i in range(len(mine), min(len(mine) + BATCH, MAX_CANDIDATES))]
        if not todo:
            break
        with multiprocessing.get_context("spawn").Pool(args.workers) as pool:
            d["candidates"] += pool.map(run_candidate, todo, chunksize=1)
        d["candidates"].sort(key=lambda c: ([p for p, _ in PLAN].index(c["place"]), c["index"]))
        save(RECORD, rec)
    print("every place has its episodes")


def cmd_make(args):
    rec = load(RECORD)
    check_frozen(rec)
    d = open_draw(rec)
    chosen = []
    for place, need in PLAN:
        kept = [c for c in d["candidates"] if c["place"] == place and c["status"] == "kept"][:need]
        if len(kept) < need:
            raise SystemExit("a place is short: run first")
        for c in kept:
            c["selected"] = True
            chosen.append({"seed": c["seed"], "spot": place})
    for i in range(len(chosen) - 1, 0, -1):
        j = keyed(d["key"], "order|%d" % i) % (i + 1)
        chosen[i], chosen[j] = chosen[j], chosen[i]
    d.update(status="made", made=utc_now())
    save(RECORD, rec)
    save(OUT, {"format": SET_FORMAT, "episodes": chosen})
    print("wrote %d episodes" % len(chosen))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("draw")
    p.add_argument("--void")
    p.add_argument("--refreeze", action="store_true")
    p = sub.add_parser("run")
    p.add_argument("--work", required=True)
    p.add_argument("--workers", type=int, default=12)
    sub.add_parser("make")
    args = ap.parse_args()
    {"draw": cmd_draw, "run": cmd_run, "make": cmd_make}[args.cmd](args)


if __name__ == "__main__":
    main()
