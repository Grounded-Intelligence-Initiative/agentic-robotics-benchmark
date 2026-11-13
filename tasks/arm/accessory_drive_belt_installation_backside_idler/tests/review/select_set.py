"""Auditable selection of the fixed hidden set (maintainers' tool; never run by the grader).

    python3 select_set.py register              # 1. the rule and the sha256 of every frozen file, timestamped
    python3 select_set.py draw                  # 2. 8 candidate seeds from os.urandom, timestamped after the registration
    python3 select_set.py run <i> <panda dir>   # 3. one candidate's start state, in draw order: frozen files re-hashed
                                                #    first (frozen_ok); the cell built and left idle for 1 s
    python3 select_set.py select                # 4. seeds.json = the first 4 candidates whose start state is sound
    python3 select_set.py amend <note>          # a grader-side amendment: the previous and the new frozen hashes and the
                                                #    files that changed, recorded; every candidate is re-run after it
                                                #    and the rule re-applied (the hidden seeds must not change)

Rule (written by `register` before any draw): candidates are taken in draw order; a candidate is kept when its start
state is sound (no contact deeper than 0.5 mm when the episode starts, the belt on its peg, no belt link moving more
than 5 mm over 1 s idle with the arms at home); the first 4 kept form the hidden set. No candidate is skipped or
re-drawn for any other reason: not for the belt's or the tensioner's physics (length, stiffness, friction, spring),
not for any program's result. The records go to tests/hidden/selection.json.
"""
import datetime
import hashlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
PKG = os.path.dirname(TESTS)
SEL = os.path.join(TESTS, "hidden", "selection.json")
SEEDS = os.path.join(TESTS, "hidden", "seeds.json")
RUNS0 = os.path.join(TESTS, "hidden", "selection_runs")
RULE = ("candidates in draw order; keep a candidate whose start state is sound (no contact deeper than 0.5 mm when the "
        "episode starts, the belt on its peg, no belt link moving more than 5 mm over 1 s idle with the arms at home); "
        "the first 4 kept are the hidden set; nothing is skipped or re-drawn for any other reason (not for the belt's or "
        "the tensioner's physics, not for any program's result)")
N_CAND, N_KEEP = 8, 4
START = {"contact_mm": 0.5, "idle_move_mm": 5.0}


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def frozen_files():
    """everything that decides a graded outcome: the cell, the grader, the givens, the oracle (when present), the spec"""
    out = []
    roots = [os.path.join(TESTS, d) for d in ("beltcell", "cellkit", "robotics_grader", "setup_payload")]
    roots += [os.path.join(HERE, "honest"), os.path.join(PKG, "environment"), os.path.join(PKG, "solution")]
    for root in roots:
        for dp, dns, fns in os.walk(root):
            dns[:] = sorted(d for d in dns if d != "__pycache__")
            for fn in sorted(fns):
                if fn != ".DS_Store":
                    out.append(os.path.relpath(os.path.join(dp, fn), PKG))
    for rel in ("tests/verify.py", "tests/test.sh", "tests/grader_config.json", "tests/given_hashes.json", "task.toml",
                "instruction.md", "tests/review/full_at_rule.md", "tests/review/fresh_seeds.json"):
        if os.path.exists(os.path.join(PKG, rel)):
            out.append(rel)
    return sorted(out)


def hashes():
    return {rel: hashlib.sha256(open(os.path.join(PKG, rel), "rb").read()).hexdigest() for rel in frozen_files()}


def load():
    return json.load(open(SEL))


def save(rec):
    json.dump(rec, open(SEL, "w"), indent=1)


def runs_dir(rec):
    am = rec.get("amendments") or []
    return RUNS0 if not am else os.path.join(TESTS, "hidden", "selection_runs_" + am[-1]["cell_version"].split("/")[-1])


def cell_version():
    sys.path.insert(0, TESTS)
    from beltcell import CELL_VERSION
    return CELL_VERSION


def tool_sha():
    return hashlib.sha256(open(os.path.abspath(__file__), "rb").read()).hexdigest()


def start_state(srv):
    """(max contact depth at the episode's start in mm, max displacement of any belt link over 1 s idle in mm, the
    belt's lowest point above the base plate in mm)"""
    import mujoco
    import numpy as np
    sc = srv.scene
    m, d = sc.model, sc.data
    mujoco.mj_forward(m, d)
    depth = max([0.0] + [-float(d.contact[i].dist) for i in range(d.ncon)])
    c0 = np.array(sc.belt.centres(d), float)
    for _ in range(int(round(1.0 / srv.tick_s))):
        srv.physics_tick()
    c1 = np.array(sc.belt.centres(d), float)
    move = float(np.max(np.linalg.norm(c1 - c0, axis=1)))
    low = float(np.min(c1[:, 2]))
    return round(depth * 1000, 3), round(move * 1000, 3), round(low * 1000, 1)


def cmd_register():
    if os.path.exists(SEL):
        sys.exit("already registered: use amend")
    os.makedirs(os.path.dirname(SEL), exist_ok=True)
    rec = {"version": 1, "rule": RULE, "start_limits": START, "registered_utc": now(), "cell": cell_version(),
           "tool_sha256": tool_sha(), "frozen": hashes(), "candidates": [], "runs": [], "kept": None}
    save(rec)
    print("registered", rec["registered_utc"], len(rec["frozen"]), "files")


def cmd_draw():
    rec = load()
    if rec["candidates"]:
        sys.exit("already drawn")
    rec["drawn_utc"] = now()
    rec["candidates"] = [int.from_bytes(os.urandom(8), "big") >> 1 for _ in range(N_CAND)]
    save(rec)
    print("drawn", rec["drawn_utc"], rec["candidates"])


def cmd_run(i, panda):
    rec = load()
    seed = rec["candidates"][i]
    frozen_ok = hashes() == rec["frozen"]
    RUNS = runs_dir(rec)
    sys.path.insert(0, TESTS)
    from beltcell.server import BeltCellServer
    cfg_p = os.path.join(TESTS, "grader_config.json")
    cfg = json.load(open(cfg_p)) if os.path.exists(cfg_p) else None
    started = now()
    t0 = time.time()
    srv = BeltCellServer(os.path.abspath(panda), config=cfg, render=False, log=lambda s: None)
    srv.reset(seed)
    depth_mm, move_mm, low_mm = start_state(srv)
    import numpy
    import mujoco
    where = ("task image " + os.environ["IMAGE_DIGEST"]) if os.environ.get("IMAGE_DIGEST") else "host"
    run = {"i": i, "seed": seed, "started_utc": started, "frozen_ok": frozen_ok, "where": where, "cell": cell_version(),
           "python": sys.version.split()[0], "numpy": numpy.__version__, "mujoco": mujoco.__version__,
           "tool_sha256": tool_sha(), "start_contact_depth_mm": depth_mm, "idle_1s_max_move_mm": move_mm,
           "belt_lowest_mm": low_mm, "engine_id": srv.layout["engine_id"],
           "start_ok": depth_mm < START["contact_mm"] and move_mm < START["idle_move_mm"],
           "wall_s": round(time.time() - t0, 1)}
    os.makedirs(RUNS, exist_ok=True)
    json.dump(run, open(os.path.join(RUNS, "run_%d.json" % i), "w"), indent=1)
    print(json.dumps(run))


def cmd_select():
    rec = load()
    RUNS = runs_dir(rec)
    runs = {}
    for fn in sorted(os.listdir(RUNS)):
        r = json.load(open(os.path.join(RUNS, fn)))
        runs[r["i"]] = r
    kept = []
    for i, seed in enumerate(rec["candidates"]):
        r = runs.get(i)
        if r is None:
            sys.exit("candidate %d has not run yet: candidates are judged in draw order" % i)
        if not r["frozen_ok"] or r["started_utc"] < rec["drawn_utc"]:
            sys.exit("candidate %d: frozen files changed or run before the draw" % i)
        if rec.get("amendments") and r["started_utc"] < rec["amendments"][-1]["utc"]:
            sys.exit("candidate %d: run before the amendment" % i)
        if r["start_ok"]:
            kept.append(seed)
        if len(kept) == N_KEEP:
            break
    if len(kept) < N_KEEP:
        sys.exit("fewer than %d candidates kept" % N_KEEP)
    if rec.get("amendments"):
        am = rec["amendments"][-1]
        am["runs"] = [runs[i] for i in sorted(runs)]
        am["kept"] = kept
        am["kept_unchanged"] = kept == rec["kept"]
        am["selected_utc"] = now()
        save(rec)
        print("amendment re-selection: kept", kept, "unchanged" if kept == rec["kept"] else "CHANGED")
        return
    rec["runs"] = [runs[i] for i in sorted(runs)]
    rec["kept"] = kept
    rec["selected_utc"] = now()
    save(rec)
    json.dump({"seeds": kept, "rule": RULE, "selection": "tests/hidden/selection.json"}, open(SEEDS, "w"), indent=1)
    print("kept", kept)


def cmd_amend(note):
    rec = load()
    prev, new = rec["frozen"], hashes()
    am = {"utc": now(), "note": note, "cell_version": cell_version(),
          "previous_frozen_sha256": hashlib.sha256(json.dumps(prev, sort_keys=True).encode()).hexdigest(),
          "previous_frozen": prev, "changed": sorted(k for k in set(prev) | set(new) if prev.get(k) != new.get(k)),
          "tool_sha256": tool_sha()}
    rec.setdefault("amendments", []).append(am)
    rec["frozen"] = new
    save(rec)
    print("amendment", am["utc"], am["cell_version"], len(new), "frozen files;", len(am["changed"]), "changed:", am["changed"])


if __name__ == "__main__":
    c = sys.argv[1]
    if c == "register":
        cmd_register()
    elif c == "draw":
        cmd_draw()
    elif c == "run":
        cmd_run(int(sys.argv[2]), sys.argv[3])
    elif c == "select":
        cmd_select()
    elif c == "amend":
        cmd_amend(" ".join(sys.argv[2:]))
    else:
        sys.exit(__doc__)
