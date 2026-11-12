"""The fixed hidden set's selection (a maintainer's tool; the grader never runs it).

The rule was pre-registered in tests/hidden/selection.json ("rule", "candidates", "frozen") and committed before any
candidate ran: the frozen reference solution (solution/) runs each candidate kit ONCE, in drawn order, through the
grader itself (tests/review/regrade.sh: a fresh container of the task image in the separate verifier's layout, the
reference's submission from solution/solve.sh, a one-episode plan); a candidate is kept when the reference completes
its kit (kit_complete 1, no abort, no fault, no budget cut); the hidden set (tests/ref/episodes.json) is the first
SET_SIZE kept candidates in drawn order. Should fewer be kept, more candidates are drawn the same way, appended and
run. Every drop is recorded with its reason; the hollowing check compares the kept kits' features with all
candidates'.

    # once, before anything runs (host, standard library): the candidates and the frozen files' sha256
    python3 -B tests/review/select_set.py draw --n 16 --frozen-commit <sha>
    # inside the task image (it draws each kit with keyed_env.sample_kit): one tests/ copy per candidate
    docker run --rm --network none -v <task>/tests:/stage:ro -v <work>:/work --entrypoint /opt/venv/bin/python3 \\
        <image> -I -B /stage/review/select_set.py prepare --out /work
    # host: each candidate once through the grader (regrade.sh), --jobs at a time
    python3 -B tests/review/select_set.py run --work <work> --image <image> --submission <reference submission> --jobs 6
    # host (standard library): the rule over the runs, the hollowing check; --write records the set
    python3 -B tests/review/select_set.py make --work <work> [--write]
"""

import argparse
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
SELECTION = os.path.join(TESTS, "hidden", "selection.json")
EPISODES = os.path.join(TESTS, "ref", "episodes.json")
SET_SIZE = 10
FROZEN = ("solution/solve.sh", "solution/solver/oracle_ctrl.py", "solution/solver/oracle_main.py",
          "tests/keyed_env.py", "tests/keyed_credit.py", "tests/verify.py", "tests/grader_config.json",
          "tests/nonet_exec.py", "tests/test.sh", "tests/oracle_token.sha256",
          "environment/Dockerfile", "environment/requirements.txt")
EPISODE_LINE = re.compile(r"\[grader\] episode 0 \((?P<id>[^)]+)\): kit_complete=(?P<kit>\d) nuts_assembled=(?P<nuts>\d) "
                          r"progress=(?P<prog>[0-9.]+) steps=(?P<steps>\d+) controller_s=(?P<cs>[0-9.]+) (?P<reason>.*)$")


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def frozen_hashes():
    out = {rel: sha256_file(os.path.join(TASK, rel)) for rel in FROZEN}
    for p in sorted(glob.glob(os.path.join(TESTS, "robotics_grader", "*.py"))):
        out["tests/robotics_grader/" + os.path.basename(p)] = sha256_file(p)
    return out


def load_selection():
    with open(SELECTION) as f:
        return json.load(f)


def save_selection(sel):
    with open(SELECTION, "w") as f:
        json.dump(sel, f, indent=1)
        f.write("\n")


# ---------------------------------------------------------------------------------------------------------- draw
def cmd_draw(a):
    sel = load_selection() if os.path.exists(SELECTION) else {}
    if sel.get("candidates") and not a.append:
        sys.exit("candidates already drawn; use --append to add a batch under the rule")
    cands = list(sel.get("candidates", []))
    for _ in range(a.n):
        kit_seed = int.from_bytes(os.urandom(8), "little") >> 1          # 63-bit
        layout_seed = int.from_bytes(os.urandom(8), "little") >> 1
        cands.append({"i": len(cands), "id": "C%02d" % len(cands), "kit_seed": kit_seed, "layout_seed": layout_seed})
    sel["candidates"] = cands
    if not a.append:
        sel["frozen_commit"] = a.frozen_commit
        sel["frozen"] = frozen_hashes()
    save_selection(sel)
    print("%d candidates in %s" % (len(cands), SELECTION))


# ---------------------------------------------------------------------------------------------------------- prepare
def kit_features(kit, layout):
    """The hollowing check's features of one candidate: per kit, from its three nuts and its layout."""
    import math
    nuts = kit["nuts"]
    turns = [abs(int(k)) for sp in nuts for k in sp["groove"]["turns"]]
    same = sum(1 for sp in nuts if (sp["groove"]["turns"][0] > 0) == (sp["groove"]["turns"][1] > 0))
    net = [abs(int(sp["groove"]["turns"][0]) + int(sp["groove"]["turns"][1])) for sp in nuts]
    base = (-0.56, 0.0)
    reach = max(math.hypot(x - base[0], y - base[1]) for (x, y, _yaw) in layout["pegs"] + layout["nuts"])
    gaps = [math.hypot(p[0] - q[0], p[1] - q[1]) for i, p in enumerate(layout["pegs"]) for q in layout["pegs"][i + 1:]]
    return {"turn_sectors_mean": round(sum(turns) / len(turns), 3),
            "turns_of_4": sum(1 for t in turns if t == 4),
            "nuts_turning_one_way_twice": same,
            "net_turn_sectors_max": max(net),
            "dead_end_sectors": sum(int(d) for sp in nuts for d in sp["groove"]["dead_ends"]),
            "false_entrances": sum(len(sp["groove"]["false_entrances"]) for sp in nuts),
            "mass_mean": round(sum(float(sp["mass"]) for sp in nuts) / len(nuts), 4),
            "friction_mean": round(sum(float(sp["friction"]) for sp in nuts) / len(nuts), 4),
            "max_reach_m": round(reach, 4),
            "min_peg_gap_m": round(min(gaps), 4)}


def cmd_prepare(a):
    import numpy as np
    sys.path.insert(0, TESTS)
    import keyed_env as E
    sel = load_selection()
    idx = a.index if a.index else [c["i"] for c in sel["candidates"]]
    os.makedirs(a.out, exist_ok=True)
    for i in idx:
        c = sel["candidates"][i]
        kit = E.sample_kit(np.random.default_rng(int(c["kit_seed"])), c["id"])
        seed = int(c["layout_seed"]) & (2 ** 64 - 1)
        layout = E.sample_layout(np.random.default_rng([seed & 0xFFFFFFFF, seed >> 32, 7]), E.NUTS)
        dst = os.path.join(a.out, "tests_%02d" % i)
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(TESTS, dst, ignore=shutil.ignore_patterns("__pycache__", "hidden", "review"))
        ep = {"id": c["id"], "kit": kit, "seed": int(c["layout_seed"])}
        with open(os.path.join(dst, "ref", "episodes.json"), "w") as f:
            json.dump({"note": "selection candidate %s (one-episode plan)" % c["id"], "episodes": [ep]}, f, indent=1)
        with open(os.path.join(a.out, "cand_%02d.json" % i), "w") as f:
            json.dump({"i": i, "episode": ep, "features": kit_features(kit, layout)}, f, indent=1)
        print("prepared candidate %d (%s)" % (i, c["id"]), flush=True)


# ---------------------------------------------------------------------------------------------------------- run
def cmd_run(a):
    sel = load_selection()
    idx = a.index if a.index else [c["i"] for c in sel["candidates"]]
    regrade = os.path.join(HERE, "regrade.sh")

    def one(i):
        out = os.path.join(a.work, "run_%02d" % i)
        if os.path.exists(os.path.join(out, "regrade.log")) and not a.force:
            return i, "kept earlier run"
        r = subprocess.run(["bash", regrade, a.image, "%s-%02d" % (a.name, i), a.submission, out,
                            os.path.join(a.work, "tests_%02d" % i)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True)
        return i, "rc=%d" % r.returncode

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for i, what in pool.map(one, idx):
            print("candidate %2d: %s" % (i, what), flush=True)


# ---------------------------------------------------------------------------------------------------------- make
def read_run(work, i):
    rec = {"i": i}
    with open(os.path.join(work, "cand_%02d.json" % i)) as f:
        cand = json.load(f)
    rec["id"] = cand["episode"]["id"]
    rec["features"] = cand["features"]
    out = os.path.join(work, "run_%02d" % i)
    m = None
    try:
        with open(os.path.join(out, "verify.stderr")) as f:
            for line in f:
                m = EPISODE_LINE.search(line.strip()) or m
        with open(os.path.join(out, "verdict.json")) as f:
            verdict = json.load(f)
        with open(os.path.join(out, "regrade.log")) as f:
            wall = re.findall(r"wall_s=(\d+)", f.read())
    except OSError as exc:
        rec["error"] = "no complete run: %s" % exc
        return rec, cand
    if m is None:
        rec["error"] = "no episode line in the grader's log"
        return rec, cand
    rec.update({"kit_complete": int(m.group("kit")), "nuts_assembled": int(m.group("nuts")),
                "progress": float(m.group("prog")), "steps": int(m.group("steps")),
                "controller_s": float(m.group("cs")), "reason": m.group("reason"),
                "reference_token": verdict["metrics"].get("reference_token"),
                "aborted": verdict["metrics"].get("episodes_aborted"), "sim_faults": verdict["metrics"].get("sim_faults"),
                "budget_cuts": verdict["metrics"].get("controller_budget_cuts"),
                "wall_s": int(wall[-1]) if wall else None})
    return rec, cand


def kept(rec):
    """The pre-registered rule: the reference completes the candidate's kit."""
    why = []
    if "error" in rec:
        return False, [rec["error"]]
    if rec["reference_token"] != 1.0:
        why.append("the reference's token was not accepted")
    if rec["kit_complete"] != 1:
        why.append("kit not complete (%d of 3 nuts assembled, %s)" % (rec["nuts_assembled"], rec["reason"]))
    if rec["aborted"] or rec["sim_faults"] or rec["budget_cuts"]:
        why.append("abort, fault or budget cut")
    return not why, why


def cmd_make(a):
    sel = load_selection()
    n = len(sel["candidates"])
    rows, cands, kept_ids = [], {}, []
    for c in sel["candidates"]:
        rec, cand = read_run(a.work, c["i"])
        assert cand["episode"]["id"] == c["id"] and cand["episode"]["seed"] == c["layout_seed"], c["id"]
        ok, why = kept(rec)
        rec.update({"kept": ok, "drop_reasons": why})
        rows.append(rec)
        cands[c["id"]] = cand
        if ok:
            kept_ids.append(c["id"])
    hidden = kept_ids[:SET_SIZE]
    if len(hidden) < SET_SIZE:
        print("only %d kept: draw more candidates (draw --append) and run them" % len(hidden))
    cut = max((r["i"] for r in rows if r["id"] in hidden), default=-1)
    dropped = ["%s (%s)" % (r["id"], "; ".join(r["drop_reasons"])) for r in rows if not r["kept"] and r["i"] < cut]
    keys = sorted(rows[0]["features"])
    ks = [r for r in rows if r["id"] in hidden]
    hollow = {k: {"kept_mean": round(sum(r["features"][k] for r in ks) / max(1, len(ks)), 4),
                  "all_mean": round(sum(r["features"][k] for r in rows) / len(rows), 4)} for k in keys}
    summary = ("%d of %d candidates completed by the reference; the hidden set is %s (the first %d completed in drawn "
               "order); dropped before the last of them: %s" % (
                   len(kept_ids), n, ", ".join(hidden), SET_SIZE, ", ".join(dropped) or "none"))
    print(summary)
    for r in rows:
        print("  %s %s steps %s nuts %s %s" % (r["id"], "kept" if r["kept"] else "drop", r.get("steps"),
                                                r.get("nuts_assembled"), "; ".join(r["drop_reasons"])))
    print("hollowing check:", json.dumps(hollow))
    if a.write and len(hidden) == SET_SIZE:
        sel["selection_run"] = {"where": a.where, "tool_sha256": sha256_file(os.path.abspath(__file__)),
                                "regrade_sha256": sha256_file(os.path.join(HERE, "regrade.sh")),
                                "runs": rows, "summary": summary, "hollowing_check": hollow}
        sel["hidden_set"] = hidden
        save_selection(sel)
        eps = [cands[h]["episode"] for h in hidden]
        with open(EPISODES, "w") as f:
            json.dump({"note": "fixed hidden grading episodes: a kit (three nuts and the work order) drawn with "
                               "keyed_env.sample_kit from a seed, and the layout/noise seed; the first %d candidates "
                               "of tests/hidden/selection.json the reference completed, in drawn order" % SET_SIZE,
                       "episodes": eps}, f, indent=1)
            f.write("\n")
        print("wrote %s and %s" % (EPISODES, SELECTION))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("draw")
    d.add_argument("--n", type=int, default=16)
    d.add_argument("--frozen-commit", default=None)
    d.add_argument("--append", action="store_true")
    p = sub.add_parser("prepare")
    p.add_argument("--out", required=True)
    p.add_argument("--index", type=int, nargs="*")
    r = sub.add_parser("run")
    r.add_argument("--work", required=True)
    r.add_argument("--image", required=True)
    r.add_argument("--submission", required=True)
    r.add_argument("--jobs", type=int, default=4)
    r.add_argument("--name", default="keyed-sel")
    r.add_argument("--index", type=int, nargs="*")
    r.add_argument("--force", action="store_true")
    m = sub.add_parser("make")
    m.add_argument("--work", required=True)
    m.add_argument("--write", action="store_true")
    m.add_argument("--where", default="")
    a = ap.parse_args()
    {"draw": cmd_draw, "prepare": cmd_prepare, "run": cmd_run, "make": cmd_make}[a.cmd](a)


if __name__ == "__main__":
    main()
