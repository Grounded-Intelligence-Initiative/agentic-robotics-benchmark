"""Pre-registered selection of the hidden sites (tests/review/selection.md), run inside the task image.

    draw:   python3 select_sites.py draw  --out tests/hidden/candidates.json     (os.urandom, before any run)
    run:    python3 select_sites.py run   --candidates tests/hidden/candidates.json --program <oracle dir>
                                          --out tests/hidden/selection.json --sites tests/hidden/sites.json

`run` must execute as root in a container laid out like the separate verifier (the tests stage at
/opt/ale/tests): it runs every candidate ONCE, in drawn order, through the grader's own `run_mission`
(the graded robot service, the program as the agent account under the no-network launcher), keeps the
first passes per cell, records every outcome and writes the kept seeds.
"""
import argparse
import json
import os
import pwd
import shutil
import sys
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
sys.path.insert(0, str(STAGE/"runtime"))

PER_CELL = 4
CANDIDATES = 24


def cell_of(site):
    """Cell A: the wide opening's centre at y >= 0 (above the hall's midline, the carrier's side); B: below."""
    return "A" if (site.wide[0]+site.wide[1])/2 >= 0 else "B"


def features(seed):
    import world as W
    site = W.Site(seed)
    return {"cell": cell_of(site), "wall_x": round(site.wall_x, 3),
            "wide_centre_y": round((site.wide[0]+site.wide[1])/2, 3),
            "wide_width": round(site.wide[1]-site.wide[0], 3), "crates": len(site.crates),
            "carrier_side": round(2*(site.carrier_half+W.LEG), 3), "carrier_mass": round(site.carrier_mass+.08, 2)}


def draw(args):
    seeds = [int.from_bytes(os.urandom(8), "little") for _ in range(CANDIDATES)]
    Path(args.out).write_text(json.dumps({"source": "os.urandom(8), little-endian, in drawn order",
                                          "seeds": seeds}, indent=1)+"\n")


def run(args):
    import grader as G
    candidates = json.loads(Path(args.candidates).read_text())["seeds"]
    account = pwd.getpwnam(G.agent_user())
    uid, gid = account.pw_uid, account.pw_gid
    G.secure_exec_path()
    G.WORK.mkdir(parents=True, exist_ok=True)
    os.chmod(G.WORK, 0o700)
    G.RUN.mkdir(parents=True, exist_ok=True)
    os.chmod(G.RUN, 0o755)
    problem = G.copy_submission(args.program, G.RUN/"program")
    assert problem is None, problem
    G.stage_public_client()
    token_sha256 = (G.HIDDEN/"oracle_token.sha256").read_text().split()[0]
    config = json.loads((STAGE/"grader_config.json").read_text())
    rows, kept = [], {"A": [], "B": []}
    for order, seed in enumerate(candidates):
        info = features(seed)
        if len(kept[info["cell"]]) >= PER_CELL:
            rows.append({"order": order, "seed": seed, **info, "run": False, "note": "cell already full"})
            continue
        record = G.run_mission(order, int(seed), uid, gid, config, token_sha256, attempt=0)
        row = {"order": order, "seed": seed, **info, "run": True, "success": record.get("success"),
               "carrier_actions": record.get("carrier_actions"), "reason": record.get("reason"),
               "infra": record.get("infra")}
        if record.get("success") == 1.0 and not record.get("infra"):
            kept[info["cell"]].append(seed)
            row["kept"] = True
        else:
            row["kept"] = False
        rows.append(row)
        print(json.dumps(row), flush=True)
        if all(len(v) >= PER_CELL for v in kept.values()):
            break
    sites = [s for c in ("A", "B") for s in kept[c]]
    Path(args.out).write_text(json.dumps({
        "rule": "tests/review/selection.md", "per_cell": PER_CELL,
        "cells": {"A": "wide opening centre y >= 0", "B": "wide opening centre y < 0"},
        "candidates": rows, "kept": sites}, indent=1)+"\n")
    Path(args.sites).write_text(json.dumps({"seeds": sites}, indent=1)+"\n")
    shutil.rmtree(G.RUN/"program", ignore_errors=True)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("draw")
    d.add_argument("--out", required=True)
    r = sub.add_parser("run")
    r.add_argument("--candidates", required=True)
    r.add_argument("--program", required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--sites", required=True)
    args = parser.parse_args()
    draw(args) if args.cmd == "draw" else run(args)


if __name__ == "__main__":
    main()
