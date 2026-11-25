"""dual_copies.py -- the dual mode's byte copies of freeflyer_uncrewed_station_caretaking, checked (maintainers' tool;
nothing runs it at verify).

    python3 tests/review/dual_copies.py            # check; exit 1 on a difference
    python3 tests/review/dual_copies.py --sync     # rewrite the copies from their sources, then check

The graded episode's service (environment/setup/online/) serves the offline grader's own cell, so its data folder
holds byte copies of the grader's files, in the layout of tests/:
  online/data/cell/cabincell/                       = tests/cabincell/
  online/data/cell/grader_config.json               = tests/grader_config.json
  online/data/cell/reference/                       = tests/reference/
  online/data/cell/setup_payload/given/station_map/ = the three JSON files of tests/setup_payload/given/station_map/
  online/data/sites/<package>                       = the drawn site's package of tests/hidden/sites/
  online/data/oracle_token.sha256                   = tests/hidden/oracle_token.sha256
and the graded episode is named consistently in online/data/episode.json, tests/online.json and
tests/hidden/selection.json. tests/setup_payload/ stays a byte copy of environment/setup/payload/ (the graded
episode's client cabinkit/graded.py included) and tests/given_hashes.json pins it; tests/offline_verify.py is main's
tests/verify.py with the two hooks its header names; tests/anchor_offline.json is main's tests/anchor.json.
"""
import argparse
import filecmp
import hashlib
import json
import os
import shutil
import sys

TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TASK = os.path.dirname(TESTS)
ONLINE = os.path.join(TASK, "environment", "setup", "online")
DATA = os.path.join(ONLINE, "data")


def _sha(p):
    with open(p, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _tree_diff(a, b):
    """Relative paths that differ between two trees (ignoring __pycache__)."""
    out = []
    cmp = filecmp.dircmp(a, b, ignore=["__pycache__"])

    def walk(c, rel):
        for n in c.left_only + c.right_only + c.funny_files:
            out.append(os.path.join(rel, n))
        for n in c.common_files:
            if _sha(os.path.join(c.left, n)) != _sha(os.path.join(c.right, n)):
                out.append(os.path.join(rel, n))
        for n, sub in c.subdirs.items():
            walk(sub, os.path.join(rel, n))
    walk(cmp, "")
    return out


def pairs(episode):
    sel = json.load(open(os.path.join(TESTS, "hidden", "selection.json")))
    hit = [e for e in sel["set"] if e.get("tag") == episode["episode"]]
    if len(hit) != 1:
        raise SystemExit("the graded episode %r is not one entry of the hidden set" % episode["episode"])
    pkg = hit[0]["package"]
    files = [(os.path.join(TESTS, "grader_config.json"), os.path.join(DATA, "cell", "grader_config.json")),
             (os.path.join(TESTS, "hidden", pkg), os.path.join(DATA, episode["package"])),
             (os.path.join(TESTS, "hidden", "oracle_token.sha256"), os.path.join(DATA, "oracle_token.sha256"))]
    for f in ("zones.json", "fixtures.json", "landmarks.json"):
        files.append((os.path.join(TESTS, "setup_payload", "given", "station_map", f),
                      os.path.join(DATA, "cell", "setup_payload", "given", "station_map", f)))
    trees = [(os.path.join(TESTS, "cabincell"), os.path.join(DATA, "cell", "cabincell")),
             (os.path.join(TESTS, "reference"), os.path.join(DATA, "cell", "reference")),
             (os.path.join(TASK, "environment", "setup", "payload"), os.path.join(TESTS, "setup_payload"))]
    return hit[0], files, trees


def check():
    problems = []
    episode = json.load(open(os.path.join(DATA, "episode.json")))
    entry, files, trees = pairs(episode)
    for src, dst in files:
        if not os.path.isfile(dst) or _sha(src) != _sha(dst):
            problems.append("differs: %s" % os.path.relpath(dst, TASK))
    for src, dst in trees:
        for rel in _tree_diff(src, dst):
            problems.append("differs: %s" % os.path.relpath(os.path.join(dst, rel), TASK))
    if episode["sha256"] != entry["sha256"] or episode.get("seed") != entry.get("seed") or \
            abs(float(episode["ref_energy_wh"]) - float(entry["ref_energy_wh"])) > 0.0:
        problems.append("online/data/episode.json does not match its entry of hidden/selection.json")
    online = json.load(open(os.path.join(TESTS, "online.json")))
    if online["episode"] != episode["episode"] or online["seed_sha256"] != episode["seed_sha256"]:
        problems.append("tests/online.json does not name the episode of online/data/episode.json")
    seed = entry["seed"] if entry.get("seed") is not None else int(entry["sha256"][:16], 16) >> 2
    if hashlib.sha256(str(int(seed)).encode()).hexdigest() != online["seed_sha256"]:
        problems.append("tests/online.json's seed_sha256 is not the graded site's noise seed's")
    service = json.load(open(os.path.join(ONLINE, "service.json")))
    if service["cell_limits"] != online["cell_limits"]:
        problems.append("service.json and tests/online.json give different cell_limits")
    hashes = json.load(open(os.path.join(TESTS, "given_hashes.json")))
    for root, _dirs, fns in os.walk(os.path.join(TESTS, "setup_payload")):
        for fn in fns:
            p = os.path.join(root, fn)
            rel = os.path.relpath(p, os.path.join(TESTS, "setup_payload"))
            if hashes.get(rel) != _sha(p):
                problems.append("given_hashes.json does not pin %s" % rel)
    return problems


def sync():
    episode = json.load(open(os.path.join(DATA, "episode.json")))
    _entry, files, trees = pairs(episode)
    for src, dst in files:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src, dst)
    for src, dst in trees[:2]:                        # the payload mirror is synced by its own generator
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sync", action="store_true")
    a = ap.parse_args()
    if a.sync:
        sync()
    problems = check()
    for p in problems:
        print(p)
    print("dual copies: %s" % ("OK" if not problems else "%d problem(s)" % len(problems)))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
