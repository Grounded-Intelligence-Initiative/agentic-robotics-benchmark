#!/usr/bin/env python3
"""Draw the fixed hidden set and the practice rows (maintainers' tool; nothing runs it at grading).

    python3 select_set.py --write      # once: draws the four keys and writes both sets
    python3 select_set.py --refresh    # the same keys: rewrites the files and the record's summaries (after a change
                                       # of rowcell/farm.py that leaves the seeds as they are)
    python3 select_set.py              # re-derives both sets from the recorded keys and checks the files

The rules below were written and committed before any key was drawn. Each has a key of its own: 32 random bytes
(os.urandom), drawn once by `--write` and recorded in tests/hidden/selection.json, independent of each other and of
every seed. No row is drawn again, dropped or reordered for any property it turns out to have, and none for how the
reference or any other program does on it.

Rule 1, the rows (key `key_hex`). The hidden set is six row sections of the cultivar "Rossa"; row k's seed is the
first 8 bytes of BLAKE2b("hidden|k", key = the first key) read little-endian. The practice rows of the commissioning
service are four row sections of the cultivar "Ambra", drawn the same way under "dev|k". Everything else about a
row section (its plants, their trusses and leaves, which trusses are ready and every fruit's ripeness, each
peduncle's size and stiffness) follows from its seed and its cultivar (rowcell/farm.py `draw_row`, whose own
promises are part of this rule: four to six ready trusses, on each plant the lowest ones; every ready truss within
the stated straight, clear approach).

Rule 2, the robots' camera mountings (key `cameras_key_hex`). Every row section is worked by a robot of its own,
whose two cameras sit off their nominal mounting (rowcell/farm.py `CAMERAS`, `MOUNT_TOL`, `camera_mount`). The
robot of row k of the set <set> ("hidden" or "dev") has, for its camera c (overview, wrist), the offset component j
(0 to 5: dx, dy, dz in mm, then rx, ry, rz in degrees, a rotation vector; in the camera's own frame) (2 u - 1) b,
rounded to 3 decimals, where u is the first 8 bytes of BLAKE2b("camera|<set>|<k>|<c>|<j>", key = the second key)
read little-endian over 2^64, and b is MOUNT_TOL[c]["mm"] for j < 3 and MOUNT_TOL[c]["deg"] otherwise.

Rule 3, a row's light (key `light_key_hex`). Row k of the set <set> has the light seed = the first 8 bytes of
BLAKE2b("light|<set>|<k>", key = the third key) read little-endian; the grow light's brightness, its colour and the
lamps' differences follow from it (`draw_row`'s `light_seed`, the ranges of `farm.ROW`).

Rule 4, a row's mechanics (key `mech_key_hex`). Row k of the set <set> has the mechanics seed = the first 8 bytes of
BLAKE2b("mech|<set>|<k>", key = the fourth key) read little-endian; the row's toughness (what a cut takes), the
stems' and the support strings' stiffness, the damping and the cart's odometer slip follow from it (`draw_row`'s
`mech_seed`; the ranges of `farm.CUT`, `farm.CULTIVARS` and `farm.ROW`).

The practice rows differ from the graded ones on purpose and by rule: another cultivar (its own colour chart,
readiness rule, fruit and peduncle sizes, stiffness ranges), other robots (their own camera mountings), their own
light and mechanics. They show what a row section is like, not what a graded one's values are.
"""

import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from rowcell import farm as F  # noqa: E402
from rowcell import wire  # noqa: E402

SETS = {"hidden": {"n": 6, "cultivar": "Rossa"}, "dev": {"n": 4, "cultivar": "Ambra"}}
SEL = os.path.join(TASK, "tests", "hidden", "selection.json")
ROWS = os.path.join(TASK, "tests", "hidden", "rows.json")
DEVFILE = os.path.join(TASK, "environment", "commission_service", "dev_rows.json")
KEYS = ("key_hex", "cameras_key_hex", "light_key_hex", "mech_key_hex")


def service():
    return {"sessions": 5, "idle_s": 900, "start_timeout_s": 900, "busy_wait_s": 120,
            "cell": {"sim_cap_s": wire.DEFAULTS["sim_cap_s"], "frame_budget": wire.DEFAULTS["frame_budget"]}}


def _u64(key, text):
    return int.from_bytes(hashlib.blake2b(text.encode("ascii"), key=key, digest_size=8).digest(), "little")


def draw(sel, name):
    k1, k2, k3, k4 = (bytes.fromhex(sel[k]) for k in KEYS)
    out = []
    for k in range(SETS[name]["n"]):
        cams = {}
        for c in sorted(F.CAMERAS):
            bounds = [F.MOUNT_TOL[c]["mm"]] * 3 + [F.MOUNT_TOL[c]["deg"]] * 3
            cams[c] = [round((2.0 * _u64(k2, "camera|%s|%d|%s|%d" % (name, k, c, j)) / float(2 ** 64) - 1.0) * b, 3)
                       for j, b in enumerate(bounds)]
        out.append({"seed": _u64(k1, "%s|%d" % (name, k)), "cultivar": SETS[name]["cultivar"], "cameras": cams,
                    "light_seed": _u64(k3, "light|%s|%d" % (name, k)), "mech_seed": _u64(k4, "mech|%s|%d" % (name, k))})
    return out


def files(sel):
    hidden, dev = draw(sel, "hidden"), draw(sel, "dev")
    record = dict(sel, rule="tests/review/select_set.py (its docstring), rules 1 to 4",
                  hidden=[F.summary(F.draw_entry(e)) for e in hidden], dev=[F.summary(F.draw_entry(e)) for e in dev])
    return {ROWS: {"rows": hidden}, DEVFILE: dict(service(), rows=dev), SEL: record}


def main():
    args = sys.argv[1:]
    if "--write" in args:
        if os.path.exists(SEL):
            raise SystemExit("the keys are drawn already (once): use --refresh")
        sel = {k: os.urandom(32).hex() for k in KEYS}
        sel["drawn_utc"] = __import__("datetime").datetime.now(__import__("datetime").timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    else:
        with open(SEL) as f:
            old = json.load(f)
        sel = {k: old[k] for k in KEYS + ("drawn_utc",)}
    want = files(sel)
    if "--write" in args or "--refresh" in args:
        for path, data in want.items():
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                json.dump(data, f, indent=1)
                f.write("\n")
        print("wrote %d hidden rows, %d practice rows" % (len(want[ROWS]["rows"]), len(want[DEVFILE]["rows"])))
        return 0
    bad = []
    for path, data in want.items():
        with open(path) as f:
            if json.load(f) != json.loads(json.dumps(data)):
                bad.append(os.path.relpath(path, TASK))
    for p in bad:
        print("PROBLEM: %s is not what the recorded keys give" % p)
    print("sets ok: %d hidden rows, %d practice rows" % (len(want[ROWS]["rows"]), len(want[DEVFILE]["rows"])) if not bad else "%d problem(s)" % len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
