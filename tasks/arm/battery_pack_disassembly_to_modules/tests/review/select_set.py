#!/usr/bin/env python3
"""Draw the fixed hidden set and the development packs (maintainers' tool; nothing runs it at grading).

    python3 select_set.py --write          # draws a new key and writes both sets (only when the sets are to change)
    python3 select_set.py --refresh        # the same key: rewrites the files and the record's summaries (after a change
                                           # of packcell/series.py that leaves the seeds as they are)
    python3 select_set.py --draw-cells     # rule 2, once: draws the cells' key and writes every pack's camera offsets
    python3 select_set.py                  # re-derives both sets from the recorded keys and checks the files

Rule 1, fixed before any pack was drawn or run:

- A key of 32 random bytes (os.urandom) is drawn once and recorded in tests/hidden/selection.json.
- The hidden set is six packs. Their variants and harness cases are fixed by this table, so that every variant
  appears twice and every harness case twice: (P4, lid), (P4, busbars), (P5, none), (P5, lid), (P6, busbars),
  (P6, none). Pack k's seed is the first 8 bytes of BLAKE2b(key, "hidden|k"). Everything else about a pack (its
  placement, head types, replaced, corroded and missing fasteners, the harness' path) follows from its seed
  (packcell/series.py `draw_pack`).
- The development packs of the commissioning service are four packs drawn the same way under "dev|k": (P4, busbars),
  (P5, lid), (P6, none), (P6, lid).
- No pack is drawn again, dropped or reordered for any property it turns out to have, and none for how the
  reference or any other program does on it.

Rule 2, the cells' camera offsets (2026-11-25; written and committed before any offset was drawn). Every pack is
taken apart in a cell of its own, and a cell's two cameras sit off their nominal mounting (packcell/series.py,
`CAMERAS`, `MOUNT_TOL`, `camera_mount`):

- A second key of 32 random bytes (os.urandom) is drawn once (`--draw-cells`) and recorded in
  tests/hidden/selection.json as `cells_key_hex`. It is independent of the first key and of every seed.
- Pack k of the hidden set is taken apart in the cell "hidden|k", development pack k in the cell "dev|k". That
  cell's camera c (overhead, wrist) has the offset component j (0 to 5: dx, dy, dz in mm, then rx, ry, rz in
  degrees, a rotation vector; in the camera's own frame) (2 u - 1) b, rounded to 3 decimals, where u is the first
  8 bytes of BLAKE2b("cell|<set>|<k>|<c>|<j>", key = the second key) read little-endian over 2^64, and b is
  MOUNT_TOL[c]["mm"] for j < 3 and MOUNT_TOL[c]["deg"] otherwise (overhead 15 mm and 1.5 degrees, wrist 4 mm and
  1.0 degree).
- The offsets go into each pack's entry (`cameras`) of tests/hidden/packs.json and
  environment/commission_service/dev_packs.json. None is drawn again or changed for any value it turns out to have
  or for how any program does with it.
"""

import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from packcell import series as S  # noqa: E402

HIDDEN = [("P4", "lid"), ("P4", "busbars"), ("P5", "none"), ("P5", "lid"), ("P6", "busbars"), ("P6", "none")]
DEV = [("P4", "busbars"), ("P5", "lid"), ("P6", "none"), ("P6", "lid")]
SEL = os.path.join(TASK, "tests", "hidden", "selection.json")
PACKS = os.path.join(TASK, "tests", "hidden", "packs.json")
DEVFILE = os.path.join(TASK, "environment", "commission_service", "dev_packs.json")
SERVICE = {"sessions": 10, "idle_s": 900, "start_timeout_s": 900, "cell": {"sim_cap_s": 2700.0, "frame_budget": 400}}


def draw(key, tag, table):
    out = []
    for k, (variant, harness) in enumerate(table):
        seed = int.from_bytes(hashlib.blake2b(b"%s|%d" % (tag, k), key=key, digest_size=8).digest(), "little")
        out.append({"seed": seed, "variant": variant, "harness": harness})
    return out


def cells(key2, tag, n):
    """Rule 2: the camera offsets of the cells "<tag>|0" .. "<tag>|n-1"."""
    out = []
    for k in range(n):
        cams = {}
        for c in sorted(S.CAMERAS):
            bounds = [S.MOUNT_TOL[c]["mm"]] * 3 + [S.MOUNT_TOL[c]["deg"]] * 3
            vals = []
            for j, b in enumerate(bounds):
                h = hashlib.blake2b(b"cell|%s|%d|%s|%d" % (tag, k, c.encode("ascii"), j), key=key2, digest_size=8).digest()
                u = int.from_bytes(h, "little") / float(2 ** 64)
                vals.append(round((2.0 * u - 1.0) * b, 3))
            cams[c] = vals
        out.append(cams)
    return out


def with_cells(sel, tag, packs):
    if "cells_key_hex" not in sel:
        return packs
    offs = cells(bytes.fromhex(sel["cells_key_hex"]), tag, len(packs))
    return [dict(p, cameras=o) for p, o in zip(packs, offs)]


def main():
    if "--draw-cells" in sys.argv[1:]:
        with open(SEL) as f:
            sel = json.load(f)
        if "cells_key_hex" in sel:
            raise SystemExit("the cells' key is drawn already (rule 2: once)")
        sel["cells_key_hex"] = os.urandom(32).hex()
        sel["cells_rule"] = "tests/review/select_set.py (its docstring), rule 2"
        sel["cells_drawn_utc"] = __import__("datetime").datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
        sys.argv.append("--refresh")
    if "--write" in sys.argv[1:]:
        key = os.urandom(32)
        with open(SEL, "w") as f:
            json.dump({"key_hex": key.hex(), "rule": "tests/review/select_set.py (its docstring)", "series_version": S.VERSION,
                       "summaries": []}, f, indent=1)
    with open(SEL) as f:
        sel = json.load(f)
    key = bytes.fromhex(sel["key_hex"])
    hidden, dev = with_cells(sel, b"hidden", draw(key, b"hidden", HIDDEN)), with_cells(sel, b"dev", draw(key, b"dev", DEV))
    if "--write" in sys.argv[1:] or "--refresh" in sys.argv[1:]:
        sel["series_version"] = S.VERSION
        with open(PACKS, "w") as f:
            json.dump({"packs": hidden, "note": "the fixed hidden set: tests/review/select_set.py"}, f, indent=1)
            f.write("\n")
        with open(DEVFILE, "w") as f:
            json.dump(dict(SERVICE, packs=dev), f, indent=1)
            f.write("\n")
        sel["summaries"] = [S.summary(S.draw_pack(**p)) for p in hidden]
        sel["dev_summaries"] = [S.summary(S.draw_pack(**p)) for p in dev]
        with open(SEL, "w") as f:
            json.dump(sel, f, indent=1)
            f.write("\n")
    with open(PACKS) as f:
        ok = json.load(f)["packs"] == hidden
    with open(DEVFILE) as f:
        ok = ok and json.load(f)["packs"] == dev
    for name, packs in (("hidden", hidden), ("dev", dev)):
        for k, p in enumerate(packs):
            s = S.summary(S.draw_pack(**p))
            print(name, k, s["variant"], "harness", s["harness"], s["harness_covers"], "missing", s["missing"], "corroded", s["corroded"],
                  s["heads"], "value", s["value"], "cameras", "drawn" if "cameras" in p else "NOT DRAWN")
    print("files match the key:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
