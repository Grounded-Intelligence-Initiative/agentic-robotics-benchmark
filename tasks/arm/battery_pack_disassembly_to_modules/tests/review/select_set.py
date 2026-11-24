#!/usr/bin/env python3
"""Draw the fixed hidden set and the development packs (maintainers' tool; nothing runs it at grading).

    python3 select_set.py --write          # draws a new key and writes both sets (only when the sets are to change)
    python3 select_set.py                  # re-derives both sets from the recorded key and checks the files

The rule, fixed before any pack was drawn or run:

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


def main():
    if "--write" in sys.argv[1:]:
        key = os.urandom(32)
        with open(SEL, "w") as f:
            json.dump({"key_hex": key.hex(), "rule": "tests/review/select_set.py (its docstring)", "series_version": S.VERSION,
                       "summaries": []}, f, indent=1)
    with open(SEL) as f:
        sel = json.load(f)
    key = bytes.fromhex(sel["key_hex"])
    hidden, dev = draw(key, b"hidden", HIDDEN), draw(key, b"dev", DEV)
    if "--write" in sys.argv[1:]:
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
                  s["heads"], "value", s["value"])
    print("files match the key:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
