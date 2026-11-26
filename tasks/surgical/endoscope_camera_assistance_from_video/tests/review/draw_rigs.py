"""Maintainers' tool: draw each fixed procedure's rig (the camera head's angle on the eyepiece, the video chain's
latency, the instruments' shaft sizes) by the rule of endosim/procedure.py `draw_rig`, written down and committed
before this was run.

    python3 tests/review/draw_rigs.py draw     -> adds factors.rig to every procedure of tests/hidden/procedures.json and
                                                 of environment/commission_service/dev_procedures.json, and the draw's
                                                 record (`rig_draw`) to each file
    python3 tests/review/draw_rigs.py check    -> recomputes every rig from its file's key and compares

The key. Each set gets one 64-bit key from os.urandom, of its own (not the procedures' seeds, not the other set's);
procedure i's rig is draw_rig(Rng(key, "rig-<i>")). The key is drawn once; nothing about a drawn rig (how anything
scores with it) decides whether it is kept, and it is never drawn again.
"""

import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from endosim import procedure  # noqa: E402

SETS = {"hidden": os.path.join(TASK, "tests", "hidden", "procedures.json"),
        "dev": os.path.join(TASK, "environment", "commission_service", "dev_procedures.json")}


def rigs(key, n):
    return [procedure.draw_rig(procedure.Rng(key, "rig-%d" % i)) for i in range(n)]


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "check"
    for name, path in SETS.items():
        with open(path) as f:
            doc = json.load(f)
        items = doc["procedures"]
        if what == "draw":
            if "rig_draw" in doc:
                sys.exit("{} already has its rigs (drawn {}): they are never drawn again".format(name, doc["rig_draw"]["drawn"]))
            key = int.from_bytes(os.urandom(8), "big")
            for item, rig in zip(items, rigs(key, len(items))):
                item.setdefault("factors", {})["rig"] = rig
            doc["rig_draw"] = {"drawn": datetime.date.today().isoformat(), "key": key, "rule": "endosim/procedure.py draw_rig; "
                               "procedure i: draw_rig(Rng(key, 'rig-<i>')); tests/review/draw_rigs.py"}
            with open(path, "w") as f:
                json.dump(doc, f, indent=1)
                f.write("\n")
        key = doc["rig_draw"]["key"]
        want = rigs(key, len(items))
        bad = [i for i, (item, rig) in enumerate(zip(items, want)) if item["factors"].get("rig") != rig]
        print("{}: {} rigs, key {}, {}".format(name, len(items), key, "ok" if not bad else "MISMATCH at {}".format(bad)))
        for i, item in enumerate(items):
            r = item["factors"]["rig"]
            print("  {:>2} head {:+7.2f} deg  latency {}  shafts {}".format(i, r["head_deg"], r["latency_steps"], r["shaft_mm"]))
        if bad:
            sys.exit(1)


if __name__ == "__main__":
    main()
