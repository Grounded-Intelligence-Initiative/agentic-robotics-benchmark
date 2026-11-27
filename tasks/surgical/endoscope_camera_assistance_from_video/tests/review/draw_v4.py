"""Maintainers' tool: version 0.4's draws, by rules written down and committed before this was run
(endosim/procedure.py `draw_unit`; the datasheet's tolerances in endosim/model.py).

    python3 tests/review/draw_v4.py draw     -> every procedure of tests/hidden/procedures.json,
                                               environment/commission_service/dev_procedures.json and
                                               environment/commission_service/commission_procedures.json gets
                                               `factors.unit`: the unit of the holder series it runs on
    python3 tests/review/draw_v4.py check    -> recomputes every unit from its file's key and compares

The keys. Each of the three sets gets one 64-bit key from os.urandom, of its own; procedure i's unit is
draw_unit(Rng(key, "unit-<i>")). Everything else of a procedure (its seed, salt, rig, head changes) is as version 0.3
drew it. A key is drawn once; nothing about a drawn unit (how anything scores with it) decides whether it is kept,
and nothing is drawn again.
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
        "dev": os.path.join(TASK, "environment", "commission_service", "dev_procedures.json"),
        "commission": os.path.join(TASK, "environment", "commission_service", "commission_procedures.json")}
RULE = "tests/review/draw_v4.py (its docstring); endosim/procedure.py draw_unit; endosim/model.py ZERO_TOL, GAIN_RANGE, VIEW_TOL_DEG"


def units(key, n):
    return [procedure.draw_unit(procedure.Rng(key, "unit-%d" % i)) for i in range(n)]


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "check"
    docs = {name: json.load(open(path)) for name, path in SETS.items()}
    if what == "draw":
        if any("v4_draw" in d for d in docs.values()):
            sys.exit("version 0.4's units are already drawn: they are never drawn again")
        today = datetime.date.today().isoformat()
        for name, doc in docs.items():
            key = int.from_bytes(os.urandom(8), "big")
            for item, unit in zip(doc["procedures"], units(key, len(doc["procedures"]))):
                item["factors"]["unit"] = unit
            doc["v4_draw"] = {"drawn": today, "key": key, "rule": RULE}
            with open(SETS[name], "w") as f:
                json.dump(doc, f, indent=1)
                f.write("\n")
    bad = 0
    for name, doc in docs.items():
        key = doc["v4_draw"]["key"]
        want = units(key, len(doc["procedures"]))
        for i, (item, unit) in enumerate(zip(doc["procedures"], want)):
            ok = item["factors"].get("unit") == unit
            bad += not ok
            u = item["factors"].get("unit") or {}
            print("{:>10} {:>2} zero {} gain {} view {:+.2f} deg  {}".format(
                name, i, [round(z, 4) for z in u.get("zero", [])], u.get("gain"), u.get("view_dev_deg", 0.0), "ok" if ok else "MISMATCH"))
    if bad:
        sys.exit(1)


if __name__ == "__main__":
    main()
