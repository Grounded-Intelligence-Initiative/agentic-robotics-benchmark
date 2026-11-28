"""Maintainers' tool: version 0.3's draws, by rules written down and committed before this was run
(endosim/procedure.py `draw_head_changes`, `draw_rig`; the salt below).

    python3 tests/review/draw_v3.py draw     -> for tests/hidden/procedures.json and
                                               environment/commission_service/dev_procedures.json: every procedure's
                                               `factors.salt` and `factors.rig.head_changes`; and the commissioning set,
                                               environment/commission_service/commission_procedures.json, made whole
    python3 tests/review/draw_v3.py check    -> recomputes every draw from its file's key and compares

The keys. Each of the three sets gets one 64-bit key from os.urandom, of its own. Procedure i of a set gets
    salt          int(2**53 * Rng(key, "salt-<i>").random())    mixed into every stream of the procedure's generator, so
                                                                that everything version 0.3 adds or changes (the
                                                                breathing, the traction, the stations in the tissue, the
                                                                assistant, the critical actions) is drawn after its rule
    head_changes  draw_head_changes(Rng(key, "head-<i>"), the rig's first camera head angle)
The hidden and development sets keep their seeds, their table of factors and version 0.2's rig draws (the first
camera head angle, the latency, the shafts). The commissioning set (version 0.3: what the commissioning sessions play,
not recorded, nothing about it on file for the agent) takes the development set's table of factors in its order, names
c01 ... c10, and per procedure i: seed int(2**63 * Rng(key, "seed-<i>").random()), rig draw_rig(Rng(key, "rig-<i>")).
A key is drawn once; nothing about a drawn procedure (how anything scores on it) decides whether it is kept, and
nothing is drawn again.
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
COMMISSION = os.path.join(TASK, "environment", "commission_service", "commission_procedures.json")
RULE = "tests/review/draw_v3.py (its docstring); endosim/procedure.py draw_head_changes, draw_rig"


def salt(key, i):
    return int(2 ** 53 * procedure.Rng(key, "salt-%d" % i).random())


def v3(key, i, head_deg):
    return salt(key, i), procedure.draw_head_changes(procedure.Rng(key, "head-%d" % i), head_deg)


def commission_items(key, dev_items):
    out = []
    for i, d in enumerate(dev_items):
        f = {k: d["factors"][k] for k in ("kind", "scope", "exchanges", "assistant", "smoke")}
        rig = procedure.draw_rig(procedure.Rng(key, "rig-%d" % i))
        s, hc = v3(key, i, rig["head_deg"])
        rig["head_changes"] = hc
        f.update(rig=rig, salt=s)
        out.append({"name": "c%02d" % (i + 1), "seed": int(2 ** 63 * procedure.Rng(key, "seed-%d" % i).random()), "factors": f})
    return out


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "check"
    docs = {name: json.load(open(path)) for name, path in SETS.items()}
    if what == "draw":
        if any("v3_draw" in d for d in docs.values()) or os.path.exists(COMMISSION):
            sys.exit("version 0.3's values are already drawn: they are never drawn again")
        today = datetime.date.today().isoformat()
        for name, doc in docs.items():
            key = int.from_bytes(os.urandom(8), "big")
            for i, item in enumerate(doc["procedures"]):
                s, hc = v3(key, i, item["factors"]["rig"]["head_deg"])
                item["factors"]["salt"] = s
                item["factors"]["rig"]["head_changes"] = hc
            doc["v3_draw"] = {"drawn": today, "key": key, "rule": RULE}
            with open(SETS[name], "w") as f:
                json.dump(doc, f, indent=1)
                f.write("\n")
        key = int.from_bytes(os.urandom(8), "big")
        cdoc = {"steps": docs["dev"]["steps"], "sessions": 12,
                "note": "What the commissioning sessions play (version 0.3): drawn like the graded procedures (their look pool), "
                        "rigs of their own; not recorded; the agent's documents name them and nothing else.",
                "procedures": commission_items(key, docs["dev"]["procedures"]), "v3_draw": {"drawn": today, "key": key, "rule": RULE}}
        with open(COMMISSION, "w") as f:
            json.dump(cdoc, f, indent=1)
            f.write("\n")
        docs = {name: json.load(open(path)) for name, path in SETS.items()}
    bad = 0
    for name, doc in docs.items():
        key = doc["v3_draw"]["key"]
        for i, item in enumerate(doc["procedures"]):
            want = v3(key, i, item["factors"]["rig"]["head_deg"])
            got = (item["factors"]["salt"], item["factors"]["rig"]["head_changes"])
            ok = list(want) == list(got)
            bad += not ok
            print("{:>6} {:>2} salt {:>16} head {:+7.2f} -> {}  {}".format(name, i, got[0], item["factors"]["rig"]["head_deg"], got[1],
                                                                       "ok" if ok else "MISMATCH"))
    cdoc = json.load(open(COMMISSION))
    want = commission_items(cdoc["v3_draw"]["key"], docs["dev"]["procedures"])
    strip = lambda item: dict(item, factors={k: v for k, v in item["factors"].items() if k != "unit"})   # noqa: E731
    ok = want == [strip(item) for item in cdoc["procedures"]]          # (version 0.4's units: draw_v4.py checks them)
    bad += not ok
    for item in cdoc["procedures"]:
        r = item["factors"]["rig"]
        print("commission {} {:<12} scope {:>2} head {:+7.2f} -> {} latency {} shafts {}".format(
            item["name"], item["factors"]["kind"], item["factors"]["scope"], r["head_deg"], r["head_changes"], r["latency_steps"], r["shaft_mm"]))
    print("commission set:", "ok" if ok else "MISMATCH")
    if bad:
        sys.exit(1)


if __name__ == "__main__":
    main()
