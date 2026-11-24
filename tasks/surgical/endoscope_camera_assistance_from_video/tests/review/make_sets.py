"""Maintainers' tool: draw the fixed hidden set and the development set, by a rule written down before any draw.

    python3 tests/review/make_sets.py hidden     -> tests/hidden/procedures.json, tests/hidden/selection.json
    python3 tests/review/make_sets.py dev        -> environment/commission_service/dev_procedures.json
    python3 tests/review/make_sets.py remeasure  -> the same seeds and order, the records measured again (after a change
                                                    of the simulator; nothing is drawn)

The rule. A set is a balanced design: a table of factor rows (procedure kind, scope, number of instrument exchanges,
assistant instrument, smoke), fixed below. Each row gets one 63-bit seed from os.urandom, in table order; the hidden
set's rows are then put in an order drawn from os.urandom as well. No draw is ever rejected or repeated: nothing about
a drawn procedure (how the reference or anything else scores on it) decides whether it is kept. Everything the factors
do not pin (the cavity, the sites and the path, the gestures and their timing, when an exchange happens, every look)
follows from the seed (endosim/procedure.py). The file records, for every procedure, the generator's facts and two
render-free measurements made after the draw: what a holder that never moves scores, and what the reference scores
(tests/review/fast_eval.py's loop). They are a record, not a filter.

Running this again draws a new set: the anchor and the status log then have to be redone.
"""

import datetime
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(TASK, "tests"))
sys.path.insert(0, os.path.join(TASK, "environment", "setup", "payload", "given"))
sys.path.insert(0, os.path.join(TASK, "solution", "reference"))

STEPS = 800
#           kind            scope exchanges assistant smoke
HIDDEN = [("dissection",    "0",  0, True,  True),
          ("dissection",    "30", 1, True,  True),
          ("dissection",    "0",  1, False, True),
          ("suturing",      "30", 0, True,  False),
          ("suturing",      "0",  1, False, False),
          ("suturing",      "30", 0, True,  False),
          ("exploration",   "0",  0, False, False),
          ("exploration",   "30", 0, True,  False),
          ("exploration",   "0",  1, True,  False),
          ("clip_and_cut",  "30", 2, True,  True),
          ("clip_and_cut",  "0",  1, False, False),
          ("clip_and_cut",  "30", 2, True,  True)]
#        kind            scope exchanges assistant smoke  the recorded assistant touches tissue
DEV = [("dissection",    "0",  1, True,  True,  False),
       ("dissection",    "30", 0, False, True,  True),
       ("suturing",      "0",  0, True,  False, False),
       ("suturing",      "30", 1, False, False, True),
       ("exploration",   "30", 0, True,  False, False),
       ("exploration",   "0",  1, False, False, False),
       ("clip_and_cut",  "0",  2, True,  True,  True),
       ("clip_and_cut",  "30", 1, False, False, False),
       ("dissection",    "30", 1, True,  False, False),
       ("suturing",      "0",  1, True,  True,  False)]
DEV_SESSIONS = 12
RECORD_EVERY = 2


def draw_seed():
    return int.from_bytes(os.urandom(8), "big") >> 1


def factors(row):
    return {"kind": row[0], "scope": row[1], "exchanges": row[2], "assistant": row[3], "smoke": row[4]}


def measure(items, pool="all"):
    import controller as ref
    import fast_eval
    from endosim import procedure
    out = []
    for item in items:
        proc = procedure.generate(item["seed"], item["factors"], steps=STEPS, pool=pool)
        idle = fast_eval.run(proc, fast_eval.Idle())["useful_view_fraction"]
        res = fast_eval.run(proc, ref.Controller(), probe_rate=0.06, rng=random.Random(1))
        out.append({"facts": proc.facts(), "never_moves": round(idle, 4), "reference_render_free": round(res["useful_view_fraction"], 4),
                    "reference_lens_contacts": res["lens_contacts"]})
    return out


def main():
    which = sys.argv[1]
    today = datetime.date.today().isoformat()
    if which == "hidden":
        rows = [{"seed": draw_seed(), "factors": factors(r)} for r in HIDDEN]
        order = list(range(len(rows)))
        random.Random(int.from_bytes(os.urandom(8), "big")).shuffle(order)
        items = [rows[i] for i in order]
        record = measure(items)
        doc = {"steps": STEPS, "generator": record[0]["facts"]["generator"], "procedures": items,
               "note": "the fixed hidden set: {} procedures drawn once by tests/review/make_sets.py on {} (rule and record: selection.json); "
                       "the same procedures for every submission".format(len(items), today)}
        with open(os.path.join(TASK, "tests", "hidden", "procedures.json"), "w") as f:
            json.dump(doc, f, indent=1)
        sel = {"drawn": today, "rule": __doc__.split("The rule.")[1].split("Running this again")[0].strip().replace("\n", " "),
               "design_rows_in_table_order": [factors(r) for r in HIDDEN], "order_drawn": order, "rejected_draws": 0,
               "record": [dict(index=i, seed=it["seed"], **rec) for i, (it, rec) in enumerate(zip(items, record))]}
        with open(os.path.join(TASK, "tests", "hidden", "selection.json"), "w") as f:
            json.dump(sel, f, indent=1)
    elif which == "dev":
        items = [{"name": "dev%02d" % (i + 1), "seed": draw_seed(), "factors": factors(r), "touch": r[5]} for i, r in enumerate(DEV)]
        record = measure(items, pool="dev")
        doc = {"steps": STEPS, "sessions": DEV_SESSIONS, "record_every": RECORD_EVERY, "procedures": items,
               "note": "the development procedures, drawn once by the task's make_sets tool on {} with the same rule as the graded set "
                       "(a balanced table of factors, one os.urandom seed per row, no draw rejected); none of them is graded".format(today)}
        with open(os.path.join(TASK, "environment", "commission_service", "dev_procedures.json"), "w") as f:
            json.dump(doc, f, indent=1)
    elif which == "remeasure":
        hp, sp = os.path.join(TASK, "tests", "hidden", "procedures.json"), os.path.join(TASK, "tests", "hidden", "selection.json")
        dp = os.path.join(TASK, "environment", "commission_service", "dev_procedures.json")
        doc, sel, dev = json.load(open(hp)), json.load(open(sp)), json.load(open(dp))
        items = doc["procedures"]
        record = measure(items)
        doc["generator"] = record[0]["facts"]["generator"]
        sel["record"] = [dict(index=i, seed=it["seed"], **rec) for i, (it, rec) in enumerate(zip(items, record))]
        sel["record_measured"] = today
        json.dump(doc, open(hp, "w"), indent=1)
        json.dump(sel, open(sp, "w"), indent=1)
        dev_record = measure(dev["procedures"], pool="dev")
        print("development procedures (not written anywhere): reference {:.4f}, never-moves {:.4f}".format(
            sum(r["reference_render_free"] for r in dev_record) / len(dev_record), sum(r["never_moves"] for r in dev_record) / len(dev_record)))
    else:
        raise SystemExit(__doc__)
    for i, (it, rec) in enumerate(zip(items, record)):
        f = rec["facts"]
        print("{:2d} {:<13} scope {:>2} x{} a{} s{} | never moves {:.4f} | reference (render-free) {:.4f} | path {:.2f} m fine {:.2f}".format(
            i, f["kind"], f["scope"], f["exchanges"], int(f["assistant"]), int(f["smoke"]), rec["never_moves"], rec["reference_render_free"],
            f["path_m"], f["fine_fraction"]))
    print("mean reference {:.4f}, mean never-moves {:.4f}".format(sum(r["reference_render_free"] for r in record) / len(record),
                                                                    sum(r["never_moves"] for r in record) / len(record)))


if __name__ == "__main__":
    main()
