"""Hollowing check of the hidden-site selection (tests/review/selection.md, step 5), inside the task image.

Compares the kept sites' features with all 24 candidates' and with a development sample's (seeds 7000-7199, the
sample of map_ablation.json, whose `full` rows give the oracle's carrier presses there).

    /opt/venv/bin/python3 -B tests/review/hollowing.py > /tmp/hollowing.md
"""
import json
import sys
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE/"runtime"))
sys.path.insert(0, str(STAGE/"review"))
import numpy as np  # noqa: E402
from select_sites import features  # noqa: E402

FIELDS = ("wall_x", "wide_centre_y", "wide_width", "crates", "carrier_side", "carrier_mass")


def summary(rows, presses=None):
    out = []
    for f in FIELDS:
        v = np.array([r[f] for r in rows], float)
        out.append("{:.2f} [{:.2f}, {:.2f}]".format(v.mean(), v.min(), v.max()))
    if presses:
        v = np.array(presses, float)
        out.append("{:.1f} [{:.0f}, {:.0f}]".format(v.mean(), v.min(), v.max()))
    else:
        out.append("-")
    return out


def main():
    selection = json.loads((STAGE/"hidden"/"selection.json").read_text())
    candidates = json.loads((STAGE/"hidden"/"candidates.json").read_text())["seeds"]
    kept = set(selection["kept"])
    kept_rows = [r for r in selection["candidates"] if r["seed"] in kept]
    all_rows = [features(s) for s in candidates]
    ablation = STAGE/"review"/"map_ablation.json"
    dev_presses = None
    if ablation.exists():
        dev_presses = [r["carrier_actions"] for r in json.loads(ablation.read_text())["rows"] if r["map"] == "full"]
    dev_rows = [features(s) for s in range(7000, 7200)]
    head = "| set | n | " + " | ".join(FIELDS) + " | oracle carrier presses |"
    print(head)
    print("|" + "---|"*(len(FIELDS)+3))
    for name, rows, presses in (("kept (hidden)", kept_rows, [r["carrier_actions"] for r in kept_rows]),
                                ("all 24 candidates", all_rows, None),
                                ("development 7000-7199", dev_rows, dev_presses)):
        print("| {} | {} | ".format(name, len(rows)) + " | ".join(summary(rows, presses)) + " |")
    cells = {c: sum(1 for r in all_rows if r["cell"] == c) for c in ("A", "B")}
    dev_cells = {c: sum(1 for r in dev_rows if r["cell"] == c) for c in ("A", "B")}
    print("\nCells among the 24 candidates: A {A}, B {B}; among the development sample: A {da}, B {db}.".format(
        A=cells["A"], B=cells["B"], da=dev_cells["A"], db=dev_cells["B"]))


if __name__ == "__main__":
    main()
