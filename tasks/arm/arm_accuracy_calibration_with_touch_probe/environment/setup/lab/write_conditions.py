"""write_conditions.py <lab folder> <output folder>: one conditions file per unit (setup, as root)."""
import json
import os
import sys

sys.path.insert(0, sys.argv[1])

import calib_cell as cc  # noqa: E402

with open(os.path.join(sys.argv[1], "units.json")) as handle:
    units = json.load(handle)
for group in ("practice", "graded"):
    for uid, seed in units[group].items():
        with open(os.path.join(sys.argv[2], uid + ".json"), "w") as out:
            json.dump(cc.write_conditions(cc.draw_unit(seed)), out)
