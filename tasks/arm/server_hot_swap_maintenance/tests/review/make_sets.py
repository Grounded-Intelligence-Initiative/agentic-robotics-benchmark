"""Maintainers' tool: draw the development units (the commissioning service's) and the candidate hidden episodes,
from salts of their own, disjoint by construction.

    python3 review/make_sets.py dev  OUT_dev_units.json
    python3 review/make_sets.py cand N OUT_units.json OUT_candidates.json

A candidate i: unit seed and episode seed are 64-bit BLAKE2b values of (HIDDEN_SALT, "unit"/"episode", i); the unit's
values are srv_sim.draw_unit(unit seed), written in full (the grader reads them from tests/units.json and does not
redraw them). The hidden set is then chosen by review/select_set.py from candidates the reference completed over
the real wire."""

import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import srv_sim as S  # noqa: E402

HIDDEN_SALT = "srv-hidden-7c41e09a2b"
DEV_SALT = "srv-dev-3e9d55f0c1"


def h64(*parts):
    return int.from_bytes(hashlib.blake2b("|".join(str(p) for p in parts).encode(), digest_size=8).digest(), "little")


def main():
    kind = sys.argv[1]
    if kind == "dev":
        units = {"d%d" % (i + 1): S.draw_unit(h64(DEV_SALT, "unit", i)) for i in range(4)}
        json.dump({"sessions": 5, "salt": DEV_SALT + "-sessions", "units": units}, open(sys.argv[2], "w"), indent=1,
                  sort_keys=True)
    elif kind == "cand":
        n = int(sys.argv[2])
        units, cands = {}, []
        for i in range(n):
            uid = "c%02d" % i
            units[uid] = S.draw_unit(h64(HIDDEN_SALT, "unit", i))
            cands.append({"index": i, "unit": uid, "seed": h64(HIDDEN_SALT, "episode", i)})
        json.dump({"units": units}, open(sys.argv[3], "w"), indent=1, sort_keys=True)
        json.dump({"episodes": cands}, open(sys.argv[4], "w"), indent=1)


if __name__ == "__main__":
    main()
