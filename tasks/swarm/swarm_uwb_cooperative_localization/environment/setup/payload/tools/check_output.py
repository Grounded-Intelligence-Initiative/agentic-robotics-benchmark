"""Check that <out_dir>/trajectories.csv has a finite row for every drone and every output
instant of the flight. It knows nothing about accuracy.

  python3 ~/tools/check_output.py <flight_dir> <out_dir>
"""

import csv
import json
import math
import os
import sys


def main():
    flight, out = sys.argv[1], sys.argv[2]
    meta = json.load(open(os.path.join(flight, "flight.json")))
    n = int(round(meta["duration_s"] * meta["output_rate_hz"]))
    want = {(d, k) for d in meta["drones"] for k in range(n)}
    seen, bad = set(), 0
    with open(os.path.join(out, "trajectories.csv"), newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["t_s"])
                d = int(row["drone"])
                p = [float(row["x_m"]), float(row["y_m"]), float(row["z_m"])]
            except (KeyError, TypeError, ValueError):
                bad += 1
                continue
            k = int(round(t * meta["output_rate_hz"]))
            if abs(t * meta["output_rate_hz"] - k) > 0.05 or not all(math.isfinite(v) for v in p):
                bad += 1
                continue
            seen.add((d, k))
    missing = want - seen
    print("{} rows expected, {} missing, {} malformed".format(len(want), len(missing), bad))
    print("ok" if not missing and not bad else "not ok")


if __name__ == "__main__":
    main()
