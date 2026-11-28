"""Check a robot.xml against the QM-24 catalog and the client's limits, the same check the grader makes first.

    /opt/venv/bin/python3 ~/qm24/check_design.py [~/submission/robot.xml]

Prints the first problem, or the design's price, mass, footprint, reach, clearance and joints, and the sizes the
standard gait derives from it (step length, swing height, period, nominal speed). Kinematics and bookkeeping only:
nothing here simulates the robot.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import qm_design as D  # noqa: E402
import qm_gait as G  # noqa: E402


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/submission/robot.xml")
    cat = D.load_catalog()
    with open(path, "rb") as f:
        data = f.read()
    try:
        design, summ = D.check(data, cat)
    except D.DesignError as exc:
        print("REJECTED:", exc)
        sys.exit(1)
    print("OK")
    print(json.dumps(summ, indent=1, default=float))
    print("gait:", json.dumps({k: round(v, 4) for k, v in G.gait_sizes(design, cat).items()}))


if __name__ == "__main__":
    main()
