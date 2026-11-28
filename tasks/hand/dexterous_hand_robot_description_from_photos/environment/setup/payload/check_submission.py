#!/usr/bin/env python3
"""Format check of the hand description (the grader's own format-check code, in checker/).

    /opt/venv/bin/python3 check_submission.py [DIR]      # default: ~/submission
    options: --json FILE (write the result and every violation)

Prints PASS or FAIL and every violated rule. It runs rules 1 to 9 of the contract: the files and size caps, the MJCF
whitelist, loading (radian header, a compile without warnings), names and structure, the coupling form of each
finger's last hinge, the pad sites, the actuators, the masses and config.yaml. Rule 10 (nothing written after the
session ends) is applied by the grader alone. The check knows nothing about the photographed hand: a PASS says the
description can be graded, not how accurate it is.
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "checker"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--json", default=None, help="write the result to this file")
    ap.add_argument("dir", nargs="?", default=os.path.join(os.environ.get("HOME", "/home/user"), "submission"))
    a = ap.parse_args(argv)
    import gate as GT
    ok, det, _models = GT.gate_model_loads(os.path.abspath(a.dir))
    print("format check: {}".format("PASS" if ok else "FAIL"))
    for v in det.get("violations", []):
        print("  - " + v)
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"model_loads": 1.0 if ok else 0.0, "details": det}, f, indent=1, default=str)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
