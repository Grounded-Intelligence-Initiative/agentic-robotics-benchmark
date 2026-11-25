"""The oracle's graded episode (dual mode, the live item): the privileged reference program flies the graded episode
through the agent-side client, as solve.sh's second step.

    python3 -B graded_oracle.py --token <oracle_token file>

It connects to the graded episode's service (~/given/lib/cabinkit/graded.py: connect_graded), starts the graded
episode with the oracle token (the service compares its sha256 with its own digest and, on a match, allows the cell's
privileged call `reference_run` on the graded site: the reference program of tests/reference inside the cell, every
move through the robot API's calls), runs it and finishes the episode. The program in the submission folder (main's
oracle) must be in place first: the start freezes it.
"""
import argparse
import json
import os
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--token", required=True)
    ap.add_argument("--wait-s", type=float, default=600.0, help="how long to wait for the service's socket")
    a = ap.parse_args()
    home = os.environ.get("ALE_HOME", "/home/user")
    sys.path.insert(0, os.path.join(home, "given", "lib"))
    from cabinkit.client import CellError
    from cabinkit.graded import connect_graded
    token = open(a.token).read().strip()
    deadline = time.time() + a.wait_s
    while True:
        try:
            robot = connect_graded()
            break
        except CellError as e:
            if time.time() > deadline:
                raise
            print("[oracle] the graded episode's service is not ready (%s); waiting" % e, file=sys.stderr)
            time.sleep(5.0)
    st = robot._call("graded_start", oracle_token=token)
    print("[oracle] graded episode started: %s" % json.dumps(st), file=sys.stderr)
    t0 = time.time()
    res = robot._call("reference_run")
    print("[oracle] reference_run: %s (%.0f s of wall clock)" % (json.dumps(res), time.time() - t0), file=sys.stderr)
    fin = robot.graded_finish()
    print("[oracle] graded episode finished: %s" % json.dumps(fin), file=sys.stderr)
    robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
