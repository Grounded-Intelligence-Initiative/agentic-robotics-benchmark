"""The oracle's driver of the graded episode (run by solve.sh as the agent account): connects to the graded service
with the agent's own client (shoecell.client.connect_graded), starts the graded episode with the 256-bit reference
token (the service compares its sha256 with its root-only digest and, on a match, gives this episode's cell the
privileged calls), runs the existing privileged reference program (reference/packer.py: describe, two camera frames,
reference_run, the scripted reference controller inside the cell) and finishes the episode."""

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given")), "lib"))
sys.path.insert(0, os.path.join(HERE, "reference"))

from shoecell.client import connect_graded  # noqa: E402

import packer  # noqa: E402  (the reference program)


def main():
    with open(os.path.join(HERE, "reference", "oracle_token")) as fh:
        token = fh.read().strip()
    t0 = time.time()
    with connect_graded() as cell:
        print("[oracle] before: %s" % json.dumps(cell.status()), file=sys.stderr, flush=True)
        cell._call("start", token=token)
        packer.solve(cell)
        cell.finish()
        print("[oracle] after %.0f s: %s" % (time.time() - t0, json.dumps(cell.status())), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
