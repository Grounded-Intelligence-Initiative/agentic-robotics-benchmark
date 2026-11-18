"""The oracle of the online variant: the task's privileged reference submission (operate.py: describe, one overview and
one wrist frame, then reference_run, the scripted reference program inside the cell) played on the graded episode
through the agent-side client, as the agent account, then finished. solution/solve.sh runs it.

The start carries the 256-bit oracle token; the graded episode's service compares its sha256 with the one it holds and,
on a match, gives the episode's cell the privileged calls, as the offline grader gives them to a submission that ships
the token. The graded episode then runs exactly as the oracle's episode of the offline grader: the same requests, then
`finish` once the reference program returns.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GIVEN = os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))
sys.path.insert(0, os.path.join(GIVEN, "lib"))
sys.path.insert(0, HERE)

import operate  # noqa: E402  (the reference submission)
from skidcell.client import connect_graded  # noqa: E402


def main():
    with open(os.path.join(HERE, "oracle_token")) as f:
        token = f.read().strip()
    cell = connect_graded()
    if cell.status()["started"]:
        sys.exit("[oracle] the graded episode was started already")
    cell.start(token=token)
    operate.solve(cell)
    cell.finish()
    st = cell.status()
    cell.close()
    print("[oracle] graded episode played: ended %s (%s), %d steps, sim %.1f s" % (
        st["ended"], st["end"], st["steps"], st["sim_time_s"]), file=sys.stderr)


if __name__ == "__main__":
    main()
