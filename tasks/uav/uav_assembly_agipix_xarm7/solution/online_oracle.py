"""Oracle driver of uav_assembly_agipix_xarm7's graded episode (dual mode; solution/solve.sh runs it as the agent
account after it has put main's reference program into the submission): connects to the graded episode's cell service
with the agent-side client (~/graded_client.py), starts the graded episode with the oracle token (the service compares
its sha256 with the digest it holds and, on a match, builds the episode's workcell as the offline grader builds a
privileged program's, on the nominal layout of the episode's seed, and gives it the privileged call reference_assemble),
runs the reference program (solution/assemble.py's solve(cell): one frame of each camera, then the privileged physical
reference inside the workcell) and finishes the episode. The start freezes the submission folder, so the program the
offline item grades is main's reference program.

Usage: online_oracle.py <the folder holding the given clients (the agent's home)> <the oracle token file>"""
import os
import sys
import time

HOME, TOKEN = sys.argv[1], sys.argv[2]
sys.path.insert(0, HOME)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from graded_client import connect_graded  # noqa: E402

import assemble  # noqa: E402  (the reference program, solution/assemble.py)


def log(msg):
    print("[oracle] %s" % (msg,), file=sys.stderr, flush=True)


def main():
    token = open(TOKEN).read().strip()
    cell = None
    for attempt in range(60):               # the service is up once setup has returned; a busy socket is retried
        try:
            cell = connect_graded()
            break
        except RuntimeError as e:
            log("connect: %s (attempt %d)" % (e, attempt + 1))
            time.sleep(5.0)
    if cell is None:
        raise SystemExit("[oracle] the graded episode's service could not be reached")
    log("status before the start: %s" % (cell.status(),))
    log("start: %s" % (cell._call("start", oracle_token=token),))
    t0 = time.time()
    assemble.solve(cell)
    log("finish: %s (%.0f s after the start)" % (cell.finish(), time.time() - t0))
    log("status at the end: %s" % (cell.status(),))
    cell.close()


if __name__ == "__main__":
    main()
