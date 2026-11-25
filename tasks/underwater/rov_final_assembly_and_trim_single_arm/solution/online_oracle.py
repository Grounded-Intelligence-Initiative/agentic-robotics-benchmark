"""Oracle driver of rov_final_assembly_and_trim_single_arm's graded episode (dual mode; solution/solve.sh runs it as the
agent account after it has put main's reference program into the submission): connects to the graded episode's cell
service with the agent-side client (~/given/lib/arm/assembly_rov/graded.py), starts the graded episode with the oracle
token (the service compares its sha256 with the digest it holds and, on a match, gives this episode the privileged
calls, as the offline grader gives a submission with the token), runs the reference program (solution/assemble.py's
solve(cell): the work order, the load cells and one frame of each camera through the public calls, then the privileged
reference program inside the cell, `reference_assemble`) and finishes the episode. The start freezes the submission
folder, so the program the offline item grades is main's reference program.

Usage: online_oracle.py <the given library dir> <the oracle token file>"""
import os
import sys
import time

LIB, TOKEN = sys.argv[1], sys.argv[2]
sys.path.insert(0, LIB)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm.assembly_rov.graded import CellError, connect_graded  # noqa: E402

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
        except CellError as e:
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
