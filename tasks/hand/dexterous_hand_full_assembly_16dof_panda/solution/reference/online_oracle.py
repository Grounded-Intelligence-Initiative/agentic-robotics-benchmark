"""Oracle driver of dexterous_hand_full_assembly_16dof_panda's graded episode (dual mode; solution/solve.sh runs it as
the agent account, after it has put main's reference program into the submission, so the freeze at the start holds
it): connects to the graded episode's service with the agent-side client (~/given/lib/graded_episode.py), starts THE
graded episode with the oracle token (the service compares its sha256 with the digest it holds and, on a match, builds
the privileged cell, as the offline grader does for a submission that carries the token: the nominal layout and the
call `reference_assemble`), runs the reference program's solve(cell) (reference/assemble.py: one frame of each camera,
then the privileged reference program inside one call) and finishes the episode.

Usage: online_oracle.py <the given client library dir> <the oracle token file>"""
import os
import sys
import time

LIB, TOKEN = sys.argv[1], sys.argv[2]
sys.path.insert(0, LIB)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from graded_episode import CellError, connect_graded  # noqa: E402

import assemble  # noqa: E402  (the privileged oracle program, solution/reference/assemble.py)


def log(msg):
    print("[oracle] %s" % (msg,), file=sys.stderr, flush=True)


def main():
    with open(TOKEN) as fh:
        token = fh.read().strip()
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
