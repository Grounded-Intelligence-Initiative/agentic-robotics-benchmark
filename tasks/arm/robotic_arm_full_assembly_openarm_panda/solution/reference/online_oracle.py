"""Oracle driver of robotic_arm_full_assembly_openarm_panda's graded episode (dual mode; solution/solve.sh runs it as the
agent account): connects to the graded episode's cell service with the agent-side client, starts the graded episode
with the oracle token (the privileged mode, as offline: the cell built again from the episode's seed on the bench
plan's nominal layout, and the call reference_assemble), runs the task's privileged program (assemble.py's solve: one
frame of each camera, then the privileged program inside one call) and finishes the episode. solution/solve.sh has put
main's privileged program into the submission first, so the freeze at the start holds it.

Usage: online_oracle.py <the given client library dir> <the oracle token file>"""
import os
import sys
import time

LIB, TOKEN = sys.argv[1], sys.argv[2]
sys.path.insert(0, LIB)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm.assembly_openarm.graded import CellError, connect_graded  # noqa: E402

import assemble  # noqa: E402  (the privileged program, solution/reference/assemble.py)


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
    t0 = time.time()
    log("start: %s (%.0f s)" % (cell._call("start", oracle_token=token), time.time() - t0))
    assemble.solve(cell)
    log("finish: %s (%.0f s after the start)" % (cell.finish(), time.time() - t0))
    log("status at the end: %s" % (cell.status(),))
    cell.close()


if __name__ == "__main__":
    main()
