"""Oracle driver of earthquake_search_and_rescue's graded episode (dual mode; solution/solve.sh runs it as the agent
account): connects to the graded episode's service with the agent-side client, starts the graded episode with the
oracle token (the privileged mode: reference_run and get_state_privileged, as offline), runs the task's privileged
oracle program (rescue.py's solve: a camera frame, a lidar sweep and a thermal frame, then the reference program inside
one call) and finishes the episode. solution/solve.sh has put main's reference program into the submission first, so
the freeze at the start holds it.

Usage: online_oracle.py <the given client library dir> <the oracle token file>"""
import os
import sys
import time

LIB, TOKEN = sys.argv[1], sys.argv[2]
sys.path.insert(0, LIB)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rubblecell.client import CellError, connect_graded  # noqa: E402

import rescue  # noqa: E402  (the privileged oracle program, solution/reference/rescue.py)


def log(msg):
    print("[oracle] %s" % (msg,), file=sys.stderr, flush=True)


def main():
    token = open(TOKEN).read().strip()
    robot = None
    for attempt in range(60):               # the service is up once setup has returned; a busy socket is retried
        try:
            robot = connect_graded()
            break
        except CellError as e:
            log("connect: %s (attempt %d)" % (e, attempt + 1))
            time.sleep(5.0)
    if robot is None:
        raise SystemExit("[oracle] the graded episode's service could not be reached")
    log("status before the start: %s" % (robot.status(),))
    log("start: %s" % (robot._call("start", oracle_token=token),))
    t0 = time.time()
    rescue.solve(robot)
    log("finish: %s (%.0f s after the start)" % (robot.finish(), time.time() - t0))
    log("status at the end: %s" % (robot.status(),))
    robot.close()


if __name__ == "__main__":
    main()
