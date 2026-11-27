"""The oracle of the dual mode's graded episode (experiment): the task's privileged reference (oracle/reference.py,
driven by oracle/wire_world.Oracle on a shadow of the world that each observation's `truth` sets) played on the graded
episode through the agent-side client (~/graded/client.py), as the agent account, then finished. solution/solve.sh
runs it after it has left main's reference submission in the submission folder, which the start then freezes as the
program item.

The start carries the reference's token (solution/oracle_token); the graded episode's service compares its sha256 with
the one it holds and, on a match, attaches `truth` (JarEnv.truth) to every observation, as the offline grader does for
the token-holding submission. The reference then acts exactly as in its offline episode on this unit and seed (the
graded episode never asks a step twice).
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "oracle"))
sys.path.insert(0, os.path.join(os.environ.get("ALE_HOME", "/home/user"), "graded"))

from client import connect_graded  # noqa: E402
from wire_world import Oracle  # noqa: E402


def main():
    with open(os.path.join(HERE, "oracle_token")) as f:
        token = f.read().strip()
    g = connect_graded()
    if g.status()["started"]:
        sys.exit("[oracle] the graded episode was started already")
    obs = g._call("start", {"token": token})["obs"]     # the client's start() has no token: the reference speaks the wire
    oracle = Oracle()
    steps = 0
    try:
        while obs is not None and not g.done:
            obs = g.step(oracle.act(int(obs["t"]), obs))
            steps += 1
    finally:
        oracle.stop()
    g.finish()
    st = g.status()
    g.close()
    print("[oracle] graded episode played: %d steps, ended %s (%s), images %d" % (
        steps, st["ended"], st["end"], st["images_requested"]), file=sys.stderr)


if __name__ == "__main__":
    main()
