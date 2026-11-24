"""The oracle of the dual mode's graded episode (experiment): the task's privileged reference controller
(oracle_ctrl.KitOracle) played on the graded episode through the agent-side client (~/commission/client.py), as the
agent account, then finished. solution/solve.sh runs it after it has left main's reference submission in the
submission folder, which the start then freezes as the program item.

The start carries the reference's token (solution/oracle_token); the graded episode's service compares its sha256 with
the one it holds and, on a match, attaches `_gt` (keyed_credit.ground_truth) to every observation, as the offline
grader does for the token-holding submission. The reference acts on the real observation's proprioception
(`_gt["proprio"]`) and the ground truth, as in its offline episodes (oracle_main.py without the repeated-step
bookkeeping, which the graded episode has no use for: it never asks a step twice).
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.environ.get("ALE_HOME", "/home/user"), "commission"))

import oracle_ctrl  # noqa: E402
from client import connect_graded  # noqa: E402

NOOP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0]


def main():
    with open(os.path.join(os.path.dirname(HERE), "oracle_token")) as f:
        token = f.read().strip()
    g = connect_graded()
    if g.status()["started"]:
        sys.exit("[oracle] the graded episode was started already")
    obs = g._call("start", {"token": token})["obs"]     # the client's start() has no token: the reference speaks the wire
    ctrl = oracle_ctrl.KitOracle()
    steps = 0
    while obs is not None and not g.done:
        gt = obs.get("_gt")
        action = ctrl.act(dict(obs, **gt.get("proprio", {})), gt) if gt else list(NOOP)
        obs = g.step([float(a) for a in action])
        steps += 1
    g.finish()
    st = g.status()
    g.close()
    print("[oracle] graded episode played: %d steps, ended %s (%s), images %d" % (
        steps, st["ended"], st["end"], st["images_requested"]), file=sys.stderr)


if __name__ == "__main__":
    main()
