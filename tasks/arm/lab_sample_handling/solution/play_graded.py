"""The oracle of the dual mode's graded episode: the task's privileged reference (solution/oracle: the worklist reference
driven by wire_world.Oracle on a shadow of the world that the token truth sets every step) played on the graded
episode through the agent-side client (~/graded/client.py), as the agent account, then finished. solution/solve.sh
runs it after it has put main's reference submission in place (the graded start freezes the submission folder).

The start carries the reference's token; the graded episode's service compares its sha256 with the one it holds and,
on a match, adds `truth` (the unit, the seed, the full simulation state) to every observation of the episode, as the
offline grader adds it to every observation it sends a submission that ships the token. The reference then answers
each observation as it answers the grader's `act` requests (oracle_main.py's loop, without the repeated requests the
graded episode does not have), until the episode ends by its own rules (the reference shuts the lid when the
worklist is done: the episode ends 20 steps later).
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "oracle"))
sys.path.insert(0, os.environ.get("ALE_HOME", "/home/user"))

from graded.client import GradedEpisode  # noqa: E402  (the agent-side client)
from wire_world import Oracle  # noqa: E402  (the reference, as oracle_main.py runs it)


def main():
    with open(os.path.join(HERE, "oracle_token")) as f:
        token = f.read().strip()
    g = GradedEpisode()
    if g.status()["started"]:
        sys.exit("[oracle] the graded episode was started already")
    t0 = time.time()
    r = g.call("start", token=token)
    obs = r["obs"]
    if "truth" not in obs:
        sys.exit("[oracle] the graded episode's start did not accept the reference's token (no truth)")
    oracle = Oracle()
    oracle.reset()
    done, steps, ended = bool(r.get("done")), 0, None
    try:
        while not done:
            action = oracle.act(int(obs["t"]), obs)
            r = g.step(action)
            obs, done, ended = r["obs"], bool(r["done"]), r.get("ended")
            steps += 1
            if steps % 2000 == 0:
                print("[oracle] graded episode: %d steps, %.0f s" % (steps, time.time() - t0), file=sys.stderr,
                      flush=True)
    finally:
        oracle.stop()
    st = g.status()
    if st["started"] and not st["ended"]:
        g.finish()
        st = g.status()
    g.close()
    print("[oracle] graded episode played: %d steps, ended %s (%s: %s), %.0f s" % (
        steps, st["ended"], st["end"], ended, time.time() - t0), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
