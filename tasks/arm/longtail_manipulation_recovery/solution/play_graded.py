"""The oracle of the dual mode's graded episode: the task's privileged reference (oracle_main.py's replay of a recorded
rescue) played on the graded episode through the agent-side client (~/graded/client.py), as the agent account.
solution/solve.sh runs it after it has put main's reference submission in place (the graded start freezes the
submission folder).

The start carries the reference's token (the token file's bytes, as the grader hashes the submission's .oracle_token);
the graded episode's service compares its sha256 with the one it holds and, on a match, adds `_gt` (the state's id) to
every observation, as the offline grader does for a submission that ships the token. The reference then answers each
observation as oracle_main.py answers the grader's `act` requests: the recorded rescue's action for step t, then the
hold action past its end, until the episode ends by itself (the job's success check held for 10 consecutive steps, or
the step budget).
"""

import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("ALE_HOME", "/home/user"))

from graded.client import GradedEpisode  # noqa: E402  (the agent-side client)


def hold():
    """oracle_main.hold(): the gripper open, the arm mode, no base motion."""
    a = np.zeros(12, np.float32)
    a[6] = -1.0
    a[11] = -1.0
    return a


def main():
    with open(os.path.join(HERE, "oracle_token")) as f:
        token = f.read()
    g = GradedEpisode()
    if g.status()["started"]:
        sys.exit("[oracle] the graded episode was started already")
    t0 = time.time()
    r = g.call("start", token=token)
    obs = r["obs"]
    state = (obs.get("_gt") or {}).get("state_id")
    if not state:
        sys.exit("[oracle] the graded episode's start did not accept the reference's token (no _gt)")
    with np.load(os.path.join(HERE, "rescues", state + ".npz")) as z:
        actions = np.asarray(z["action"], np.float32)
    done, ended, steps = bool(r.get("done")), None, 0
    while not done:
        t = int(obs["t"])
        a = actions[t] if t < len(actions) else hold()
        r = g.step([float(x) for x in a])
        obs, done, ended = r["obs"], bool(r["done"]), r.get("ended")
        steps += 1
    st = g.status()
    if st["started"] and not st["ended"]:
        g.finish()
        st = g.status()
    g.close()
    print("[oracle] graded episode played: %d steps (rescue of %d actions), ended %s (%s: %s), %.0f s" % (
        steps, len(actions), st["ended"], st["end"], ended, time.time() - t0), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
