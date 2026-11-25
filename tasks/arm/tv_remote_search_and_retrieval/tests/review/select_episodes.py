"""Draws the hidden episode set (maintainers' tool; nothing runs it at grading time).

For each hiding spot, candidate seeds are drawn from os.urandom and the reference controller of
solution/ plays each candidate once, in process, on the simulator of tests/remote_env.py. The first
`PER_SPOT` candidates of a spot in which the reference retrieves the remote are the spot's graded
episodes. Every candidate and its outcome is recorded in tests/hidden/selection.json, so the
reference's own failure rate on unselected draws is on record.

    REMOTE_ROOM_DIR=environment/room python3 tests/review/select_episodes.py
"""

import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(TASK, "tests"), os.path.join(TASK, "solution", "solver")]
os.environ.setdefault("REMOTE_ROOM_DIR", os.path.join(TASK, "environment", "room"))

import remote_env as E  # noqa: E402
from reference import Reference  # noqa: E402

PER_SPOT = 2
CANDIDATES = 5
# Before any candidate is played, the remote must show in neither camera.


def play(env, spec):
    obs = env.reset(spec)
    behaviour = Reference().run(obs)
    done = False
    try:
        action = next(behaviour)
        while not done:
            obs, done = env.step(action)
            if not done:
                action = behaviour.send(obs)
    except StopIteration:
        pass
    return env.score(), env.steps, env.stopped


def main():
    env = E.Env()
    record, chosen = [], {}
    for spot in E.HIDING_SPOTS:
        chosen[spot] = []
        for _ in range(CANDIDATES):
            seed = struct.unpack("<Q", os.urandom(8))[0]
            env.reset(E.make_episode(seed, spot))
            if max(env._remote_pixels(name) for name in E.CAMERAS) > 0:
                raise SystemExit("the remote shows at the start of {} {}".format(spot, seed))
            score, steps, ended = play(env, E.make_episode(seed, spot))
            picked = score >= 1.0 and len(chosen[spot]) < PER_SPOT
            if picked:
                chosen[spot].append(seed)
            record.append({"spot": spot, "seed": seed, "reference_score": score, "steps": steps,
                           "ended": ended, "selected": picked})
            print(record[-1], flush=True)
        if len(chosen[spot]) < PER_SPOT:
            raise SystemExit("too few episodes for " + spot)
    # Graded order: the spots in turn, twice.
    episodes = [{"seed": chosen[spot][i], "spot": spot} for i in range(PER_SPOT) for spot in E.HIDING_SPOTS]
    with open(os.path.join(TASK, "tests", "hidden", "episodes.json"), "w") as handle:
        json.dump({"episodes": episodes}, handle, indent=1)
    with open(os.path.join(TASK, "tests", "hidden", "selection.json"), "w") as handle:
        json.dump({"rule": "first {} of {} os.urandom candidates per spot that the reference retrieves".format(
            PER_SPOT, CANDIDATES), "candidates": record}, handle, indent=1)
    solved = sum(1 for r in record if r["reference_score"] >= 1.0)
    print("reference retrieved {} of {} candidates".format(solved, len(record)))


if __name__ == "__main__":
    main()
