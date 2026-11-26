"""Draws the hidden episode set (maintainers' tool; nothing runs it at grading time).

For each hiding spot, candidate seeds are drawn from os.urandom and the reference controller of
solution/ plays each candidate once, in process, on the simulator of tests/remote_env.py. The first
`per_spot(spot)` candidates of a spot in which the reference retrieves the remote, and retrieves it
again in `REPLAYS` replays with the loose things nudged by a hair (pouring a basket out does not
come out the same on every machine's arithmetic), are the spot's graded episodes. Every candidate and its outcome is recorded in tests/hidden/selection.json, so the
reference's own failure rate on unselected draws is on record.

    REMOTE_ROOM_DIR=environment/room python3 tests/review/select_episodes.py
"""

import json
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(TASK, "tests"), os.path.join(TASK, "solution", "solver")]
os.environ.setdefault("REMOTE_ROOM_DIR", os.path.join(TASK, "environment", "room"))

import episodes as R  # noqa: E402
import remote_env as E  # noqa: E402
from reference import Reference  # noqa: E402

PER_SPOT = 2
# The two seat pads are one place, "under a seat pad", with one episode on each side, so that
# the place weighs as much in the result as any other.
ONE_EACH = ("left_cushion", "right_cushion")


def per_spot(spot):
    return 1 if spot in ONE_EACH else PER_SPOT
CANDIDATES = 8
REPLAYS = 3            # a candidate is graded only if the reference also retrieves in this many nudged replays
# A candidate in which more than HIDDEN_PIXELS pixels of the remote show in either camera at the
# start is not played (60 pixels count as seen).
HIDDEN_PIXELS = 8


def play(env, spec, jitter=None):
    obs = env.reset(spec)
    if jitter is not None:
        # A replay with the loose things nudged by a hair: what another machine's arithmetic does.
        env.data.qvel[:] += np.random.default_rng(jitter).normal(0.0, 1e-5, env.model.nv)
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
    for spot in R.HIDING_SPOTS:
        chosen[spot] = []
        for _ in range(CANDIDATES):
            seed = struct.unpack("<Q", os.urandom(8))[0]
            env.reset(R.make_episode(E, seed, spot))
            if max(env._remote_pixels(name) for name in E.CAMERAS) > HIDDEN_PIXELS:
                record.append({"spot": spot, "seed": seed, "shows_at_start": True, "selected": False})
                print(record[-1], flush=True)
                continue
            score, steps, ended = play(env, R.make_episode(E, seed, spot))
            replays = []
            if score >= 1.0 and len(chosen[spot]) < per_spot(spot):
                replays = [play(env, R.make_episode(E, seed, spot), jitter=k)[0] for k in range(REPLAYS)]
            picked = score >= 1.0 and len(chosen[spot]) < per_spot(spot) and all(r >= 1.0 for r in replays)
            if picked:
                chosen[spot].append(seed)
            record.append({"spot": spot, "seed": seed, "reference_score": score, "steps": steps,
                           "ended": ended, "replays": replays, "selected": picked})
            print(record[-1], flush=True)
        if len(chosen[spot]) < per_spot(spot):
            raise SystemExit("too few episodes for " + spot)
    # Graded order: the spots in turn, twice (a seat pad only the first time).
    episodes = [{"seed": chosen[spot][i], "spot": spot} for i in range(PER_SPOT) for spot in R.HIDING_SPOTS
                if i < per_spot(spot)]
    with open(os.path.join(TASK, "tests", "hidden", "episodes.json"), "w") as handle:
        json.dump({"episodes": episodes}, handle, indent=1)
    with open(os.path.join(TASK, "tests", "hidden", "selection.json"), "w") as handle:
        json.dump({"rule": "first {} (one for each seat pad) of {} os.urandom candidates per spot that the reference retrieves, also in {} replays with the loose things nudged by 1e-5".format(
            PER_SPOT, CANDIDATES, REPLAYS), "candidates": record}, handle, indent=1)
    played = [r for r in record if "reference_score" in r]
    solved = sum(1 for r in played if r["reference_score"] >= 1.0)
    print("reference retrieved {} of {} candidates played".format(solved, len(played)))


if __name__ == "__main__":
    main()
