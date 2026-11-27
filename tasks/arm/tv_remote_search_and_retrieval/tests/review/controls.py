"""The pre-registered controls (verify / maintainers only; nothing runs this at grading time).

Three controllers that must NOT do well, played in process on the simulator of tests/remote_env.py, on development
seeds or (after the draw) on the fixed hidden set:

  still     sends the hold-still action for the whole episode.
  surfaces  knows the layout (the grader's truth) and looks straight down with the wrist camera at every surface that
            lies open at the start (the two seat pads, the chest's top beside the basket, the basket from above, the
            floor in front of the furniture), and picks the TV remote up if it sees it. It opens, lifts and moves
            nothing.
  author    the contributed controller (tests/review/author_controller.py) as it was written for six places and one
            remote: it takes the first thing of the remote's size it sees.

    python3 -B tests/review/controls.py <control> dev <first seed> <seeds per place> [<place> ...]
    python3 -B tests/review/controls.py <control> hidden

Prints one JSON line per episode (by index in a hidden run, never the seed or the place; in a development run with a
hash of the final state, to compare machines) and the mean. Needs the task
image's Python and the numerics pin (tests/nonet_exec.NUMERICS_ENV), like tests/review/select_episodes.py.
"""

import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(TASK, "tests"), os.path.join(TASK, "solution", "solver")]
os.environ.setdefault("REMOTE_ROOM_DIR", os.path.join(TASK, "environment", "room"))

import episodes as R  # noqa: E402
import remote_env as E  # noqa: E402
import skills  # noqa: E402
from author_controller import AuthorController  # noqa: E402
from oracle import Oracle  # noqa: E402
from search_skills import CAB_CY  # noqa: E402


class Still(object):
    def run(self, obs):
        hold = [float(v) for v in obs["q"][:8]] + [0.08, 0]
        while True:
            yield hold


class SurfacesOnly(Oracle):
    def run(self, obs):
        self.setup(obs)
        band = lambda z: (z + 0.007, z + 0.03)                                        # noqa: E731
        looks = []
        top = E.DECK_Z + E.PAD_SIZE[2]
        for side in ("left", "right"):
            y = E.PAD_Y[side]
            half = 0.5 * E.PAD_SIZE[1] - 0.02
            looks.append(("sofa", (E.SOFA_X0 + 0.22, y), top, ((E.SOFA_X0 - 0.02, E.SOFA_X0 + 0.42), (y - half, y + half)),
                          [y + o for o in (-0.3, 0.3, -0.45, 0.45, 0.0)]))
        hw = E.CAB_HALF_W
        chest = ((E.CAB_X0 + 0.01, E.CAB_X0 + E.CAB_DEPTH - 0.02), (CAB_CY - hw, CAB_CY + hw))
        rails = [CAB_CY + o for o in (0.45, -0.45, 0.3, -0.3)]
        looks.append(("cabinet", (E.CAB_X0 + 0.22, CAB_CY), E.CAB_H, chest, rails))
        looks.append(("cabinet", (E.CAB_X0 + 0.22, CAB_CY), E.CAB_H + E.BOX_FLOOR_Z, chest, rails))      # into the basket
        for piece, y0 in (("sofa", E.SEAT_MID_Y), ("cabinet", CAB_CY)):
            for dy in (-0.3, 0.3):
                y = y0 + dy
                looks.append((piece, (0.20, y), 0.0, ((0.02, E.SOFA_X0 - 0.01), (y - 0.3, y + 0.3)), [y + o for o in (-0.4, 0.4, 0.0)]))
        for piece, centre, z, box, rails in looks:
            self.kin.use(piece)
            hit = yield from self.look_down(centre, z, band(z), box, rails)
            self.log.append(("surface", piece, z, hit))
            if hit is not None:
                yield from self.pick(hit, z, rails)
                return
        yield from self.go_home()
        while True:
            yield self.action()


CONTROLS = {"still": (Still, False, False), "surfaces": (SurfacesOnly, True, True), "author": (AuthorController, False, False)}


def play(env, name, seed, place):
    maker, truth, black = CONTROLS[name]
    skills.TV_BLACK_ONLY = black
    env.place = place
    obs = env.reset(R.make_episode(E, seed, place))

    def wire(o):
        if not truth:
            return o
        w = dict(o)
        w["truth"] = env.truth()
        return w

    behaviour = maker().run(wire(obs))
    done = False
    try:
        action = next(behaviour)
        while not done:
            obs, done = env.step(action)
            if not done:
                action = behaviour.send(wire(obs))
    except StopIteration:
        pass
    return env.score(), env.steps, env.stopped, hashlib.sha256(env.data.qpos.tobytes()).hexdigest()[:12]


def main(argv):
    name, mode = argv[0], argv[1]
    env = E.Env()
    scores = []
    if mode == "dev":
        first, per = int(argv[2]), int(argv[3])
        places = argv[4:] or list(R.HIDING_SPOTS)
        for place in places:
            for seed in range(first, first + per):
                score, steps, ended, state = play(env, name, seed, place)
                scores.append(score)
                print(json.dumps({"control": name, "place": place, "seed": seed, "score": score, "steps": steps, "ended": ended,
                                  "state": state}), flush=True)
    else:
        with open(os.path.join(TASK, "tests", "hidden", "episodes.json")) as f:
            episodes = json.load(f)["episodes"]
        for index, ep in enumerate(episodes):
            score, steps, ended, _ = play(env, name, int(ep["seed"]), ep["spot"])
            scores.append(score)
            print(json.dumps({"control": name, "episode": index, "score": score, "steps": steps, "ended": ended}), flush=True)
    print(json.dumps({"control": name, "episodes": len(scores), "mean": round(sum(scores) / max(1, len(scores)), 4)}), flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
