"""The reference: the search-and-retrieve skills of search_skills.py driven by the grader's truth.

Only a submission that carries the oracle token gets `truth` in its observations (tests/verify.py). From it the
reference takes where the furniture stands (instead of fitting it to the overview depth image), where the basket
stands, and which place the remote is in: it goes to that place first and does what the place takes (lift the pad,
fetch the hanger and sweep, reach in under the chest, open the hat's drawer and take the hat out, pour the basket
out). If that does not end with the remote in the hand it goes through the other places in the usual order. The
remote itself is still found in the camera images the skills ask for, as a controller without the token has to.
"""

import numpy as np

import remote_env as E
import skills
from search_skills import CAB_CY, SearchSkills

skills.TV_BLACK_ONLY = True

FIRST = {"left_cushion": "left", "right_cushion": "right", "under_sofa": "under", "under_chest": "chest", "in_box": "box"}
USUAL = ("under", "chest", "box", "left", "right", "lower", "mid", "upper")


class Oracle(SearchSkills):
    def setup(self, obs):
        """The layout and the basket's place from the truth."""
        self.obs = obs
        self.log = []
        truth = obs["truth"]
        scene = truth["scene"]
        sofa, cab = tuple(scene["sofa_pose"]), tuple(scene["cabinet_pose"])
        self.layout = (sofa, cab)
        self.kin.use("cabinet")
        self.box_xy = tuple(float(v) for v in scene["box_xy"])
        self.kin.box_xy = self.box_xy
        self.kin.set_layout(np.array(sofa), np.array(cab))
        return truth

    def run(self, obs):
        truth = self.setup(obs)
        scene = truth["scene"]
        place = truth["place"]
        first = scene["hat_drawer"] if place == "under_hat" else FIRST.get(place)
        if place == "in_drawer":
            z = truth["loose"]["remote"][2]
            first = min(E.DRAWERS, key=lambda n: abs(E.DRAWER_FLOOR_Z[n] + E.REMOTE_HALF[2] - z))
        # Under a seat pad: the basket is emptied and put aside first, as the skills were developed (with the basket on
        # the chest the arm's way to a pad is planned around it, and the pad's lift then failed in 3 of 12
        # development episodes).
        lead = ["box", first] if first in ("left", "right") else ([first] if first else [])
        order = lead + [n for n in USUAL if n not in lead]
        for name in order:
            if name == "under":
                found = yield from self.search_under()
            elif name == "chest":
                found = yield from self.search_under_chest()
            elif name == "box":
                found = yield from self.search_box()
            elif name in E.DRAWERS:
                found = yield from self.search_drawer(name)
            else:
                found = yield from self.search_cushion(name)
            if found:
                return
        yield from self.wait(10)
