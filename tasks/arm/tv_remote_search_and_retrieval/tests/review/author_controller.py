"""The contributed controller, kept as review material (verify / maintainers only; no submission may use it and
nothing ships it to the agent): it has no knowledge of the episode. It fits the furniture to the first overview depth
image and goes through the places in a fixed order with the skills of solution/solver/search_skills.py, taking the
first thing of the remote's size it sees. tests/review/controls.py runs it as one of the pre-registered controls.
"""

import numpy as np
from scipy.ndimage import median_filter

import remote_env as E
from search_skills import CAB_CY, SearchSkills
from skills import overview_pose


class AuthorController(SearchSkills):
    def run(self, obs):
        """The whole episode as a generator of actions."""
        self.obs = obs
        self.log = []
        # Where does the furniture stand? Everything after this works in the nominal layout's
        # coordinates of the piece it deals with.
        first = obs["images"]["overview"]
        sofa, cab = self.locator.locate(self.kin, E.decode_image(first["depth"]), obs["q"])
        self.layout = (sofa, cab)
        self.log.append(("layout", [round(float(v), 4) for v in sofa], [round(float(v), 4) for v in cab]))
        # ... and the box on the chest, so that the arm keeps clear of it.
        self.kin.use("cabinet")
        depth = median_filter(E.decode_image(first["depth"]), size=3)
        rim = self.box_rim(self.points("overview", depth, overview_pose(), depth > 0))
        self.box_xy = (E.CAB_X0 + E.BOX_X, CAB_CY)
        if len(rim) >= 80:
            self.box_xy = tuple(float(0.5 * (np.percentile(rim[:, i], 2) + np.percentile(rim[:, i], 98))) for i in (0, 1))
        self.log.append(("box", len(rim), [round(v, 3) for v in self.box_xy]))
        self.kin.box_xy = self.box_xy
        self.kin.set_layout(sofa, cab)
        order = getattr(self, "order", ("under", "chest", "box", "left", "right", "lower", "mid", "upper"))
        for name in order:
            if name == "under":
                found = yield from self.search_under()
            elif name == "chest":
                found = yield from self.search_under_chest()
            elif name == "box":
                found = yield from self.search_box()
            elif name in ("lower", "mid", "upper"):
                found = yield from self.search_drawer(name)
            else:
                found = yield from self.search_cushion(name)
            if found:
                return
        yield from self.wait(10)
