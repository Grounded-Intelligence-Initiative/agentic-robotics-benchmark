"""One survey, one irreversible switch, one physical dock attempt."""
import math

import numpy as np

from articulated import Go1, MAX_SCOUT, Survey, layout


class World:
    def __init__(self, seed):
        self.rng, self.racks, self.goal, self.walls = layout(seed)
        self.survey = Survey(self.walls, self.goal)
        self.go1 = None
        self.phase = self.scout_actions = 0

    @property
    def done(self):
        return self.go1 is not None and self.go1.outcome != "running"

    def observe(self):
        data = self.go1.observe() if self.phase else {
            **self.survey.observe(), "scout_actions_remaining": MAX_SCOUT-self.scout_actions,
        }
        return {"phase": self.phase, "done": self.done, **data}

    def act(self, action):
        if self.done:
            raise ValueError("the mission has ended")
        if self.phase:
            self.go1.act(action)
        else:
            if self.scout_actions >= MAX_SCOUT:
                raise ValueError("scout action budget spent; advance to the Go1")
            if (not isinstance(action, list) or len(action) != 2
                    or any(type(v) not in (float, int) or not math.isfinite(v) for v in action)
                    or math.hypot(*action) > .60):
                raise ValueError("expected [dx, dy] within 0.60 m")
            self.survey.move(np.asarray(action))
            self.scout_actions += 1
        return self.observe()

    def advance(self):
        if self.phase or self.scout_actions < 4:
            raise ValueError("switch exactly once, after at least four scout moves")
        self.survey.close()
        self.go1 = Go1(self.rng, self.walls, self.goal, self.racks)
        self.phase = 1
        return self.observe()

    def metrics(self):
        dog = self.go1
        first, _, second = dog.choices if dog else (None, None, None)
        fell = dog is not None and dog.outcome == "fell"
        wall = dog.contact_wall if dog else None
        return {
            "success": int(dog is not None and dog.outcome == "docked"),
            "switched": self.phase,
            "scout_actions": self.scout_actions,
            "target_actions": dog.actions if dog else 0,
            "target_ticks": dog.ticks if dog else 0,
            "target_contact": int(dog is not None and dog.outcome == "contact"),
            "target_fell": int(fell),
            "target_budget_stop": int(dog is not None and dog.outcome == "budget"),
            "target_outcome": dog.outcome if dog else "not_deployed",
            "contact_wall": wall,
            "contact_first": int(bool(wall and wall.startswith("rack1_"))),
            "contact_middle": int(bool(wall and wall.startswith("rack2_"))),
            "contact_second": int(bool(wall and wall.startswith("rack3_"))),
            "contact_boundary": int(bool(wall and not wall.startswith("rack"))),
            "corridor_entered": int(first is not None),
            "first_aisle_correct": int(first == "wide"),
            "first_shortcut_selected": int(first == "shortcut"),
            "fell_before_aisle": int(fell and first is None),
            "partition_crossed": int(bool(dog and dog.crossed[0])),
            "middle_crossed": int(bool(dog and dog.crossed[1])),
            "second_aisle_entered": int(second is not None),
            "second_aisle_correct": int(second == "wide"),
            "second_shortcut_selected": int(second == "shortcut"),
            "second_partition_crossed": int(bool(dog and dog.crossed[2])),
            "dock_zone_entered": int(bool(dog and dog.dock_zone_entered)),
            "passage_choices": dog.choices if dog else [None]*3,
            "passage_lateral_margin_m": dog.lateral_margin if dog else [None]*3,
            "max_forward_m": dog.max_x if dog else 0.,
            "travel_m": dog.travel if dog else 0.,
            "final_goal_distance": dog.dock_error if dog else -1.,
        }

    def close(self):
        self.survey.close()
