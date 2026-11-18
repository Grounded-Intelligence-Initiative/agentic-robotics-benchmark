"""One survey, one irreversible switch, one physical dock attempt."""
import math

import numpy as np

from articulated import Go1, MAX_SCOUT, Survey, layout


class World:
    def __init__(self, seed):
        self.rng, self.side, self.goal, self.walls = layout(seed)
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
        self.go1 = Go1(self.rng, self.walls, self.goal)
        self.phase = 1
        return self.observe()

    def metrics(self):
        dog = self.go1
        entered = bool(dog and dog.corridor)
        fell = dog is not None and dog.outcome == "fell"
        return {
            "success": int(dog is not None and dog.outcome == "docked"),
            "switched": self.phase,
            "scout_actions": self.scout_actions,
            "target_actions": dog.actions if dog else 0,
            "target_ticks": dog.ticks if dog else 0,
            "target_contact": int(dog is not None and dog.outcome == "contact"),
            "target_fell": int(fell),
            "corridor_entered": int(entered),
            "first_aisle_correct": int(entered and dog.corridor == self.side),
            "fell_before_aisle": int(fell and not entered),
            "partition_crossed": int(dog is not None and dog.max_x > self.walls["island"][1]+.10),
            "max_forward_m": dog.max_x if dog else 0.,
            "travel_m": dog.travel if dog else 0.,
            "final_goal_distance": dog.dock_error if dog else -1.,
        }

    def close(self):
        self.survey.close()
