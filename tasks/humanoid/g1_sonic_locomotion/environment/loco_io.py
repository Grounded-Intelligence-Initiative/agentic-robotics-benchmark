"""Public 50 Hz control packets for Unitree G1 SONIC locomotion."""

from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np

from sonic_loco.score import PhysicalScore


CHUNK = 5
CODE_DIM = 64
TASKS = ("straight_stop", "right_turn_stop")
PROPRIO = (
    "q", "dq", "base_pos", "base_quat", "base_gyro", "gravity_body",
    "torso_pos", "torso_quat", "base_velocity_world", "heading_rad", "yaw_rate_rad_s",
    "left_foot_pos", "right_foot_pos", "left_foot_contact", "right_foot_contact",
)


@dataclass(frozen=True)
class Route:
    """Distance along the initial heading and signed yaw from the initial heading."""

    name: str
    distance_m: float
    heading_delta_rad: float

    def __post_init__(self):
        if self.name not in TASKS:
            raise ValueError(f"unknown route {self.name!r}")
        if not math.isfinite(self.distance_m) or self.distance_m < 0:
            raise ValueError("distance_m must be finite and nonnegative")
        if not math.isfinite(self.heading_delta_rad):
            raise ValueError("heading_delta_rad must be finite")

    def descriptor(self):
        return {"distance_m": self.distance_m, "heading_delta_rad": self.heading_delta_rad}


def practice_route(name):
    if name == "straight_stop":
        return Route(name, 5.0, 0.0)
    if name == "right_turn_stop":
        return Route(name, 0.0, -math.pi / 2)
    raise ValueError(f"unknown route {name!r}")


def stamp_observation(obs):
    """Hash the sensor and goal payload, independent of the preceding receipt."""
    fields = {k: v for k, v in obs.items() if k not in ("observation_id", "last_observation_id")}
    data = json.dumps(fields, sort_keys=True, separators=(",", ":"), allow_nan=False)
    obs["observation_id"] = hashlib.sha256(data.encode("utf-8")).hexdigest()
    return obs


def rebase_observation(sample, tick, last_observation_id):
    """Present a measured sample at a decision with the shared committed receipt."""
    return stamp_observation({**sample, "tick": tick, "last_observation_id": last_observation_id})


def validate_action(action, tick, horizon):
    """Check the entire requested chunk before any torque is applied."""
    if not isinstance(action, dict) or set(action) != {"rows"}:
        raise ValueError("action requires a rows field")
    rows = action["rows"]
    limit = min(CHUNK - tick % CHUNK, horizon - tick)
    if not isinstance(rows, list) or not 1 <= len(rows) <= limit:
        raise ValueError(f"tick {tick} accepts 1 to {limit} rows")
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != {"codes"}:
            raise ValueError(f"row {i} requires a codes field")
        codes = row["codes"]
        if (not isinstance(codes, list) or len(codes) != CODE_DIM
                or any(type(c) is not int or not 0 <= c <= 31 for c in codes)):
            raise ValueError(f"row {i} requires 64 integer codes in [0,31]")
    return rows


class LiveEpisode:
    """Bind a reset native session and a route to the solver wire."""

    def __init__(self, session, route):
        self.session = session
        self.route = route
        state = session.state
        if state["tick"] != 0:
            raise ValueError("create a LiveEpisode at reset")
        start = state["observation"]
        self.start_xy = np.array(start["base_pos"][:2], dtype=float)
        self.start_heading = float(start["heading_rad"])
        self.last_observation_id = None
        self.score = PhysicalScore(route.name, route.distance_m, route.heading_delta_rad,
                                   state["metrics"], start["dq"], session.horizon)

    def target(self, route):
        direction = np.array([math.cos(self.start_heading), math.sin(self.start_heading)])
        return self.start_xy + route.distance_m * direction, self.start_heading + route.heading_delta_rad

    def observe(self, other_route=None):
        """Sample the current native state with a route, without advancing it."""
        state = self.session.state
        route = self.route if other_route is None else other_route
        target_xy, target_heading = self.target(route)
        pro = state["observation"]
        goal = self.session.relative_goal(target_xy)
        command = {
            "route": route.name,
            "target_forward_m": goal["forward_m"],
            "target_left_m": goal["left_m"],
            "heading_error_rad": target_heading - float(pro["heading_rad"]),
        }
        return stamp_observation({
            "protocol": "g1-sonic-locomotion/1",
            "tick": state["tick"],
            "horizon": self.session.horizon,
            "goal": route.descriptor(),
            "command": command,
            "observation": {k: np.asarray(pro[k]).tolist() for k in PROPRIO},
            "decoder_history": np.asarray(self.session.decoder_history()).tolist(),
            "last_observation_id": self.last_observation_id,
        })

    def advance(self, action, observation_id, *, max_ticks=None):
        """Apply the validated chunk through its first terminal row or debug cutoff."""
        tick = self.session.tick
        if self.score.done:
            raise ValueError("episode has ended")
        rows = validate_action(action, tick, self.session.horizon)
        if max_ticks is not None:
            if type(max_ticks) is not int or max_ticks < 1:
                raise ValueError("max_ticks must be a positive integer")
            rows = rows[:max_ticks]
        for row in rows:
            codes = row["codes"]
            token = (np.asarray(codes, dtype=np.float32) - 16) / 16
            state = self.session.step(token)
            self.score.update_tick(codes, state["observation"]["dq"], self.session.last_substeps)
            if self.score.done:
                break
        self.last_observation_id = observation_id
        return self.session.tick - tick
