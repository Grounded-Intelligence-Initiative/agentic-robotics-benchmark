"""Navigate the visual course using only the public egocentric RGB-D and local proprioception."""

from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

from gait import GaitPolicy
from sonic_sim.encoder import ReferenceEncoder
from vision import beacons, horizontal_axes


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Estimate:
    """All positions are in a private odometric frame with the robot's reset heading."""

    tick: int
    xy: np.ndarray
    yaw: float
    velocity: np.ndarray
    yaw_rate: float
    leg: str = "search"
    entered: int = 0
    entry_yaw: float = 0.0
    desired_yaw: float = 0.0
    goal: np.ndarray | None = None
    marker: np.ndarray | None = None
    turn_sign: int = 0
    parked: bool = False


class VisualPolicy:
    def __init__(self, encoder):
        self.gait = GaitPolicy(encoder)
        self.reset()

    def reset(self):
        self.parent_id = None
        self.parent = None
        self.candidates = {}

    @staticmethod
    def integrate(obs, parent):
        pro = obs["observation"]
        projected = np.asarray(horizontal_axes(pro["gravity_body"])) @ np.asarray(
            pro["velocity_body"]
        )
        rate = float(pro["yaw_rate_rad_s"])
        tick = obs["tick"]
        if parent is None:
            return Estimate(tick, np.zeros(2), 0.0, projected, rate)
        dt = (tick - parent.tick) * 0.02
        if dt <= 0.0:
            raise ValueError("observation tick must follow its committed predecessor")
        yaw = parent.yaw + 0.5 * (parent.yaw_rate + rate) * dt
        velocity = rotation(yaw) @ projected
        xy = parent.xy + 0.5 * (parent.velocity + velocity) * dt
        return replace(
            parent, tick=tick, xy=xy, yaw=yaw, velocity=velocity, yaw_rate=rate
        )

    @staticmethod
    def localize(s, visible):
        rotate = rotation(s.yaw)
        sign = int("red" in visible) - int("magenta" in visible)
        if "green" in visible:
            leg, color = "departure", "green"
        elif sign or "yellow" in visible or s.leg == "pivot":
            leg, color = "pivot", None
        elif "blue" in visible:
            leg, color = "approach", "blue"
        else:
            return s
        if leg != s.leg:
            s = replace(
                s,
                leg=leg,
                entered=s.tick,
                entry_yaw=s.yaw,
                desired_yaw=s.yaw,
                goal=s.xy.copy() if leg == "pivot" else None,
                marker=None,
                parked=False,
            )
        if leg == "pivot":
            if sign and s.turn_sign != sign:
                # The left/right light gives a usable initial yaw before the yellow target enters view.
                s = replace(
                    s,
                    turn_sign=sign,
                    desired_yaw=s.entry_yaw + sign * math.pi / 2,
                    parked=False,
                )
            if "yellow" in visible:
                ray = s.xy + rotate @ visible["yellow"] - s.goal
                measured = math.atan2(ray[1], ray[0])
                s = replace(
                    s, desired_yaw=s.desired_yaw + 0.55 * wrap(measured - s.desired_yaw)
                )
            return s
        ray = visible[color]
        if s.marker is None:
            # The walking beacon lies along the entry heading. Use its optical range
            # to initialize a fixed bearing reference in the reset-local frame.
            s = replace(
                s, marker=s.xy + rotation(s.entry_yaw)[:, 0] * np.linalg.norm(ray)
            )
        else:
            to_marker = s.marker - s.xy
            optical_yaw = math.atan2(to_marker[1], to_marker[0]) - math.atan2(
                ray[1], ray[0]
            )
            s = replace(s, yaw=s.yaw + 0.15 * wrap(optical_yaw - s.yaw))
        target = s.xy + rotation(s.yaw) @ ray - 1.50 * rotation(s.entry_yaw)[:, 0]
        if s.goal is not None:
            target = 0.35 * target + 0.65 * s.goal
        return replace(s, goal=target)

    def compute(self, obs, parent):
        pro = obs["observation"]
        s = self.integrate(obs, parent)
        s = self.localize(s, beacons(obs["camera"], pro["gravity_body"]))
        reverse = rotation(s.yaw).T
        forward, left = reverse @ (s.goal - s.xy) if s.goal is not None else (0.0, 0.0)
        speed = (reverse @ s.velocity)[0]
        yaw_error = wrap(s.desired_yaw - s.yaw)
        motion = "pivot" if s.leg == "pivot" else "walk"
        if s.goal is None:
            motion = "stand"
        elif s.leg != "pivot":
            distance = math.hypot(forward, left)
            if s.parked:
                if distance > 0.245 or abs(yaw_error) > 0.174:
                    s = replace(s, parked=False)
            elif distance < 0.155 and abs(yaw_error) < 0.145 and abs(speed) < 0.13:
                s = replace(s, parked=True)
            if s.parked:
                motion = "stand"
            elif distance < 0.205 and abs(yaw_error) > 0.17:
                # A shrinking walking stride cannot correct a residual heading error at the waypoint.
                motion = "pivot"
        codes = self.gait.codes(
            s.tick - s.entered, motion, forward, left, yaw_error, speed, s.yaw_rate
        )
        return s, {"rows": [{"codes": codes.tolist()}]}

    def act(self, obs):
        # Same-tick queries can contain independent visual alternatives. Only the receipt cited by
        # the next observation contributed to the real physical trajectory.
        key = obs["observation_id"]
        ancestry = obs["last_observation_id"]
        if ancestry != self.parent_id:
            self.parent = self.candidates[ancestry][0]
            self.parent_id = ancestry
            self.candidates.clear()
        if key not in self.candidates:
            self.candidates[key] = self.compute(obs, self.parent)
        return self.candidates[key][1]


def main():
    assets = Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))
    policy = VisualPolicy(ReferenceEncoder(assets / "policy/model_encoder.onnx"))
    print("READY", flush=True)
    for line in sys.stdin:
        kind = None
        try:
            request = json.loads(line)
            kind = request["type"]
            if kind == "act":
                obs = request["obs"]
                if (
                    obs["protocol"] != "g1-sonic-visual-navigation/1"
                    or request["t"] != obs["tick"]
                ):
                    raise ValueError(
                        "expected a synchronized visual navigation observation"
                    )
                result = {"action": policy.act(obs)}
            elif kind == "reset":
                policy.reset()
                result = {"ok": True}
            elif kind == "close":
                result = {"ok": True}
            else:
                raise ValueError("unknown request type")
        except (ValueError, KeyError, TypeError) as exc:
            result = {"error": str(exc)[:200]}
        print(json.dumps(result, allow_nan=False, separators=(",", ":")), flush=True)
        if kind == "close":
            break


if __name__ == "__main__":
    main()
