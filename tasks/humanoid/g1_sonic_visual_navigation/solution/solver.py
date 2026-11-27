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
from vision import beacons, camera_arrays, gate_openings, horizontal_axes


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
    quiet_ticks: int = 0
    aligning: bool = False
    parked: bool = False
    gates: tuple = ()


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
                quiet_ticks=0,
                aligning=False,
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
        arrays = camera_arrays(obs["camera"])
        s = self.localize(s, beacons(obs["camera"], pro["gravity_body"], arrays=arrays))
        if s.leg == "approach":
            observed = gate_openings(*arrays, pro["gravity_body"], s.xy, s.yaw)
            gates = list(s.gates)
            for center in observed:
                if center[0] < s.xy[0] + 0.35:
                    continue
                nearest = min(
                    range(len(gates)),
                    key=lambda i: abs(gates[i][0] - center[0]),
                    default=None,
                )
                if nearest is None or abs(gates[nearest][0] - center[0]) > 0.50:
                    gates.append(center)
                else:
                    gates[nearest] = 0.85 * gates[nearest] + 0.15 * center
            s = replace(s, gates=tuple(sorted(gates, key=lambda g: g[0])))
        reverse = rotation(s.yaw).T
        forward, left = reverse @ (s.goal - s.xy) if s.goal is not None else (0.0, 0.0)
        speed = (reverse @ s.velocity)[0]
        yaw_error = wrap(s.desired_yaw - s.yaw)
        motion = "pivot" if s.leg == "pivot" else "walk"
        if s.goal is None:
            motion = "stand"
        elif s.leg == "approach" and s.gates:
            pending = [g for g in s.gates if g[0] + 0.31 > s.xy[0]]
            if pending or s.goal[0] - s.xy[0] > 0.85:
                # Keep the camera facing along the aisle and sidestep on the
                # walking cycle. Hold the current opening until the trailing
                # arm and foot have passed its plane.
                target = pending[0] if pending else s.goal
                forward, left = target - s.xy
                yaw_error = wrap(s.entry_yaw - s.yaw)
                motion = "gate"
            else:
                # Wait for several measured quiet samples before committing
                # the static reference. It then persists until the cue changes,
                # with a recovery margin for a genuine positional drift.
                distance = math.hypot(forward, left)
                held = s.parked
                if held and (distance > 0.244 or abs(yaw_error) > 0.170):
                    held = False
                quiet = (
                    distance < 0.178
                    and abs(yaw_error) < 0.104
                    and np.linalg.norm(s.velocity) < 0.13
                    and abs(s.yaw_rate) < 0.19
                )
                n = s.quiet_ticks + 1 if quiet else 0
                parked = held or n >= 3
                s = replace(s, aligning=False, quiet_ticks=n, parked=parked)
                motion = "stand" if parked else "dock"
        elif s.leg != "pivot":
            distance = math.hypot(forward, left)
            if s.parked and (distance > 0.235 or abs(yaw_error) > 0.165):
                s = replace(s, parked=False)
            if not s.parked:
                if distance < 0.21 and abs(yaw_error) > 0.14:
                    s = replace(s, aligning=True)
                elif distance > 0.24 or abs(yaw_error) < 0.09:
                    s = replace(s, aligning=False)
                if (
                    distance < 0.175
                    and abs(yaw_error) < 0.14
                    and not s.aligning
                    and np.linalg.norm(s.velocity) < 0.13
                ):
                    s = replace(s, parked=True)
            if s.parked:
                motion = "stand"
            elif s.aligning:
                # A short stance-turn corrects heading even as walking stride
                # shrinks. Hysteresis keeps it active through the correction.
                motion = "pivot"
        codes = self.gait.codes(
            s.tick - s.entered,
            motion,
            forward,
            left,
            yaw_error,
            speed,
            s.yaw_rate,
            s.velocity[1],
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
