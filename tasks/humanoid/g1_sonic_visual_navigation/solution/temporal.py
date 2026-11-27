"""Control a native G1 using local motion and measured space between sliding shutters."""

from collections import deque
from dataclasses import replace
import math

import numpy as np

from perception import shutters_from_rgbd
from route import RoutePolicy, wrap
from vision import beacons, camera_arrays, gate_openings


class TemporalPolicy:
    """Stage outside the barrier, observe it opening, then cross the selected aperture."""

    def __init__(self, encoder, mode="live"):
        if mode not in ("live", "snapshot"):
            raise ValueError("expected live or snapshot")
        self.mode = mode
        self.after = RoutePolicy(encoder)
        self.gait = self.after.gait
        self.reset()

    def reset(self):
        self.pose = None
        self.gate_x = None
        self.side = None
        self.ingress = {"left": 0.72, "right": -0.72}
        self.history = {side: deque(maxlen=35) for side in self.ingress}
        self.onsets = {side: None for side in self.ingress}
        self.downstream_gates = []
        self.stage = "survey"
        self.first = None
        self.cache = None
        self.debug = {}

    def _measure(self, packet, t):
        for side, samples in self.history.items():
            item = packet[side]
            if not item["visible"] or item["confidence"] < 0.45:
                continue
            if item["ingress_y"] is not None:
                self.ingress[side] = float(item["ingress_y"])
            c = float(item["clearance"])
            recent = [v for last, v in samples if 0.07 <= t - last <= 0.32]
            rising = (
                (0.24 if self.side is None else 0.08) < c < 0.88
                and recent
                and c - min(recent) > 0.075
            )
            if rising:
                # Locate onset within the measured rise using the published
                # one-second cosine travel. The motor's unknown phase never enters.
                travel = np.clip((c + 0.03) / 1.06, 0.0, 1.0)
                elapsed = math.acos(1.0 - 2.0 * travel) / math.pi
                estimate = t - elapsed
                last = self.onsets[side]
                if last is None or estimate - last > 3.0:
                    self.onsets[side] = estimate
                elif abs(estimate - last) < 0.45:
                    self.onsets[side] = 0.85 * last + 0.15 * estimate
            samples.append((t, c))

    def act(self, obs):
        query = obs["observation_id"]
        if self.cache is not None and self.cache[0] == query:
            return self.cache[1]
        if self.cache is not None and obs["last_observation_id"] != self.cache[0]:
            raise ValueError(
                "expected the next physically committed public observation"
            )
        if self.stage == "cleared":
            s, action = self.after.compute(obs, self.pose)
        else:
            s = self.after.integrate(obs, self.pose)
            gravity = obs["observation"]["gravity_body"]
            arrays = camera_arrays(obs["camera"])
            s = self.after.localize(s, beacons(obs["camera"], gravity, arrays=arrays))
            t = 0.02 * s.tick
            packet = (
                self.first
                if self.mode == "snapshot" and self.first is not None
                else shutters_from_rgbd(
                    obs["camera"], gravity, s.xy, s.yaw, barrier_hint=self.gate_x
                )
            )
            if self.first is None:
                self.first = packet
            readings = packet if self.mode == "live" else self.first
            if readings["barrier_x"] is not None and readings["confidence"] >= 0.65:
                estimate = readings["barrier_x"]
                if self.gate_x is None:
                    self.gate_x = estimate
                elif self.gate_x - s.xy[0] > 1.4 and abs(self.gate_x - estimate) < 0.25:
                    self.gate_x = 0.90 * self.gate_x + 0.1 * estimate
            if self.gate_x is not None:
                for center in gate_openings(*arrays, gravity, s.xy, s.yaw):
                    if center[0] < self.gate_x + 0.75 or center[0] < s.xy[0] + 0.35:
                        continue
                    if not self.downstream_gates:
                        self.downstream_gates = [center]
                    elif abs(center[0] - self.downstream_gates[0][0]) < 0.55:
                        self.downstream_gates[0] = (
                            0.85 * self.downstream_gates[0] + 0.15 * center
                        )
                if self.side is not None:
                    passed = np.array([self.gate_x, self.ingress[self.side]])
                    s = replace(s, gates=(passed, *self.downstream_gates))
            if self.mode == "live":
                self._measure(readings, t)
            if self.gate_x is None:
                # Keep the camera stationary until the cabinet plane is observable.
                target = s.xy
                motion = "stand"
            else:
                if self.side is None:
                    if self.mode == "snapshot":
                        self.side = max(
                            self.ingress, key=lambda k: readings[k]["clearance"] or 0.0
                        )
                    else:
                        rising = [
                            k
                            for k, v in self.onsets.items()
                            if v is not None and t - v < 0.95
                        ]
                        if rising:
                            self.side = max(rising, key=lambda k: self.onsets[k])
                y = self.ingress[self.side] if self.side else 0.0
                target = np.array(
                    [self.gate_x - (1.75 if self.side is None else 1.15), y]
                )
                motion = "stage"
                error = target - s.xy
                if self.stage != "cross":
                    self.stage = "stage" if self.side is not None else "survey"
                    if (
                        np.linalg.norm(error) < 0.086
                        and np.linalg.norm(s.velocity) < 0.14
                    ):
                        motion = "stand"
                    ready = (
                        self.side is not None
                        and abs(error[1]) < 0.16
                        and abs(error[0]) < 0.22
                    )
                    if ready:
                        self.stage = "wait"
                        onset = self.onsets[self.side]
                        # Allow the leading and trailing leg to traverse the
                        # door completely before it starts closing.
                        eta = (self.gate_x - s.xy[0] + 0.10) / 0.41 + 0.38
                        enough = (
                            onset is not None
                            and 0.10 < t - onset
                            and t - onset + eta < 5.15
                        )
                        if self.mode == "snapshot" or enough:
                            self.stage = "cross"
                if self.stage == "cross":
                    target = np.array([self.gate_x + 1.3, y])
                    motion = "gate"
                    if s.xy[0] > self.gate_x + 0.36:
                        self.stage = "cleared"
            forward, left = target - s.xy
            codes = self.gait.codes(
                s.tick,
                motion,
                forward,
                left,
                wrap(-s.yaw),
                s.velocity[0],
                s.yaw_rate,
                s.velocity[1],
            )
            action = {"rows": [{"codes": codes.tolist()}]}
            self.debug = dict(
                stage=self.stage,
                side=self.side,
                gate_x=self.gate_x,
                xy=s.xy.tolist(),
                onsets=self.onsets.copy(),
                clearance={k: readings[k]["clearance"] for k in self.ingress},
            )
        self.pose = s
        self.cache = (query, action)
        return action
