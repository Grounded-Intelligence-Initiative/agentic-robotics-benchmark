"""PRIVILEGED feasibility controller. Never stage this in participant setup.

Reads the private scene and integration state. It does not perform image-only
perception and is not evidence of participant difficulty. Every real transition
is an ordinary budgeted service operation; lookahead uses separate MjData.
"""
from dataclasses import replace
import math

import mujoco
import numpy as np

from world import ACTION_LIMIT, MAX_FRAMES, REQUEST_LIMIT, wrap


def stopped(data, response):
    return (np.linalg.norm(data.qvel[:2]) < .012 and abs(data.qvel[2]) < .018
            and abs(response.drive) < .012 and abs(response.turn) < .018)


class Controller:
    def __init__(self, world, call):
        self.w, self.call = world, call
        self.scratch = mujoco.MjData(world.model)
        self.buttons = {name: world.scene.mapping.index(name) for name in world.scene.mapping}

    def pulse(self, name, frames):
        return self.call("act", {"action": [self.buttons[name]/3], "frames": int(frames)})

    def settle(self):
        for _ in range(8):
            if self.w.done or stopped(self.w.data, self.w.response):
                return
            self.pulse("brake", MAX_FRAMES)
        raise RuntimeError("reference could not settle within 9.6 simulated seconds")

    def forecast(self, name, frames):
        mujoco.mj_copyData(self.scratch, self.w.model, self.w.data)
        response = replace(self.w.response)
        for _ in range(frames):
            self.w.integrate(self.scratch, response, self.buttons[name])
        for _ in range(800):
            self.w.integrate(self.scratch, response, self.buttons["brake"])
            if stopped(self.scratch, response):
                break
        return self.scratch.qpos.copy()

    def duration(self, name, target, turning):
        """Monotone pulse search including coast-to-stop, not constant displacement."""
        start = self.w.data.qpos.copy()
        sign = 1 if name == "turn_left" else -1
        forward = np.array([math.cos(start[2]), math.sin(start[2])])
        wanted = sign*(target-start[2]) if turning else float((target-start[:2])@forward)
        candidates = {}

        def evaluate(n):
            if n not in candidates:
                end = self.forecast(name, n)
                progress = sign*(end[2]-start[2]) if turning else float((end[:2]-start[:2])@forward)
                error = abs(end[2]-target) if turning else float(np.linalg.norm(end[:2]-target))
                candidates[n] = (error, progress)
            return candidates[n][1]

        lo, hi = 0, MAX_FRAMES
        evaluate(lo)
        evaluate(hi)
        while hi-lo > 1:
            mid = (lo+hi)//2
            if evaluate(mid) < wanted:
                lo = mid
            else:
                hi = mid
        frames = min(candidates, key=lambda n: (candidates[n][0], n))
        return frames, candidates[frames][0]

    def align(self, angle):
        for _ in range(16):
            if self.w.done:
                return
            error = wrap(angle-self.w.heading())
            if abs(error) < .035 and stopped(self.w.data, self.w.response):
                return
            name = "turn_left" if error > 0 else "turn_right"
            frames, remaining = self.duration(name, self.w.heading()+error, True)
            if frames == 0:
                self.pulse("brake", MAX_FRAMES)
            else:
                self.pulse(name, frames)
                if frames < MAX_FRAMES or remaining < .06:
                    self.settle()
        raise RuntimeError("reference alignment did not converge")

    def travel(self, target):
        for _ in range(40):
            if self.w.done:
                return
            delta = target-self.w.position()
            if np.linalg.norm(delta) < .065:
                self.settle()
                if np.linalg.norm(target-self.w.position()) < .07:
                    return
                delta = target-self.w.position()
            angle = math.atan2(delta[1], delta[0])
            if abs(wrap(angle-self.w.heading())) > .05:
                self.settle()
                delta = target-self.w.position()
                self.align(math.atan2(delta[1], delta[0]))
            frames, remaining = self.duration("forward", target, False)
            if frames == 0:
                self.pulse("brake", MAX_FRAMES)
            else:
                self.pulse("forward", frames)
                if frames < MAX_FRAMES or remaining < .07:
                    self.settle()
        raise RuntimeError("reference waypoint did not converge")

    def route(self):
        p, g = self.w.position(), self.w.goal()
        direction = 1 if g[0] > p[0] else -1
        walls = list(zip(self.w.scene.walls_x, self.w.scene.openings))
        if direction < 0:
            walls.reverse()
        route = []
        for x, openings in walls:
            if min(p[0], g[0]) < x < max(p[0], g[0]):
                y, _ = max(openings, key=lambda gap: gap[1])
                route.extend((np.array([x-direction*.47, y]), np.array([x+direction*.47, y])))
        return route+[g]

    def solve(self):
        while not self.w.done:
            for waypoint in self.route():
                self.travel(waypoint)
            self.align(self.w.scene.goal_headings[self.w.current])
            for _ in range(6):
                if self.w.delivered > self.w.current or self.w.done:
                    break
                self.pulse("brake", MAX_FRAMES)
            if self.w.delivered <= self.w.current:
                raise RuntimeError("reference failed to acquire the precision dock")
            if not self.w.done:
                self.call("advance", {})
        if self.w.delivered != 6:
            raise RuntimeError(f"reference exhausted {ACTION_LIMIT} presses before all deliveries")


class Session:
    """Author ledger/mirror; optionally send the very same requests to a live service."""
    def __init__(self, world, client=None):
        self.world, self.client, self.events = world, client, []

    def call(self, method, params):
        if len(self.events) >= REQUEST_LIMIT:
            raise RuntimeError("reference exhausted the real request budget")
        # No private socket protocol: the live service sees only these operations.
        live = self.client.request(method, **params) if self.client else None
        obs = getattr(self.world, method)(**params)
        if live is not None:
            for key, value in self.world.status().items():
                if live[key] != value:
                    raise RuntimeError(f"private mirror diverged from live service: {key}")
        self.events.append({"method": method, "params": params, "frame_id": obs["frame_id"],
                            "steps": self.world.steps, "delivered": self.world.delivered,
                            "state_fingerprint": self.world.state_fingerprint()})
        return obs
