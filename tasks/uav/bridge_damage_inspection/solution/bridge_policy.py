"""Calibrated feasibility reference; RGB defects, privileged actuator calibration.

No seed or defect list is read. Exact response calibration is an oracle advantage,
so this is not a fair frontier-model baseline and says nothing about difficulty.
"""
import base64
import io
import json

import numpy as np
from PIL import Image


class CalibratedMotion:
    def __init__(self, client):
        self.client = client
        self.position = np.array([-2., -5.5])
        self.velocity = np.zeros(2)

    def go(self, target, observation):
        target = np.array(target, dtype=float)
        while np.linalg.norm(target-self.position) > .08 and not observation["done"]:
            if observation["steps_remaining"] == 0:
                break
            action = np.clip((target-self.position)*.55, -1., 1.)
            observation = self.client.act(action.tolist())
            for _ in range(50):
                self.velocity = (self.velocity + .35*(3*action-self.velocity))/1.05
                self.position += .01*self.velocity
        return observation


def aperture_in_view(observation):
    image = np.asarray(Image.open(io.BytesIO(base64.b64decode(observation["images"][0]))), dtype=float)
    center = image[181:203, 181:203]
    r, g, b = np.moveaxis(center, -1, 0)
    water = (b > r*1.15) & (g > r*1.1) & (b > 45)
    return float(water.mean()) > .30


def solve(client, kind="bridge", artifacts=None, memory="all"):
    obs = client.observe()
    motion = CalibratedMotion(client)
    evidence = []
    for bay in range(1, 21):
        sides = (-1, 1) if bay % 2 else (1, -1)
        for side in sides:
            point = [(bay-.5)*6., side*3.2]
            obs = motion.go(point, obs)
            if obs["done"] or obs["steps_remaining"] == 0:
                break
            if aperture_in_view(obs):
                evidence.append({"frame_id": obs["frame_id"], "point": point})
                if len(evidence) == 3:
                    break
        if len(evidence) == 3 or obs["done"] or obs["steps_remaining"] == 0:
            break
    if artifacts:
        (artifacts/"reference_evidence.json").write_text(json.dumps(evidence, indent=2)+"\n")
    if not obs["done"]:
        reports = evidence if memory == "all" else evidence[-1:]
        for report in reports:
            obs = client.report(**report)
        obs = client.advance()
    return obs
