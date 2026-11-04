#!/usr/bin/env python3
"""Reference controller using only camera pixels and robot proprioception."""
import base64
import io
import json
import math
import sys

import numpy as np
from PIL import Image


def locate_red_can(obs):
    image = np.asarray(Image.open(io.BytesIO(base64.b64decode(
        obs["external_rgb_png_base64"]))).convert("RGB"), dtype=np.int16)
    red, green, blue = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    mask = ((red >= 45) & (red - green >= 18) & (red - blue >= 16)
            & (red * 10 >= green * 15) & (red * 10 >= blue * 13))
    visited = np.zeros(mask.shape, dtype=bool)
    best = []
    height, width = mask.shape
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or visited[y, x]:
                continue
            stack, component = [(y, x)], []
            visited[y, x] = True
            while stack:
                cy, cx = stack.pop()
                component.append((cy, cx))
                for ny, nx in ((cy - 1, cx), (cy + 1, cx),
                               (cy, cx - 1), (cy, cx + 1)):
                    if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            if len(component) > len(best):
                best = component
    if len(best) < 12:
        return None
    pixels = np.asarray(best, dtype=float)
    v, u = pixels.mean(axis=0)
    camera = obs["external_camera"]
    origin = np.asarray(camera["origin"], dtype=float)
    basis = np.asarray(camera["right_down_forward_to_world"], dtype=float)
    focal = .5 * image.shape[0] / math.tan(math.radians(camera["fovy_deg"]) / 2)
    ray = basis @ np.array([(u - image.shape[1] / 2) / focal,
                            (v - image.shape[0] / 2) / focal, 1.])
    point = origin + ray * ((.873 - origin[2]) / ray[2])
    point[2] = .86
    return point


class Policy:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stage = "hover"
        self.stage_start = 0
        self.object_pos = None
        self.grasp_offset = None
        self.last_action = [0.] * 6 + [-1.]

    def switch(self, stage, t):
        self.stage = stage
        self.stage_start = t

    def step(self, t, obs):
        eef = np.asarray(obs["eef_pos"], dtype=float)
        if self.object_pos is None:
            self.object_pos = locate_red_can(obs)
        if self.object_pos is None:
            return self.last_action
        obj = self.object_pos
        age = t - self.stage_start

        def move(target, grip, speed=1.0):
            xyz = np.clip((np.asarray(target) - eef) / .035, -speed, speed)
            self.last_action = [float(v) for v in xyz] + [0., 0., 0., float(grip)]
            return self.last_action

        if self.stage == "hover":
            target = obj + [0., 0., .16]
            if np.linalg.norm(target - eef) < .025 or age > 30:
                self.switch("descend", t)
            return move(target, -1.)
        if self.stage == "descend":
            target = obj + [0., 0., .025]
            if np.linalg.norm(target - eef) < .015 or age > 30:
                self.switch("close", t)
            return move(target, -1., .9)
        if self.stage == "close":
            if age >= 6:
                self.grasp_offset = eef - obj
                self.switch("lift", t)
            return move(eef, 1.)
        if self.stage == "lift":
            if eef[2] > .94 or age > 25:
                self.switch("carry", t)
            return move(eef + [0., 0., .12], 1.)
        if self.stage == "carry":
            target = np.array([.1975, .4025, .95]) + self.grasp_offset
            if np.linalg.norm(eef[:2] - target[:2]) < .035 or age > 50:
                self.switch("lower", t)
            return move(target, 1.)
        if self.stage == "lower":
            target = np.array([.1975, .4025, .86]) + self.grasp_offset
            if (np.linalg.norm(eef[:2] - target[:2]) < .025
                    and eef[2] < target[2] + .035) or age > 25:
                self.switch("release", t)
            return move(target, 1., .7)
        if self.stage == "release":
            if age >= 5:
                self.switch("retreat", t)
            return move(eef, -1.)
        return move(eef + [0., 0., .12], -1.)


def main():
    policy = Policy()
    print("READY", flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get("type") == "reset":
                policy.reset()
                reply = {"ok": True}
            elif request.get("type") == "close":
                print(json.dumps({"ok": True}), flush=True)
                break
            elif request.get("type") == "act":
                reply = {"action": policy.step(int(request["t"]), request["obs"])}
            else:
                reply = {"error": "unknown request"}
            print(json.dumps(reply, separators=(",", ":")), flush=True)
        except Exception as exc:
            print("controller error: " + repr(exc), file=sys.stderr, flush=True)
            print(json.dumps({"action": policy.last_action}), flush=True)


if __name__ == "__main__":
    main()
