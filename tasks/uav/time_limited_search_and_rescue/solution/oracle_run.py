"""ALE reference: uses only public camera frames and the published controls."""
import base64
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/opt/visual_robotics/client")

LANES = (-2.7, -.9, .9, 2.7)
PASSAGES = ((.92, .75), (-.92, 2.20))
PACE = np.array([.32, .32, .19])
CAMERA_SCALE = 3.5/256
PAD_Y = -5.25
TRAVEL = 1.16


def down(observation):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(observation["images"][0]))),
                      dtype=np.int16)


class Pilot:
    def __init__(self, client):
        self.client = client
        self.command = np.array([0., PAD_Y, 1.12])
        self.obs = client.observe()

    def act(self, action):
        if not self.obs["done"]:
            self.obs = self.client.act(action)
            self.command += PACE*action[:3]

    def go(self, x, y, z=TRAVEL, settle=0):
        target = np.array([x, y, z])
        while not self.obs["done"] and np.max(np.abs(target-self.command)) > 1e-7:
            self.act([*np.clip((target-self.command)/PACE, -1., 1.).tolist(), 0.])
        for _ in range(settle):
            self.act([0., 0., 0., 0.])

    def grip(self, signal):
        self.act([0., 0., 0., signal])

    def scan(self, x, y):
        target = np.array([x, y, TRAVEL])
        while not self.obs["done"]:
            if find_pod(self.obs, self.command) is not None:
                self.go(*self.command, settle=2)
                pod = find_pod(self.obs, self.command)
                if pod is not None:
                    return pod
            error = target-self.command
            if np.max(np.abs(error)) <= 1e-7:
                self.go(*target, settle=2)
                return find_pod(self.obs, self.command)
            self.act([*np.clip(error/PACE, -1., 1.).tolist(), 0.])
        return None


def forecast(observation, height=False):
    rgb = down(observation)
    # Read the board directly under the UAV; the other board can be at an edge.
    rgb = rgb[113:173]
    r, g, b = rgb.transpose(2, 0, 1)
    rows, columns = np.where((g > 105) & (g > r+65) & (b > r+28))
    if rows.size < 120:
        raise RuntimeError("green forecast tile not in view")
    x = (columns.mean()+.5-128)*CAMERA_SCALE
    tiles = [-.30, .30] if height else [-.72, -.24, .24, .72]
    return int(np.argmin(np.abs(x-np.array(tiles))))


def find_pod(observation, command):
    rgb = down(observation)
    r, g, b = rgb.transpose(2, 0, 1)
    # Orange shell, not the yellow mannequin or the red/yellow corridor.
    rows, columns = np.where((r > 150) & (g > 60) & (g < .65*r)
                             & (b < 48) & (r > 1.7*g))
    if rows.size < 260:
        return None
    if (rows.min() < 4 or columns.min() < 4 or rows.max() >= 252 or columns.max() >= 252
            or np.ptp(rows) > 60 or np.ptp(columns) > 50):
        return None
    u, v = (columns.min()+columns.max())/2, (rows.min()+rows.max())/2
    return command[:2]+[(u+.5-128)*CAMERA_SCALE, (128-v-.5)*CAMERA_SCALE+.20]


def run(client, lane=None, height=None, target=None, locate_only=False):
    p = Pilot(client)
    p.go(0., -3.28, settle=2)
    if lane is None:
        lane = forecast(p.obs)
    p.go(1., -2.12, settle=2)
    if height is None:
        height = forecast(p.obs, height=True)
    x = LANES[lane]
    hx, hz = PASSAGES[height]
    p.go(x, -1.12)
    p.go(x, 1.12)
    p.go(hx, 1.47, hz, settle=1)
    p.go(hx, 3.78, hz)
    p.go(hx, 3.78)
    if p.obs["done"]:
        return p.obs
    if target is None:
        # Read along the pass, settle before using a detection for precise alignment.
        side = -1 if hx < 0 else 1
        p.go(side*1.72, 4.68)
        for sx, sy in ((-side*1.72, 4.68), (-side*1.72, 6.73), (side*1.72, 6.73)):
            target = p.scan(sx, sy)
            if target is not None or p.obs["done"]:
                break
        if target is None and not p.obs["done"]:
            raise RuntimeError("no fully visible rescue sled")
    if p.obs["done"]:
        return p.obs
    px, py = map(float, target)
    p.go(px, py, settle=1)
    if locate_only:
        return p.obs
    p.go(px, py, .43, settle=2)
    p.grip(1.)
    p.go(px, py)
    p.go(hx, 3.78)
    p.go(hx, 3.78, hz, settle=1)
    p.go(hx, 1.47, hz)
    p.go(x, 1.1)
    p.go(x, -1.1)
    p.go(0., PAD_Y)
    p.go(0., PAD_Y, .43, settle=2)
    p.grip(-1.)
    for _ in range(3):
        p.grip(0.)
    return p.obs


if __name__ == "__main__":
    from client import Client

    result = run(Client())
    out = Path("/home/user/submission")
    for i, png in enumerate(result["images"]):
        (out/f"final-camera-{i}.png").write_bytes(base64.b64decode(png))
    print(json.dumps({k: v for k, v in result.items() if k != "images"}), flush=True)
