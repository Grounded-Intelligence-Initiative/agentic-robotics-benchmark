"""Image-only scripted solution. This is not a frontier-model result."""
import base64
import copy
import io
import json
import sys

import numpy as np
from PIL import Image


def decode(obs, index=0):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][index]))))


def color_mask(image, color):
    r, g, b = (image[:, :, i].astype(float) for i in range(3))
    masks = {
        "red": (r > 80) & (r > 2*g) & (r > 2*b),
        "blue": (b > 80) & (b > 2*r) & (b > 2*g),
        "yellow": (r > 100) & (g > .55*r) & (g < 1.15*r) & (g > 2*b),
        "green": (g > 80) & (g > 2*r) & (g > 2*b),
        "magenta": (r > 80) & (b > 80) & (r > 2*g) & (b > 2*g) & (r < 1.5*b),
        "cyan": (g > 80) & (b > 80) & (g > 2*r) & (b > 2*r) & (b < 1.5*g),
    }
    return masks[color]


def location(image, color):
    r, g, b = (image[:, :, i].astype(float) for i in range(3))
    selected = color_mask(image, color)
    yy, xx = np.mgrid[:image.shape[0], :image.shape[1]]
    weights = np.maximum(0, r-np.maximum(g, b))*selected if color == "red" else selected
    if not weights.sum():
        return None
    u, v = (xx*weights).sum()/weights.sum(), (yy*weights).sum()/weights.sum()
    return np.array([(u+.5)*8/image.shape[1]-4, 4-(v+.5)*8/image.shape[0]])


def gate_direction(top, color, center):
    """Read a gate's orientation and the painted arrow from the public top image."""
    rows, columns = np.nonzero(color_mask(top, color))
    points = np.column_stack(((columns+.5)/64-4, 4-(rows+.5)/64))
    _, axes = np.linalg.eigh(np.cov(points.T))
    tangent = axes[:, -1]
    normal = np.array([-tangent[1], tangent[0]])
    yy, xx = np.mgrid[:top.shape[0], :top.shape[1]]
    x, y = (xx+.5)/64-4, 4-(yy+.5)/64
    rgb = top.astype(float)
    white = (rgb.min(axis=2) > 235) & (rgb.max(axis=2)-rgb.min(axis=2) < 18)
    white &= (x-center[0])**2+(y-center[1])**2 < .65**2
    if white.sum() < 12:
        raise ValueError("gate direction arrow not visible")
    offset = np.array([x[white].mean(), y[white].mean()])-center
    if np.dot(normal, offset) < 0:
        normal = -normal
    return np.r_[normal, 0.]


def xyz(obs, fallback=None):
    top, side = location(decode(obs), "red"), location(decode(obs, 1), "red")
    if fallback is None and (top is None or side is None):
        raise ValueError("tracking sticker not visible")
    return np.array([top[0] if top is not None else side[0] if side is not None else fallback[0],
                     top[1] if top is not None else fallback[1],
                     side[1]+2-.017 if side is not None else fallback[2]])


class Controller:
    def __init__(self, kind="flight"):
        self.last_pose = self.last = None
        self.velocity = np.zeros(3)
        self.goals = None
        self.phase = 0

    def action(self, obs):
        p = xyz(obs, self.last_pose)
        self.last_pose = p.copy()
        if self.goals is None:
            top, side = decode(obs), decode(obs, 1)
            gates, directions = [], []
            for color in ("yellow", "green", "magenta", "cyan"):
                xy, xz = location(top, color), location(side, color)
                if xy is None or xz is None:
                    raise ValueError("gate not visible in both cameras")
                gates.append(np.array([*xy, xz[1]+2]))
                directions.append(gate_direction(top, color, xy))
            target = location(top, "blue")
            if target is None:
                raise ValueError("goal not visible")
            self.goals = [np.r_[p[:2], gates[0][2]]]
            for gate, direction in zip(gates, directions):
                self.goals.extend([gate-.55*direction, gate+.55*direction])
            self.goals.append(np.r_[target, 1.3])
        if self.last is not None:
            self.velocity = .55*(p-self.last)/.2+.45*self.velocity
        self.last = p.copy()
        if self.phase < len(self.goals)-1 and np.linalg.norm(p-self.goals[self.phase]) < .12:
            self.phase += 1
        error = self.goals[self.phase]-p
        acceleration = np.clip(1.6*error[:2]-2.3*self.velocity[:2], -1.3, 1.3)
        attitude = np.array([-acceleration[1], acceleration[0]])/9.81/.3
        thrust = .027*(9.81+2.2*error[2]-2.3*self.velocity[2])/.175-1
        return np.clip([thrust, *attitude, 0], -1, 1).tolist()


class ControllerSession:
    """Branch repeated-t histories; retain the branch consistent with the next image."""
    def __init__(self, kind="flight"):
        self.t = -1
        self.parents, self.candidates = [], []

    def act(self, t, obs):
        if t < self.t:
            raise ValueError("step index moved backwards without reset")
        history = self.candidates if t != self.t else self.parents
        p = xyz(obs, history[0][0] if history else None)
        if t != self.t:
            self.parents, self.candidates = self.candidates, []
            self.t = t
        controller = copy.deepcopy(min(self.parents, key=lambda row: np.linalg.norm(p-row[0]))[1]) if self.parents else Controller()
        action = controller.action(obs)
        self.candidates.append((p, controller))
        return action


def solve(client, kind="flight", artifacts=None):
    obs = client.observe()
    session = ControllerSession()
    t = 0
    while not obs["done"]:
        obs = client.act(session.act(t, obs))
        t += 1
    return obs


def controller_main(kind="flight"):
    session = ControllerSession()
    print("READY", flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        if request["type"] == "reset":
            session = ControllerSession()
            result = {"ok": True}
        elif request["type"] == "act":
            result = {"action": session.act(request["t"], request["obs"])}
        elif request["type"] == "close":
            print('{"ok":true}', flush=True)
            return
        else:
            raise ValueError("unknown protocol request")
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    controller_main()
