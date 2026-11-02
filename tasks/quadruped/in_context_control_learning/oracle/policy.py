"""Scripted visual reference, not an LLM result. Reads only the public observation."""
import base64
import copy
from collections import deque
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation
from scipy import ndimage

from coordinates import SIZE, pixel_to_world


def decode(obs, index=0):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][index]))))


def mask(im, color):
    r, g, b = (im[:, :, i].astype(float) for i in range(3))
    return {"red": (r > 100) & (r > 2*g) & (r > 2*b) & (np.abs(g-b) < 25),
            "blue": (b > 80) & (b > 2*r) & (b > 1.8*g),
            "green": (g > 100) & (g > 2*r) & (g > 2*b),
            "yellow": (r > 100) & (g > .55*r) & (g < 1.15*r) & (g > 1.4*b) & (b < .75*r),
            "orange": (r > 150) & (g > 45) & (g < .85*r) & (g > 1.8*b)}[color]


def location(im, color):
    selected = mask(im, color)
    if color == "red":
        # Photographic brick/rust can contain isolated red pixels. The robot's
        # saturated connected tracking sticker, not their global mean, is the pose cue.
        rgb = im.astype(float)
        selected &= (rgb[:, :, 0] > 3*rgb[:, :, 1]) & (rgb[:, :, 0] > 3*rgb[:, :, 2])
        labels, count = ndimage.label(selected)
        if count:
            weight = rgb[:, :, 0]-np.maximum(rgb[:, :, 1], rgb[:, :, 2])
            totals = ndimage.sum(weight, labels, np.arange(1, count+1))
            main = labels == int(np.argmax(totals)+1)
            selected &= binary_dilation(main, iterations=8)
    ys, xs = np.where(selected)
    if not len(xs):
        return None
    if color == "green":
        # Intensity-weighted subpixel centroid avoids a half-pixel (1.6 cm)
        # quantization error in the reconstructed object's initial position.
        roi = np.zeros(im.shape[:2], dtype=bool)
        roi[max(0, ys.min()-2):ys.max()+3, max(0, xs.min()-2):xs.max()+3] = True
        pixels = im.astype(float)
        weights = np.maximum(0, pixels[:, :, 1]-np.maximum(pixels[:, :, 0], pixels[:, :, 2]))*roi
        yy, xx = np.mgrid[:im.shape[0], :im.shape[1]]
        return np.array(pixel_to_world(float((xx*weights).sum()/weights.sum()),
                                       float((yy*weights).sum()/weights.sum())))
    return np.array(pixel_to_world(float(np.mean(xs)), float(np.mean(ys))))


def xyz(obs, color="red", fallback=None):
    xy = location(decode(obs), color)
    xz = location(decode(obs, 1), color)
    if fallback is None and (xy is None or xz is None):
        raise ValueError("robot not visible in both public cameras")
    # A gate/gantry can briefly hide the marker from one camera. Keep only the
    # unobserved coordinates from the last measurement, not hidden simulator state.
    return np.array([xy[0] if xy is not None else xz[0] if xz is not None else fallback[0],
                     xy[1] if xy is not None else fallback[1],
                     xz[1]+2 if xz is not None else fallback[2]])


def search_detection(im, kind):
    """Visual reporter shared by coverage-search and random-motion comparators."""
    if kind == "rescue":
        return location(im, "orange")
    r, g, b = (im[:, :, i].astype(float) for i in range(3))
    water = (b > 100) & (b > 1.1*g) & (b > 1.5*r)
    # The shifted bridge axis is estimated from observed asphalt, never from the
    # hidden seed. A deck row contains a narrow water gap between its two sides.
    asphalt = ((im.max(axis=2)-im.min(axis=2) < 35) & (im.mean(axis=2) > 28)
               & (im.mean(axis=2) < 175))
    rows = np.arange(112, 144)
    row = int(rows[np.argmax(asphalt[rows, 35:220].sum(axis=1))])
    columns = np.where(water[max(0, row-3):row+4].sum(axis=0) >= 5)[0]
    columns = columns[(columns > 35) & (columns < 220)]
    if not 5 <= len(columns) <= 22:
        return None
    # Once near the gap, both deck sides reveal its centreline. Taking the
    # brightest asphalt row biases the estimate toward a partially seen edge.
    left, right = max(0, columns.min()-5), min(SIZE-1, columns.max()+5)
    band = (~water[:, left] & ~water[:, right]
            & (im[:, left].mean(axis=1) > 20) & (im[:, right].mean(axis=1) > 20))
    band[:85], band[171:] = False, False
    labels, count = ndimage.label(band)
    groups = [np.where(labels == label)[0] for label in range(1, count+1)]
    groups = [ys for ys in groups if len(ys) >= 15 and ys.min() <= 140 and ys.max() >= 112]
    centre = float(max(groups, key=len).mean()) if groups else 127.5
    return np.array(pixel_to_world(columns.mean(), centre))


class Controller:
    def __init__(self, kind):
        self.kind, self.last, self.phase = kind, None, 0
        self.block = self.tray = None
        self.command = np.array([-1.9, -1.7, 1.])
        self.takeoff_xy = None
        self.last_pose = None

    def action(self, obs):
        p = xyz(obs, fallback=self.last_pose)
        self.last_pose = p.copy()
        if self.kind == "flight":
            target = location(decode(obs), "blue")
            if target is None:
                raise ValueError("goal not visible")
            gate = location(decode(obs), "yellow")
            if gate is None:
                raise ValueError("gate not visible")
            velocity = np.zeros(3) if self.last is None else (p-self.last)/.2
            self.last = p
            if self.takeoff_xy is None:
                self.takeoff_xy = p[:2].copy()
            goals = [np.r_[self.takeoff_xy, 1.3],
                     np.array([gate[0]-.7, gate[1], 1.3]),
                     np.array([gate[0]+.65, gate[1], 1.3]), np.r_[target, 1.3]]
            if self.phase < 3 and np.linalg.norm(p-goals[self.phase]) < .2:
                self.phase += 1
            goal = goals[self.phase]
            acceleration = np.clip(1.1*(goal[:2]-p[:2]) - 1.8*velocity[:2], -.7, .7)
            attitude = np.array([-acceleration[1], acceleration[0]])/9.81/.3
            thrust = (.82 * (9.81 + 2*(goal[2]-p[2])-2*velocity[2]))/8.2 - 1
            return np.clip([thrust, attitude[0], attitude[1], 0], -1, 1).tolist()
        if self.phase < 3 or self.block is None:
            observed_block = location(decode(obs), "green")
            # Partial occlusion by the gripper shifts the visible centroid. Retain
            # the unobstructed estimate unless the object has materially moved.
            if self.block is None or (observed_block is not None and
                                      np.linalg.norm(observed_block-self.block) > .15):
                self.block = observed_block
        observed_tray = location(decode(obs), "blue")
        if self.tray is None or (observed_tray is not None and
                                 np.linalg.norm(observed_tray-self.tray) > .15):
            self.tray = observed_tray
        if self.block is None or self.tray is None:
            raise ValueError("object or tray not visible")
        waypoints = [
            [*self.block, .9], [*self.block, .32], [*self.block, .30],
            [*self.block, 1.15], [*self.tray, 1.15], [*self.tray, .32],
            [*self.tray, .30], [self.tray[0], self.tray[1]-.5, .9],
        ]
        goal = np.array(waypoints[min(self.phase, 7)])
        grip = 1 if 2 <= self.phase <= 5 else -1
        error = goal - p
        if np.linalg.norm(error) < .075:
            self.phase += 1
        command_step = np.clip((goal-self.command)/.14, -1, 1)
        self.command += command_step*.14
        return np.r_[command_step, grip].tolist()


def navigation_button(occupied, source, target, radius, mapping):
    """Rank whole button pulses on a pixel-derived distance field."""
    grid = occupied.reshape(128, 2, 128, 2).max(axis=(1, 3))
    n = int(np.ceil(radius/.0625))
    yy, xx = np.mgrid[-n:n+1, -n:n+1]
    blocked = binary_dilation(grid, structure=xx*xx+yy*yy <= (radius/.0625)**2)
    # Unknown pixels outside the permitted workspace are not traversable shortcuts.
    blocked[:5, :] = blocked[-5:, :] = True
    blocked[:, :5] = blocked[:, -5:] = True
    def cell(p):
        return (int(np.clip((4-p[1])*16, 1, 126)),
                int(np.clip((p[0]+4)*16, 1, 126)))
    goal = cell(target)
    blocked[goal] = False
    clearance = ndimage.distance_transform_edt(~grid)
    inside_margin = blocked[cell(source)]
    distances = np.full((128, 128), 10000.)
    distances[goal] = 0
    queue = deque([goal])
    while queue:
        y, x = queue.popleft()
        d = distances[y, x]+1
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            ny, nx = y+dy, x+dx
            if 0 < ny < 127 and 0 < nx < 127 and not blocked[ny, nx] and d < distances[ny, nx]:
                distances[ny, nx] = d
                queue.append((ny, nx))
    def cost(button):
        delta = mapping[button]
        end = source+delta
        if np.any(np.abs(end) > 3.65):
            return 1e8
        if inside_margin:
            # A newly observed wall can place the current pose inside the
            # conservative planning margin. Back out instead of repeatedly
            # choosing a goal-directed pulse into the obstacle.
            return -100*clearance[cell(end)] + np.linalg.norm(end-target)
        collision = any(blocked[cell(source+delta*f)] for f in (.25, .5, .75, 1.))
        return (1000 if collision else 0) + distances[cell(end)] + .3*np.linalg.norm(end-target)
    return min(mapping, key=cost)


def solve(client, kind, mode="reference", artifacts=None):
    obs = client.observe()
    if kind in ("flight", "arm"):
        ctrl = Controller(kind)
        if kind == "arm" and artifacts:
            from reconstruction import create_scene, identify_response
            top = decode(obs)
            block, tray = location(top, "green"), location(top, "blue")
            if block is None or tray is None:
                raise ValueError("object or tray is not visually detectable")
            scene = create_scene(top, block, tray)
            if mode != "initial_image_only":
                obs, gains, diagnostics = identify_response(client, obs, scene)
                scene = create_scene(top, block, tray, gains)
                Path(artifacts, "identification.json").write_text(json.dumps(diagnostics, indent=2))
            Path(artifacts, "scene.xml").write_text(scene)
        while not obs["done"]:
            obs = client.act([0]*4 if mode == "noop" else ctrl.action(obs))
        return obs
    if kind in ("icl", "cross"):
        occupied = np.zeros((SIZE, SIZE), dtype=bool)
        mapping, old_phase, scout, max_radius = {}, -1, 0, 0.
        initial_budget = obs["steps_remaining"]
        scans = [np.array(p) for p in ((-1.2, -3.1), (-1.2, 3.1), (1.2, 3.1), (2.6, 1.7))]
        while not obs["done"]:
            phase = obs["phase"]
            if phase != old_phase:
                max_radius = 0.
                if kind == "cross" or mode == "clear_context":
                    mapping = {}
                if mode in ("clear_context", "no_scout"):
                    occupied[:] = False
                old_phase = phase
            im = decode(obs)
            p, goal = location(im, "red"), location(im, "blue")
            if p is None or goal is None:
                raise ValueError("visual robot/goal missing")
            red = mask(im, "red")
            ys, xs = np.where(red)
            max_radius = max(max_radius, max(np.ptp(xs), np.ptp(ys))/SIZE*4)
            # The red plate spans 64% of the nominal body footprint; its visible
            # width is not the collision envelope of the detailed robot mesh.
            radius = max_radius/.64 + (.13 if kind == "cross" and phase == 1 else .07)
            if kind in ("icl", "cross"):
                # Whole fixed-duration button pulses cannot align every lattice of
                # reachable positions with a narrow opening. Use a wider safe route.
                radius = max(.32, radius)
            if mode == "no_adaptation":
                mapping = {0: np.array([.21, 0]), 1: np.array([-.21, 0]),
                           2: np.array([0, .21]), 3: np.array([0, -.21])}
            if len(mapping) < 4:
                button = len(mapping)
                newer = client.act([button/3])
                new_p = location(decode(newer), "red")
                mapping[button] = (new_p-p) if new_p is not None else np.zeros(2)
                if len(mapping) == 4:
                    # Remove momentum carried over from the previous calibration
                    # pulse. The observed dominant axis identifies each button.
                    distance = float(np.median([np.max(np.abs(v)) for v in mapping.values()]))*1.15
                    for key, delta in mapping.items():
                        axis = int(np.argmax(np.abs(delta)))
                        vector = np.zeros(2)
                        vector[axis] = np.sign(delta[axis])*distance
                        mapping[key] = vector
                obs = newer
                continue
            # Use the wall's neutral-blue tint as well as brightness; a global
            # brightness cutoff either loses shaded wall tips or marks the floor.
            tint = im[:, :, 2].astype(float)-im[:, :, 0].astype(float)
            green_tint = im[:, :, 1].astype(float)-im[:, :, 0].astype(float)
            gray = ((tint > 20) & (tint < 80) & (im.mean(axis=2) > 100)
                    & (green_tint > .3*tint) & (green_tint < .7*tint))
            yy, xx = np.mgrid[:SIZE, :SIZE]
            gx, gy = (p[0]+4)*32, (4-p[1])*32
            body_pixels = (max_radius+.07)*32
            gray[(np.abs(xx-gx) < body_pixels) & (np.abs(yy-gy) < body_pixels)] = False
            occupied |= gray
            if kind == "cross" and phase == 0:
                if mode == "no_scout" and initial_budget-obs["steps_remaining"] >= 12:
                    obs = client.advance()
                    continue
                target = scans[scout]
                if np.linalg.norm(p-target) < .35:
                    scout += 1
                    if scout == len(scans):
                        obs = client.advance()
                        continue
                    target = scans[scout]
            else:
                target = goal
                if kind == "icl" and obs["completed_stages"] > phase:
                    obs = client.advance()
                    continue
            button = navigation_button(occupied, p, target, radius, mapping)
            obs = client.act([button/3])
        return obs
    # A generic coverage sweep, not privileged access to the target position.
    targets = [np.array([x, y]) for j, y in enumerate((-1.8, -.6, .6, 1.8))
               for x in ((-2.8, 2.8) if j % 2 == 0 else (2.8, -2.8))]
    index = 0
    while not obs["done"]:
        im = decode(obs)
        p = location(im, "red")
        if p is None:
            raise ValueError("robot missing")
        detected = search_detection(im, kind)
        if detected is not None:
            if np.linalg.norm(detected-p) < (.45 if kind == "bridge" else 1.0):
                obs = client.report(obs["frame_id"], detected.tolist())
                if obs["done"]:
                    break
            else:
                obs = client.act(np.clip((detected-p)*2, -1, 1).tolist())
                continue
        target = targets[min(index, len(targets)-1)]
        if np.linalg.norm(p-target) < .25:
            index += 1
        if index >= len(targets):
            break
        obs = client.act(np.clip((targets[index]-p)*3, -1, 1).tolist())
    return obs


class ControllerSession:
    """Repeated t values branch state; physical motion disambiguates prior candidates."""
    def __init__(self, kind):
        self.kind, self.t = kind, -1
        self.parents, self.candidates = [], []

    def act(self, t, obs):
        if t < self.t:
            raise ValueError("step index moved backwards without reset")
        history = self.candidates if t != self.t else self.parents
        p = xyz(obs, fallback=history[0][0] if history else None)
        if t != self.t:
            self.parents, self.candidates = self.candidates, []
            self.t = t
        if self.parents:
            # At a repeated step the simulator has not moved; do not integrate twice.
            state = min(self.parents, key=lambda row: np.linalg.norm(p-row[0]))[1]
            ctrl = copy.deepcopy(state)
        else:
            ctrl = Controller(self.kind)
        action = ctrl.action(obs)
        self.candidates.append((p, ctrl))
        return action


def controller_main(kind):
    import sys
    session = ControllerSession(kind)
    print("READY", flush=True)
    for line in sys.stdin:
        req = json.loads(line)
        if req["type"] == "reset":
            session = ControllerSession(kind)
            result = {"ok": True}
        elif req["type"] == "act":
            result = {"action": session.act(req["t"], req["obs"])}
        elif req["type"] == "close":
            print('{"ok":true}', flush=True)
            break
        else:
            result = {"error": "unknown operation"}
        print(json.dumps(result), flush=True)
