"""Single-camera geometric reference; no simulator imports or state channels.

AUTHOR-SUPPLIED design priors: calibrated onboard camera, metric square markers,
gate color order, upright gates, nominal rigid-body/servo dynamics, startup
height, goal-panel standoff, and left-turn route topology. These are not all
stated in the public instruction; this is not a fair uninformed baseline.
Landmark positions and headings are recovered from pixels, not a coordinate map.
"""
import base64
import copy
import hashlib
import io
import json
import math
import sys

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


COLORS = ("yellow", "green", "magenta", "cyan", "blue")
FOCAL = 256 / math.tan(math.radians(50))
TILT = math.radians(8)
CAMERA = np.array([[0, -math.sin(TILT), math.cos(TILT)],
                   [-1, 0, 0], [0, -math.cos(TILT), -math.sin(TILT)]])
OFFSET = np.array([.06, 0, .015])


def decode(obs, index=0):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][index]))).convert("RGB"))


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def color_mask(image, color):
    r, g, b = (image[:, :, i].astype(float) for i in range(3))
    return {
        "yellow": (r > 90) & (g > .52*r) & (g < 1.15*r) & (g > 1.8*b),
        "green": (g > 65) & (g > 1.7*r) & (g > 1.6*b),
        "magenta": (r > 70) & (b > 70) & (r > 1.8*g) & (b > 1.8*g),
        "cyan": (g > 70) & (b > 70) & (g > 1.8*r) & (b > 1.8*r),
        "blue": (b > 75) & (b > 1.8*r) & (b > 1.8*g),
    }[color]


def corners(image, color):
    """Associate achromatic connected components with a colored entry rim."""
    mask = color_mask(image, color)
    if mask.sum() < 20:
        return None
    rgb = image.astype(float)
    white = (rgb.min(axis=2) > 195) & (np.ptp(rgb, axis=2) < 22)
    labels, count = ndimage.label(white)
    near = ndimage.maximum_filter(mask, size=13)
    points = []
    for i, sl in enumerate(ndimage.find_objects(labels), 1):
        if sl is None:
            continue
        component = labels[sl] == i
        area = component.sum()
        if area < 2 or area > 2500 or not (near[sl] & component).any():
            continue
        yy, xx = np.nonzero(component)
        x, y = xx.mean()+sl[1].start, yy.mean()+sl[0].start
        if min(x, y, 511-x, 511-y) < 2:
            continue
        points.append((x, y, area))
    if len(points) < 4:
        return None
    points = np.asarray(points)[:, :2]
    if len(points) > 4:
        yy, xx = np.nonzero(mask)
        box = np.array([[xx.min(), yy.min()], [xx.max(), yy.min()],
                        [xx.min(), yy.max()], [xx.max(), yy.max()]])
        selected = np.argmin(np.linalg.norm(box[:, None]-points[None], axis=2), axis=1)
        if len(set(selected)) != 4:
            return None
        points = points[selected]
    points = points[np.argsort(points[:, 1])]
    top, bottom = points[:2], points[2:]
    top, bottom = top[np.argsort(top[:, 0])], bottom[np.argsort(bottom[:, 0])]
    ordered = np.array([top[0], top[1], bottom[0], bottom[1]])
    if min(np.linalg.norm(ordered[0]-ordered[1]), np.linalg.norm(ordered[2]-ordered[3])) < 5:
        return None
    return ordered


def marker_points(index):
    half = .27 if index == 4 else .48
    face = -.020 if index == 4 else -.041
    return np.array([[face, half, half], [face, -half, half],
                     [face, half, -half], [face, -half, -half]])


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def marker_pose(pixels, index, attitude):
    """Fit an upright metric square; camera attitude is supplied by the observer."""
    model = marker_points(index)
    ray = np.c_[(pixels-255.5)/FOCAL, np.ones(4)]
    rows = []
    for (y, z), (u, v, _) in zip(model[:, 1:], ray):
        rows.extend([[y, z, 1, 0, 0, 0, -u*y, -u*z, -u],
                     [0, 0, 0, y, z, 1, -v*y, -v*z, -v]])
    _, _, vh = np.linalg.svd(rows)
    hom = vh[-1].reshape(3, 3)
    if hom[2, 2] < 0:
        hom = -hom
    scale = 2/(np.linalg.norm(hom[:, 0])+np.linalg.norm(hom[:, 1]))
    yaxis, zaxis, trans = (hom[:, i]*scale for i in range(3))
    normal = np.cross(yaxis, zaxis)
    basis = attitude @ CAMERA
    n = basis @ normal
    heading = math.atan2(n[1], n[0])
    initial = np.r_[basis @ (trans-model[0, 0]*normal)+attitude@OFFSET, heading]

    def residual(x):
        xyz = (model @ rz(x[3]).T + x[:3] - attitude@OFFSET) @ basis
        uv = xyz[:, :2] / np.maximum(xyz[:, 2:], .01)*FOCAL+255.5
        return (uv-pixels).ravel()

    fit = least_squares(residual, initial, max_nfev=35, ftol=1e-7)
    if np.linalg.norm(fit.fun) > 4 or np.linalg.norm(fit.x[:3]) > 7:
        return None
    return fit.x[:3], wrap(fit.x[3]), float(np.linalg.norm(fit.fun))


class Controller:
    def __init__(self, kind="flight"):
        self.p = np.array([0., 0., .055])
        self.v = np.zeros(3)
        self.rotation = np.eye(3)
        self.omega = np.zeros(3)
        self.landmarks = {}
        self.index = 0
        self.mode = "takeoff"
        self.steps = 0
        self.last_action = None
        self.last_seen = {}
        self.exit = None
        self.heading = 0.
        self.innovation = 0.
        self.search_start = 0
        self.thumbnail = None

    def predict(self):
        if self.last_action is None:
            return
        action = np.asarray(self.last_action)
        desired = action[1:]*[.3, .3, np.pi]
        inertia = np.array([2.3951e-5, 2.3951e-5, 3.2347e-5])
        dt = .005
        for _ in range(40):
            angles = Rotation.from_matrix(self.rotation).as_euler("xyz")
            torque = np.clip(inertia*(100*wrap(desired-angles)-20*self.omega), -.002, .002)
            angular = (torque-np.cross(self.omega, inertia*self.omega))/inertia
            accel = self.rotation[:, 2]*(action[0]+1)*.175/.027 - [0, 0, 9.81]
            accel -= .08*self.v + .08*np.linalg.norm(self.v)*self.v
            self.p += self.v*dt + .5*accel*dt*dt
            self.v += accel*dt
            self.rotation = self.rotation @ Rotation.from_rotvec((self.omega+.5*angular*dt)*dt).as_matrix()
            self.omega += angular*dt
            if self.p[2] < .055:
                self.p[2], self.v[2] = .055, max(0., self.v[2])

    def observe(self, image):
        self.innovation = 0.
        observations = {}
        for index in range(5):
            pixels = corners(image, COLORS[index])
            if pixels is None or self.steps < 4:
                continue
            pose = marker_pose(pixels, index, self.rotation)
            if pose is not None:
                observations[index] = pose
        corrections = []
        for index, (relative, heading, quality) in observations.items():
            if index in self.landmarks:
                center, old_heading = self.landmarks[index]
                corrections.append((np.linalg.norm(relative), center-self.p-relative))
        if corrections:
            _, correction = min(corrections, key=lambda row: row[0])
            self.innovation = float(np.linalg.norm(correction))
            if self.innovation < .7:
                self.p += .6*correction
                self.v += .25*correction/.2
        for index, (relative, heading, quality) in observations.items():
            if index not in self.landmarks:
                self.landmarks[index] = (self.p+relative, heading)
            else:
                center, old_heading = self.landmarks[index]
                self.landmarks[index] = (center, old_heading+.15*wrap(heading-old_heading))
            self.last_seen[index] = self.steps
        return observations

    def continuation_error(self, previous):
        """Rotation-compensated photometric evidence for a repeated-step branch.

        Landmark-only scores are ambiguous while a new gate is first acquired.
        Static scene pixels also distinguish competing image/action histories.
        """
        if previous.thumbnail is None:
            return self.innovation
        yy, xx = np.mgrid[:64, :64]
        rays = np.stack(((xx-31.5)/(FOCAL/8), (yy-31.5)/(FOCAL/8), np.ones_like(xx)), axis=-1)
        transform = (self.rotation@CAMERA).T @ (previous.rotation@CAMERA)
        rays = rays @ transform
        uv = rays[:, :, :2]/np.maximum(rays[:, :, 2:], .001)*(FOCAL/8)+31.5
        valid = (rays[:, :, 2] > .1) & (uv.min(axis=2) >= 0) & (uv.max(axis=2) <= 63)
        if valid.sum() < 300:
            return 100.
        warped = np.stack([ndimage.map_coordinates(previous.thumbnail[:, :, i],
                            [uv[:, :, 1], uv[:, :, 0]], order=1, mode="nearest") for i in range(3)], axis=-1)
        return float(np.abs(warped-self.thumbnail)[valid].mean()) + 12*min(2., self.innovation)

    def target(self, image):
        yaw = Rotation.from_matrix(self.rotation).as_euler("xyz")[2]
        if self.mode == "takeoff":
            height = self.landmarks[0][0][2] if 0 in self.landmarks else 1.3
            goal = np.array([0., 0., height])
            if self.steps >= 12 and abs(self.p[2]-height) < .16 and 0 in self.landmarks:
                self.mode = "entry"
            return goal, 0.
        if self.index not in self.landmarks:
            rows, cols = np.nonzero(color_mask(image, COLORS[self.index]))
            if len(rows) > 15:
                self.heading = wrap(yaw-math.atan((cols.mean()-255.5)/FOCAL))
                ray = self.rotation @ CAMERA @ np.array([(cols.mean()-255.5)/FOCAL,
                                                         (rows.mean()-255.5)/FOCAL, 1.])
                self.exit[2] = self.p[2]+np.clip(ray[2], -.3, .3)
            else:
                if self.steps-self.search_start > 15:
                    self.heading = wrap(self.heading+.045)
            return self.exit.copy(), self.heading
        center, heading = self.landmarks[self.index]
        normal = np.array([math.cos(heading), math.sin(heading), 0.])
        if self.index == 4:
            goal = center-.65*normal
            return goal, math.atan2(center[1]-self.p[1], center[0]-self.p[0])
        if self.mode == "acquire":
            self.mode = "entry"
        if self.mode == "entry":
            goal = center-.85*normal
            if np.linalg.norm(self.p-goal) < .16 and np.linalg.norm(self.v) < .4:
                self.mode = "cross"
            return goal, math.atan2(center[1]-self.p[1], center[0]-self.p[0])
        goal = center+.65*normal
        if np.dot(self.p-center, normal) > .43:
            self.index += 1
            self.mode = "acquire"
            self.exit = center+1.7*normal
            self.heading = wrap(heading+(.4 if self.index < 4 else -.45))
            self.search_start = self.steps
        return goal, heading

    def action(self, obs):
        self.predict()
        image = decode(obs)
        self.observe(image)
        goal, heading = self.target(image)
        error = goal-self.p
        desired_velocity = 1.5*error[:2]
        desired_velocity *= min(1., .8/max(np.linalg.norm(desired_velocity), 1e-6))
        acceleration = 2.6*(desired_velocity-self.v[:2])
        acceleration *= min(1., 1.5/max(np.linalg.norm(acceleration), 1e-6))
        yaw = Rotation.from_matrix(self.rotation).as_euler("xyz")[2]
        body_accel = rz(yaw).T @ np.r_[acceleration, 0.]
        roll, pitch = -body_accel[1]/9.81, body_accel[0]/9.81
        az = np.clip(4*error[2]-3*self.v[2], -3., 3.)
        thrust = .027*(9.81+az)/max(.85, self.rotation[2, 2])
        action = np.clip([thrust/.175-1, roll/.3, pitch/.3, wrap(heading)/np.pi], -1, 1).tolist()
        self.last_action = action
        self.thumbnail = np.asarray(Image.fromarray(image).resize((64, 64), Image.Resampling.BILINEAR), dtype=float)
        self.steps += 1
        return action


class ControllerSession:
    """Repeated-t probes branch from a snapshot, never from a previous probe."""
    def __init__(self, kind="flight"):
        self.t = -1
        self.parents = []
        self.candidates = []
        self.cache = {}

    def act(self, t, obs):
        if t < self.t:
            raise ValueError("step index moved backwards without reset")
        if t != self.t:
            self.parents, self.candidates = self.candidates, []
            self.cache = {}
            self.t = t
        fingerprint = hashlib.sha256(obs["images"][0].encode()).digest()
        if fingerprint in self.cache:
            return self.cache[fingerprint].copy()
        possibilities = []
        for parent in self.parents or [Controller()]:
            candidate = copy.deepcopy(parent)
            action = candidate.action(obs)
            possibilities.append((candidate.continuation_error(parent), candidate, action))
        _, controller, action = min(possibilities, key=lambda row: row[0])
        self.candidates.append(controller)
        self.cache[fingerprint] = action
        return action.copy()


def solve(client, kind="flight", artifacts=None):
    obs = client.observe()
    session = ControllerSession(kind)
    t = 0
    while not obs["done"]:
        obs = client.act(session.act(t, obs))
        t += 1
    return obs


def controller_main(kind="flight"):
    session = ControllerSession(kind)
    print("READY", flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        if request["type"] == "reset":
            session = ControllerSession(kind)
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
