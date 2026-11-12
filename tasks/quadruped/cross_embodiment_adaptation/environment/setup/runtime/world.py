"""Trusted MuJoCo world of one cross-embodiment mission. Never agent-readable in a task.

A mission: a small scout maps an enclosed hall it sees only near itself, then the operator
switches to a large carrier (new button permutation and gain) that must reach the delivery
pad. The carrier sees only its immediate surroundings and stops at its first contact, so it
can arrive only along a route the scout has mapped: through the one opening of the partition
wide enough for it, around the crates.
"""
import base64
import io
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("LP_NUM_THREADS", "2")
import mujoco
import numpy as np
from PIL import Image

from geometry import SIZE, box, header
from scene_assets import decorate, sync_visuals
from pbr_renderer import Renderer

KIND = "cross"
FIELD = 3.8                      # inner faces of the hall's boundary walls: |x|, |y| = FIELD
WALL_HALF = .12                  # half thickness of the partition
LEG = .025                       # leg capsule radius, at the corners of the body box
SCOUT_HALF = .13                 # scout body box half side (collision square 0.31 m with the legs)
CARRIER_HALF = (.355, .405)      # carrier body box half side (collision square 0.76-0.86 m)
SCOUT_MASS = 1.0                 # body box mass, kg (the four legs add 0.08)
CARRIER_MASS = (5.0, 8.0)        # the carrier is heavy: its velocity responds with a 0.13-0.2 s lag
WIDE = (1.20, 1.35)              # the wide opening (the carrier fits)
NARROW = (.62, .66)              # the two narrow openings (only the scout fits)
SCOUT_ACTIONS = 200              # phase 0 budget
CARRIER_ACTIONS = 160            # phase 1 budget
MIN_SCOUT_ACTIONS = 12           # actions before the switch is allowed
VIEW = (1.4, .5)                 # view radius around the robot, per phase
GOAL_VIEW = .25                  # the delivery pad's beacon is always visible
GOAL_TOLERANCE = .3              # carrier centre within this of the pad centre
SPEED = (1.2, 1.0)               # m/s per unit gain, per phase
GAIN = ((.85, 1.15), (.7, .9))   # per-site response gain range, per phase
DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))
BUTTONS = (0., 1/3, 2/3, 1.)
SUBSTEPS = 20                    # 20 x 10 ms = 0.2 s per action


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode()


def _rect_gap(a, b):
    """Edge-to-edge distance between axis-aligned rectangles (cx, cy, hx, hy); 0 when they touch."""
    dx = max(0., abs(a[0]-b[0])-a[2]-b[2])
    dy = max(0., abs(a[1]-b[1])-a[3]-b[3])
    return math.hypot(dx, dy)


def _point_gap(rect, p):
    dx = max(0., abs(p[0]-rect[0])-rect[2])
    dy = max(0., abs(p[1]-rect[1])-rect[3])
    return math.hypot(dx, dy)


def occupancy(rects, inflate, resolution=.04):
    """Boolean grid (rows = y descending from +FIELD, cols = x ascending from -FIELD) of cells whose centre
    lies within `inflate` of an obstacle rectangle or of the boundary; used for the feasibility check."""
    n = int(round(2*FIELD/resolution))
    centres = -FIELD+(np.arange(n)+.5)*resolution
    xx, yy = np.meshgrid(centres, centres[::-1])
    blocked = (np.abs(xx) > FIELD-inflate) | (np.abs(yy) > FIELD-inflate)
    for cx, cy, hx, hy in rects:
        dx = np.maximum(0., np.abs(xx-cx)-hx)
        dy = np.maximum(0., np.abs(yy-cy)-hy)
        blocked |= dx*dx+dy*dy < inflate*inflate
    return blocked, resolution


def _reachable(blocked, resolution, start, goal):
    n = blocked.shape[0]

    def cell(p):
        return (int(np.clip((FIELD-p[1])/resolution, 0, n-1)), int(np.clip((p[0]+FIELD)/resolution, 0, n-1)))
    s, g = cell(start), cell(goal)
    if blocked[s] or blocked[g]:
        return False
    seen = np.zeros_like(blocked)
    seen[s] = True
    frontier = [s]
    while frontier:
        nxt = []
        for r, c in frontier:
            if (r, c) == g:
                return True
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r+dr, c+dc
                if 0 <= rr < n and 0 <= cc < n and not blocked[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    nxt.append((rr, cc))
        frontier = nxt
    return False


class Site:
    """The hall of one mission, drawn from its seed (deterministic)."""

    def __init__(self, seed):
        self.seed = int(seed)
        rng = np.random.default_rng([self.seed % (2**64), 0xC805])
        for _ in range(2000):
            if self._draw(rng):
                break
        else:  # pragma: no cover - the draw succeeds within a few attempts
            raise RuntimeError("no feasible site for this seed")
        self.look_seed = int(rng.integers(0, 2**62))

    def _draw(self, rng):
        self.wall_x = float(rng.uniform(-.45, .45))
        widths = [float(rng.uniform(*WIDE))] + [float(rng.uniform(*NARROW)) for _ in range(2)]
        openings = []
        for width in widths:
            for _ in range(200):
                centre = float(rng.uniform(-3.6+width/2, 3.6-width/2))
                lo, hi = centre-width/2, centre+width/2
                if all(lo >= b+.6 or hi <= a-.6 for a, b, _ in openings):
                    openings.append((lo, hi, width))
                    break
            else:
                return False
        self.wide = openings[0][:2]
        self.openings = sorted(openings)
        self.scout_start = np.array([-2.8, -1.8])+rng.uniform(-.12, .12, 2)
        self.carrier_start = np.array([-2.8, 1.8])+rng.uniform(-.12, .12, 2)
        self.goal = np.array([2.6, -1.7])+rng.uniform(-.15, .15, 2)
        self.carrier_half = float(rng.uniform(*CARRIER_HALF))
        self.carrier_mass = float(rng.uniform(*CARRIER_MASS))
        reach = self.carrier_half+LEG
        partition = (self.wall_x, 0., WALL_HALF, FIELD)
        crates = []
        for _ in range(int(rng.integers(2, 5))):
            for _ in range(300):
                hx, hy = (float(v) for v in rng.uniform(.2, .45, 2))
                cx = float(rng.uniform(-FIELD+1.2+hx, FIELD-1.2-hx))
                cy = float(rng.uniform(-FIELD+1.2+hy, FIELD-1.2-hy))
                rect = (cx, cy, hx, hy)
                if (_rect_gap(rect, partition) >= 1.2 and all(_rect_gap(rect, c) >= 1.2 for c in crates)
                        and _point_gap(rect, self.carrier_start) >= reach+.5
                        and _point_gap(rect, self.goal) >= reach+.5
                        and _point_gap(rect, self.scout_start) >= .6):
                    crates.append(rect)
                    break
        if len(crates) < 2:
            return False
        self.crates = crates
        self.partition = []
        cursor = -FIELD-.12
        for lo, hi, _ in self.openings + [(FIELD+.12, FIELD+.12, 0)]:
            self.partition.append((self.wall_x, (cursor+lo)/2, WALL_HALF, (lo-cursor)/2))
            cursor = hi
        rects = self.partition+self.crates
        blocked, res = occupancy(rects, reach+.08)
        if not _reachable(blocked, res, self.carrier_start, self.goal):
            return False
        blocked, res = occupancy(rects, SCOUT_HALF+LEG+.03)
        return _reachable(blocked, res, self.scout_start, self.goal)

    def boundary(self):
        t = .06
        return [(0., FIELD+t, FIELD+2*t, t), (0., -FIELD-t, FIELD+2*t, t),
                (FIELD+t, 0., t, FIELD+2*t), (-FIELD-t, 0., t, FIELD+2*t)]

    def obstacles(self):
        """Every collision rectangle (cx, cy, hx, hy): the partition's segments, the crates, the boundary."""
        return list(self.partition)+list(self.crates)+self.boundary()


class World:
    def __init__(self, seed, render=True):
        self.seed = int(seed)
        self.site = Site(self.seed)
        control = np.random.default_rng([self.seed % (2**64), 0xB077])
        self.mappings = [control.permutation(4).tolist() for _ in range(2)]
        self.gains = [float(control.uniform(*GAIN[k])) for k in range(2)]
        self.render_enabled = render
        self.phase = 0
        self.steps = [0, 0]
        self.done = self.success = self.contact_stop = False
        self.switched = False
        self.scout_contacts = 0
        self.bumper = False
        self.frame_id = 0
        self.renderer = None
        self._build()

    # --- model ------------------------------------------------------------- #
    @property
    def half(self):
        return SCOUT_HALF if self.phase == 0 else self.site.carrier_half

    def _build(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        site, rad = self.site, self.half
        mass = SCOUT_MASS if self.phase == 0 else site.carrier_mass
        xml = header()
        for i, (cx, cy, hx, hy) in enumerate(site.partition):
            xml += box(f"wall{i}", f"{cx} {cy} .35", f"{hx} {hy} .35", ".55 .6 .65 1")
        for i, (cx, cy, hx, hy) in enumerate(site.crates):
            xml += box(f"crate{i}", f"{cx} {cy} .3", f"{hx} {hy} .3", ".5 .45 .35 1")
        for i, (cx, cy, hx, hy) in enumerate(site.boundary()):
            xml += box(f"boundary{i}", f"{cx} {cy} .35", f"{hx} {hy} .35", ".55 .6 .65 1")
        gx, gy = site.goal
        xml += box("goal", f"{gx} {gy} .02", ".2 .2 .02", ".03 .12 1 1", False)
        xml += f"""<body name="agent" pos="0 0 .23">
          <joint name="x" type="slide" axis="1 0 0" damping="5"/>
          <joint name="y" type="slide" axis="0 1 0" damping="5"/>
          <geom name="robot" type="box" size="{rad} {rad} .06" mass="{mass}" rgba="1 .02 .02 1"/>"""
        for lx in (-rad, rad):
            for ly in (-rad, rad):
                xml += (f'<geom type="capsule" fromto="{lx} {ly} 0 {lx} {ly} -.15" size="{LEG}" '
                        f'mass=".02" rgba=".4 .45 .5 1"/>')
        xml += "</body></worldbody></mujoco>"
        xml = decorate(xml, KIND, rad, seed=site.look_seed)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.data.qpos[:2] = site.scout_start if self.phase == 0 else site.carrier_start
        mujoco.mj_forward(self.model, self.data)
        self._contacts = set()
        if self.render_enabled:
            self.renderer = Renderer(self.model, SIZE, SIZE)

    def position(self):
        # The slide joints are the base's x and y (qpos, not xpos: mj_step leaves xpos one substep behind).
        return self.data.qpos[:2].copy()

    def _touching(self):
        """Geoms other than the floor in contact with the robot, at the current configuration."""
        touching = set()
        for contact in self.data.contact[:self.data.ncon]:
            b1 = self.model.geom_bodyid[contact.geom1]
            b2 = self.model.geom_bodyid[contact.geom2]
            if (b1 == self.agent_id) != (b2 == self.agent_id):
                other = contact.geom2 if b1 == self.agent_id else contact.geom1
                if self.model.geom(other).name != "floor":
                    touching.add(int(other))
        return touching

    def _contact(self, touching):
        if touching:
            self.bumper = True
        if self.phase == 0:
            self.scout_contacts += len(touching-self._contacts)
            self._contacts = touching
        elif touching:
            # The carrier's safety stop: halted at its first contact, mission over.
            self.data.qvel[:] = 0
            self.contact_stop = True
        return self.contact_stop

    # --- observation ------------------------------------------------------- #
    def image(self):
        sync_visuals(self.model, self.data)
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera="top", scene_option=option)
        im = self.renderer.render().copy()
        xx = (np.arange(SIZE)+.5)/SIZE*8-4
        yy = 4-(np.arange(SIZE)+.5)/SIZE*8
        p, g = self.position(), self.site.goal
        visible = (xx[None, :]-p[0])**2+(yy[:, None]-p[1])**2 < VIEW[self.phase]**2
        visible |= (xx[None, :]-g[0])**2+(yy[:, None]-g[1])**2 < GOAL_VIEW**2
        im[~visible] = 10
        return im

    def observe(self):
        self.frame_id += 1
        obs = {"frame_id": self.frame_id, "phase": self.phase, "done": self.done,
               "steps_remaining": self.remaining(), "bumper": self.bumper,
               "contact_stop": self.contact_stop}
        if self.render_enabled:
            obs["images"] = [png(self.image())]
        return obs

    def remaining(self):
        return max(0, (SCOUT_ACTIONS, CARRIER_ACTIONS)[self.phase]-self.steps[self.phase])

    # --- actions ----------------------------------------------------------- #
    def act(self, action):
        if self.done:
            raise ValueError("the mission has ended")
        if not isinstance(action, list) or len(action) != 1 or isinstance(action[0], bool) \
                or not isinstance(action[0], (int, float)) or not math.isfinite(action[0]):
            raise ValueError("action must be a one-element list: [0], [1/3], [2/3] or [1]")
        index = next((i for i, b in enumerate(BUTTONS) if abs(action[0]-b) < 1e-6), None)
        if index is None:
            raise ValueError("buttons are 0, 1/3, 2/3 and 1")
        if self.remaining() == 0:
            raise ValueError("the scout's action budget is spent; call advance()" if self.phase == 0
                             else "the carrier's action budget is spent")
        self.steps[self.phase] += 1
        direction = np.array(DIRECTIONS[self.mappings[self.phase][index]], float)
        velocity = direction*self.gains[self.phase]*SPEED[self.phase]
        self.bumper = False
        for _ in range(SUBSTEPS):
            self.data.xfrc_applied[:] = 0
            self.data.qfrc_applied[:2] = 35*(velocity-self.data.qvel[:2])
            mujoco.mj_step(self.model, self.data)       # contacts of the configuration it started from
            if self._contact(self._touching()):
                break
        mujoco.mj_forward(self.model, self.data)        # and of the one it ended in
        if not self.contact_stop:
            self._contact(self._touching())
        if self.phase == 1:
            if not self.contact_stop and np.linalg.norm(self.position()-self.site.goal) < GOAL_TOLERANCE:
                self.success = True
            self.done = bool(self.success or self.contact_stop or self.remaining() == 0)
        return self.observe()

    def advance(self):
        if self.done or self.phase != 0:
            raise ValueError("the body switch happens once, from the scout to the carrier")
        if self.steps[0] < MIN_SCOUT_ACTIONS:
            raise ValueError(f"scout for at least {MIN_SCOUT_ACTIONS} actions before the switch")
        self.phase = 1
        self.switched = True
        self.bumper = False
        self._build()
        return self.observe()

    # --- records ----------------------------------------------------------- #
    def metrics(self):
        return {"success": float(self.success), "switched": float(self.switched),
                "scout_actions": self.steps[0], "carrier_actions": self.steps[1],
                "scout_contacts": self.scout_contacts, "contact_stop": float(self.contact_stop),
                "final_goal_distance": float(np.linalg.norm(self.position()-self.site.goal))
                if self.phase == 1 else -1.0}

    def truth(self):
        """The whole site and the robot's true state: answered only to the token-holding oracle."""
        s = self.site
        return {"phase": self.phase, "position": self.position().tolist(),
                "velocity": self.data.qvel[:2].tolist(),
                "buttons": [list(DIRECTIONS[m]) for m in self.mappings[self.phase]],
                "gain": self.gains[self.phase], "speed": SPEED[self.phase], "half": self.half, "leg": LEG,
                "mass": float(self.model.body_mass[self.agent_id]), "damping": float(self.model.dof_damping[0]),
                "drive": 35., "timestep": float(self.model.opt.timestep), "substeps": SUBSTEPS,
                "obstacles": [list(map(float, r)) for r in s.obstacles()],
                "wide_opening": list(s.wide), "wall_x": s.wall_x,
                "carrier_start": s.carrier_start.tolist(), "carrier_half": s.carrier_half,
                "goal": s.goal.tolist(), "steps": list(self.steps)}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
