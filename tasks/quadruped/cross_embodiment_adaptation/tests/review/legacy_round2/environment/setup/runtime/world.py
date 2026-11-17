"""Trusted MuJoCo world of one cross-embodiment mission. Never agent-readable in a task.

A mission: a small scout explores an enclosed hall and maps it in its own frame; then the
operator switches to a large, heavy carrier that must reach the delivery pad. Nothing reports
a position or a heading in the hall:

* each body carries its own camera, centred on the body and turning with it: the scout a mast
  camera 2.0 m above the floor looking straight down (2.8 m of floor across; a pinhole, so walls
  and crates lean away from the image centre and hide the floor behind them), the carrier a
  floor camera under its base (about 0.6 m across, the exact span unknown: the floor beneath it
  and nothing else);
* the leg odometer over-reads whenever a foot slips, and a slip also turns the body;
* the carrier is set down near the scout's start, somewhere within a 0.8 m radius, at a
  heading of its own;
* the concrete of the floor is laid differently on every site.

The carrier never sees an obstacle and stops at its first contact, so it can arrive only along
a route the scout has mapped, and only when it has found its own pose in the scout's map: the
floor the scout recorded is the one thing both cameras see.
"""
import base64
import io
import math
import os
import xml.etree.ElementTree as ET

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
WIDE = (1.50, 1.65)              # the wide opening (the carrier fits at any heading)
NARROW = (.62, .66)              # the two narrow openings (only the scout fits)
SCOUT_ACTIONS = 500              # phase 0 budget
CARRIER_ACTIONS = 240            # phase 1 budget
MIN_SCOUT_ACTIONS = 12           # actions before the switch is allowed
SCOUT_VIEW = 2.8                 # metres of floor across the scout's mast camera image (calibrated)
SCOUT_LENS = 2.0                 # height of the mast camera's lens above the floor (a pinhole looking straight down)
FLOOR_VIEW = (.56, .64)          # metres across the carrier's floor camera image (it rides lower under a heavier load)
DOCK_CLEAR = 1.5                 # nothing stands within this distance of the dock point
DOCK_RADIUS = .8                 # the carrier's centre is set down within this distance of the dock point
IN_THE_WAY = .45                 # one crate reaches at least this near the direct way through the wide opening
PAD_WALL = 1.0                   # the pad's centre is at least this far from every wall
PAD_CRATE = 1.15                 # and at least this far from the nearest point of a crate
ROUTE_RADIUS = .70               # a route exists whose centre line keeps this far from every obstacle
GOAL_TOLERANCE = .3              # carrier centre within this of the pad centre
SPEED = (1.2, 1.0)               # m/s per unit gain, per phase
GAIN = ((.85, 1.15), (.7, .9))   # per-site response gain range, per phase
DRIVE = 35.                      # velocity servo gain, N per m/s
SLIP_RATE = (.12, .25)           # per-site, per-body probability that a press slips
SLIP_LOSS = (.3, .8)             # fraction of the commanded velocity a slipping press loses
SLIP_TURN = (1.5, 6.)            # degrees the body turns during a slipping press (either way)
BASE_LOSS = .03                  # every press loses up to this fraction (never a turn)
ODOMETER_NOISE = .02             # the odometer's reading of a press is off by up to this fraction
FLOOR_PERIOD = (2.4, 3.75)       # the concrete's pattern repeats at this distance, in two directions of its own
FLOOR_NOISE = 5.                 # floor camera: sensor noise, grey levels (1 sigma)
FLOOR_VIGNETTE = (.25, .4)       # floor camera: brightness lost at the image corners
FLOOR_GAIN = (.85, 1.15)         # floor camera: exposure relative to the scout's camera
FLOOR_TINT = (.93, 1.07)         # floor camera: per-channel colour balance
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


def _segment_gap(rect, a, b):
    """Distance from the rectangle to the segment a-b (sampled every 5 cm)."""
    n = max(2, int(math.hypot(b[0]-a[0], b[1]-a[1])/.05)+1)
    return min(_point_gap(rect, (a[0]+(b[0]-a[0])*k/(n-1), a[1]+(b[1]-a[1])*k/(n-1))) for k in range(n))


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
        rng = np.random.default_rng([self.seed % (2**64), 0xC806])
        for _ in range(4000):
            if self._draw(rng):
                break
        else:  # pragma: no cover - the draw succeeds within a few attempts
            raise RuntimeError("no feasible site for this seed")
        self.look_seed = int(rng.integers(0, 2**62))
        self.sun_azimuth = float(rng.uniform(0, 2*math.pi))
        # how the concrete is laid: the pattern's direction, its period (metres) and its offset
        self.floor_lay = (float(rng.uniform(0, 2*math.pi)), float(rng.uniform(*FLOOR_PERIOD)),
                          [float(v) for v in rng.uniform(0, 1, 2)])
        self.floor_camera = {"span": float(rng.uniform(*FLOOR_VIEW)), "gain": float(rng.uniform(*FLOOR_GAIN)),
                             "tint": [float(v) for v in rng.uniform(*FLOOR_TINT, 3)],
                             "vignette": float(rng.uniform(*FLOOR_VIGNETTE))}

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
        # The dock: where both robots are set down, in the room on the -x side of the partition.
        self.dock = np.array([rng.uniform(-FIELD+DOCK_CLEAR, self.wall_x-WALL_HALF-DOCK_CLEAR),
                              rng.uniform(-FIELD+DOCK_CLEAR, FIELD-DOCK_CLEAR)])
        self.scout_start = self.dock.copy()
        radius, bearing = DOCK_RADIUS*math.sqrt(rng.uniform()), rng.uniform(0, 2*math.pi)
        self.carrier_start = self.dock+radius*np.array([math.cos(bearing), math.sin(bearing)])
        self.scout_yaw = float(rng.uniform(0, 2*math.pi))
        self.carrier_yaw = float(rng.uniform(0, 2*math.pi))
        # The delivery pad: in the other room.
        self.goal = np.array([rng.uniform(self.wall_x+WALL_HALF+PAD_WALL, FIELD-PAD_WALL),
                              rng.uniform(-FIELD+PAD_WALL, FIELD-PAD_WALL)])
        self.carrier_half = float(rng.uniform(*CARRIER_HALF))
        self.carrier_mass = float(rng.uniform(*CARRIER_MASS))
        partition = (self.wall_x, 0., WALL_HALF, FIELD)
        # The direct way: from the carrier's start to the front of the wide opening, through it, to the pad.
        middle = (self.wide[0]+self.wide[1])/2
        way = [tuple(self.carrier_start), (self.wall_x-.9, middle), (self.wall_x+.9, middle), tuple(self.goal)]

        def allowed(rect):
            return (_rect_gap(rect, partition) >= 1.2 and all(_rect_gap(rect, c) >= 1.2 for c in crates)
                    and _point_gap(rect, self.dock) >= DOCK_CLEAR and _point_gap(rect, self.goal) >= PAD_CRATE
                    and abs(rect[0]) <= FIELD-1.2-rect[2] and abs(rect[1]) <= FIELD-1.2-rect[3])
        crates = []
        # The hall is in use: one crate stands in that way (the carrier has to go round it) ...
        for _ in range(300):
            hx, hy = (float(v) for v in rng.uniform(.2, .45, 2))
            a, b = (way[0], way[1]) if rng.uniform() < .5 else (way[2], way[3])
            t = float(rng.uniform())
            cx = a[0]+(b[0]-a[0])*t+float(rng.uniform(-.3, .3))
            cy = a[1]+(b[1]-a[1])*t+float(rng.uniform(-.3, .3))
            rect = (cx, cy, hx, hy)
            if allowed(rect) and min(_segment_gap(rect, a, b) for a, b in zip(way, way[1:])) < IN_THE_WAY:
                crates.append(rect)
                break
        else:
            return False
        # ... and one to three more stand anywhere.
        for _ in range(int(rng.integers(1, 4))):
            for _ in range(300):
                hx, hy = (float(v) for v in rng.uniform(.2, .45, 2))
                cx = float(rng.uniform(-FIELD+1.2+hx, FIELD-1.2-hx))
                cy = float(rng.uniform(-FIELD+1.2+hy, FIELD-1.2-hy))
                rect = (cx, cy, hx, hy)
                if allowed(rect):
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
        # The carrier at any heading lies inside a circle of radius <= 0.60 m: a route for a disc of
        # ROUTE_RADIUS is a route for every carrier, with at least 0.10 m to spare.
        blocked, res = occupancy(rects, ROUTE_RADIUS)
        if not _reachable(blocked, res, self.carrier_start, self.goal):
            return False
        blocked, res = occupancy(rects, SCOUT_HALF*math.sqrt(2)+LEG+.03)
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
        control = np.random.default_rng([self.seed % (2**64), 0xB078])
        self.mappings = [control.permutation(4).tolist() for _ in range(2)]
        self.gains = [float(control.uniform(*GAIN[k])) for k in range(2)]
        self.slip_rates = [float(control.uniform(*SLIP_RATE)) for _ in range(2)]
        # One stream per body: the slips of a mission depend only on its seed and the press count.
        self._slip = [np.random.default_rng([self.seed % (2**64), 0x5119, k]) for k in range(2)]
        self._noise = np.random.default_rng([self.seed % (2**64), 0xCA3E])
        self.render_enabled = render
        self.phase = 0
        self.steps = [0, 0]
        self.done = self.success = self.contact_stop = False
        self.switched = False
        self.scout_contacts = 0
        self.slips = [0, 0]
        self.bumper = False
        self.frame_id = 0
        self.odometer = np.zeros(2)
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
        if self.phase == 0:
            # A pinhole on a mast: the floor plane fills the image exactly; what stands above the floor
            # appears displaced away from the image centre and hides the floor behind it.
            fovy = 2*math.degrees(math.atan(SCOUT_VIEW/2/SCOUT_LENS))
            camera = f'<camera name="body" pos="0 0 {SCOUT_LENS-.23}" fovy="{fovy:.6f}"/>'
        else:
            camera = (f'<camera name="body" pos="0 0 4" projection="orthographic" '
                      f'fovy="{site.floor_camera["span"]}"/>')
        xml = header()
        for i, (cx, cy, hx, hy) in enumerate(site.partition):
            xml += box(f"wall{i}", f"{cx} {cy} .35", f"{hx} {hy} .35", ".55 .6 .65 1")
        for i, (cx, cy, hx, hy) in enumerate(site.crates):
            xml += box(f"crate{i}", f"{cx} {cy} .3", f"{hx} {hy} .3", ".5 .45 .35 1")
        for i, (cx, cy, hx, hy) in enumerate(site.boundary()):
            xml += box(f"boundary{i}", f"{cx} {cy} .35", f"{hx} {hy} .35", ".55 .6 .65 1")
        gx, gy = site.goal
        xml += box("goal", f"{gx} {gy} .02", ".2 .2 .02", ".03 .12 1 1", False)
        # The base: x and y of the hall (qpos 0, 1) and the heading (qpos 2). The camera rides on the body
        # and turns with it: image right is the body's +x, image up its +y.
        xml += f"""<body name="agent" pos="0 0 .23">
          <joint name="x" type="slide" axis="1 0 0" damping="5"/>
          <joint name="y" type="slide" axis="0 1 0" damping="5"/>
          <joint name="yaw" type="hinge" axis="0 0 1" damping="0"/>
          {camera}
          <geom name="robot" type="box" size="{rad} {rad} .06" mass="{mass}" rgba="1 .02 .02 1"/>"""
        for lx in (-rad, rad):
            for ly in (-rad, rad):
                xml += (f'<geom type="capsule" fromto="{lx} {ly} 0 {lx} {ly} -.15" size="{LEG}" '
                        f'mass=".02" rgba=".4 .45 .5 1"/>')
        xml += "</body></worldbody></mujoco>"
        xml = self._dress(decorate(xml, KIND, rad, seed=site.look_seed))
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.data.qpos[:2] = site.scout_start if self.phase == 0 else site.carrier_start
        self.data.qpos[2] = site.scout_yaw if self.phase == 0 else site.carrier_yaw
        mujoco.mj_forward(self.model, self.data)
        self._contacts = set()
        self.odometer = np.zeros(2)
        if self.render_enabled:
            self.renderer = Renderer(self.model, SIZE, SIZE)

    def _dress(self, xml):
        """The robot's own geoms go to a render group of their own (the floor camera looks from under the
        base and never shows them); the daylight comes from a direction drawn for the site; and the concrete
        is laid differently on every site (the pattern's direction, its period and its offset), so that no
        floor seen on one site says anything about another."""
        root = ET.fromstring(xml)
        world = root.find("worldbody")
        for item in world.find("body[@name='agent']").iter("geom"):
            item.set("group", "1")
        sun = world.find("light[@name='daylight']")
        if sun is not None:
            lean = float(np.hypot(*[float(v) for v in sun.get("dir").split()[:2]]))
            a = self.site.sun_azimuth
            sun.set("dir", f"{lean*math.cos(a):.6g} {lean*math.sin(a):.6g} -1")
        ground = root.find("asset").find("mesh[@name='site_ground_textured']")
        if ground is not None:
            angle, period, shift = self.site.floor_lay
            c, s = math.cos(angle), math.sin(angle)
            vertices = np.array(ground.get("vertex").split(), float).reshape(-1, 3)
            u = (c*vertices[:, 0]+s*vertices[:, 1])/period+shift[0]
            v = (-s*vertices[:, 0]+c*vertices[:, 1])/period+shift[1]
            ground.set("texcoord", " ".join(f"{a:.6f} {b:.6f}" for a, b in zip(u, v)))
        return ET.tostring(root, encoding="unicode")

    def position(self):
        # The slide joints are the base's x and y (qpos, not xpos: mj_step leaves xpos one substep behind).
        return self.data.qpos[:2].copy()

    def yaw(self):
        return float(self.data.qpos[2])

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
        if self.phase == 1:
            option.geomgroup[1] = 0          # the floor camera sits under the base
        self.renderer.update_scene(self.data, camera="body", scene_option=option)
        im = self.renderer.render().copy()
        if self.phase == 1:
            im = self._floor_camera(im)
        return im

    def _floor_camera(self, im):
        """The carrier's floor camera is another sensor than the scout's: its own exposure and colour
        balance, darker corners, sensor noise."""
        cam = self.site.floor_camera
        c = (np.arange(SIZE)+.5)/SIZE*2-1
        r2 = (c[None, :]**2+c[:, None]**2)/2
        out = im.astype(np.float32)*(cam["gain"]*(1-cam["vignette"]*r2))[..., None]*np.asarray(cam["tint"], np.float32)
        out += self._noise.normal(0, FLOOR_NOISE, out.shape)
        return np.clip(np.round(out), 0, 255).astype(np.uint8)

    def observe(self):
        self.frame_id += 1
        obs = {"frame_id": self.frame_id, "phase": self.phase, "done": self.done,
               "steps_remaining": self.remaining(), "bumper": self.bumper,
               "contact_stop": self.contact_stop,
               "odometer": [round(float(v), 4) for v in self.odometer]}
        if self.render_enabled:
            obs["images"] = [png(self.image())]
        return obs

    def remaining(self):
        return max(0, (SCOUT_ACTIONS, CARRIER_ACTIONS)[self.phase]-self.steps[self.phase])

    # --- actions ----------------------------------------------------------- #
    def _draw_slip(self):
        """(loss, turn in radians, the odometer's error) of the next press of the current body."""
        rng = self._slip[self.phase]
        loss = float(rng.uniform(0, BASE_LOSS))
        turn = 0.
        slipped = rng.uniform() < self.slip_rates[self.phase]
        extra, angle, sign = rng.uniform(*SLIP_LOSS), rng.uniform(*SLIP_TURN), rng.uniform() < .5
        if slipped:
            loss, turn = float(extra), math.radians(float(angle))*(1 if sign else -1)
            self.slips[self.phase] += 1
        return loss, turn, float(rng.uniform(-ODOMETER_NOISE, ODOMETER_NOISE))

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
        loss, turn, misread = self._draw_slip()
        # The button commands a velocity along one of the body's own axes. The stabilized base holds its
        # heading: only a slip turns it, steadily over the press; a contact does not.
        command = np.array(DIRECTIONS[self.mappings[self.phase][index]], float)*self.gains[self.phase]*SPEED[self.phase]
        start = float(self.data.qpos[2])
        step = self.model.opt.timestep
        stride = np.zeros(2)
        self.bumper = False
        for k in range(SUBSTEPS):
            heading = start+turn*k/SUBSTEPS
            self.data.qpos[2], self.data.qvel[2] = heading, turn/(SUBSTEPS*step)
            c, s = math.cos(heading), math.sin(heading)
            target = (1-loss)*np.array([c*command[0]-s*command[1], s*command[0]+c*command[1]])
            before = self.data.qpos[:2].copy()
            self.data.xfrc_applied[:] = 0
            self.data.qfrc_applied[:2] = DRIVE*(target-self.data.qvel[:2])
            mujoco.mj_step(self.model, self.data)       # contacts of the configuration it started from
            moved = self.data.qpos[:2]-before
            # The legs' stride in the body frame: what the body moved plus what the slipping foot lost.
            stride += np.array([c*moved[0]+s*moved[1], -s*moved[0]+c*moved[1]])
            stride += loss*command*step
            self.data.qpos[2], self.data.qvel[2] = start+turn*(k+1)/SUBSTEPS, 0.
            if self._contact(self._touching()):
                break
        mujoco.mj_forward(self.model, self.data)        # and of the one it ended in
        if not self.contact_stop:
            self._contact(self._touching())
        self.odometer = self.odometer+stride*(1+misread)
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
                "scout_slips": self.slips[0], "carrier_slips": self.slips[1],
                "final_goal_distance": float(np.linalg.norm(self.position()-self.site.goal))
                if self.phase == 1 else -1.0}

    def truth(self):
        """The whole site and the robot's true state: answered only to the token-holding oracle."""
        s = self.site
        return {"phase": self.phase, "position": self.position().tolist(), "yaw": self.yaw(),
                "velocity": self.data.qvel[:2].tolist(), "yaw_rate": float(self.data.qvel[2]),
                "buttons": [list(DIRECTIONS[m]) for m in self.mappings[self.phase]],
                "gain": self.gains[self.phase], "speed": SPEED[self.phase], "half": self.half, "leg": LEG,
                "mass": float(self.model.body_mass[self.agent_id]), "damping": float(self.model.dof_damping[0]),
                "drive": DRIVE, "timestep": float(self.model.opt.timestep), "substeps": SUBSTEPS,
                "slip_rate": self.slip_rates[self.phase], "slip_loss": list(SLIP_LOSS),
                "slip_turn_deg": list(SLIP_TURN), "base_loss": BASE_LOSS,
                "obstacles": [list(map(float, r)) for r in s.obstacles()], "field": FIELD,
                "wide_opening": list(s.wide), "wall_x": s.wall_x, "dock": s.dock.tolist(),
                "scout_yaw": s.scout_yaw, "carrier_start": s.carrier_start.tolist(),
                "carrier_yaw": s.carrier_yaw, "carrier_half": s.carrier_half,
                "scout_view": SCOUT_VIEW, "scout_lens": SCOUT_LENS, "floor_camera": dict(s.floor_camera),
                "goal": s.goal.tolist(), "goal_tolerance": GOAL_TOLERANCE, "route_radius": ROUTE_RADIUS,
                "odometer": self.odometer.tolist(), "steps": list(self.steps)}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
