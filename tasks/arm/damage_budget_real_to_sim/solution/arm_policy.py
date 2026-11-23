"""Scripted image controller for the passive-pitch workcell."""
from dataclasses import dataclass, field
import copy
import math

import numpy as np
from scipy.ndimage import label

from policy import decode, xyz
from belt_drive import CassetteProbe, HOME, motor_matrix

SEAT = .205
TIP_Z = .375
TIP_PRESSURE = .60
GAUGE_Z = .416
GAUGE_HEIGHT_THRESHOLD = .106
PROOF_Z = .61
PRESSURES = {"light": .635, "heavy": .975}
NOMINAL_OFFSET = .36
PIVOT = .125
MARK = .57
RAMP = .015
SHEET_TOP = .006


def back_orientation(image, x, y):
    """The two ochre cheeks extend toward the open side of the turquoise rail."""
    r, g, b = image.astype(float).transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    cheeks = (r-g > 15) & (g-b > 15) & (r > 100)
    cheeks &= (abs(u-x) < .75) & (abs(v-y) < .53)
    if np.count_nonzero(cheeks) < 24:
        return None
    delta = y-np.median(v[cheeks])
    return (1 if delta > 0 else -1) if abs(delta) > .09 else None


def source_frame(obs, full=False):
    """The back of the shipping sleeve is a full-width turquoise crosspiece."""
    image = decode(obs)
    r, g, b = image.astype(float).transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    back = (np.minimum(g, b)-r > 55) & (u < -.55) & (u > -2.65) & (v > -1.25) & (v < .05)
    if np.count_nonzero(back) < 25 or np.ptp(u[back]) < (1.58 if full else 1.35):
        return None, None
    x, y = (u[back].min()+u[back].max())/2, np.median(v[back])
    sign = back_orientation(image, x, y)
    return (np.array([x, y-sign*.255]), sign) if sign is not None else (None, None)


def source_location(obs):
    return source_frame(obs)[0]


def release_pocket(obs, source, back):
    """Measure the two perpendicular violet fence faces in overhead metres."""
    r, g, b = decode(obs).astype(float).transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    dx, depth = u-source[0], back*(source[1]-v)
    violet = (b-g > 50) & (r-g > 30) & (b-r > 10)
    front = violet & (abs(dx) < .62) & (depth > 1.25) & (depth < 1.73)
    if front.sum() < 15 or np.ptp(dx[front]) < .46:
        return None
    elbow = float(np.median(depth[front]) - (.17+.09+.03))
    heel = violet & (abs(depth-elbow) < .19) & (abs(dx) > .99) & (abs(dx) < 1.55)
    if heel.sum() < 6 or np.ptp(depth[heel]) < .12:
        return None
    edge = float(np.median(dx[heel]))
    return source + [edge-math.copysign(.69+.09+.03, edge), -back*elbow]


def fiducials(obs, side=False, wide=False):
    im = decode(obs, int(side)).astype(float)
    r, g, b = im.transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4,
                       (6 if side else 4)-(np.arange(256)+.5)/32)
    xmin, xmax = (-3.18, .24) if wide else (-2.48, -.58)
    ymin, ymax = (-2.35, 1.12) if wide else (-1.03, -.24)
    roi = (u > xmin) & (u < xmax)
    roi &= ((v > -.025) & (v < .75) if side else (v > ymin) & (v < ymax))
    chroma = g-np.maximum(r, b)
    components, n = label((g > 90) & (chroma > 65) & roi)
    rows = []
    for k in range(1, n+1):
        ys, xs = np.where(components == k)
        if len(xs) < 4:
            continue
        # Include partially covered edge pixels for stable subpixel estimates.
        sy = slice(max(0, ys.min()-1), ys.max()+2)
        sx = slice(max(0, xs.min()-1), xs.max()+2)
        weight = np.maximum(0., chroma[sy, sx]-32)*roi[sy, sx]
        rows.append((len(xs), np.sum(u[sy, sx]*weight)/weight.sum(),
                     np.sum(v[sy, sx]*weight)/weight.sum()))
    pairs = []
    for n0, x0, y0 in rows:
        for n1, x1, y1 in rows:
            if .96 < x1-x0 < 1.25 and abs(y0-y1) < (.43 if side else .12):
                pairs.append((n0+n1, [[x0, y0], [x1, y1]]))
    if pairs:
        return np.array(max(pairs)[1])
    if side and wide:
        # One green end can be behind the bench obstacle. Extrapolate the
        # opposite end along the visible pink centerline of the rigid shell.
        weight = np.maximum(0., np.minimum(r-g-48, b-g-19)) * (r-b > 16) * roi
        for _, x, z in sorted(rows, reverse=True):
            select = (weight > 0) & (abs(u-x) < 1.12) & (abs(v-z) < .43)
            if not np.any(select):
                continue
            du, dz, w = u[select]-x, v[select]-z, weight[select]
            sign = 1 if w[du>0].sum() > w[du<0].sum() else -1
            use = sign*du > .04
            if use.sum() < 8 or w[use].sum() < .90*w.sum() or np.max(sign*du) < .40:
                continue
            du, dz, w = du[use], dz[use], w[use]
            slope = np.sum(w*du*dz)/np.sum(w*du*du)
            if abs(slope) > .41 or np.sqrt(np.average((dz-slope*du)**2, weights=w)) > .033:
                continue
            dx = sign*2*MARK/math.hypot(1., slope)
            return np.asarray(sorted([[x,z], [x+dx,z+slope*dx]]))
    return None


def tray_location(obs):
    im = decode(obs).astype(float)
    r, g, b = im.transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    weight = np.maximum(0., b - 1.35*np.maximum(r, g) - 25)
    weight *= (u > .21) & (u < 2.93) & (v > -.18) & (v < 1.54)
    if weight.sum() <= 0.:
        return None
    return np.array([np.sum(u*weight), np.sum(v*weight)])/weight.sum()


def lug_location(obs, tray):
    """Centroid of the magenta working face in the calibrated overhead view."""
    r, g, b = decode(obs).astype(float).transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    weight = np.maximum(0., np.minimum(r, b)-g-45)
    weight *= (abs(u-tray[0]) < .53) & (abs(v-tray[1]) > .53) & (abs(v-tray[1]) < 1.7)
    if np.count_nonzero(weight) < 5:
        return None
    return np.array([np.sum(u*weight), np.sum(v*weight)])/weight.sum()


def cradle_angle(obs, tx):
    """Slope of the two brass front flags in the calibrated side camera."""
    r,g,b = decode(obs,1).astype(float).transpose(2,0,1)
    x,z = np.meshgrid((np.arange(256)+.5)/32-4, 6-(np.arange(256)+.5)/32)
    pigment = np.maximum(0., np.minimum(r,g)-b-55)*(abs(r-g)<64)*(g>134)
    flags = []
    for sign in (-1,1):
        w = pigment*(abs(x-(tx+sign*.73)) < .135)*(z>.25)*(z<.76)
        if np.count_nonzero(w)<5:
            return None
        flags.append(np.array([(x*w).sum(),(z*w).sum()])/w.sum())
    return math.atan2(flags[0][1]-flags[1][1], flags[1][0]-flags[0][0])


def rider_bay(obs, tray):
    """The orange index is rigidly attached 0.258 m in front of the pocket."""
    r,g,b = decode(obs).astype(float).transpose(2,0,1)
    x,y = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    w = np.maximum(0., r-np.maximum(g,b)-55)*(g-b>14)
    w *= (abs(x-tray[0])<.86)*(abs(y-(tray[1]-2.128))<.115)
    if np.count_nonzero(w)<5:
        return None
    return np.array([(x*w).sum()/w.sum(),(y*w).sum()/w.sum()+.258])


def shell_observation(obs):
    # The moving bridge can mask one overhead mark. The side camera still
    # resolves the endpoints; either overhead mark supplies the shared Y.
    side = fiducials(obs, side=True, wide=True)
    if side is None:
        return None, None
    top = fiducials(obs, wide=True)
    if top is not None:
        return np.mean(top, axis=0), side[:, 1]
    im = decode(obs).astype(float)
    r, g, b = im.transpose(2, 0, 1)
    u, v = np.meshgrid((np.arange(256)+.5)/32-4, 4-(np.arange(256)+.5)/32)
    select = (np.min(abs(u[..., None]-side[:, 0]), axis=2) < .12) & (v > -2.35) & (v < 1.12)
    weight = np.maximum(0., g-np.maximum(r, b)-73)*select
    if weight.sum() <= 0.:
        return None, None
    return np.array([side[:, 0].mean(), (v*weight).sum()/weight.sum()]), side[:, 1]


def pitch_pose(obs, baseline, rest_z=.08, wide=False):
    marks = fiducials(obs, side=True, wide=wide)
    if marks is None or marks[1, 0]-marks[0, 0] < .8:
        return None
    z = marks[:, 1]-baseline+rest_z
    angle = math.atan2(z[0]-z[1], marks[1, 0]-marks[0, 0])
    return np.array([np.mean(marks[:, 0]), np.mean(z), angle])


def pulse(error, limit, tick, grip):
    if grip:
        limit = np.minimum(min(limit, RAMP*(tick+1)), np.sqrt(2*RAMP*np.abs(error)))
    return np.clip(error, -limit, limit)


@dataclass
class ArmController:
    cassette: tuple | None = None
    drive_probe: CassetteProbe = field(default_factory=CassetteProbe)
    fixed_offset: float | None = None
    fixed_pressure: float | None = None
    stop_mode: str = "observed"
    packaged: bool = True
    lug: np.ndarray | None = None
    stop_side: int = 0
    command: np.ndarray = field(default_factory=lambda: HOME.copy())
    previous_command: np.ndarray = field(default_factory=lambda: HOME.copy())
    last_pose: np.ndarray = field(default_factory=lambda: HOME.copy())
    block: np.ndarray | None = None
    source: np.ndarray | None = None
    pocket: np.ndarray | None = None
    back_side: int = 1
    fixed_back: int | None = None
    trim_enabled: bool = True
    trim_after: str = "draw_approach"
    trim_gain: float = 1.95
    trim_sample: tuple | None = None
    trim_goal: float = 0.
    trim_sign: int = 1
    trim_xy: np.ndarray | None = None
    trim_center: float = 0.
    trim_angle: float = 0.
    tray: np.ndarray | None = None
    baseline: np.ndarray | None = None
    stage: str = "approach"
    dwell: int = 0
    ticks: int = 0
    offset: float = 0.
    sag: float = .112
    pressure: float = 0.
    cartridge: str | None = None
    proof_pose: np.ndarray | None = None
    proof_samples: list = field(default_factory=list)

    def plan_clearance(self, lug):
        self.stop_side = 1 if self.stop_mode == "fixed_positive" or lug[1] > self.tray[1] else -1
        self.lug = np.array([lug[0], self.tray[1]+self.stop_side*abs(lug[1]-self.tray[1])])
        self.stage = "bolt_standoff"

    def observe_trim(self, obs):
        tx = self.tray[0]
        bay = rider_bay(obs,self.tray)
        if self.stage == "trim_read" and self.dwell >= 5:
            angle = cradle_angle(obs,tx)
            if angle is None or bay is None:
                return
            if self.trim_sample is not None:
                x0,a0 = self.trim_sample
                dx,da = bay[0]-x0, angle-a0
                if abs(dx)>.055 and abs(da)>.032 and dx*da>0:
                    self.trim_gain = float(np.clip(dx/da,1.1,3.1))
            self.trim_angle = angle
            if abs(angle)<.032:
                self.stage,self.dwell,self.ticks = self.trim_after,0,0
                return
            stroke = -np.sign(angle)*.14 if self.trim_sample is None else -self.trim_gain*angle
            self.trim_sample = (bay[0],angle)
            self.trim_xy = bay
            self.trim_goal = float(np.clip(bay[0]+stroke,tx-.69,tx+.69))
            self.trim_sign = 1 if self.trim_goal>bay[0] else -1
            self.stage,self.dwell,self.ticks = "trim_over",0,0
        elif self.stage == "trim_push":
            if (bay is not None and self.trim_sign*(self.trim_goal-bay[0]) < .014) or self.dwell>=8:
                self.trim_center = float(bay[0]) if bay is not None else self.trim_goal
                self.stage,self.dwell,self.ticks = "trim_relax",0,0

    def action(self, obs):
        if self.cassette is None:
            action = self.drive_probe.action(obs)
            if action is not None:
                return action
            self.cassette = self.drive_probe.cassette
        p = xyz(obs, fallback=self.last_pose)
        if self.block is None:
            if self.packaged:
                block, sign = source_frame(obs)
                if sign is not None:
                    self.back_side = sign
            else:
                block, self.baseline = shell_observation(obs)
            tray = tray_location(obs)
            if block is None or tray is None:
                # Reacquire the scene from the next image rather than move with
                # an incomplete localization.
                self.last_pose = p
                return [0.]*4
            self.block = block.copy()
            if self.packaged:
                self.source, self.stage = block.copy(), "draw_approach"
            self.tray = tray
        if self.source is not None and self.stage in ("bolt_lift", "bolt_exit"):
            # The parked bridge can hide one rail end at reset. Re-register
            # both ends while the bridge is over the receiving fixture.
            clear, sign = source_frame(obs, full=True)
            if clear is not None and (pocket := release_pocket(obs, clear, sign)) is not None:
                self.source, self.block, self.back_side, self.pocket = clear, clear.copy(), sign, pocket
        if self.source is not None and self.pocket is None:
            self.pocket = release_pocket(obs, self.source, self.back_side)
            if self.pocket is None and self.stage == "draw_approach" and self.lug is not None:
                return [0.]*4
        if self.stage == "draw_register":
            _, baseline = shell_observation(obs)
            if baseline is not None:
                self.baseline = baseline
        if self.lug is None and self.stop_mode != "skip":
            lug = lug_location(obs, self.tray)
            if lug is None:
                return [0.]*4
            self.plan_clearance(lug)
        if self.stage.startswith("trim_"):
            self.observe_trim(obs)
        return self.command_for_pose(
            p, lambda: pitch_pose(obs, self.baseline, .08+(SHEET_TOP if self.packaged else 0.), self.packaged),
            lambda: shell_observation(obs)[0])

    def command_for_pose(self, p, read_proof, read_marks):
        self.last_pose = p
        if self.stage in ("tip", "gauge", "proof") and self.dwell >= 1:
            measured = read_proof()
            if measured is not None:
                self.proof_samples.append(measured)
        while True:
            bx, by = self.block
            tx, ty = self.tray
            pick = bx+self.offset
            stage_x = tx-1.61+self.offset
            ceiling_entry = .700+self.sag
            # Let the pads take the load before releasing the vacuum.
            seated = .604+self.sag
            route = {
                "approach": ((bx, by, .78), .14, 5, 0., "seat"),
                "seat": ((bx, by, SEAT), .092, 6, 0.,
                         "tip" if self.fixed_offset is None else "shift"),
                "tip": ((bx, by, TIP_Z), .019, 7, TIP_PRESSURE, "restore"),
                "restore": ((bx, by, .204), .027, 6, TIP_PRESSURE, "vent"),
                "vent": ((bx, by, .204), .065, 3, 0., "shift"),
                "shift": ((pick, by, .266), .095, 4, 0., "pickup"),
                "pickup": ((pick, by, SEAT), .055, 5, 0., "gauge"),
                "gauge": ((pick, by, GAUGE_Z), .021, 8,
                          PRESSURES["light"] if self.fixed_pressure is None else self.fixed_pressure, "proof"),
                "reseat": ((pick, by, SEAT), .050, 6, 0., "proof"),
                "proof": ((pick, by, PROOF_Z), .034, 8, self.pressure, "lift"),
                "lift": ((pick, by, 1.08), .067, 6, self.pressure, "align"),
                "align": ((stage_x, ty, 1.08), .058, 6, self.pressure, "entry_height"),
                "entry_height": ((stage_x, ty, ceiling_entry), .045, 7, self.pressure, "insert"),
                "insert": ((tx+self.offset, ty, ceiling_entry), .068, 7, self.pressure, "dock"),
                "dock": ((tx+self.offset, ty, seated), .012, 9, self.pressure, "release"),
                "release": ((tx+self.offset, ty, seated), .050, 3, 0., "retract"),
                "retract": ((tx+self.offset, ty, 1.10), .080, 4, 0., None),
                "abort_tip": ((bx, by, .204), .024, 8, TIP_PRESSURE, "stop"),
                "abort_gauge": ((pick, by, .204), .024, 8, PRESSURES["light"], "stop"),
                "abort_proof": ((pick, by, .204), .024, 8, self.pressure, "stop"),
                "stop": (self.command, 0., 0, 0., None),
            }
            if self.source is not None:
                sx, sy = self.source
                free_x, free_y = self.pocket if self.pocket is not None else self.source
                if self.fixed_back is not None:
                    free_y = sy-self.fixed_back*abs(free_y-sy)
                low = SEAT+SHEET_TOP
                route.update({
                    "draw_approach": ((sx, sy, .75), .14, 4, 0., "draw_seat"),
                    "draw_seat": ((sx, sy, low), .075, 6, 0., "draw_seal"),
                    "draw_seal": ((sx, sy, low), .030, 4, TIP_PRESSURE, "draw_slide"),
                    "draw_slide": ((sx, free_y, low), .037, 8, TIP_PRESSURE, "draw_turn"),
                    "draw_turn": ((free_x, free_y, low), .026, 7, TIP_PRESSURE, "draw_vent"),
                    "draw_vent": ((free_x, free_y, low), .04, 5, 0., "draw_register"),
                    "draw_register": ((free_x, free_y, .72), .085, 5, 0., "approach"),
                })
                if self.trim_enabled:
                    route["retract"] = ((tx+self.offset,ty,1.10),.080,5,0.,"trim_leave_deck")
                    bay = self.trim_xy if self.trim_xy is not None else [tx,ty-1.87]
                    probe_z = .545-(bay[0]-tx)*math.tan(self.trim_angle)
                    # A tilted pocket can catch a vertical cup on its rim. Load
                    # the trailing jaw from outside; retreat on the same side.
                    flank = .38*self.trim_sign
                    home_x = bay[0]-flank
                    clear_x = self.trim_center-flank
                    route.update({
                        "trim_leave_deck": ((tx-1.06,ty,1.10),.085,4,0.,"trim_read"),
                        "trim_read": ((tx-1.06,ty-1.87,1.02),.12,5,0.,None),
                        "trim_over": ((home_x,bay[1],1.02),.12,3,0.,"trim_seat"),
                        "trim_seat": ((home_x,bay[1],probe_z),.058,4,0.,"trim_push"),
                        "trim_push": ((self.trim_goal+self.trim_sign*.10,bay[1],probe_z),.025,8,0.,None),
                        "trim_relax": ((clear_x,bay[1],probe_z),.067,3,0.,"trim_raise"),
                        "trim_raise": ((clear_x,bay[1],1.02),.085,3,0.,"trim_read"),
                        "trim_clear": ((tx-1.06,ty-1.87,1.02),.12,5,0.,None),
                    })
            if self.stage.startswith("bolt_"):
                x, y = self.lug
                sign = self.stop_side
                inside, outside = y-sign*.23, y+sign*.53
                route.update({
                    "bolt_standoff": ((tx-1.05, inside, 1.02), .14, 3, 0., "bolt_align"),
                    "bolt_align": ((x, inside, 1.02), .12, 4, 0., "bolt_lower"),
                    "bolt_lower": ((x, inside, .710), .065, 5, 0., "bolt_push"),
                    "bolt_push": ((x, outside, .710), .043, 5, 0., "bolt_lift"),
                    "bolt_lift": ((x, outside, 1.02), .068, 4, 0., "bolt_exit"),
                    "bolt_exit": ((tx-1.05, outside, 1.02), .13, 3, 0.,
                                  ("trim_read" if self.trim_enabled else "draw_approach")
                                  if self.source is not None else "approach"),
                })
            goal, limit, settle, grip, next_stage = route[self.stage]
            if next_stage is None or self.dwell < settle:
                break
            if self.stage == "retract":
                self.trim_after = "trim_clear"
            if self.stage == "draw_register":
                block = read_marks()
                if block is None:
                    break
                self.block = block
            if self.stage == "seat" and self.fixed_offset is not None:
                self.offset = self.fixed_offset
            if self.stage == "pickup" and self.fixed_pressure is not None:
                self.pressure = self.fixed_pressure
            if self.stage == "tip" or (self.fixed_pressure is None and self.stage in ("gauge", "proof")):
                measured = np.median(self.proof_samples, axis=0) if self.proof_samples else None
                self.proof_samples.clear()
                if measured is None:
                    next_stage = "abort_"+self.stage
                elif self.stage == "tip":
                    self.offset = math.copysign(NOMINAL_OFFSET, measured[2])
                elif self.stage == "gauge":
                    self.cartridge = "light" if measured[1] > GAUGE_HEIGHT_THRESHOLD else "heavy"
                    self.pressure = PRESSURES[self.cartridge]
                    if self.cartridge == "heavy":
                        next_stage = "reseat"
                else:
                    self.proof_pose = measured
                    _, z, angle = measured
                    self.sag = float(np.clip(PROOF_Z-z-PIVOT*math.cos(angle), .085, .27))
            if self.stage == "vent":
                block = read_marks()
                if block is not None:
                    self.block = block
            self.stage, self.dwell, self.ticks = next_stage, 0, 0
        error = np.asarray(goal)-self.command
        self.dwell = self.dwell+1 if np.max(np.abs(error)) < .014 else 0
        expected = self.command+.62*(self.previous_command-self.command)
        error[:2] += np.clip(.11*(expected[:2]-p[:2]), -.011, .011)
        delta = pulse(error, limit, self.ticks, grip)
        motor = motor_matrix(self.cassette) @ delta[:2]
        scale = max(1., np.max(abs(motor))/.14)
        delta[:2] /= scale
        self.ticks += 1
        self.previous_command = self.command.copy()
        self.command = np.clip(self.command+delta, [-2.6, -2.6, .17], [2.6, 2.6, 1.5])
        return np.r_[motor/(scale*.14), delta[2]/.14, grip].tolist()


class ArmSession:
    """Reconcile repeated-step branches by continuity of the full RGB scene."""

    def __init__(self):
        self.t, self.parents, self.candidates = -1, [], []

    def act(self, t, obs):
        if t < self.t:
            raise ValueError("step index moved backwards without reset")
        if t != self.t:
            self.parents, self.candidates = self.candidates, []
            self.t = t
        # Arm markers can be occluded. Comparing the last complete views
        # also keeps stationary supports and the workcell in the association.
        frame = decode(obs)[::4, ::4].astype(np.int16)
        if self.parents:
            controller = copy.deepcopy(min(
                self.parents, key=lambda item: np.mean(abs(frame-item[0])))[1])
        else:
            controller = ArmController()
        action = controller.action(obs)
        self.candidates.append((frame, controller))
        return action
