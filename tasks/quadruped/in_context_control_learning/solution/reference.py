"""Author-only reference. The feedback is an onboard RGB frame and milestone bits, not simulator state."""
import base64
import io
from itertools import count
import math

import numpy as np
from PIL import Image
from scipy import ndimage


class PartialView(ValueError):
    """Body-frame bearings to identifiable posts; a single bearing is not a pose."""


class PhysicalFall(RuntimeError):
    pass


POSTS = {"pink": (.52, .29), "cyan": (.52, -.29), "red": (-.52, .29), "green": (-.52, -.29)}
PITCH = math.radians(25)


def wrap(angle):
    return math.remainder(angle, 2*math.pi)


def pixels(encoded):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(encoded))).convert("RGB"), dtype=float)


def outboard_side(encoded):
    """Which side of the launch view carries the violet edges of the steel plate?"""
    r, g, b = pixels(encoded).transpose(2, 0, 1)
    mask = (r > 50) & (g > 35) & (r > 1.18*g) & (b > 1.48*r)
    _, u = np.where(mask)
    if len(u) < 16:
        raise ValueError("outboard runner nosing not visible from the rack")
    return 1 if np.median(u) < (r.shape[1]-1)/2 else -1


def doorway_side(encoded):
    r,g,b = pixels(encoded).transpose(2,0,1)
    ink = (r>75) & (g>.65*r) & (b<.42*g)
    h,w = ink.shape
    left,right = ink[:h//3,:w//5].sum(),ink[:h//3,-w//5:].sum()
    return (1 if left>right else -1) if abs(int(left)-int(right))>30 else 0


def departure_turn(encoded):
    """Read left/right from the physical yellow pointer relative to its blue tail pin."""
    red, green, blue = pixels(encoded).transpose(2, 0, 1)
    pin = (blue > 155) & (blue > 5*red) & (blue > 5*green)
    v, u = np.where(pin)
    if not len(u):
        return .085
    yy, xx = np.ogrid[:red.shape[0], :red.shape[1]]
    local = (xx-u.mean())**2 + (yy-v.mean())**2 < (6*math.sqrt(len(u)))**2
    ink = local & (red > 105) & (green > .65*red) & (blue < .39*green)
    col = np.where(ink)[1]
    return -math.copysign(.085, col.mean()-u.mean()) if len(col) else .085


def fixture_view(encoded):
    """Two identified vertical staffs define a metric plane, from either side of the dock."""
    rgb = pixels(encoded)
    red, green, blue = rgb.transpose(2, 0, 1)
    masks = {"pink": (red > 145) & (blue > 100) & (red > 2.8*green) & (blue > 2.8*green),
             "cyan": (green > 135) & (blue > 145) & (green > 2.8*red) & (blue > 2.8*red),
             "red": (red > 160) & (red > 3.7*green) & (red > 4*blue),
             "green": (green > 145) & (green > 3.7*red) & (green > 3.7*blue)}
    h, w = red.shape
    focal = h/(2*math.tan(math.radians(41)))
    pairs, rays, ranges = {}, {}, {}
    for name, mask in masks.items():
        labels, _ = ndimage.label(mask)
        areas = np.bincount(labels.ravel())[1:]
        ids = np.flatnonzero(areas >= 5) + 1
        if not len(ids):
            continue
        selected = sorted(ids, key=lambda i: areas[i-1], reverse=True)[:2]
        ordered = sorted(zip(ndimage.center_of_mass(mask, labels, selected), selected))
        centres = [centre for centre, _ in ordered]
        v, u = centres[0]
        ut, vt = (u-(w-1)/2)/focal, (v-(h-1)/2)/focal
        # An unobstructed ball's angular radius supplies coarse range even if the lower ball is hidden.
        area = areas[ordered[0][1]-1]
        depth = .024*focal*math.sqrt(math.pi/area)*(1+ut*ut+vt*vt)**.25
        if len(centres) == 2 and centres[1][0]-v > 6:
            pairs[name] = [(u,v) for v,u in centres]
            vt, vb = (v-(h-1)/2)/focal, (centres[1][0]-(h-1)/2)/focal
            depth = .26*(math.cos(PITCH)-vb*math.sin(PITCH))/(vb-vt)
        bx = .11 + depth*(math.cos(PITCH)-vt*math.sin(PITCH))
        by = -depth*ut
        rays[name] = math.atan2(by, bx)
        ranges[name] = math.hypot(bx, by)
    def coarse():
        candidates = []
        for a in rays:
            for b in rays:
                if a >= b:
                    continue
                body_a = ranges[a]*np.array([math.cos(rays[a]), math.sin(rays[a])])
                body_b = ranges[b]*np.array([math.cos(rays[b]), math.sin(rays[b])])
                arm = body_b-body_a
                baseline = np.subtract(POSTS[b], POSTS[a])
                mismatch = abs(np.linalg.norm(arm)-np.linalg.norm(baseline))
                if mismatch > .19:
                    continue
                angle = wrap(math.atan2(baseline[1], baseline[0])-math.atan2(arm[1], arm[0]))
                co, si = math.cos(angle), math.sin(angle)
                pos = .5*(np.add(POSTS[a], POSTS[b])-np.array([[co,-si],[si,co]])@(body_a+body_b))
                candidates.append((mismatch, pos[0], pos[1], angle))
        if candidates:
            return min(candidates)[1:]
        raise PartialView(rays, ranges)
    options = [(abs(a[0][0]-b[0][0]), i, j) for i,a in pairs.items() for j,b in pairs.items() if i < j]
    if not options or max(options)[0] < 19:
        return coarse()
    _, i, j = max(options)
    a, b = np.array(POSTS[i]), np.array(POSTS[j])
    spacing = np.linalg.norm(a-b)
    axis = (a-b)/spacing
    basis = np.array([[axis[1], axis[0], 0], [-axis[0], axis[1], 0], [0, 0, 1]])
    rows, observations = [], []
    for (u, v), (y, z) in zip(pairs[i]+pairs[j], [(y,z) for y in (spacing/2,-spacing/2) for z in (.46,.20)]):
        u, v = (u - (w-1)/2)/focal, (v - (h-1)/2)/focal
        rows.extend([[y, z, 1, 0, 0, 0, -u*y, -u*z], [0, 0, 0, y, z, 1, -v*y, -v*z]])
        observations.extend([u, v])
    try:
        H = np.append(np.linalg.solve(rows, observations), 1).reshape(3, 3)
    except np.linalg.LinAlgError:
        return coarse()
    side, up, origin = H.T * (2/(np.linalg.norm(H[:, 0]) + np.linalg.norm(H[:, 1])))
    U, _, V = np.linalg.svd(np.column_stack([np.cross(side, up), side, up]))
    R = U @ V  # fixture axes in optical coordinates: right, down, forward
    camera = np.r_[(a+b)/2, 0.] - basis @ R.T @ origin
    forward = basis @ R.T @ [0, -math.sin(PITCH), math.cos(PITCH)]
    vertical = basis @ R.T @ [0, -math.cos(PITCH), -math.sin(PITCH)]
    trunk = camera - .11*forward - .16*vertical
    if not .14 < trunk[2] < .38 or vertical[2] < .85:
        return coarse()
    return trunk[0], trunk[1], math.atan2(forward[1], forward[0])


def pose(phase, stride, turn, lateral=0., height=.265, lift=.082, crawl=False, track=(0.,0.)):
    """Native leg IK: a turn sweeps each foot tangentially about the trunk, not just fore/aft."""
    result = []
    duty = .77 if crawl else .59
    offsets = (0,.5,.75,.25) if crawl else (0,.5,.5,0)
    for i, offset in enumerate(offsets):
        p = (phase + offset) % 1
        stroke = stride + (turn if i in (0, 2) else -turn)
        if p < duty:
            swing, h = .5-p/duty, height
        else:
            s = (p-duty)/(1-duty)
            swing = -.5+s-math.sin(2*math.pi*s)/(2*math.pi)
            h = height - lift*math.sin(math.pi*s)
        x = stroke*swing - (.035 if crawl else 0.)
        hip = -.08 if i in (0, 2) else .08
        y = hip + track[i % 2] + (lateral + (.24 if i < 2 else -.24)*turn)*swing
        sagittal = math.sqrt(y*y+h*h-hip*hip)
        abd = math.atan2(y, h)-math.atan2(hip, sagittal)
        bend = math.acos(min(1., math.hypot(x, sagittal)/.426))
        result.extend((abd, math.atan2(-x, sagittal)+bend, -2*bend))
    return result


HOME = [0., .9, -1.8]*4


def plate_pose(line,phase,stride,turn,lateral,height,lift,crawl,track):
    if line >= 0:
        return pose(phase,stride,turn,lateral,height,lift,crawl,track)
    cmd = np.array(pose(phase,stride,-turn,-lateral,height,lift,crawl,(-track[1],-track[0])))
    cmd = cmd.reshape(4,3)[[1,0,3,2]]
    cmd[:,0] *= -1
    return cmd.ravel().tolist()


def clear_pickups(encoded):
    """Read the four cyan/amber windows in their labelled physical board positions."""
    r, g, b = pixels(encoded).transpose(2,0,1)
    masks = ((g > 125) & (b > 150) & (r < .22*g) & (r < .22*b),
             (r > 165) & (g > 32) & (g < .70*r) & (b < .22*g))
    windows = []
    for clear,mask in enumerate(masks):
        labels, _ = ndimage.label(mask)
        sizes = np.bincount(labels.ravel())[1:]
        for label in np.flatnonzero(sizes > 45)+1:
            v,u = ndimage.center_of_mass(mask,labels,label)
            windows.append((int(sizes[label-1]),v,u,bool(clear)))
    if len(windows) < 4:
        raise ValueError("cannot read four pickup windows in the onboard image")
    top = sorted(sorted(windows,reverse=True)[:4],key=lambda item:item[1])
    physical = [item for row in (top[:2],top[2:]) for item in sorted(row,key=lambda item:-item[2])]
    return tuple(item[3] for item in physical)  # FR, FL, RR, RL


class Controller:
    def __init__(self, request):
        self.request = request
        self.legs_by_port = (0,1,2,3)
        self.probe_readings = []
        self.phase = 0.

    def act(self, native, ticks):
        cmd = np.asarray(native).reshape(4, 3)
        cmd = cmd[list(self.legs_by_port)]
        status = self.request("act",target_rad=cmd.ravel().tolist(),ticks=ticks)
        if status["fallen"]:
            raise PhysicalFall(f"reference fell: {status}")
        return status

    def identify(self):
        def read():
            reading = clear_pickups(self.request("observe")["images"][0])
            self.probe_readings.append(reading)
            return reading
        if any(read()):
            raise ValueError("expected four covered pickups at the settled launch pose")
        found = []
        for port in range(3):
            cmd = np.array(HOME).reshape(4,3)
            cmd[port] = [0.,1.22,-2.44]
            status = self.request("act", target_rad=cmd.ravel().tolist(), ticks=55)
            reading = read()
            if status["fallen"] or sum(reading) != 1 or reading.index(True) in found:
                raise ValueError(f"inconclusive physical port probe: {port}, {reading}")
            found.append(reading.index(True))
            self.request("recover")
        last, = set(range(4))-set(found)
        return tuple(found+[last])

    def commission(self):
        self.legs_by_port = self.identify()
        status = self.act([0., 1.09, -2.18]*4, 450)
        if status["calibration_checks"] != 1:
            raise RuntimeError(f"clearance check did not latch: {status}")
        self.request("recover")
        return self.act(([0., .93, -1.86] + [0., 1.04, -2.08])*2, 500)

    def stance(self, dock):
        # The operator manual identifies the two pieces of plant; navigation still uses live RGB.
        return "duct" if dock == 2 else ("cover" if dock == 3 else "flat")

    def foot_track(self, side):
        return (-.052,.052), side*.061

    def solve(self):
        self.commission()
        for dock in range(1,7):
            self.request("advance")
            for attempt in range(3):
                try:
                    self._walk_dock(dock,attempt)
                    break
                except PhysicalFall:
                    if attempt == 2:
                        raise
                    # Same physical checkpoint and accrued budget; try a different first support diagonal.
                    self.request("recover")

    def _walk_dock(self,dock,attempt):
        if dock == 1:
            return self._split_inspection(attempt)
        if dock == 2:
            sign = self.door_side or math.copysign(1.,departure_turn(self.request("observe")["images"][0]))
            track, line = self.foot_track(self.runner_side)
            # A fall on the long walk-off is evidence to turn sooner on the next rack.
            # Keep the approach phase from dock 1, rather than the failed turning phase.
            if not attempt:
                self.exit_phase = self.phase
            for k in range((120,77,96)[attempt]):
                p = self.exit_phase+(k+1)*.030
                self.act(plate_pose(line,p,.085,0.,0.,.25,.082,True,track),12)
            for k in range(48):
                self.act(pose((k+1)*.053,0.,sign*.077,0.,.225,.065),14)
        mode = self.stance(dock)
        height,lift = {"duct":(.194,.055),"cover":(.24,.09),"flat":(.265,.082)}[mode]
        bend = math.acos(height/.426)
        rest = HOME if height == .265 else [0.,bend,-2*bend]*4
        if dock in (2,3,4) or attempt:
            self.act(rest,160)
        self.phase = 0. if dock == 2 and attempt == 1 else (0.,.25,.50)[attempt]
        estimate = None
        search_turn = .085
        facing_travel = dock != 3
        on_top = False
        for burst in count():
            obs = self.request("observe")
            # Approach credit latches across recovery; a retry must reacquire the deck in RGB.
            near_now = not attempt or (estimate is not None and math.hypot(*estimate[:2]) < .24)
            if dock == 3 and obs["approach_reached"] and near_now and not on_top:
                # The front feet are up. Leave the rearward-biased climbing pattern for parking.
                on_top = True
                if mode == "cover":
                    mode,height,lift,rest = "flat",.265,.082,HOME
                if self.act(rest,350)["goal_reached"]:
                    break
            if not burst:
                search_turn = sign*.085 if dock == 2 else departure_turn(obs["images"][0])
            try:
                estimate = fixture_view(obs["images"][0])
            except PartialView as exc:
                rays, ranges = exc.args
                if estimate is not None and rays:
                    x, y, yaw = estimate
                    if ranges:
                        k = min(ranges, key=ranges.get)
                        dx, dy = POSTS[k][0]-x, POSTS[k][1]-y
                        gain = .65*(1-ranges[k]/max(.05, math.hypot(dx, dy)))
                        x, y = x+gain*dx, y+gain*dy
                    angles = [math.atan2(POSTS[k][1]-y, POSTS[k][0]-x)-angle for k,angle in rays.items()]
                    estimate = (x, y, math.atan2(sum(map(math.sin, angles)), sum(map(math.cos, angles))))
            pivot = False
            if estimate is None:
                stride, lateral, turn = 0., 0., search_turn
            else:
                x, y, yaw = estimate
                if dock == 2 and x < -.28:
                    # Follow the level turning slab until past the hatch.
                    y += sign*.20*np.clip((-.28-x)/.17, 0., 1.)
                distance = math.hypot(x, y)
                if distance < .11 and abs(yaw) < math.radians(15):
                    if self.act(rest, 350)["goal_reached"]:
                        break
                dx = -x*math.cos(yaw)-y*math.sin(yaw)
                dy = x*math.sin(yaw)-y*math.cos(yaw)
                # Enter the pad first, then translate in its vicinity while aligning the nose.
                error = -yaw if distance < .24 else math.atan2(dy, dx)
                stride = np.clip(.42*dx, -.053, .11 if mode=="cover" else .10)
                lateral = np.clip(.32*dy, -.035, .035)
                turn = np.clip(.09*error, -.075, .075)
                if dock == 3 and not on_top and x < -.18:
                    # Square up to the marked entry, then correct small cross-track drift.
                    # A diagonal attack puts one front foot on the apron well before its mate.
                    error = wrap(float(np.clip(-.70*y,-.13,.13))-yaw)
                    if abs(error) > (.29 if facing_travel else .09):
                        facing_travel = False
                        pivot = True
                        stride = lateral = 0.
                        turn = math.copysign(min(.078,max(.038,.12*abs(error))),error)
                    else:
                        facing_travel = True
                        stride = min(.11,max(.065,-.42*x))
                        lateral = np.clip(-.12*y,-.018,.018)
                        turn = np.clip(.15*error,-.037,.037)
                if not facing_travel:
                    if not pivot and abs(error) < .30:
                        facing_travel = True
                    else:
                        stride = lateral = 0.
            # Reobserve after a short burst, carrying phase between bursts.
            clearing_hatch = (dock==2 and attempt==0 and estimate is not None and estimate[0]<-.33
                              and abs(estimate[2])<.60)
            crawl = (mode=="cover" and facing_travel and not pivot) or clearing_hatch
            for _ in range(12):
                ticks = 12 if crawl else 14
                self.phase += ticks*.002*(1.25 if crawl else 1.9)
                status = self.act(pose(self.phase,stride,turn,lateral,height,
                                       .093 if clearing_hatch else lift,crawl),ticks)
                if status["goal_reached"]:
                    break
            if status["goal_reached"]:
                break
            if estimate is not None:
                # Short prediction across blind arcs. New visible staff pairs replace it outright.
                x, y, yaw = estimate
                estimate = (x+.65*stride*math.cos(yaw)-.60*lateral*math.sin(yaw),
                            y+.65*stride*math.sin(yaw)+.60*lateral*math.cos(yaw), wrap(yaw+4.2*turn))

    def _split_inspection(self, attempt):
        im = self.request("observe")["images"][0]
        self.runner_side = outboard_side(im)
        self.door_side = doorway_side(im)
        track, line = self.foot_track(self.runner_side)
        rest = pose(0.,0.,0.,height=.25,lift=0.,crawl=True,track=track)
        x, y, yaw = -.52, 0., 0.  # public straight arrival from a settled, forward-facing rack
        self.phase = attempt*.25
        # Move the weight over the two tracks while the rear feet still have a
        # full slab; widening planted feet on one side would push the body away.
        for k in range(32):
            self.phase += 1/32
            ramp = min(1.,(k+1)/16)
            self.act(plate_pose(line,self.phase,0.,0.,1.25*line,.25,.060,False,
                          tuple(v*ramp for v in track)),12)
        y = line
        for burst in count():
            im = self.request("observe")["images"][0]
            try:
                nx, ny, na = fixture_view(im)
                if abs(nx-x) < .23 and abs(ny-y) < .19 and abs(wrap(na-yaw)) < .55:
                    x, y, yaw = .22*x+.78*nx, .36*y+.64*ny, wrap(yaw+.74*wrap(na-yaw))
            except PartialView as exc:
                rays, ranges = exc.args
                if rays:
                    name = min(ranges, key=ranges.get)
                    bx, by = POSTS[name][0]-x, POSTS[name][1]-y
                    yaw = wrap(yaw+.5*np.clip(wrap(math.atan2(by,bx)-rays[name]-yaw),-.32,.32))
            if abs(x-.050) < .055 and abs(y) < .12 and abs(yaw) < .27:
                if self.act(rest, 320)["goal_reached"]:
                    return
            stride = .085 if x < .036 else float(np.clip(.48*(.065-x),-.045,.085))
            lateral = float(np.clip(.68*(line-y),-.060,.060))
            turn = float(np.clip(-.16*yaw,-.035,.035))
            for _ in range(12):
                self.phase += .030
                if self.act(plate_pose(line,self.phase,stride,turn,lateral,.25,.105,True,track),12)["goal_reached"]:
                    return
            x += .38*stride*math.cos(yaw)
            y += .38*stride*math.sin(yaw)+.30*lateral
            yaw = wrap(yaw+1.6*turn)
