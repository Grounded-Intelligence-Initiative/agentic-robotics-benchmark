"""Motion skills shared by the expert, the reference and the naive controller (sub-generators yielding Intents).

Every skill takes `w`, a world-model object with:
  w.R            the Runner (pending commands, unwrapped yaw)
  w.obs          the latest observation
  w.cfg          knobs: force (bool), verify (bool)
"""
import math

import numpy as np

from ctl import Intent, wrap, HOME_YAW, YAW_LO, YAW_HI

TABLE_Z = 0.80
BELT_Z = TABLE_Z + 0.040
H_PART = 0.020
GRIP_H = 0.016
FLOOR_Z = TABLE_Z + 0.010            # kit pocket floor and nest floor
SAFE_Z = 0.95
SEAT_TOL = 0.0015            # a held part within this of its seated height is in (unseated states sit >= 2.3 mm high)
CONTROL_DT = 0.1


def g_open(width, margin=0.024):
    return float(np.clip(1.0 - (width + margin) / 0.04, -1.0, 1.0))


def nest_yaw(pocket_yaw):
    """The yaw (0 or pi) to lay a part in the turnover station's input nest so that it comes out of the output
    nest (turned over about x: yaw -> -yaw) closest to pocket_yaw."""
    return 0.0 if abs(wrap(-0.0 - pocket_yaw)) < abs(wrap(-math.pi - pocket_yaw)) else math.pi


def near_home(a):
    """The equivalent (mod 2 pi) of angle a nearest the hand's home yaw."""
    return HOME_YAW + wrap(a - HOME_YAW)


def yaw_window(xy, margin_deg=6.0):
    """The hand yaws (unwrapped, rad) the arm reaches at a place xy: about 360 degrees around a centre that turns with
    the place's bearing from the robot base (measured on the cell; see the data sheet's yaw range)."""
    c = math.radians(138.0) + math.atan2(xy[1], xy[0] + 0.56)
    h = math.radians(180.0 - margin_deg)
    return c - h, c + h


def yaw_in(window, a, near=None):
    """The equivalent of angle a (mod 2 pi) inside the window, nearest to `near` (default: the window's centre), or
    None."""
    lo, hi = window
    ref = (lo + hi) / 2 if near is None else near
    best = None
    for n in range(-3, 4):
        v = a + 2 * math.pi * n
        if lo <= v <= hi and (best is None or abs(v - ref) < abs(best - ref)):
            best = v
    return best


def yaw_place(a, xy, current):
    """The equivalent of a inside the yaw window at xy nearest to the current hand yaw, preferring one well inside it
    (near a joint-7 limit the arm tracks and presses poorly); fallback: nearest home."""
    v = yaw_in(yaw_window(xy, margin_deg=35.0), a, near=current)
    if v is None:
        v = yaw_in(yaw_window(xy), a, near=current)
    return v if v is not None else near_home(a)


def grasp_plan(theta_part, theta_goal, pick_xy=(0.10, 0.0), place_xy=(-0.21, -0.25)):
    """Hand yaws (pick, place) for a part lying at yaw theta_part that must end at yaw theta_goal, both reachable,
    and the part's yaw relative to the hand's forward direction (0 or pi). The hand turns by the same amount the
    part has to turn."""
    wp, wq = yaw_window(pick_xy), yaw_window(place_xy)
    wq_c = yaw_window(place_xy, margin_deg=35.0)
    best = None
    for k in (0, 1):
        base = theta_part + math.pi / 2 + k * math.pi
        for n in range(-3, 4):
            psi_pick = base + 2 * math.pi * n
            if not (wp[0] <= psi_pick <= wp[1]):
                continue
            rot = theta_goal - theta_part
            for m in range(-2, 3):
                psi_place = psi_pick + rot + 2 * math.pi * m
                margin = min(psi_pick - wp[0], wp[1] - psi_pick, psi_place - wq[0], wq[1] - psi_place)
                if margin < -math.radians(20):
                    continue
                cost = -margin + 0.1 * abs(psi_place - psi_pick) + (100.0 if margin < 0 else 0.0) + \
                    (5.0 if not (wq_c[0] <= psi_place <= wq_c[1]) else 0.0)
                if best is None or cost < best[0]:
                    delta = wrap(theta_part - (psi_pick - math.pi / 2))
                    best = (cost, psi_pick, psi_place, delta)
    if best is None:                      # should not happen with the windows' ~310 degrees
        psi_pick = near_home(theta_part + math.pi / 2)
        delta = wrap(theta_part - (psi_pick - math.pi / 2))
        return psi_pick, near_home(theta_goal - delta + math.pi / 2), delta
    return best[1], best[2], best[3]


def wait(w, n, grip=None, cams=0):
    for _ in range(n):
        yield Intent(goal=w.hold_goal(), grip=grip, cams=cams)


def move_to(w, goal, yaw=None, grip=None, tol=0.002, yaw_tol=0.02, max_steps=80, max_step=0.03, cams=0, settle=0):
    goal = np.asarray(goal, float)
    w.hold = goal.copy()
    ok_count = 0
    for _ in range(max_steps):
        obs = w.obs
        e = np.linalg.norm(np.asarray(obs["ee_pos"]) - goal)
        ye = 0.0 if yaw is None else abs(w.R._yaw(obs) - yaw)
        if e < tol and ye < yaw_tol:
            ok_count += 1
            if ok_count > settle:
                return True
        yield Intent(goal=goal, yaw=yaw, grip=grip, max_step=max_step, cams=cams)
    if hasattr(w, "log"):
        o = w.obs
        w.log.append(("move-timeout", [round(x, 4) for x in goal], round(float(np.linalg.norm(np.asarray(o["ee_pos"]) - goal)), 4),
                      None if yaw is None else round(w.R._yaw(o) - yaw, 4), round(o["time"], 1)))
    return False


def transit(w, xy, yaw=None, grip=None, z=SAFE_Z):
    """Up to the safe height, across, (stays at the safe height)."""
    ee = np.asarray(w.obs["ee_pos"], float)
    if ee[2] < z - 0.01:
        yield from move_to(w, [ee[0], ee[1], z], yaw=None, grip=grip, tol=0.01, max_steps=30)
    yield from move_to(w, [xy[0], xy[1], z], yaw=yaw, grip=grip, tol=0.004, yaw_tol=0.03, max_steps=120)


def lx(w):
    """Control steps of command latency beyond the nominal 2 (the shift's identified latency): every wait for the
    gripper or for a command to take effect lasts that much longer."""
    return max(0, int(getattr(w.R, "latency", 2)) - 2)


def place_release(w, xy, z_grip, yaw, lift_to=SAFE_Z):
    """Lower the held part to z_grip (grip-site height) at xy, open, lift."""
    yield from move_to(w, [xy[0], xy[1], z_grip + 0.02], yaw=yaw, grip=1.0, tol=0.0015, yaw_tol=0.01, max_steps=60)
    yield from move_to(w, [xy[0], xy[1], z_grip], yaw=yaw, grip=1.0, tol=0.0008, max_steps=40, max_step=0.006)
    for _ in range(4 + lx(w)):
        yield Intent(goal=np.array([xy[0], xy[1], z_grip]), yaw=yaw, grip=-0.2)
    ee = np.asarray(w.obs["ee_pos"], float)
    yield from move_to(w, [ee[0], ee[1], lift_to], grip=-0.2, tol=0.01, max_steps=40)


def insert_force(w, aim_xy, yaw, z_seat, f_touch=2.0, f_set=4.0, r_max=0.0036, ring=0.0012, max_points=24, t_end=None):
    """Force-guided insertion of the held part at aim_xy (grip-site xy), hand yaw `yaw`; z_seat = the grip-site
    height at which the part sits on the pocket floor. Returns 'seated' when the hand got there, else 'stuck'.
    Leaves the part held."""
    aim_xy = np.asarray(aim_xy, float)
    z_free = z_seat + 0.008                  # part bottom 3 mm above the wall tops
    yield from move_to(w, [aim_xy[0], aim_xy[1], z_free], yaw=yaw, grip=1.0, tol=0.0005, yaw_tol=0.004,
                       max_steps=60, max_step=0.01, settle=3)
    if hasattr(w, "log"):
        w.log.append(("t-above", round(w.obs["time"], 1)))
    fz = []
    for _ in range(3):
        yield Intent(goal=np.array([aim_xy[0], aim_xy[1], z_free]), yaw=yaw, grip=1.0)
        fz.append(np.asarray(w.obs["wrench"][:3], float))
    f0 = np.mean(fz, axis=0)
    fz0 = float(f0[2])
    w.fz0 = fz0

    def press(goal_xy, steps, use_force=True, k_lat=0.0):
        """Go down at about 1 mm per step, laterally soft (the lead-in may move the part), until the part stalls;
        returns the stall height."""
        zs, xys = [], []
        for _ in range(steps):
            o = w.obs
            z = o["ee_pos"][2]
            zs.append(z)
            xys.append(np.asarray(o["ee_pos"][:2], float))
            f = o["wrench"][2] - fz0
            if z < z_seat + SEAT_TOL:
                return z
            inside = z < z_seat + 0.0025                # the part's bottom is well below the walls' top
            f_hold = 8.0 if inside else 0.6 * f_set      # inside, push through the walls' friction (still far below
            f_stop = 9.0 if inside else f_set            # the 12 N limit); outside, touch lightly
            if use_force and f > f_stop:
                return z
            if len(zs) >= 6 and zs[-6] - z < 0.00025 and (f > (6.0 if inside else 1.5) or not use_force) and \
                    np.linalg.norm(xys[-1] - xys[-6]) < 0.0003:     # neither going down nor sliding sideways
                return z
            # slow near the expected contact (the commands in flight keep pushing for two steps after a reading)
            dz = -0.0012 if (z > z_seat + 0.0062 or z < z_seat + 0.0025) else -0.0008
            dz = dz / getattr(w.R, "stiffness", 1.0)    # the same contact force per step on a soft or stiff day
            if use_force and f > f_hold:
                dz = 0.0
            kl = 0.8 if f < 1.0 else k_lat          # firm until touching (descending drifts the hand), then soft
            if inside:
                kl = 0.0                            # inside the pocket: the walls guide the part, do not fight them

            touching = use_force and f > 1.0

            def fn(o, R, gx=goal_xy, dz=dz, kl=kl, touching=touching):
                ee = np.asarray(o["ee_pos"], float)
                lat = (np.asarray(gx) - ee[:2]) * kl
                if touching:
                    # admittance: follow the lateral reaction (on a lead-in chamfer it points to the pocket)
                    fl = np.asarray(o["wrench"][:2], float) - f0[:2]
                    lat = lat + np.clip(0.0012 * fl / getattr(R, "stiffness", 1.0), -0.0012, 0.0012)
                return np.array([lat[0], lat[1], dz])
            # touching or inside: the hand's yaw is left free too (the walls square the part up)
            yield Intent(fn=fn, yaw=None if (touching or inside) else yaw, grip=1.0)
        return w.obs["ee_pos"][2]

    zc = yield from press(aim_xy, 60, w.cfg.get("force", True))
    aim_xy = np.asarray(w.obs["ee_pos"][:2], float)   # search around where the lateral reaction led the part
    if hasattr(w, "log"):
        w.log.append(("t-pressed", round(w.obs["time"], 1), round(zc - z_seat, 4)))
    if zc < z_seat + SEAT_TOL:
        return "seated"
    if not w.cfg.get("force", True):
        return "stuck"
    pts = []
    for k in range(1, int(round(r_max / ring)) + 1):
        for j in range(6 * k):
            a = 2 * math.pi * (j + 0.5 * (k % 2)) / (6 * k)
            pts.append(k * ring * np.array([math.cos(a), math.sin(a)]))
    for j, d in enumerate(pts[:max_points]):
        if t_end is not None and w.obs["time"] > t_end:
            break
        goal = aim_xy + d
        for _ in range(8):           # lift off a little and move to the next point
            o = w.obs
            if np.linalg.norm(np.asarray(o["ee_pos"][:2]) - goal) < 0.0003 and o["ee_pos"][2] > zc + 0.0004:
                break
            yield Intent(goal=np.array([goal[0], goal[1], zc + 0.0009]), yaw=yaw, grip=1.0, max_step=0.002)
        z = yield from press(goal, 10)
        if z < zc - 0.0008:
            # in the lead-in: hold the point, keep pressing gently
            z = yield from press(goal, 50)
            if z < z_seat + SEAT_TOL:
                w.found = d
                return "seated"
            zc = max(z, zc - 0.002)
    return "stuck"
