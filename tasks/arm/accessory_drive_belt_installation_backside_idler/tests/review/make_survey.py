"""Survey photographs for the payload (maintainers' tool; never run by the grader).

    python3 make_survey.py <panda dir> <out dir>

The station with an engine at its nominal pose, the arms at home, nominal belt and tensioner physics: (a) the drive
without a belt (the new belt is not yet on its peg), (b) a correctly fitted belt (on every pulley, seated in the
groove zones, the tensioner where the chain closes). Per state: one front-camera frame, and for each wrist camera
four frames with the hand placed so that the camera looks at C, WP, ALT and I from about 0.2 m. The frames go through
the cell's own sensor model (noise, quantisation, invalid pixels; a fixed noise seed). Writes <state>_<camera>[_<i>]
.png (RGB) and _depth.png (uint16 in the camera's depth unit, 0 = invalid) and survey.json (per frame: the camera,
its intrinsics, T_world_cam, the arms' joints).
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

from beltcell import cameras as cm  # noqa: E402
from beltcell import drive as dr  # noqa: E402
from beltcell.control import ik_solve_seeds  # noqa: E402
from beltcell.physics import Scene  # noqa: E402

NOMINAL = {"mu": 0.75, "bend_EI": dr.BELT["bend_EI"], "mass_per_m": dr.BELT["mass_per_m"], "len_scale": 1.0,
           "t_rate": dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]["rate"], "t_tau_closed": dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]["tau_closed"],
           "t_damper": 1.0, "t_friction": 0.08, "drag_WP": 0.3, "drag_ALT": 0.3, "dpos": [0.0, 0.0, 0.0], "yaw": 0.0, "roll": 0.0,
           "idler_bearing": 0.0125}


def fit_belt(sc):
    """The belt on every pulley at the groove centre, the tensioner at the angle that closes the chain."""
    T = dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]
    S = dr.opening_sense(T)
    n, l = sc.belt.n, sc.belt.l
    d = sc.data

    def gap(th):
        path, tot, _ = dr.loop_path(sc.drive.circles(th, l))
        P = np.vstack([path, path])
        pts, g = dr.resample_closed(P, n, l)
        t0 = path[1] - path[0]
        t0 /= np.linalg.norm(t0)
        return float(g @ t0), pts

    lo, hi = T["theta_closed"], T["theta_closed"] + S * math.radians(50.0)
    glo = gap(lo)[0]
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        gm = gap(mid)[0]
        if gm * glo > 0:
            lo, glo = mid, gm
        else:
            hi = mid
    th = 0.5 * (lo + hi)
    _, pts = gap(th)
    P3 = np.asarray([sc.E_to_world(x, z, dr.BELT_PLANE_D) for x, z in pts])
    d.qpos[sc.drive.t_qadr] = S * (th - T["theta_closed"])
    sc.belt.set_points(d, P3, np.tile(sc.R_E[:, 1], (n, 1)))
    mujoco.mj_forward(sc.model, d)
    return th


def hide_belt(sc):
    """The belt out of the scene (5 m below the floor)."""
    d = sc.data
    n = sc.belt.n
    P = np.asarray([[0.3 * math.cos(2 * math.pi * k / n), 0.3 * math.sin(2 * math.pi * k / n), -5.0] for k in range(n)])
    sc.belt.set_points(d, P, np.tile(np.array([0.0, 0.0, 1.0]), (n, 1)))
    mujoco.mj_forward(sc.model, d)


def hand_R(approach, x_hint):
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_hint, float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def place_hand(sc, side, aim_E, dist=0.2):
    """Arm `side`'s joints so that its wrist camera looks at aim_E (x, z, d) from about dist (kinematics only)."""
    a = sc.arms[side]
    into = sc.R_E[:, 1]
    aim = sc.E_to_world(*aim_E)
    tcp = aim - into * (dist - 0.05)
    R = hand_R(into, sc.R_E[:, 2])
    q, ep, er, ok, cols = ik_solve_seeds(a, tcp, R, q_ref=a.q(), collision_free=True)
    sc.data.qpos[a.qadr] = q
    mujoco.mj_forward(sc.model, sc.data)
    return bool(ok), float(ep)


def main():
    panda, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(20261109)
    rec = []
    for state in ("no_belt", "fitted"):
        sc = Scene(dict(NOMINAL), panda)
        sc.reset_arms_home()
        if state == "fitted":
            th = fit_belt(sc)
        else:
            hide_belt(sc)
            th = None
        rig = cm.CameraRig(sc.model)
        colour = {k: (np.ones(3), 1.0) for k in rig.sizes}
        home = {s: sc.data.qpos[sc.arms[s].qadr].copy() for s in sc.arms}
        shots = [("front", None, None)]
        for side, cam in (("L", "left_wrist"), ("R", "right_wrist")):
            for i, k in enumerate(("C", "WP", "ALT", "I")):
                c = dr.VARIANTS[dr.CELL_VARIANT]["pulleys"][k]["c"]
                shots.append((cam, side, (c[0], c[1], 0.03)))
        n_wrist = {}
        for cam, side, aim in shots:
            for s in sc.arms:
                sc.data.qpos[sc.arms[s].qadr] = home[s]
            mujoco.mj_forward(sc.model, sc.data)
            ok = True
            if side is not None:
                ok, _ = place_hand(sc, side, aim)
            rgb, dep = rig.render(sc.data, cam)
            kind = cm.CAMERA_SPECS[cam]["kind"]
            rgb2, q = cm.apply_sensor_model(kind, rgb, dep, rng, colour[cam])
            i = n_wrist.get(cam, 0)
            n_wrist[cam] = i + 1
            tag = "%s_%s" % (state, cam) + ("" if side is None else "_%d" % i)
            Image.fromarray(rgb2).save(os.path.join(out, tag + ".png"))
            Image.fromarray(q.astype(np.uint16)).save(os.path.join(out, tag + "_depth.png"))
            rec.append({"file": tag + ".png", "depth_file": tag + "_depth.png", "state": state, "camera": cam,
                        "depth_unit_m": cm.SENSOR[kind]["quant_m"], "intrinsics": rig.intrinsics(cam),
                        "T_world_cam": rig.extrinsics(sc.data, cam).round(6).tolist(),
                        "arms_q": {("left" if s == "L" else "right"): [round(float(v), 6) for v in sc.data.qpos[sc.arms[s].qadr]] for s in sc.arms},
                        "pose_found": ok, "tensioner_theta_deg": None if th is None else round(math.degrees(th), 2)})
    json.dump({"note": "survey frames of the station (nominal engine pose, nominal physics); RGB and 16-bit depth (0 = "
                       "invalid) through the cell's camera model; T_world_cam in the OpenCV convention",
               "frames": rec}, open(os.path.join(out, "survey.json"), "w"), indent=1)
    print("wrote", len(rec), "frames to", out)


if __name__ == "__main__":
    main()
