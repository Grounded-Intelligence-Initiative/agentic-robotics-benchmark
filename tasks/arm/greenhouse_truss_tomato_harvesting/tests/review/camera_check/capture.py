#!/usr/bin/env python3
"""The camera-only check, step 0 (maintainers' tool; nothing runs it at grading; needs Isaac Sim and a GPU): saves
wrist frames of the calibration plate and of every ready truss's peduncle from eight sides, with the truth beside
them, for calibrate.py and peduncle.py.

    python3 capture.py <out dir> <seed> [cultivar]        (ROWCELL_ROBOT_USD names the robot's USD file;
                                                           ROWCELL_KIT_DIR, if set, Kit's cache folder)

The row section is a review row: `farm.draw_row(seed, cultivar, light_seed=seed, mech_seed=seed)` with a fixed
camera mounting offset; it is neither a graded nor a practice row. The views are aimed with the truth (where a
peduncle is), 0.36 to 0.42 m away, so that it lies below the tool's own neck in the picture: the check is about what
the frames hold once a truss is in view, not about finding trusses.
"""
import base64
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from isaacsim import SimulationApp  # noqa: E402

KIT = os.environ.get("ROWCELL_KIT_DIR")                 # where Kit keeps its cache, data and logs (default: its own)
app = SimulationApp({"headless": True, "multi_gpu": False, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"] + (
    ["--/app/folder/cache=%s/cache" % KIT, "--/app/folder/data=%s/data" % KIT, "--/app/folder/logs=%s/logs" % KIT] if KIT else [])})
from rowcell import farm as F  # noqa: E402
from rowcell import kin, wire  # noqa: E402
from rowcell.cell import Cell  # noqa: E402

OUT_ROOT, seed = sys.argv[1], int(sys.argv[2])
cultivar = sys.argv[3] if len(sys.argv) > 3 else "Rossa"
row = F.draw_row(seed, cultivar, light_seed=seed, mech_seed=seed,
                 cameras={"overview": [12, -15, 8, 1.2, -1.6, 0.7], "wrist": [3.5, -4.2, 2.1, 0.9, -1.2, 0.6]})
P = lambda *a: print("##", *a, flush=True)  # noqa: E731
cell = Cell(app, row, robot_usd=os.environ.get("ROWCELL_ROBOT_USD", "/opt/assets/isaac/Isaac/Robots/UniversalRobots/ur20/ur20.usd"), render=True, log=P)
TOOL = wire.TOOL_T


def act(**a):
    return cell.step(wire.check_action(a))


def tcp_cmd():
    return kin.flange(cell.q_cmd) @ TOOL


def goto(T, ticks=20):
    d = kin.pose_error(T, tcp_cmd())
    return act(move={"type": "tcp", "delta": [float(v) for v in d], "ticks": int(ticks)})


def hold(ticks=10, cams=None):
    a = {"move": {"type": "hold", "ticks": int(ticks)}}
    if cams:
        a["cams"] = cams
    return act(**a)


def jmove(q, ticks=40):
    d = np.array(q, float) - cell.q_cmd
    return act(move={"type": "joint", "delta": [float(v) for v in d], "ticks": int(ticks)})


OUT = os.path.join(OUT_ROOT, "row%d" % seed)
os.makedirs(OUT, exist_ok=True)
MOUNT = F.camera_mount("wrist")
def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)
FPX = F.camera_K("wrist")[0][0]
def look(cam_pos, target, up=(0, 0, 1.0), below_px=0.0, right_px=0.0):
    """tool pose (C) that puts the wrist camera (nominal mounting) at cam_pos with `target` at the image centre shifted by
    (right_px, below_px) (the tool's own neck fills the image's top centre), image up = `up`"""
    z = unit(np.array(target) - np.array(cam_pos)); x = unit(np.cross(z, up)); y = np.cross(z, x)
    dist = np.linalg.norm(np.array(target) - np.array(cam_pos))
    aim = np.array(target, float) - y * (below_px * dist / FPX) - x * (right_px * dist / FPX)
    z = unit(aim - np.array(cam_pos)); x = unit(np.cross(z, up)); y = np.cross(z, x)
    Tc = kin.pose(np.stack([x, y, z], axis=1), np.array(cam_pos, float))
    return Tc @ kin.inv(MOUNT) @ TOOL
def reach(T, ticks=30):
    for k in range(8):
        o = goto(T, min(40, ticks)); e = kin.pose_error(T, tcp_cmd())
        if np.linalg.norm(e[:3]) < 1.5e-3 and np.linalg.norm(e[3:]) < 5e-3: return o, True
    return o, False
def shot(tag, extra):
    hold(8)
    o = hold(1, cams=["wrist"])
    im = o["images"]["wrist"]
    rec = dict(extra, tag=tag, rgb=im["rgb"], depth=im["depth"], pose=im["pose"], q=o["q"], tcp=o["tcp"], t=o["time"], stops=o["status"]["protective_stops"])
    tt = cell.truth(); rec["truth_tool"] = tt["tool"]; rec["trusses"] = {k: {"peduncle": v["peduncle"], "r": v["r"], "ready": v["ready"], "fruits": v["fruits"]} for k, v in tt["trusses"].items()}
    rec["plants"] = {k: {"stem": v["stem"], "stem_r": v["stem_r"]} for k, v in tt["plants"].items()}
    pickle.dump(rec, open("%s/%s.pkl" % (OUT, tag), "wb"))
    return o
meta = {"seed": seed, "cameras": row["cameras"], "light": row["light"], "cultivar": row["cultivar"], "K": F.camera_K("wrist"), "mount_nominal": MOUNT.tolist(),
        "plate": F.CAL_PLATE, "card": F.COLOR_CARD}
pickle.dump(meta, open(OUT + "/meta.pkl", "wb"))
# 1. the plate, from six poses (the camera 0.40 to 0.50 m above it)
pc = np.array(F.CAL_PLATE["origin"]) + np.array([0.10, 0.075, 0.0])
n = 0
for dx, dy, h in ((0.0, 0.0, 0.55), (0.10, 0.0, 0.52), (-0.08, 0.05, 0.55), (0.0, -0.10, 0.50), (0.06, 0.10, 0.58), (-0.05, -0.08, 0.50)):
    T = look(pc + np.array([dx, dy, h]), pc, up=(1.0, 0, 0), right_px=300.0)
    o, ok = reach(T, 40)
    if ok: shot("plate%d" % n, {"kind": "plate"}); n += 1
P("plate views", n)
o = hold(1, cams=["overview"]); pickle.dump({"rgb": o["images"]["overview"]["rgb"], "y_cart": cell.y_cart, "odometer": o["cart"]["odometer"]}, open(OUT + "/overview0.pkl", "wb"))
jmove(F.CART["home_q"], 40); jmove(F.CART["home_q"], 40)
# 2. every ready truss's peduncle from five sides, 0.30 m away
done = 0
for p in row["plants"]:
    ready = [t for t in p["trusses"] if t["ready"]]
    if not ready: continue
    ynode = ready[0]["node"][1]
    for _ in range(12):
        if abs(cell.y_cart - ynode) < 0.01: break
        act(cart=float(np.clip((ynode - cell.y_cart) / (1 - cell.slip), -0.3, 0.3)), move={"type": "hold", "ticks": 40})
    hold(20)
    o = hold(1, cams=["overview"]); pickle.dump({"rgb": o["images"]["overview"]["rgb"], "y_cart": cell.y_cart, "odometer": o["cart"]["odometer"]}, open(OUT + "/overview_%s.pkl" % p["id"], "wb"))
    for t in ready:
        tr = cell.truth()["trusses"][t["id"]]
        a, b = np.array(tr["peduncle"][0]), np.array(tr["peduncle"][1]); mid = 0.5 * (a + b)
        k = 0
        for yaw, pitch, dist in ((0, 10, 0.40), (25, 10, 0.40), (-25, 10, 0.40), (0, 35, 0.40), (12, -10, 0.42), (-12, 22, 0.36), (35, 20, 0.42), (-35, 20, 0.42)):
            d = np.array([np.cos(np.radians(pitch)) * np.cos(np.radians(yaw)), np.cos(np.radians(pitch)) * np.sin(np.radians(yaw)), -np.sin(np.radians(pitch))])
            T = look(mid - dist * d, mid, below_px=170.0)
            if T[0, 3] > 1.03: continue
            o, ok = reach(T, 30)
            if ok and o["status"]["protective_stops"] == 0:
                shot("%s_v%d" % (t["id"], k), {"kind": "truss", "truss": t["id"], "view": [yaw, pitch, dist]}); k += 1
        P(t["id"], "views", k, "stops", o["status"]["protective_stops"])
        done += 1
    jmove(F.CART["home_q"], 40); jmove(F.CART["home_q"], 40)
P("captured", done, "trusses; events", [e for e in cell.rules.events][:6])

app.close()
