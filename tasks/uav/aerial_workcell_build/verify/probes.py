"""Measure a workcell scene in Isaac Sim: what is where, what it weighs, how the bin's mechanism behaves.

    python3 probes.py <scene> <out.json> <group>      (groups: layout settle bin_lift bin_open bin_button bin_drop)

Everything is read from PhysX or from the stage as authored, never from anything the scene reports about itself, and
everything is pushed from outside (forces on bodies). The grader runs this on the submission as an unprivileged
account after the static scan passed (no code in the scene), and compares the result with the same measurement of
the reference build (truth.json). The grader starts one process per group (a stage opened after another one in the
same process does not start from a clean simulation clock); each group records its own errors.
"""

import json
import math
import os
import sys
import time
import traceback

import numpy as np
from isaacsim import SimulationApp

SCENE, OUT, GROUP = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 1.0 / 120.0
app = SimulationApp({"headless": True})
import omni.kit.app  # noqa: E402

for _ext in ("isaacsim.core.nodes", "isaacsim.sensors.physics"):
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(_ext, True)
import omni.physics.tensors as pt  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

RESULT = {"scene": SCENE, "groups": {}}
PURPOSES = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]


# --------------------------------------------------------------------------------------------- math
def qinv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def rot(q, v):
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r @ np.asarray(v, dtype=float)


def angle_deg(q0, q1):
    return math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q0, q1))))))


def yaw_deg(q):
    """Heading of the body's own x axis in the world xy plane."""
    v = rot(q, (1.0, 0.0, 0.0))
    return math.degrees(math.atan2(v[1], v[0]))


def gfq(qd):
    return np.array([qd.GetReal(), *qd.GetImaginary()])


def r3(v):
    return [round(float(x), 4) for x in v]


# --------------------------------------------------------------------------------------------- stage
class Scene:
    def __init__(self):
        ctx = omni.usd.get_context()
        ctx.open_stage(SCENE)
        for _ in range(3):
            app.update()
        self.ctx, self.stage = ctx, ctx.get_stage()
        self.cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PURPOSES, useExtentsHint=False)
        self.sim = SimulationContext(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
        self.t = 0.0

    def start(self):
        self.sim.reset()
        self.view = pt.create_simulation_view("numpy", self.ctx.get_stage_id())

    def step(self, n=1):
        for _ in range(n):
            self.sim.step(render=False)
            self.t += DT

    def usd_pose(self, path):
        m = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(m.ExtractTranslation()), gfq(m.ExtractRotationQuat())

    def world_box(self, path):
        r = self.cache.ComputeWorldBound(self.stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(r.GetMin()), np.array(r.GetMax())

    def own_center(self, path):
        """Bounding box centre and size in the prim's own frame (its body frame for a rigid body)."""
        r = self.cache.ComputeUntransformedBound(self.stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(r.GetMidpoint()), np.array(r.GetSize())


def group(name):
    def deco(fn):
        def run():
            t0 = time.time()
            out = {}
            try:
                fn(out)
            except Exception:  # noqa: BLE001
                out["error"] = traceback.format_exc()[-1500:]
            out["wall_s"] = round(time.time() - t0, 1)
            RESULT["groups"][name] = out
            with open(OUT, "w") as f:
                json.dump(RESULT, f)
        run.group_name = name
        return run
    return deco


def rigid_bodies(stage, root):
    p = stage.GetPrimAtPath(root)
    if not p.IsValid():
        return []
    return [q.GetPath().pathString for q in Usd.PrimRange(p) if q.HasAPI(UsdPhysics.RigidBodyAPI)]


# --------------------------------------------------------------------------------------------- groups
@group("layout")
def layout(out):
    """Static things, as authored: tables, bin, button, cameras."""
    s = Scene()
    for name in ("Table_A", "Table_B"):
        lo, hi = s.world_box(f"/World/Cell/{name}")
        out[name] = dict(center=r3((lo + hi)[:2] / 2), extent=r3((hi - lo)[:2]), top=round(float(hi[2]), 4))
    lo, hi = s.world_box("/World/Bin/Body")
    out["bin_body"] = dict(center=r3((lo + hi)[:2] / 2), extent=r3((hi - lo)[:2]), top=round(float(hi[2]), 4))
    lo, hi = s.world_box("/World/Bin/Lid")
    out["bin_lid"] = dict(center=r3((lo + hi)[:2] / 2), extent=r3((hi - lo)[:2]), top=round(float(hi[2]), 4))
    lo, hi = s.world_box("/World/Bin/Lid/Handle")
    out["handle"] = r3((lo + hi) / 2)
    lo, hi = s.world_box("/World/Bin/Button")
    out["button"] = dict(center=r3((lo + hi) / 2), top=round(float(hi[2]), 4))
    out["robots"] = {}
    for drone in ("Drone_L", "Drone_S"):
        lo, hi = s.world_box(f"/World/Robots/{drone}")
        out["robots"][drone] = dict(center=r3((lo + hi) / 2), extent=r3(hi - lo))
    cams = {}
    for drone in ("Drone_L", "Drone_S"):
        root = s.stage.GetPrimAtPath(f"/World/Robots/{drone}")
        art = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        base = art[0].GetPath().pathString if art else f"/World/Robots/{drone}"
        bp, bq = s.usd_pose(base)
        lst = []
        for p in Usd.PrimRange(root):
            if not p.IsA(UsdGeom.Camera):
                continue
            c = UsdGeom.Camera(p)
            cp, cq = s.usd_pose(p.GetPath().pathString)
            focal, hap = c.GetFocalLengthAttr().Get(), c.GetHorizontalApertureAttr().Get()
            look = rot(cq, (0.0, 0.0, -1.0))  # USD cameras look along their -z
            look_b = rot(qinv(bq), look)
            lst.append(dict(path=p.GetPath().pathString, pos_in_base=r3(rot(qinv(bq), cp - bp)), look_in_base=r3(look_b),
                            pitch_down_deg=round(math.degrees(math.asin(max(-1.0, min(1.0, -look[2])))), 2),
                            hfov_deg=round(math.degrees(2 * math.atan(hap / 2 / focal)), 2) if focal else None))
        cams[drone] = dict(base=base, cameras=lst)
    out["cameras"] = cams


@group("settle")
def settle(out):
    """Start the simulation, let it run 5 s: does everything stay put; where do the movable things rest; masses."""
    s = Scene()
    bodies = {}
    for root in ("/World/Bin", "/World/Items", "/World/Robots"):
        for b in rigid_bodies(s.stage, root):
            bodies[b] = s.usd_pose(b)
    s.start()
    views = {}
    for b in bodies:
        try:
            views[b] = s.view.create_rigid_body_view(b)
        except Exception:  # noqa: BLE001
            pass
    s.step(int(5.0 / DT))
    moved = {}
    for b, v in views.items():
        tf = v.get_transforms()[0]
        moved[b] = float(np.linalg.norm(tf[:3] - bodies[b][0])) if np.all(np.isfinite(tf)) else 99.0
    out["moved"] = {k: round(v, 4) for k, v in moved.items()}
    items = {}
    for prim in s.stage.GetPrimAtPath("/World/Items").GetChildren():
        path = prim.GetPath().pathString
        if path not in views:
            continue
        tf = views[path].get_transforms()[0]
        p, q = tf[:3], np.array([tf[6], tf[3], tf[4], tf[5]])
        c_own, size = s.own_center(path)
        items[prim.GetName()] = dict(center=r3(p + rot(q, c_own)), size=r3(sorted(size)),
                                     mass=round(float(views[path].get_masses()[0][0]), 4), moved=round(moved[path], 4))
    out["items"] = items
    robots = {}
    for drone in ("Drone_L", "Drone_S"):
        root = s.stage.GetPrimAtPath(f"/World/Robots/{drone}")
        art = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if not art:
            continue
        base = art[0].GetPath().pathString
        av = s.view.create_articulation_view(base)
        meta = av.shared_metatype
        tf = av.get_root_transforms()[0]
        robots[drone] = dict(base=base, mass=round(float(np.sum(av.get_masses()[0])), 4), links=int(av.max_links),
                             dofs=int(av.max_dofs), dof_types=[int(t) for t in meta.dof_types],
                             fixed_base=bool(meta.fixed_base), position=r3(tf[:3]),
                             yaw=round(yaw_deg(np.array([tf[6], tf[3], tf[4], tf[5]])), 2))
    out["robots"] = robots


class BinRig:
    """The bin, pushed from outside: forces at the handle (along the direction that turns the lid about its hinge)
    and on the button; the lid's opening = its rotation relative to the body, against the stage as authored."""

    def __init__(self):
        self.s = Scene()
        st = self.s.stage
        self.handle0 = sum(self.s.world_box("/World/Bin/Lid/Handle")) / 2
        bp, bq = self.s.usd_pose("/World/Bin/Body")
        lp, lq = self.s.usd_pose("/World/Bin/Lid")
        self.rel0 = qmul(qinv(bq), lq)
        self.handle_in_lid = rot(qinv(lq), self.handle0 - lp)
        j = st.GetPrimAtPath("/World/Bin/LidHinge")
        axis = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[j.GetAttribute("physics:axis").Get() or "X"]
        lpos = np.array(j.GetAttribute("physics:localPos0").Get() or (0, 0, 0), dtype=float)
        lrot = j.GetAttribute("physics:localRot0").Get()
        lrot = gfq(lrot) if lrot is not None else np.array([1.0, 0, 0, 0])
        self.hinge_p = bp + rot(bq, lpos)  # (the body is fixed: its authored pose is its pose)
        self.hinge_a = rot(bq, rot(lrot, axis))
        self.s.start()
        self.art = self.s.view.create_articulation_view("/World/Bin/Body")
        names = list(self.art.shared_metatype.link_names)
        self.i_body, self.i_lid, self.i_btn = names.index("Body"), names.index("Lid"), names.index("Button")
        self.nl = self.art.max_links
        self.btn0 = self.pose(self.i_btn)[0]
        self.probe = None

    def pose(self, i):
        tf = self.art.get_link_transforms()[0, i]
        return tf[:3].copy(), np.array([tf[6], tf[3], tf[4], tf[5]])

    def opening(self):
        return angle_deg(self.rel0, qmul(qinv(self.pose(self.i_body)[1]), self.pose(self.i_lid)[1]))

    def handle(self):
        p, q = self.pose(self.i_lid)
        return p + rot(q, self.handle_in_lid)

    def open_dir(self):
        h = self.handle()
        d = np.cross(self.hinge_a, h - self.hinge_p)
        n = np.linalg.norm(d)
        if n < 1e-6:
            return np.array([0.0, 0.0, 1.0])
        d /= n
        return d if np.dot(np.cross(self.hinge_a, self.handle0 - self.hinge_p), (0, 0, 1)) >= 0 else -d

    def travel_mm(self):
        return float((self.btn0[2] - self.pose(self.i_btn)[0][2]) * 1000.0)

    def step(self, lid=0.0, button=0.0):
        if lid or button:
            f = np.zeros((1, self.nl, 3), dtype=np.float32)
            pos = np.zeros((1, self.nl, 3), dtype=np.float32)
            if lid:
                f[0, self.i_lid] = lid * self.open_dir()
                pos[0, self.i_lid] = self.handle()
            if button:
                f[0, self.i_btn] = (0.0, 0.0, -button)
                pos[0, self.i_btn] = self.pose(self.i_btn)[0]
            self.art.apply_forces_and_torques_at_position(f, np.zeros_like(f), pos, np.array([0], dtype=np.int32), True)
        self.s.step()

    def run(self, seconds, **kw):
        tr = []
        for _ in range(int(round(seconds / DT))):
            self.step(**kw)
            tr.append(self.opening())
        return tr

    def drive_to(self, target, closing=False, f_max=8.0, timeout=10.0):
        """Push (or pull) the handle until the lid passes `target`: a proportional push plus one that grows with
        time, so a lid that needs a steady force near the target still gets there (up to f_max)."""
        for i in range(int(timeout / DT)):
            a = self.opening()
            if (a <= target) if closing else (a >= target):
                return True
            f = min(f_max, 1.5 + 0.4 * abs(target - a) + 3.0 * i * DT)
            self.step(lid=-f if closing else f)
        return False


@group("bin_lift")
def bin_lift(out):
    b = BinRig()
    out["rest_deg"] = round(max(b.run(1.0)), 2)
    out["hold_1N_max_deg"] = round(max(b.run(3.0, lid=1.0)), 2)
    f_open = None
    for i in range(int(8.0 / DT)):
        f = (i * DT) * 1.0  # 0 .. 8 N over 8 s
        b.step(lid=f)
        if b.opening() > 3.0:
            f_open = f
            break
    out["lift_threshold_N"] = round(f_open, 2) if f_open is not None else None
    out["after_release_deg"] = round(b.run(4.0)[-1], 2)
    out["lifted_to_20"] = round(b.opening(), 2) if b.drive_to(20.0) else None
    out["fall_back_deg"] = round(b.run(4.0)[-1], 2)


@group("bin_open")
def bin_open(out):
    b = BinRig()
    b.run(1.0)
    out["lifted_to_35"] = round(b.opening(), 2) if b.drive_to(35.0) else None
    tr = b.run(8.0)
    out["open_after_8s_deg"] = round(tr[-1], 2)
    tr = b.run(10.0)
    out["stay_drift_deg"] = round(max(tr) - min(tr), 2)
    stop = tr[-1]
    out["pulled_back_to"] = round(b.opening(), 2) if b.drive_to(stop - 11.0, closing=True, f_max=4.0) else None
    tr = b.run(40.0)
    out["soft_close_end_deg"] = round(tr[-1], 2)
    out["soft_close_s"] = next((round(i * DT, 2) for i, a in enumerate(tr) if a < 3.0), None)
    out["soft_close_max_rate_dps"] = round(float(np.max(np.abs(np.diff(tr))) / DT), 2) if len(tr) > 1 else None


@group("bin_button")
def bin_button(out):
    b = BinRig()
    b.run(2.0)
    peak = 0.0
    tr = []
    for _ in range(int(0.5 / DT)):
        b.step(button=8.0)
        peak = max(peak, b.travel_mm())
        tr.append(b.opening())
    out["button_travel_mm"] = round(peak, 2)
    tr += b.run(31.5)
    at = lambda s: round(tr[min(len(tr) - 1, int(s / DT))], 2)  # noqa: E731
    out.update(open_3s=at(3), open_6s=at(6), open_15s=at(15), open_19s=at(19), open_25s=at(25), open_30s=at(30),
               open_max=round(max(tr), 2))
    # how long the lid stays open: from the press until it has dropped 5 deg below its highest point
    i_top = int(np.argmax(tr))
    i_drop = next((i for i in range(i_top, len(tr)) if tr[i] < tr[i_top] - 5.0), None)
    out["open_duration_s"] = round(i_drop * DT, 2) if i_drop is not None and tr[i_top] > 60 else None
    # a second press while the lid is shut again: the bin must answer every press, not just the first
    b.run(1.0)
    for _ in range(int(0.5 / DT)):
        b.step(button=8.0)
    tr2 = b.run(6.0)
    out["second_press_open_6s"] = round(tr2[-1], 2)


@group("bin_drop")
def bin_drop(out):
    """Open the lid with the button, drop a 5 cm, 50 g cube from 0.3 m above the lid's closed footprint: inside?"""
    b = BinRig()
    lo, hi = b.s.world_box("/World/Bin/Lid")  # the closed lid covers the bin's opening (the post is part of the body)
    mid = (lo + hi) / 2
    b.run(1.0)
    for _ in range(int(0.5 / DT)):
        b.step(button=8.0)
    b.run(5.0)
    out["open_before_drop_deg"] = round(b.opening(), 2)
    # the probe cube is created now, on the running stage, and dropped
    from isaacsim.core.api.objects import DynamicCuboid
    cube = DynamicCuboid(prim_path="/World/GraderProbe/cube", name="probe_cube", size=0.05, mass=0.05,
                         position=np.array([mid[0], mid[1], hi[2] + 0.30]))
    cube.initialize()
    b.s.step(int(3.0 / DT))
    p = cube.get_world_pose()[0]
    out["cube_end"] = r3(p)
    # inside: under where the closed lid's underside is, within its footprint (a cube on a shut lid, or beside the
    # bin, is not inside)
    out["cube_inside"] = bool(lo[0] < p[0] < hi[0] and lo[1] < p[1] < hi[1] and p[2] < lo[2])


if __name__ == "__main__":
    {g.group_name: g for g in (layout, settle, bin_lift, bin_open, bin_button, bin_drop)}[GROUP]()
    RESULT["done"] = True
    with open(OUT, "w") as f:
        json.dump(RESULT, f)
    print("PROBES DONE", flush=True)
    os._exit(0)
