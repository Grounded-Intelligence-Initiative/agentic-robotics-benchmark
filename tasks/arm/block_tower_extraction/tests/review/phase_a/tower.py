"""Phase-A prototype: block tower generator + MuJoCo model (optionally with the Panda)."""
import math
import os

import mujoco
import numpy as np

L, W, H = 0.075, 0.025, 0.015          # nominal block
PITCH = 0.0265                          # slot pitch (1.5 mm nominal side gap)
LEVELS = 18
TOWER_XY = np.array([0.65, 0.0])
TOWER_YAW = 0.0                         # level 0 blocks lie along this yaw (turntable at 0); odd levels +90 deg
PED_TOP = 0.115                         # pedestal top (tower base) z
PED_HALF = 0.030                        # turntable half side (square, inside the tower footprint)

MARGIN = 5e-5                           # per-geom contact margin (pair margin = sum); see DECISIONS
PHYS = dict(dt=0.001, solref=(0.002, 1.0), solimp=(0.99, 0.999, 0.0002, 0.5, 2.0), impratio=10.0,
            noslip=0, iterations=100, ls_iterations=30, tol=1e-10)

PATTERNS = {            # tight slot sets per level pattern
    "M": (1,), "SS": (0, 2), "MS0": (0, 1), "MS2": (1, 2), "ALL": (0, 1, 2),
}


def level_yaw(k):
    return TOWER_YAW + (math.pi / 2 if k % 2 else 0.0)


def gen_tower(seed, p_all=0.15, mu=(0.35, 0.5), gap=(0.12e-3, 0.30e-3), mid_tight=0.0):
    """Hidden per-block parameters. mid_tight: extra probability of ALL-tight in levels 5..12."""
    rng = np.random.default_rng(seed)
    blocks = []
    pats = []
    for k in range(LEVELS):
        pa = p_all + (mid_tight if 5 <= k <= 12 else 0.0)
        if rng.random() < pa:
            pat = "ALL"
        else:
            pat = rng.choice(["M", "SS", "MS0", "MS2"], p=[0.3, 0.3, 0.2, 0.2])
        pats.append(pat)
        tight = PATTERNS[pat]
        h_t = H + rng.uniform(-0.1e-3, 0.1e-3)
        for s in range(3):
            if s in tight:
                h = h_t
            else:
                h = h_t - rng.uniform(*gap)
            blocks.append(dict(level=k, slot=s, h=h, tight=s in tight,
                               mu=float(rng.uniform(*mu)), mass=float(rng.uniform(0.0145, 0.0185)),
                               dax=float(rng.uniform(-2e-3, 2e-3)), dlat=float(rng.uniform(-0.3e-3, 0.3e-3)),
                               dyaw=float(rng.uniform(-math.radians(0.6), math.radians(0.6)))))
    # z of each level's base: rests on the tallest block of the level below
    z = PED_TOP
    for k in range(LEVELS):
        lv = [b for b in blocks if b["level"] == k]
        for b in lv:
            b["z0"] = z
        z += max(b["h"] for b in lv)
    for b in blocks:
        yaw = level_yaw(b["level"])
        ax = np.array([math.cos(yaw), math.sin(yaw)])
        lat = np.array([-math.sin(yaw), math.cos(yaw)])
        c = TOWER_XY + ax * b["dax"] + lat * ((b["slot"] - 1) * PITCH + b["dlat"])
        b["pos"] = [float(c[0]), float(c[1]), float(b["z0"] + b["h"] / 2 + 2 * MARGIN * (b["level"] + 1))]
        b["yaw"] = float(yaw + b["dyaw"])
    return dict(seed=seed, blocks=blocks, patterns=pats)


def yaw_quat(y):
    return [math.cos(y / 2), 0.0, 0.0, math.sin(y / 2)]


def build_spec(tower, panda_dir=None, phys=PHYS, condim=3):
    if panda_dir:
        spec = mujoco.MjSpec.from_file(os.path.join(panda_dir, "panda.xml"))
        for b in spec.bodies:
            if b.name != "world":
                b.gravcomp = 1.0
    else:
        spec = mujoco.MjSpec()
    spec.option.timestep = phys["dt"]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = phys["impratio"]
    spec.option.iterations = phys["iterations"]
    spec.option.ls_iterations = phys["ls_iterations"]
    spec.option.tolerance = phys["tol"]
    spec.option.noslip_iterations = phys["noslip"]
    wb = spec.worldbody
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.5, 1.5, 0.01], rgba=[0.5, 0.5, 0.52, 1])
    wb.add_geom(name="pedestal", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[TOWER_XY[0], TOWER_XY[1], (PED_TOP - 0.012) / 2],
                size=[0.02, (PED_TOP - 0.012) / 2, 0], rgba=[0.3, 0.3, 0.33, 1])
    tt = wb.add_body(name="turntable", pos=[TOWER_XY[0], TOWER_XY[1], PED_TOP - 0.005])
    tt.add_joint(name="turntable_j", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=2.0, armature=0.05)
    g = tt.add_geom(name="turntable_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[PED_HALF, PED_HALF, 0.005],
                    rgba=[0.3, 0.3, 0.33, 1], friction=[0.8, 0.005, 0.0001], solref=list(phys["solref"]),
                    solimp=list(phys["solimp"]), margin=MARGIN)
    g.mass = 0.5
    a = spec.add_actuator(name="turntable", target="turntable_j", trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = 200.0
    a.biasprm[:3] = [0.0, -200.0, -20.0]
    for i, b in enumerate(tower["blocks"]):
        bd = wb.add_body(name="blk%d" % i, pos=b["pos"], quat=yaw_quat(b["yaw"]))
        bd.add_freejoint(name="blk%d_j" % i)
        g = bd.add_geom(name="blk%d_g" % i, type=mujoco.mjtGeom.mjGEOM_BOX, size=[L / 2, W / 2, b["h"] / 2],
                        rgba=[0.80, 0.62, 0.40, 1], friction=[b["mu"], 0.002, 0.0001], condim=condim,
                        solref=list(phys["solref"]), solimp=list(phys["solimp"]), margin=MARGIN)
        g.mass = b["mass"]
    return spec


def block_ids(m, n):
    return [m.body("blk%d" % i).id for i in range(n)]


def add_gantry_probe(spec, start):
    """A test probe on three world slide joints with stiff position servos (Phase-A force tests only)."""
    b = spec.worldbody.add_body(name="gprobe", pos=list(start))
    for ax, nm in zip(([1, 0, 0], [0, 1, 0], [0, 0, 1]), "xyz"):
        b.add_joint(name="gp_" + nm, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=ax, damping=5.0, armature=0.05)
    g = b.add_geom(name="gprobe_g", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.003, 0, 0], rgba=[0.9, 0.1, 0.1, 1],
                   friction=[0.3, 0.002, 0.0001], solref=[0.002, 1.0], solimp=[0.99, 0.999, 0.0002, 0.5, 2.0])
    g.mass = 0.05
    for nm in "xyz":
        a = spec.add_actuator(name="gpa_" + nm, target="gp_" + nm, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = 20000.0
        a.biasprm[:3] = [0.0, -20000.0, -200.0]
    return b


def settle(m, d, seconds=2.0):
    for _ in range(int(round(seconds / m.opt.timestep))):
        mujoco.mj_step(m, d)


def block_loads(m, d, ids):
    """Per block: (normal force pressing on its top face from above, total normal force from below) [N]."""
    idx = {b: i for i, b in enumerate(ids)}
    top = np.zeros(len(ids))
    f6 = np.zeros(6)
    for k in range(d.ncon):
        c = d.contact[k]
        if c.efc_address < 0:
            continue
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if b1 not in idx or b2 not in idx:
            continue
        mujoco.mj_contactForce(m, d, k, f6)
        n = c.frame[:3]
        if abs(n[2]) < 0.9:
            continue
        lo, hi = (b1, b2) if d.xpos[b1][2] < d.xpos[b2][2] else (b2, b1)
        top[idx[lo]] += abs(f6[0])
    return top
