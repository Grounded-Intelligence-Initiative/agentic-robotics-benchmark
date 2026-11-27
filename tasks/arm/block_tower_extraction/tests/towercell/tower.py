"""The tower: hidden per-tower parameters (drawn from the tower's seed) and its MuJoCo bodies.

Tightness is geometry: per level a pattern of load-bearing slots; the other blocks of the level are shorter by the
loose gap, so after settling nothing presses on them from above. Every block also carries its own friction, mass and
small placement offsets; the whole tower sits slightly off the turntable's axis and slightly rotated.

Contact model (DECISIONS): every block / turntable geom has a contact margin of 5e-5 m (a pair's margin is the sum),
solref (0.002, 1), solimp (0.99, 0.999, 0.0002): with margin 0 MuJoCo's box-box collider drops points of face pairs
that tilt by micro-radians and an 18-level tower creeps. Blocks rest ~0.1 mm apart (invisible).
"""

import math

import mujoco
import numpy as np

from .config import CONFIG, RANGES, level_yaw

MARGIN = 5e-5
SOLREF = [0.002, 1.0]
SOLIMP = [0.99, 0.999, 0.0002, 0.5, 2.0]
PATTERNS = {"M": (1,), "SS": (0, 2), "MS0": (0, 1), "MS2": (1, 2), "ALL": (0, 1, 2)}


def draw_tower(seed, overrides=None):
    """The hidden parameters of tower `seed`: per-block height, friction, mass, offsets; the level patterns; the
    tower's offset on the turntable. `overrides` (hidden-set packages): p_all, mid_tight, mu (pair)."""
    o = dict(overrides or {})
    rng = np.random.default_rng([int(seed), 20261124])
    blk = CONFIG["block"]
    tw = CONFIG["tower"]
    p_all = float(o.get("p_all", 0.15))
    mid_tight = float(o.get("mid_tight", 0.0))
    mu_lo, mu_hi = o.get("mu", RANGES["block_friction"])
    gap_lo, gap_hi = RANGES["loose_block_gap_m"]
    blocks, pats = [], []
    for k in range(tw["levels"]):
        pa = p_all + (mid_tight if 5 <= k <= 12 else 0.0)
        pat = "ALL" if rng.random() < pa else str(rng.choice(["M", "SS", "MS0", "MS2"], p=[0.3, 0.3, 0.2, 0.2]))
        # never two middle-only levels in a row (a 25 x 25 mm pillar)
        if pat == "M" and pats and pats[-1] == "M":
            pat = "SS"
        pats.append(pat)
        h_t = blk["height_m"] + rng.uniform(-1e-4, 1e-4)
        for s in range(3):
            tight = s in PATTERNS[pat]
            h = h_t if tight else h_t - rng.uniform(gap_lo, gap_hi)
            blocks.append(dict(level=k, slot=s, h=float(h), tight=tight, mu=float(rng.uniform(mu_lo, mu_hi)),
                               mass=float(rng.uniform(*RANGES["block_mass_kg"])),
                               dax=float(rng.uniform(*RANGES["block_axial_offset_m"])),
                               dlat=float(rng.uniform(*RANGES["block_lateral_offset_m"])),
                               dyaw=float(math.radians(rng.uniform(*RANGES["block_yaw_offset_deg"])))))
    off = rng.uniform(*RANGES["tower_offset_on_turntable_m"], size=2)
    tyaw = math.radians(rng.uniform(*RANGES["tower_yaw_offset_deg"]))
    return {"seed": int(seed), "blocks": blocks, "patterns": pats, "offset_xy": [float(off[0]), float(off[1])],
            "yaw_offset": float(tyaw), "overrides": o}


def initial_poses(tower):
    """World poses (pos, yaw) of the blocks at turntable angle 0, before settling."""
    tw = CONFIG["tower"]
    axis = np.array(tw["turntable_axis_xy"], float)
    c = axis + np.array(tower["offset_xy"])
    ty = tower["yaw_offset"]
    pitch = tw["slot_pitch_m"]
    z = tw["base_z"]
    out = []
    by_level = {}
    for b in tower["blocks"]:
        by_level.setdefault(b["level"], []).append(b)
    zbase = {}
    for k in range(tw["levels"]):
        zbase[k] = z
        z += max(b["h"] for b in by_level[k])
    for b in tower["blocks"]:
        yaw = level_yaw(b["level"]) + ty
        ax = np.array([math.cos(yaw), math.sin(yaw)])
        lat = np.array([-math.sin(yaw), math.cos(yaw)])
        xy = c + ax * b["dax"] + lat * ((b["slot"] - 1) * pitch + b["dlat"])
        pz = zbase[b["level"]] + b["h"] / 2 + 2 * MARGIN * (b["level"] + 1)
        out.append(([float(xy[0]), float(xy[1]), float(pz)], float(yaw + b["dyaw"])))
    return out


def yaw_quat(y):
    return [math.cos(y / 2), 0.0, 0.0, math.sin(y / 2)]


def add_tower(spec, tower, wood_rgba=(0.80, 0.62, 0.40, 1.0)):
    """Turntable (hinge + position servo `turntable`), its pedestal and the 54 free blocks."""
    tw = CONFIG["tower"]
    wb = spec.worldbody
    ax = tw["turntable_axis_xy"]
    base = tw["base_z"]
    wb.add_geom(name="pedestal", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[ax[0], ax[1], (base - 0.012) / 2],
                size=[0.02, (base - 0.012) / 2, 0], rgba=[0.3, 0.3, 0.33, 1])
    tt = wb.add_body(name="turntable", pos=[ax[0], ax[1], base - 0.005])
    tt.add_joint(name="turntable_j", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=2.0, armature=0.05)
    half = tw["turntable_top_half_m"]
    g = tt.add_geom(name="turntable_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[half, half, 0.005],
                    rgba=[0.3, 0.3, 0.33, 1], friction=[0.8, 0.005, 0.0001], solref=SOLREF, solimp=SOLIMP,
                    margin=MARGIN)
    g.mass = 0.5
    # a white index mark on the turntable's +x edge (angle 0 faces +x)
    tt.add_geom(name="turntable_mark", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[half - 0.003, 0, 0.0051],
                size=[0.003, 0.002, 0.0001], rgba=[0.95, 0.95, 0.95, 1], contype=0, conaffinity=0)
    a = spec.add_actuator(name="turntable", target="turntable_j", trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = 200.0
    a.biasprm[:3] = [0.0, -200.0, -20.0]
    L, W = CONFIG["block"]["length_m"], CONFIG["block"]["width_m"]
    for i, (b, (pos, yaw)) in enumerate(zip(tower["blocks"], initial_poses(tower))):
        bd = wb.add_body(name="blk%d" % i, pos=pos, quat=yaw_quat(yaw))
        bd.add_freejoint(name="blk%d_j" % i)
        shade = 0.92 + 0.08 * ((i * 37) % 11) / 10.0
        rgba = [wood_rgba[0] * shade, wood_rgba[1] * shade, wood_rgba[2] * shade, 1.0]
        g = bd.add_geom(name="blk%d_g" % i, type=mujoco.mjtGeom.mjGEOM_BOX, size=[L / 2, W / 2, b["h"] / 2],
                        rgba=rgba, friction=[b["mu"], 0.002, 0.0001], condim=3, solref=SOLREF, solimp=SOLIMP,
                        margin=MARGIN)
        g.mass = b["mass"]
