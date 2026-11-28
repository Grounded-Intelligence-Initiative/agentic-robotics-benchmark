"""collide: the interpenetration check on the photographed unit (grader only; never shipped to the agent).

The collision envelope is the convex hull of each listed part of the unit (tests/gt/robot.xml carries one hull geom
per part, named by the part's instance id). Pairs that count: parts of different digits, and a digit's part with a
palm part. Excluded: pairs whose two bodies are a parent and a child link (a jointless body belongs to its nearest
jointed ancestor, the palm's bodies to the palm) and pairs whose hulls touch (closer than TOUCH_TOL_M) at q = 0.

    pairs = counted_pairs(unit.truth)                        # [(instance a, instance b), ...]
    depth_mm = max_penetration_mm(unit, ex.q_exec)          # 0.0 when no counted pair interpenetrates

Distances are MuJoCo's mj_geomDistance between the hull geoms (negative = penetration depth).
"""
import mujoco
import numpy as np

try:
    from . import contract as C
except ImportError:  # loaded as a plain module beside contract.py
    import contract as C

TOUCH_TOL_M = 1e-6
DISTMAX_M = 0.01


def _digit_of_body(m, b, first):
    x = b
    while x > 0:
        for d, fb in first.items():
            if x == fb:
                return d
        x = int(m.body_parentid[x])
    return C.ROOT_BODY


def _link_of(m, b, palm):
    x = b
    while x not in (0, palm) and m.body_jntnum[x] == 0:
        x = int(m.body_parentid[x])
    return x


class Collider(object):
    """The counted hull pairs of a unit model, found once."""

    def __init__(self, hm):
        m = hm.model
        self.hm = hm
        self.data = mujoco.MjData(m)
        palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.ROOT_BODY)
        first = {d: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.FIRST_BODY[d]) for d in C.DIGITS}
        geoms = []
        for g in range(m.ngeom):
            if int(m.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_MESH):
                continue
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            if not name:
                continue
            b = int(m.geom_bodyid[g])
            geoms.append((name, g, b, _digit_of_body(m, b, first), _link_of(m, b, palm)))
        d = self.data
        d.qpos[:] = m.qpos0
        mujoco.mj_kinematics(m, d)
        pairs = []
        for i, (na, ga, ba, da, la) in enumerate(geoms):
            for nb, gb, bb, db, lb in geoms[i + 1:]:
                if da == db:
                    continue
                if int(m.body_parentid[la]) == lb or int(m.body_parentid[lb]) == la:
                    continue
                if float(mujoco.mj_geomDistance(m, d, ga, gb, DISTMAX_M, None)) <= TOUCH_TOL_M:
                    continue
                pairs.append((na, nb, ga, gb))
        self.pairs = pairs

    def names(self):
        return [(a, b) for a, b, _, _ in self.pairs]

    def max_penetration_m(self, qpos):
        m, d = self.hm.model, self.data
        d.qpos[:] = qpos
        mujoco.mj_kinematics(m, d)
        worst = 0.0
        for _, _, ga, gb in self.pairs:
            dist = float(mujoco.mj_geomDistance(m, d, ga, gb, 0.0, None))
            if -dist > worst:
                worst = -dist
        return worst


def collider_of(unit):
    if unit._collider is None:
        unit._collider = Collider(unit.truth)
    return unit._collider


def counted_pairs(hm):
    return Collider(hm).names()


def max_penetration_mm(unit, q_exec):
    """The deepest interpenetration (mm) of a counted pair of the unit at 13 driven angles q_exec (dict or KinState)."""
    st = q_exec if hasattr(q_exec, "qpos") else unit.solve(q_exec)
    return 1000.0 * collider_of(unit).max_penetration_m(np.asarray(st.qpos, float))
