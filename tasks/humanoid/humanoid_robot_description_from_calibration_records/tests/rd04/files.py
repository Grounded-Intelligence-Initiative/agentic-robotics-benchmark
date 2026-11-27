"""Loading a submission: load(sub_dir) -> Files | GateFail(reasons).

The submission folder holds unit.xml and calibration.yaml. The gate (gate.py beside this module: the same file as the
agent's checker imports) decides pass or fail; after a pass this module reads what the grader uses and checks it
again (defence in depth: anything it cannot read is a GateFail with code 'reader_*', never an exception). Its own
mass check is the gate's: a mass above 0 on each of the 26 link bodies (a jointless body added below a link may have
none, e.g. an imu board). Nothing submitted is executed.

Files: the compiled model (its kinematics in the given frames), the home table (sign, home per slot), per foot the
ankle map (AnkleMap), the inertial unit's site and the two sole boxes:
    F.angles(r)            the files' angles (slot order) at the readings r (25): joints sign r + home; a foot's two
                           motors' q = sign r + home through its map
    F.readings(th)         the readings the grader sends for planned angles th: joints (th - home) / sign; a foot through
                           the inverse of its map, the command clamped at the firmware's box (|pitch| 0.35, |roll|
                           0.10 rad in the linear map's coordinates of these files' q) -> (r, n_box_clamps); raises
                           grid.InverseFail by the map inverse's failure rules (grid.py)
    F.sole[side]           Sole: the -z face of the box in the foot link's frame (centre, normal, corners, axes)
"""
from __future__ import annotations

import importlib
import math
import os

import numpy as np
import mujoco

from .given import given_model
from .grid import AnkleMap, InverseFail  # noqa: F401  (InverseFail re-exported)
from .model import ANKLE_SLOTS, BODIES, FOOT, JOINT_SLOTS, SIDES, Kin
from .vendor_chain import K_PITCH, K_ROLL, PITCH_LIMIT, ROLL_LIMIT, fw_pitch_roll

# The gate: the first importable of this package's own gate module (gate.py beside this module) and the module named
# by the environment variable RD04_GATE_MODULE; without one, an optional local module (absent from the task folder)
# may name or stand in for a gate; without any, rd04 raises GateMissing, a fault of the setup, never a score.


class GateMissing(RuntimeError):
    """no gate module is importable (tests/rd04/gate.py missing): a fault of the task's setup, never of a submission"""


class GateFail(Exception):
    """the gate refused the submission: reasons = [(code, message)]; the score is 0"""

    def __init__(self, reasons):
        self.reasons = [tuple(r) if isinstance(r, (list, tuple)) else ("gate", str(r)) for r in reasons]
        super().__init__("; ".join(f"{c}: {m}" for c, m in self.reasons[:5]))

    @property
    def codes(self):
        return sorted({c for c, _ in self.reasons})


class Sole:
    """a sole box's -z face in its foot link's frame: centre c, unit normal n (the box's +z), in-plane axes ex, ey,
    half extents hx, hy, corners (4, 3)"""

    __slots__ = ("c", "n", "ex", "ey", "hx", "hy", "corners")

    def __init__(self, p, R, half):
        p = np.asarray(p, float); R = np.asarray(R, float)
        hx, hy, hz = (float(x) for x in half)
        self.n = R[:, 2].copy(); self.ex = R[:, 0].copy(); self.ey = R[:, 1].copy()
        self.c = p - hz * self.n
        self.hx, self.hy = hx, hy
        self.corners = np.array([self.c + sx * hx * self.ex + sy * hy * self.ey
                                 for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])

    @classmethod
    def from_rect(cls, r: dict, half_height: float = 0.004):
        """from an axis-aligned rectangle {z, x0, x1, y0, y1} (the face at z), box half height given"""
        c = np.array([(r["x0"] + r["x1"]) / 2.0, (r["y0"] + r["y1"]) / 2.0, r["z"] + half_height])
        return cls(c, np.eye(3), ((r["x1"] - r["x0"]) / 2.0, (r["y1"] - r["y0"]) / 2.0, half_height))


class Files:
    def __init__(self, model, sign, home, maps, imu_site, sole_sites, source=""):
        self.model = model
        self.kin = Kin(model)
        self.sign = np.asarray(sign, float)
        self.home = np.asarray(home, float)
        self.maps = maps
        self.imu_site = int(imu_site)
        self.sole_site = dict(sole_sites)
        self.source = source
        k = self.kin.fk(np.zeros(25))
        self.sole = {}
        for s in SIDES:
            fb = k.foot[s]
            xp, xm = k.xpos(fb), k.xmat(fb)
            sp, sm = k.site(self.sole_site[s])
            self.sole[s] = Sole(xm.T @ (sp - xp), xm.T @ sm, model.site_size[self.sole_site[s]])

    # ---------------------------------------------------------------- readings <-> the files' angles
    def angles(self, r):
        r = np.asarray(r, float)
        th = np.zeros(25)
        for j in JOINT_SLOTS:
            th[j] = self.sign[j] * r[j] + self.home[j]
        for s in SIDES:
            a, b = ANKLE_SLOTS[s]
            A = self.sign[a] * r[a] + self.home[a]
            B = self.sign[b] * r[b] + self.home[b]
            th[a], th[b] = self.maps[s](A, B)
        return th

    def readings(self, th, clamp_box=True):
        th = np.asarray(th, float)
        r = np.zeros(25); nclamp = 0
        for j in JOINT_SLOTS:
            r[j] = (th[j] - self.home[j]) / self.sign[j]
        for s in SIDES:
            a, b = ANKLE_SLOTS[s]
            A, B = self.maps[s].inverse(th[a], th[b])
            if clamp_box:
                pf, rf = fw_pitch_roll(A, B)
                pc, rc = min(max(pf, -PITCH_LIMIT), PITCH_LIMIT), min(max(rf, -ROLL_LIMIT), ROLL_LIMIT)
                if pc != pf or rc != rf:
                    nclamp += 1
                    A, B = K_PITCH * pc - K_ROLL * rc, -K_PITCH * pc - K_ROLL * rc
            r[a] = (A - self.home[a]) / self.sign[a]
            r[b] = (B - self.home[b]) / self.sign[b]
        return r, nclamp


def _first_gate(names):
    for name in names:
        try:
            mod = importlib.import_module(name)
        except ImportError:
            continue
        if hasattr(mod, "check"):
            return mod
    return None


def _gate_module():
    names = [__package__ + ".gate"]
    if os.environ.get("RD04_GATE_MODULE"):
        names.append(os.environ["RD04_GATE_MODULE"])
    mod = _first_gate(names)
    if mod is not None:
        return mod
    try:
        from . import gate_shim                          # the optional local module; absent from the task folder
    except ImportError:
        raise GateMissing("rd04: no gate module is importable (tests/rd04/gate.py is missing)") from None
    mod = _first_gate(gate_shim.GATE_MODULES)
    return gate_shim if mod is None else mod


def _is_shim(gate) -> bool:
    return gate.__name__.rsplit(".", 1)[-1] == "gate_shim"


# the compiled arrays rd04 reads (kinematics, masses and centres, sites): a light copy is used only when it equals the
# gate's model in every one of them, bit for bit
READ_ARRAYS = ("body_parentid", "body_jntnum", "body_pos", "body_quat", "body_mass", "body_ipos", "jnt_type",
               "jnt_bodyid", "jnt_qposadr", "jnt_pos", "jnt_axis", "qpos0", "site_bodyid", "site_type", "site_pos",
               "site_quat", "site_size")


def light_compile(xml_text: str):
    """the model compiled without its mesh assets and mesh geoms (no file is read): the kinematics, masses and sites
    of a model whose inertials do not come from its meshes, at a small part of the memory (a compile of the 26 given
    link meshes peaks near 1.2 GB); None when it does not compile"""
    try:
        spec = mujoco.MjSpec.from_string(xml_text)
        for g in list(spec.geoms):
            if g.type == mujoco.mjtGeom.mjGEOM_MESH:
                spec.delete(g)
        for me in list(spec.meshes):
            spec.delete(me)
        return spec.compile()
    except Exception:                                    # noqa: BLE001 - no light copy: the gate's model is used
        return None


def _same_read_arrays(a, b) -> bool:
    if a is None or b is None or a.nbody != b.nbody or a.njnt != b.njnt or a.nsite != b.nsite or a.nq != b.nq:
        return False
    for n in READ_ARRAYS:
        x, y = np.asarray(getattr(a, n)), np.asarray(getattr(b, n))
        if x.shape != y.shape or not np.array_equal(x, y):
            return False
    for o in (mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_SITE):
        n = {mujoco.mjtObj.mjOBJ_BODY: a.nbody, mujoco.mjtObj.mjOBJ_JOINT: a.njnt, mujoco.mjtObj.mjOBJ_SITE: a.nsite}[o]
        if any(mujoco.mj_id2name(a, o, i) != mujoco.mj_id2name(b, o, i) for i in range(n)):
            return False
    return True


def _read_files(res, source, xml_text=None):
    """Files from a passed gate's (model, calibration document); GateFail('reader_*') for anything unreadable. When
    the submission's model compiled without its meshes equals the gate's model in every array rd04 reads, that light
    copy is kept (the memory of the meshes is released)"""
    m, doc = res.model, res.calib
    if xml_text is not None:
        lm = light_compile(xml_text)
        if _same_read_arrays(lm, m):
            m = lm
    S, B = mujoco.mjtObj.mjOBJ_SITE, mujoco.mjtObj.mjOBJ_BODY
    try:
        rows = doc["home_table"]
        sign = np.array([float(rows[i]["sign"]) for i in range(25)])
        home = np.array([float(rows[i]["home_rad"]) for i in range(25)])
        maps = {s: AnkleMap(doc["ankle_map"][s]["pitch"], doc["ankle_map"][s]["roll"]) for s in SIDES}
    except (KeyError, TypeError, ValueError, IndexError) as e:
        raise GateFail([("reader_calibration", f"calibration.yaml cannot be read: {str(e)[:200]}")])
    if not (np.all(np.isin(sign, (1.0, -1.0))) and np.all(np.isfinite(home))
            and all(np.all(np.isfinite(maps[s].P)) and np.all(np.isfinite(maps[s].R)) for s in SIDES)):
        raise GateFail([("reader_calibration", "a sign is not +-1 or a number is not finite")])
    sid = {n: int(mujoco.mj_name2id(m, S, n)) for n in ("imu", "left_sole", "right_sole")}
    if min(sid.values()) < 0:
        raise GateFail([("reader_sites", "a site is missing")])
    for s in SIDES:
        k = sid[f"{s}_sole"]
        if int(m.site_type[k]) != int(mujoco.mjtGeom.mjGEOM_BOX) or not np.all(np.asarray(m.site_size[k]) > 0):
            raise GateFail([("reader_sites", f"site {s}_sole is not a box with positive size")])
        foot = mujoco.mj_name2id(m, B, FOOT[s]); b = int(m.site_bodyid[k])
        while b != foot and b > 0 and int(m.body_jntnum[b]) == 0:
            b = int(m.body_parentid[b])
        if b != foot:
            raise GateFail([("reader_sites", f"site {s}_sole is not fixed to {FOOT[s]}")])
    link_ids = [int(mujoco.mj_name2id(m, B, n)) for n in BODIES]
    if min(link_ids) < 0:
        raise GateFail([("reader_model", "a link body of the given tree is missing")])
    if not (np.all(np.asarray(m.body_mass)[link_ids] > 0) and np.all(np.isfinite(m.body_mass)) and
            np.all(np.isfinite(m.body_ipos))):
        raise GateFail([("reader_masses", "a link body's mass is not positive or a number is not finite")])
    try:
        F = Files(m, sign, home, maps, sid["imu"], {"left": sid["left_sole"], "right": sid["right_sole"]}, source)
    except (KeyError, ValueError, IndexError) as e:
        raise GateFail([("reader_model", f"the compiled model cannot be read: {str(e)[:200]}")])
    for s in SIDES:
        so = F.sole[s]
        if not (np.all(np.isfinite(so.corners)) and math.isfinite(so.hx) and math.isfinite(so.hy)):
            raise GateFail([("reader_sites", f"site {s}_sole: a number is not finite")])
    return F


def load(sub_dir, given=None, late_entries=None):
    """Files, or GateFail(reasons) (returned, not raised). late_entries: the lockdown's entries of the submission
    folder written after the session ended (the gate zeroes them). A setup fault of the gate (its
    GateSetupError: the given meshes missing) is raised, never scored."""
    g = given if given is not None else given_model()
    gate = _gate_module()                                # GateMissing propagates (a setup fault)
    try:
        if _is_shim(gate):
            if late_entries:
                return GateFail([("files.late_write", f"{len(late_entries)} entries written after the session ended")])
            res = gate.check(str(sub_dir), g)
        else:
            res = gate.check(str(sub_dir), g, late_entries=late_entries)
    except Exception as e:                               # noqa: BLE001 - a gate crash is a failed gate, never a crash
        if type(e).__name__ == "GateSetupError":
            raise
        return GateFail([("gate_error", f"{type(e).__name__}: {str(e)[:200]}")])
    if not getattr(res, "ok", False):
        return GateFail(list(getattr(res, "reasons", [])) or [("gate", "refused")])
    try:
        with open(os.path.join(str(sub_dir), "unit.xml"), encoding="utf-8") as f:
            xml_text = f.read()
    except (OSError, UnicodeDecodeError):
        xml_text = None
    try:
        F = _read_files(res, str(sub_dir), xml_text)
    except GateFail as e:
        return e
    res.model = None                                     # the gate's model is not kept beyond the Files
    return F


class _Res:
    def __init__(self, model, calib):
        self.model, self.calib, self.ok = model, calib, True


def load_truth(gt_dir):
    """the truth files (the grader's own tests/gt/: unit.xml, calibration.yaml) without the gate: the model compiled
    light (no mesh asset is read; the truth's inertials are explicit) and read as a submission is; a fault is an
    error of the task (ValueError), never a score"""
    import yaml
    with open(os.path.join(gt_dir, "unit.xml"), encoding="utf-8") as f:
        xml_text = f.read()
    with open(os.path.join(gt_dir, "calibration.yaml"), encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    m = light_compile(xml_text)
    if m is None:
        raise ValueError(f"{gt_dir}: the truth's unit.xml does not compile without its meshes")
    try:
        return _read_files(_Res(m, doc), str(gt_dir))
    except GateFail as e:
        raise ValueError(f"{gt_dir}: the truth files cannot be read: {e}")


def gate_module_name():
    return _gate_module().__name__
