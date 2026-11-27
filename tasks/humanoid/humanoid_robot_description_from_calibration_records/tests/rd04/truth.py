"""The grader's true side: the true unit (TrueUnit) and the hidden set (Hidden).

TrueUnit (what the record says about the unit as crated: the TRUE unit at the readings):
    sign (25)          the true senses
    home (25)          the true table: joint slots the model angle at the stored zero; ankle motors FN23's
                       homes (the readout convention), the firmware map's coordinates of the levelled foot attitude at
                       Calibrate Both
    zero_crank         per foot the crank angles (rad) at the ankle motors' stored zeros
    L_now_mm           per foot the bars as crated (mm, centre to centre)
    stops              per joint (the 21 joint slots' joints) the true stops (lo, hi) rad
  T.angles(r) -> (th, n_stop_clamps, n_region_clamps): joints home + sign r clamped at the stops; a foot's cranks
  zero_crank + sign r, the foot by the vendored exact chain on L_now (Newton from the firmware map's attitude of the
  true table's q), moved to the stop region's nearest point when outside it; TrueSideFail when the chain cannot place
  the foot (the posture scores 0). T.readings(th): the exact readings of true angles th (the bus samples).
  JSON form (rd04-true-unit/1): {"schema", "sign", "home_rad", "zero_crank_rad": {side: [A, B]},
  "L_now_mm": {side: [A, B]}, "stops_rad": {joint: [lo, hi]}}.

Hidden: the true unit, the truth files (loaded through rd04.load: masses, soles, the imu site as crated),
the bus samples (theta (n, 25) slot order with the foot hinges in the ankle slots, pelvis_R (n, 3, 3), readings
(n, 25), g_imu (n, 3)), the truth-planned reading vectors (plan_readings (10, 25), plan_pose (10, 4, 4)) and the
posture file. The true per-sample quantities are computed once at construction.
"""
from __future__ import annotations

import json
import math

import numpy as np

from .model import ANKLE_SLOTS, ARM_LINKS, JOINT_SLOTS, SIDES, SLOT_JOINT
from .topple import feet_frame_coords
from .vendor_chain import Chain, fw_pitch_roll, stop_region


class TrueSideFail(Exception):
    def __init__(self, code, detail=""):
        super().__init__(f"{code}: {detail}")
        self.code = code


class TrueUnit:
    def __init__(self, sign, home, zero_crank, L_now_mm, stops):
        self.sign = np.asarray(sign, float).copy()
        self.home = np.asarray(home, float).copy()
        self.zero_crank = {s: (float(zero_crank[s][0]), float(zero_crank[s][1])) for s in SIDES}
        self.L_now = {s: (float(L_now_mm[s][0]), float(L_now_mm[s][1])) for s in SIDES}
        self.stops = {k: (float(v[0]), float(v[1])) for k, v in stops.items()}
        missing = [SLOT_JOINT[j] for j in JOINT_SLOTS if SLOT_JOINT[j] not in self.stops]
        if missing:
            raise ValueError(f"true stops missing for {missing}")
        self.chain_now = {s: Chain(s, self.L_now[s]) for s in SIDES}
        self.region = {s: stop_region(s) for s in SIDES}

    # ---------------------------------------------------------------- JSON
    def to_json(self) -> dict:
        return {"schema": "rd04-true-unit/1", "sign": [int(x) for x in self.sign], "home_rad": [float(x) for x in self.home],
                "zero_crank_rad": {s: list(self.zero_crank[s]) for s in SIDES},
                "L_now_mm": {s: list(self.L_now[s]) for s in SIDES},
                "stops_rad": {k: list(v) for k, v in sorted(self.stops.items())}}

    @classmethod
    def from_json(cls, d: dict) -> "TrueUnit":
        if d.get("schema") != "rd04-true-unit/1":
            raise ValueError("not an rd04-true-unit/1 document")
        return cls(d["sign"], d["home_rad"], d["zero_crank_rad"], d["L_now_mm"], d["stops_rad"])

    # ---------------------------------------------------------------- readings <-> true angles
    def angles(self, r, clamp=True):
        r = np.asarray(r, float)
        th = np.zeros(25); n_stop = 0; n_reg = 0
        for j in JOINT_SLOTS:
            v = self.home[j] + self.sign[j] * r[j]
            if clamp:
                lo, hi = self.stops[SLOT_JOINT[j]]
                if v < lo or v > hi:
                    n_stop += 1; v = min(max(v, lo), hi)
            th[j] = v
        for s in SIDES:
            a, b = ANKLE_SLOTS[s]
            z = self.zero_crank[s]
            phi = (z[0] + self.sign[a] * r[a], z[1] + self.sign[b] * r[b])
            if not (math.isfinite(phi[0]) and math.isfinite(phi[1])):
                raise TrueSideFail("true_ankle", f"{s}: crank angles not finite")
            g = fw_pitch_roll(phi[0] - z[0] + self.home[a], phi[1] - z[1] + self.home[b])
            try:
                p, rr = self.chain_now[s].fk(phi[0], phi[1], guess=g)
            except ValueError as e:
                raise TrueSideFail("true_ankle", f"{s}: {e}")
            if not (math.isfinite(p) and math.isfinite(rr)):
                raise TrueSideFail("true_ankle", f"{s}: foot attitude not finite")
            if clamp and not self.region[s].contains(p, rr):
                n_reg += 1
                p, rr = self.region[s].nearest(p, rr)
            th[a], th[b] = p, rr
        return th, n_stop, n_reg

    def readings(self, th):
        th = np.asarray(th, float)
        r = np.zeros(25)
        for j in JOINT_SLOTS:
            r[j] = self.sign[j] * (th[j] - self.home[j])
        for s in SIDES:
            a, b = ANKLE_SLOTS[s]
            phi = self.chain_now[s].ik(th[a], th[b], (0.0, 0.0))
            if phi is None:
                raise TrueSideFail("true_ankle", f"{s}: bus sample ankle out of reach")
            r[a] = self.sign[a] * (phi[0] - self.zero_crank[s][0])
            r[b] = self.sign[b] * (phi[1] - self.zero_crank[s][1])
        return r

    def cranks(self, th):
        """the true crank angles (left A, B, right A, B) of true angles th on the bars as crated"""
        out = []
        for s in SIDES:
            a, b = ANKLE_SLOTS[s]
            phi = self.chain_now[s].ik(th[a], th[b], (0.0, 0.0))
            if phi is None:
                raise TrueSideFail("true_ankle", f"{s}: out of reach")
            out += [phi[0], phi[1]]
        return np.array(out)


def pose44(R, t):
    T = np.eye(4); T[:3, :3] = np.asarray(R, float); T[:3, 3] = np.asarray(t, float)
    return T


class Hidden:
    def __init__(self, true_unit: TrueUnit, truth_files, theta, pelvis_R, readings, g_imu, plan_readings, plan_pose,
                 postures: dict, crank=None, check_readings=True):
        self.true = true_unit
        self.truth = truth_files
        self.theta = np.asarray(theta, float)
        self.pelvis_R = np.asarray(pelvis_R, float)
        self.readings = np.asarray(readings, float)
        self.g_imu = np.asarray(g_imu, float)
        self.plan_readings = np.asarray(plan_readings, float)
        self.plan_pose = np.asarray(plan_pose, float)
        self.postures = postures
        self.crank = None if crank is None else np.asarray(crank, float)
        n = self.theta.shape[0]
        if self.theta.shape != (n, 25) or self.readings.shape != (n, 25) or self.pelvis_R.shape != (n, 3, 3) or \
                self.g_imu.shape != (n, 3) or self.plan_readings.shape[1:] != (25,) or self.plan_pose.shape[1:] != (4, 4):
            raise ValueError("hidden set shapes")
        for a in (self.theta, self.pelvis_R, self.readings, self.g_imu, self.plan_readings, self.plan_pose):
            if not np.all(np.isfinite(a)):
                raise ValueError("hidden set: a value is not finite")
        self._prepare()

    def _prepare(self):
        T = self.truth; k = T.kin
        g_world = np.array([0.0, 0.0, -1.0])
        self.t_feet, self.t_arms, self.t_g_torso = [], [], []
        for i in range(self.theta.shape[0]):
            k.fk(self.theta[i])
            feet = {}
            for s in SIDES:
                fb = k.foot[s]
                xp, xm = k.xpos(fb), k.xmat(fb)
                feet[s] = (xp + xm @ T.sole[s].c, xm, xm @ T.sole[s].n)
            self.t_feet.append(feet)
            self.t_arms.append({s: k.xmat(k.bid[a]).T @ k.xmat(k.bid[b]) for s, (a, b) in ARM_LINKS.items()})
            Rt = self.pelvis_R[i] @ k.xmat(k.torso)
            self.t_g_torso.append(Rt.T @ g_world)
        self.t_plan = []
        self.t_plan_clamps = []
        for kk in range(self.plan_readings.shape[0]):
            tht, ns, nr = self.true.angles(self.plan_readings[kk])
            self.t_plan_clamps.append((ns, nr))
            Rp, tp = self.plan_pose[kk][:3, :3], self.plan_pose[kk][:3, 3]
            k.fk(tht, Rp, tp)
            com = k.com()
            cen = {s: k.xpos(k.foot[s]) + k.xmat(k.foot[s]) @ T.sole[s].c for s in SIDES}
            xf = 0.5 * (k.xmat(k.foot["left"])[:, 0] + k.xmat(k.foot["right"])[:, 0])
            self.t_plan.append(feet_frame_coords(com, cen, xf))
        # the ankle words' references (metrics only): the true attitude as crated and on the dry-fit bars at the zero
        # readings are filled by the caller that knows the dry-fit bars (set_dryfit); default: none
        self.dryfit_zero_attitude = None

    def set_dryfit(self, L_cal_mm: dict):
        """the true foot attitude at the ankle motors' zero readings on the dry-fit bars (metrics: the bars word)"""
        out = {}
        for s in SIDES:
            z = self.true.zero_crank[s]
            a, b = ANKLE_SLOTS[s]
            g = fw_pitch_roll(self.true.home[a], self.true.home[b])
            out[s] = Chain(s, L_cal_mm[s]).fk(z[0], z[1], guess=g)
        self.dryfit_zero_attitude = out

    @classmethod
    def from_files(cls, npz_path, postures_path, true_unit_path, truth_files, dryfit_json_key="L_cal_mm"):
        with open(true_unit_path, encoding="utf-8") as f:
            tj = json.load(f)
        tu = TrueUnit.from_json(tj)
        with np.load(npz_path, allow_pickle=False) as z:
            arr = {k: np.array(z[k]) for k in z.files}
        with open(postures_path, encoding="utf-8") as f:
            post = json.load(f)
        H = cls(tu, truth_files, arr["theta"], arr["pelvis_R"], arr["readings"], arr["g_imu"], arr["plan_readings"],
                arr["plan_pose"], post, crank=arr.get("crank"))
        if dryfit_json_key in tj:
            H.set_dryfit(tj[dryfit_json_key])
        return H


def make_plan_set(truth_files, postures: dict):
    """the truth-planned reading vectors and pelvis poses: each posture planned on the truth files by rd04's
    planner, the readings through the truth files' table and the inverse of their ankle map with the firmware clamp"""
    from .planner import plan
    R, P = [], []
    for sp in postures["postures"]:
        pl = plan(truth_files, sp, postures["frame"])
        r, _ = truth_files.readings(pl.th)
        R.append(r); P.append(pose44(pl.Rp, pl.tp))
    return np.array(R), np.array(P)


def make_bus_arrays(true_unit: TrueUnit, truth_files, postures_th, pelvis_R):
    """the bus samples' exact readings and inertial-unit gravity for true angles postures_th (n, 25) and
    pelvis attitudes pelvis_R (n, 3, 3): readings from the true table; g_imu = the world's gravity in the axes of the
    truth files' imu site"""
    k = truth_files.kin
    th = np.asarray(postures_th, float); Rp = np.asarray(pelvis_R, float)
    rd, g, cr = [], [], []
    for i in range(th.shape[0]):
        rd.append(true_unit.readings(th[i]))
        cr.append(true_unit.cranks(th[i]))
        k.fk(th[i])
        _, sm = k.site(truth_files.imu_site)
        g.append((Rp[i] @ sm).T @ np.array([0.0, 0.0, -1.0]))
    return np.array(rd), np.array(g), np.array(cr)
