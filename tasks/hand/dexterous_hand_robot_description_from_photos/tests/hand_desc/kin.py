"""kin: the grader's kinematics of a hand model in contract form (MuJoCo; static, nothing is ever stepped).

HandModel.load(xml) compiles a model and reads its contract structure: the 13 driven hinges, the three DIP hinges and
how each finger couples its DIP to its PIP. Two coupling forms are read:
  "joint"  one joint equality joint1 = <f>_dip, joint2 = <f>_pip with a polynomial (MuJoCo's polycoef: dip - dip0 =
           c0 + c1 (pip - pip0) + ... + c4 (pip - pip0)^4, dip0 / pip0 = the hinges' reference values);
  "loop"   connect equalities between bodies of the finger (a closed chain). Every hinge of the finger that is not one
           of its three driven hinges (its DIP and the hinges of its loop bodies) is passive: solve() closes the loop by
           Gauss-Newton on the passive hinges, by continuation from q = 0 in steps of at most 2 degrees of the driven
           hinges that change the loop's shape (for the usual layout: the PIP alone), so that the solution stays on the
           assembly branch of q = 0. A configuration at which the loop does not close to the model's tolerance raises
           LoopError.
Everything returned is expressed in the frame of the body "palm" (positions in metres, angles in radians).

  st = hm.solve({"index_pip": -0.4, ...})     # driven angles in (missing driven hinges = 0)
  p, n = hm.pad(st, "index")                  # pad site position and its +z axis
  J = hm.pad_jacobian(st, "index")            # 3 x n: d pad / d driven angles of that digit (coupling included),
                                              # columns in contract.DRIVEN_BY_DIGIT order
  m, c = hm.group_mass_com(st, "hand")        # "thumb" / "index" / "middle" / "ring" / "hand"
Deterministic: the same model and the same input give the same result; nothing global is changed.
"""
import math

import numpy as np
import mujoco

try:
    from . import contract as C
except ImportError:  # loaded as a plain module beside contract.py
    import contract as C

STEP_RAD = math.radians(2.0)      # continuation step of the loop's driven hinges
GN_TOL_M = 1e-12                  # Gauss-Newton iterates to this residual (m) or until it stops improving
GN_MAX_IT = 60
DEFAULT_LOOP_TOL_M = 1e-9         # a loop closes when every connect residual is at most this (m)
GRID_MARGIN_RAD = math.radians(6.0)


class ContractError(Exception):
    """The model does not have the contract's structure (a missing name, an unsupported coupling)."""


class LoopError(Exception):
    """A finger's closed chain does not close at the requested configuration."""

    def __init__(self, finger, residual_m, message):
        super().__init__("%s: %s (residual %.3e m)" % (finger, message, residual_m))
        self.finger = finger
        self.residual_m = residual_m


class KinState(object):
    """A solved configuration: q = the 16 contract hinge angles, qpos = the model's full position vector, residual_m
    = the closing residual per finger (0.0 for a joint-equality coupling), and a snapshot of the world-frame
    kinematics (MuJoCo's xpos, xmat, xipos, xanchor, xaxis, site_xpos, site_xmat)."""

    __slots__ = ("q", "qpos", "residual_m", "xpos", "xmat", "xipos", "xanchor", "xaxis", "site_xpos", "site_xmat")

    def __init__(self, q, qpos, residual_m, d):
        self.q = q
        self.qpos = qpos
        self.residual_m = residual_m
        self.xpos = d.xpos.copy()
        self.xmat = d.xmat.copy()
        self.xipos = d.xipos.copy()
        self.xanchor = d.xanchor.copy()
        self.xaxis = d.xaxis.copy()
        self.site_xpos = d.site_xpos.copy()
        self.site_xmat = d.site_xmat.copy()


def _name(m, objtype, i):
    return mujoco.mj_id2name(m, objtype, i)


class _Loop(object):
    """The coupling of one finger."""

    def __init__(self):
        self.form = None          # "joint" | "loop"
        self.poly = None          # joint form: polycoef c0..c4
        self.eqs = []             # loop form: [(kind, a, b, anchor_a, anchor_b)], kind "site" | "body"
        self.passive = []         # loop form: joint ids of the passive hinges
        self.drivers = []         # loop form: driven contract joint names that change the loop's shape
        self.grid = None          # loop form with one driver: {k: passive values at k * STEP_RAD}
        self.grid_fail = {}       # side (+1 / -1) -> first k at which the continuation failed


class HandModel(object):
    """A compiled hand model with its contract maps."""

    def __init__(self, model, loop_tol=DEFAULT_LOOP_TOL_M):
        self.model = model
        self.loop_tol = float(loop_tol)
        self._d = mujoco.MjData(model)
        self.driven = tuple(C.MOTOR_ORDER)
        m = model
        self.jid = {}
        for j in C.CONTRACT_JOINTS:
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if i < 0:
                raise ContractError("joint %s is missing" % j)
            if m.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ContractError("joint %s is not a hinge" % j)
            self.jid[j] = int(i)
        for i in range(m.njnt):
            if m.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ContractError("joint %s is not a hinge" % (_name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or i))
        self.palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.ROOT_BODY)
        if self.palm < 0:
            raise ContractError("body %s is missing" % C.ROOT_BODY)
        self.sid = {}
        for dg, s in C.PAD_SITES.items():
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, s)
            if i < 0:
                raise ContractError("site %s is missing" % s)
            self.sid[dg] = int(i)
        self.first_body = {}
        for dg in C.DIGITS:
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.FIRST_BODY[dg])
            if b < 0:
                raise ContractError("body %s is missing" % C.FIRST_BODY[dg])
            self.first_body[dg] = int(b)
        # ancestry: anc[b] = set of bodies on the path from b to the world (b included)
        self._anc = []
        for b in range(m.nbody):
            s, x = set(), b
            while True:
                s.add(x)
                if x == 0:
                    break
                x = int(m.body_parentid[x])
            self._anc.append(frozenset(s))
        self._digit_bodies = {dg: [b for b in range(m.nbody) if self.first_body[dg] in self._anc[b]]
                              for dg in C.DIGITS}
        self._digit_of_joint = {}
        for i in range(m.njnt):
            b = int(m.jnt_bodyid[i])
            for dg in C.DIGITS:
                if self.first_body[dg] in self._anc[b]:
                    self._digit_of_joint[i] = dg
        for j, i in self.jid.items():
            if self._digit_of_joint.get(i) != C.DIGIT_OF_JOINT[j]:
                raise ContractError("joint %s is not below body %s" % (j, C.FIRST_BODY[C.DIGIT_OF_JOINT[j]]))
        self._qadr = {i: int(m.jnt_qposadr[i]) for i in range(m.njnt)}
        self._qpos0 = m.qpos0.copy()
        self._loops = {}
        self._read_couplings()
        for f in C.FINGERS:
            lp = self._loops[f]
            if lp.form == "loop" and len(lp.drivers) == 1:
                self._build_grid(f)

    # ------------------------------------------------------------------------------------------------ loading
    @classmethod
    def load(cls, xml_path, assets_dir=None, loop_tol=DEFAULT_LOOP_TOL_M):
        """Compile robot.xml. With assets_dir, every file below it is handed to the compiler under the name
        "<meshdir>/<relative path>" (for a model whose meshes are kept apart from its robot.xml)."""
        import os
        try:
            if assets_dir is None:
                model = mujoco.MjModel.from_xml_path(str(xml_path))
            else:
                import xml.etree.ElementTree as ET
                text = open(str(xml_path), "rb").read().decode("utf-8")
                comp = ET.fromstring(text).find("compiler")
                meshdir = (comp.get("meshdir") if comp is not None else None) or ""
                assets = {}
                for root, _, files in os.walk(str(assets_dir)):
                    for fn in files:
                        p = os.path.join(root, fn)
                        rel = os.path.relpath(p, str(assets_dir)).replace(os.sep, "/")
                        key = (meshdir.rstrip("/") + "/" + rel) if meshdir else rel
                        with open(p, "rb") as fh:
                            assets[key] = fh.read()
                model = mujoco.MjModel.from_xml_string(text, assets=assets)
        except ValueError as exc:
            raise ContractError("the model does not compile: %s" % exc)
        return cls(model, loop_tol=loop_tol)

    def _finger_of_body(self, b):
        for f in C.FINGERS:
            if self.first_body[f] in self._anc[b]:
                return f
        return None

    def _read_couplings(self):
        m = self.model
        for f in C.FINGERS:
            self._loops[f] = _Loop()
        for e in range(m.neq):
            if not m.eq_active0[e]:
                continue  # an inactive equality couples nothing (MuJoCo's own reading)
            t = int(m.eq_type[e])
            if t == mujoco.mjtEq.mjEQ_JOINT:
                j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
                f = None
                for ff in C.FINGERS:
                    if j1 == self.jid[C.DIP_OF_FINGER[ff]]:
                        f = ff
                if f is None or j2 != self.jid[C.PIP_OF_FINGER[f]]:
                    raise ContractError("a joint equality must couple <finger>_dip (joint1) to <finger>_pip (joint2)")
                lp = self._loops[f]
                if lp.form is not None:
                    raise ContractError("%s: more than one coupling" % f)
                lp.form = "joint"
                lp.poly = np.array(m.eq_data[e][:5], float)
            elif t == mujoco.mjtEq.mjEQ_CONNECT:
                ot = int(m.eq_objtype[e])
                if ot == mujoco.mjtObj.mjOBJ_SITE:
                    a, b = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
                    ba, bb = int(m.site_bodyid[a]), int(m.site_bodyid[b])
                    rec = ("site", a, b, None, None)
                elif ot == mujoco.mjtObj.mjOBJ_BODY:
                    ba, bb = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
                    rec = ("body", ba, bb, np.array(m.eq_data[e][0:3], float), np.array(m.eq_data[e][3:6], float))
                else:
                    raise ContractError("connect equality %d: unsupported object type" % e)
                fa, fb = self._finger_of_body(ba), self._finger_of_body(bb)
                if fa is None or fa != fb:
                    raise ContractError("a connect equality must join two bodies of one finger")
                lp = self._loops[fa]
                if lp.form == "joint":
                    raise ContractError("%s: more than one coupling" % fa)
                lp.form = "loop"
                lp.eqs.append(rec + (ba, bb))
            else:
                raise ContractError("equality %d: only joint and connect equalities are read" % e)
        driven_ids = {self.jid[j] for j in C.MOTOR_ORDER}
        for f in C.FINGERS:
            lp = self._loops[f]
            extra = [i for i in range(m.njnt) if self._digit_of_joint.get(i) == f and i not in driven_ids]
            if lp.form is None:
                raise ContractError("%s: the last hinge is not coupled" % C.DIP_OF_FINGER[f])
            if lp.form == "joint":
                if extra != [self.jid[C.DIP_OF_FINGER[f]]]:
                    raise ContractError("%s: extra hinges without a closed chain" % f)
                continue
            lp.passive = extra
            # driven hinges on the loop: the joints that move exactly one side of some connect equality
            drivers = set()
            for rec in lp.eqs:
                ba, bb = rec[5], rec[6]
                ja = {i for i in range(m.njnt) if int(m.jnt_bodyid[i]) in self._anc[ba]}
                jb = {i for i in range(m.njnt) if int(m.jnt_bodyid[i]) in self._anc[bb]}
                drivers |= (ja ^ jb) & driven_ids
            lp.drivers = [j for j in C.MOTOR_ORDER if self.jid[j] in drivers]
            if self.jid[C.DIP_OF_FINGER[f]] not in lp.passive:
                raise ContractError("%s: the DIP hinge is not part of the closed chain" % f)

    # ------------------------------------------------------------------------------------------------ the loop
    def _moves(self, j, b):
        return int(self.model.jnt_bodyid[j]) in self._anc[b]

    def _residual(self, lp, d):
        r = []
        pts = []
        for kind, a, b, anc_a, anc_b, ba, bb in lp.eqs:
            if kind == "site":
                pa, pb = d.site_xpos[a].copy(), d.site_xpos[b].copy()
            else:
                pa = d.xpos[a] + d.xmat[a].reshape(3, 3) @ anc_a
                pb = d.xpos[b] + d.xmat[b].reshape(3, 3) @ anc_b
            r.append(pa - pb)
            pts.append((pa, pb, ba, bb))
        return np.concatenate(r), pts

    def _constraint_jac(self, lp, d, pts, joints):
        J = np.zeros((3 * len(pts), len(joints)))
        for k, (pa, pb, ba, bb) in enumerate(pts):
            for c, j in enumerate(joints):
                ax, an = d.xaxis[j], d.xanchor[j]
                col = np.zeros(3)
                if self._moves(j, ba):
                    col += np.cross(ax, pa - an)
                if self._moves(j, bb):
                    col -= np.cross(ax, pb - an)
                J[3 * k:3 * k + 3, c] = col
        return J

    @staticmethod
    def _rmax(r):
        return float(np.max(np.linalg.norm(r.reshape(-1, 3), axis=1))) if len(r) else 0.0

    def _gauss_newton(self, lp, d, qpos):
        """Close the loop lp from the passive values in qpos (changed in place). Returns the final residual (m)."""
        m = self.model
        adr = [self._qadr[j] for j in lp.passive]
        d.qpos[:] = qpos
        mujoco.mj_kinematics(m, d)
        r, pts = self._residual(lp, d)
        best = self._rmax(r)
        for _ in range(GN_MAX_IT):
            if best <= GN_TOL_M:
                break
            J = self._constraint_jac(lp, d, pts, lp.passive)
            dx = -np.linalg.lstsq(J, r, rcond=None)[0]
            step, improved = 1.0, False
            x0 = qpos[adr].copy()
            for _h in range(8):
                qpos[adr] = x0 + step * dx
                d.qpos[:] = qpos
                mujoco.mj_kinematics(m, d)
                r2, pts2 = self._residual(lp, d)
                v = self._rmax(r2)
                if v < best:
                    improved = True
                    break
                step *= 0.5
            if not improved:
                qpos[adr] = x0
                d.qpos[:] = qpos
                mujoco.mj_kinematics(m, d)
                break
            r, pts, best = r2, pts2, v
        return best

    def _continue(self, f, qpos, start_vals, start_q, target_q):
        """Continuation of finger f's loop from driver values start_q (passive values start_vals) to target_q, in
        steps of at most STEP_RAD; qpos holds everything else. Returns the passive values and the residual."""
        lp = self._loops[f]
        adr = [self._qadr[j] for j in lp.passive]
        dadr = [self._qadr[self.jid[j]] for j in lp.drivers]
        start_q = np.asarray(start_q, float)
        target_q = np.asarray(target_q, float)
        span = float(np.max(np.abs(target_q - start_q))) if len(dadr) else 0.0
        n = max(1, int(math.ceil(span / STEP_RAD - 1e-12)))
        q = qpos.copy()
        q[adr] = start_vals
        res = 0.0
        for k in range(1, n + 1):
            q[dadr] = start_q + (target_q - start_q) * (k / n)
            res = self._gauss_newton(lp, self._d, q)
            if res > self.loop_tol:
                raise LoopError(f, res, "the closed chain does not close")
        return q[adr].copy(), res

    def _build_grid(self, f):
        """For a loop driven by one hinge: the passive values at every multiple of STEP_RAD inside the hinge's range
        (widened by GRID_MARGIN_RAD), each reached by continuation from 0. solve() starts its last step from the
        grid point between 0 and the target that is nearest to the target: the same continuation from q = 0."""
        m = self.model
        lp = self._loops[f]
        j = self.jid[lp.drivers[0]]
        lo, hi = float(m.jnt_range[j][0]), float(m.jnt_range[j][1])
        if not m.jnt_limited[j]:
            lo, hi = -math.pi, math.pi
        lo, hi = min(lo, 0.0) - GRID_MARGIN_RAD, max(hi, 0.0) + GRID_MARGIN_RAD
        adr = [self._qadr[i] for i in lp.passive]
        base = self._qpos0.copy()
        grid = {}
        try:
            vals, _ = self._continue(f, base, base[adr], [0.0], [0.0])
        except LoopError:
            raise ContractError("%s: the closed chain does not close at q = 0" % f)
        grid[0] = vals
        for side, end in ((1, hi), (-1, lo)):
            k, prev = 0, vals
            while abs((k + side) * STEP_RAD) <= abs(end):
                k += side
                try:
                    prev, _ = self._continue(f, base, prev, [(k - side) * STEP_RAD], [k * STEP_RAD])
                except LoopError:
                    lp.grid_fail[side] = k
                    break
                grid[k] = prev
        lp.grid = grid

    def _solve_finger(self, f, qpos, target):
        lp = self._loops[f]
        adr = [self._qadr[j] for j in lp.passive]
        tq = [target[j] for j in lp.drivers]
        if lp.grid is not None:
            t = tq[0]
            k = int(math.trunc(t / STEP_RAD))
            side = 1 if k > 0 else -1
            kf = lp.grid_fail.get(side)
            if kf is not None and abs(k) >= abs(kf):
                raise LoopError(f, float("nan"), "the closed chain does not close on the way from q = 0")
            kmax = max((kk for kk in lp.grid if kk * side >= 0), key=abs)
            if abs(k) > abs(kmax):
                vals, _ = self._continue(f, qpos, lp.grid[kmax], [kmax * STEP_RAD], [t])
            else:
                vals, _ = self._continue(f, qpos, lp.grid[k], [k * STEP_RAD], [t])
        else:
            vals, _ = self._continue(f, qpos, self._qpos0[adr], [0.0] * len(tq), tq)
        qpos[adr] = vals

    # ------------------------------------------------------------------------------------------------ public
    def ranges(self):
        """(lo, hi) of the 16 contract hinges in the model's own convention (rad)."""
        m = self.model
        out = {}
        for j in C.CONTRACT_JOINTS:
            i = self.jid[j]
            if not m.jnt_limited[i]:
                raise ContractError("joint %s is not limited" % j)
            out[j] = (float(m.jnt_range[i][0]), float(m.jnt_range[i][1]))
        return out

    def solve(self, q):
        """Driven angles in (dict: contract name -> rad; missing driven hinges = 0); DIPs and loop hinges solved."""
        for k in q:
            if k not in self.jid or k in C.DIP_JOINTS:
                raise ValueError("not a driven contract joint: %r" % (k,))
        target = {j: float(q.get(j, 0.0)) for j in C.MOTOR_ORDER}
        m = self.model
        qpos = self._qpos0.copy()
        for j, v in target.items():
            qpos[self._qadr[self.jid[j]]] = v
        res = {}
        for f in C.FINGERS:
            lp = self._loops[f]
            if lp.form == "joint":
                jd, jp = self.jid[C.DIP_OF_FINGER[f]], self.jid[C.PIP_OF_FINGER[f]]
                x = target[C.PIP_OF_FINGER[f]] - self._qpos0[self._qadr[jp]]
                c = lp.poly
                qpos[self._qadr[jd]] = self._qpos0[self._qadr[jd]] + c[0] + x * (c[1] + x * (c[2] + x * (c[3] + x * c[4])))
                res[f] = 0.0
            else:
                self._solve_finger(f, qpos, target)
        d = self._d
        d.qpos[:] = qpos
        mujoco.mj_kinematics(m, d)
        for f in C.FINGERS:
            lp = self._loops[f]
            if lp.form == "loop":
                r, _ = self._residual(lp, d)
                res[f] = self._rmax(r)
                if res[f] > self.loop_tol:
                    raise LoopError(f, res[f], "the closed chain does not close")
        qd = {j: float(qpos[self._qadr[self.jid[j]]]) for j in C.CONTRACT_JOINTS}
        return KinState(qd, qpos, res, d)

    def _palm_pose(self, st):
        return st.xpos[self.palm].copy(), st.xmat[self.palm].reshape(3, 3).copy()

    def pad(self, st, digit):
        """Pad site position (m) and its +z axis, palm frame."""
        x0, R0 = self._palm_pose(st)
        s = self.sid[digit]
        R = st.site_xmat[s].reshape(3, 3)
        n = R @ np.asarray(C.PAD_NORMAL_AXIS, float)
        return R0.T @ (st.site_xpos[s] - x0), R0.T @ n

    def _point_jac(self, st, p, body, joints):
        J = np.zeros((3, len(joints)))
        for c, j in enumerate(joints):
            if self._moves(j, body):
                J[:, c] = np.cross(st.xaxis[j], p - st.xanchor[j])
        return J

    def pad_jacobian(self, st, digit):
        """3 x n reduced Jacobian of the pad point with respect to the digit's driven hinges (m / rad), the coupling
        included; columns in contract.DRIVEN_BY_DIGIT[digit] order; palm frame."""
        m = self.model
        s = self.sid[digit]
        body = int(m.site_bodyid[s])
        p = st.site_xpos[s]
        dj = [self.jid[j] for j in C.DRIVEN_BY_DIGIT[digit]]
        J = self._point_jac(st, p, body, dj)
        if digit in C.FINGERS:
            lp = self._loops[digit]
            if lp.form == "joint":
                jd, jp = self.jid[C.DIP_OF_FINGER[digit]], self.jid[C.PIP_OF_FINGER[digit]]
                x = st.q[C.PIP_OF_FINGER[digit]] - self._qpos0[self._qadr[jp]]
                c = lp.poly
                dd = c[1] + x * (2 * c[2] + x * (3 * c[3] + x * 4 * c[4]))
                Jd = self._point_jac(st, p, body, [jd])[:, 0]
                J[:, dj.index(jp)] += Jd * dd
            else:
                pts = []
                for kind, a, b, anc_a, anc_b, ba, bb in lp.eqs:
                    if kind == "site":
                        pa, pb = st.site_xpos[a], st.site_xpos[b]
                    else:
                        pa = st.xpos[a] + st.xmat[a].reshape(3, 3) @ anc_a
                        pb = st.xpos[b] + st.xmat[b].reshape(3, 3) @ anc_b
                    pts.append((pa, pb, ba, bb))
                Cp = self._constraint_jac_state(st, pts, lp.passive)
                Cd = self._constraint_jac_state(st, pts, dj)
                dx = -np.linalg.lstsq(Cp, Cd, rcond=None)[0]
                J = J + self._point_jac(st, p, body, lp.passive) @ dx
        R0 = self._palm_pose(st)[1]
        return R0.T @ J

    def _constraint_jac_state(self, st, pts, joints):
        J = np.zeros((3 * len(pts), len(joints)))
        for k, (pa, pb, ba, bb) in enumerate(pts):
            for c, j in enumerate(joints):
                col = np.zeros(3)
                if self._moves(j, ba):
                    col += np.cross(st.xaxis[j], pa - st.xanchor[j])
                if self._moves(j, bb):
                    col -= np.cross(st.xaxis[j], pb - st.xanchor[j])
                J[3 * k:3 * k + 3, c] = col
        return J

    def group_mass_com(self, st, group):
        """Mass (kg) and centre of mass (palm frame) of a digit's subtree or of the whole hand ("hand")."""
        m = self.model
        if group == "hand":
            bodies = list(range(1, m.nbody))
        elif group in C.DIGITS:
            bodies = self._digit_bodies[group]
        else:
            raise ValueError("unknown group %r" % (group,))
        mass = float(sum(m.body_mass[b] for b in bodies))
        if mass <= 0.0:
            return 0.0, np.zeros(3)
        c = sum(m.body_mass[b] * st.xipos[b] for b in bodies) / mass
        x0, R0 = self._palm_pose(st)
        return mass, R0.T @ (c - x0)

    def dip_of_pip(self, finger, q_pip):
        """The finger's DIP angle at PIP angle q_pip, every other hinge at 0."""
        return self.solve({C.PIP_OF_FINGER[finger]: float(q_pip)}).q[C.DIP_OF_FINGER[finger]]

    def coupling_form(self, finger):
        return self._loops[finger].form

    def dip_sense(self, finger):
        """+1 if the model flexes the finger's DIP with negative angles (the contract's sense rule), else -1."""
        m, d = self.model, self._d
        s = self.sid[finger]
        jd = self.jid[C.DIP_OF_FINGER[finger]]
        d.qpos[:] = self._qpos0
        for j in C.CONTRACT_JOINTS:
            d.qpos[self._qadr[self.jid[j]]] = 0.0
        mujoco.mj_kinematics(m, d)
        p0 = d.site_xpos[s].copy()
        R0 = d.xmat[self.palm].reshape(3, 3).copy()
        d.qpos[self._qadr[jd]] = C.DIP_SENSE_STEP_RAD
        mujoco.mj_kinematics(m, d)
        delta = R0.T @ (d.site_xpos[s] - p0)
        return 1 if float(delta @ np.asarray(C.PALM_SIDE_DIR[finger], float)) > 0.0 else -1

    def force_ranges(self):
        """f per driven hinge: the upper end of its actuator's force range (contract: act_<joint>)."""
        m = self.model
        out = {}
        for j in C.MOTOR_ORDER:
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, C.ACTUATOR_OF_JOINT[j])
            if a < 0:
                raise ContractError("actuator %s is missing" % C.ACTUATOR_OF_JOINT[j])
            if not m.actuator_forcelimited[a]:
                out[j] = float("inf")
            else:
                out[j] = float(m.actuator_forcerange[a][1])
        return out
