"""The plant session of one hand unit: the hand, its servos, the cell and the specimen, advanced tick by tick.

One tick is one millisecond: the host's pending writes take effect (in the contract's order), the arm's field and the
nests are set for this instant, the firmware ticks (encoder, estimators, its controller), the electrical stage gives
the servo torques, and the physics steps once. Reads sample the register file the last tick left. A specimen placed
in a nest is part of the physics until it is removed; the model is rebuilt with it (the hand's state carried over
unchanged) and without it again when it goes.
measure() and truth() report the state the last physics step saw (positions and contacts at the start of that step);
nothing of them reaches the controller. Every random stream comes from the unit's seeds (noise.py); there is no clock
and no global random state, so two sessions of one unit driven alike are bit-identical.
The tactile modules (tactile.py): at the first tick at or after each frame's capture time the board's frame is made
from the pad-specimen contacts of the last physics step; it arrives the transfer time later. A re-zero takes effect
at the next tick, like a write. Every frame's bytes enter the state trace.
"""
import copy
import hashlib
import os
import struct

import mujoco
import numpy as np

from contract import constants as C
from contract import names as N
from plant import MODEL_DIR, MODEL_FILE, TRUTH, bus, cell, noise, specimens, tactile
from plant import firmware as F

_CON = TRUTH["contact"]
PAD_GROUP = 1
CLEARANCE_M = C.PLACEMENT["clearance_mm"] / 1000.0
TACTILE_MODELLED = True


class PlantError(RuntimeError):
    """The physics went somewhere it cannot come back from (a non-finite state)."""


class Session:
    """See the module docstring. unit: schemas/unit_truth.json; model_dir: None = the plant's own model folder;
    ceilings: the world's ceiling values (only the soft packs' travel limit is used), or None."""

    def __init__(self, unit, *, model_dir=None, ceilings=None):
        self.unit = copy.deepcopy(unit)
        servos = sorted(self.unit["servos"], key=lambda s: s["id"])
        if [s["id"] for s in servos] != list(range(N.N_SERVOS)):
            raise ValueError("unit truth must hold servos 0..12")
        self.soft_limit_mm = None if not ceilings else ceilings.get("SP_travel_mm")
        md = MODEL_DIR if model_dir is None else model_dir
        spec = mujoco.MjSpec.from_file(os.path.join(md, MODEL_FILE))
        for s in servos:
            j = spec.joint(N.SERVOS[s["id"]].joint)
            j.frictionloss = float(s["friction_nm"])
            j.armature = float(s["armature"])
        m_hand = spec.compile()
        self.taught = cell.Taught(m_hand)
        cell.add_cell(spec, self.taught)
        self._spec = spec
        self.m_empty = spec.compile()
        self.m = self.m_empty
        self.g = float(np.linalg.norm(self.m.opt.gravity))
        self.field = cell.Field(self.taught, self.g)
        self._job = None
        self._geo = None
        self._maps(self.m)
        self.d = mujoco.MjData(self.m)
        self._set_contact_params(self.m, _CON["nest_friction"])
        self._set_mocap(0.0)
        mujoco.mj_forward(self.m, self.d)
        self.fw = F.ServoBank(self.unit, q0=self.d.qpos[self.sq].copy())
        self.enc = noise.NormalStream(int(self.unit["seeds"]["encoder"]), N.N_SERVOS)
        self.maps = tactile.Maps(md)
        self.tac = tactile.Stream(self.unit, self.maps)
        self._t = 0
        self._pending = None
        self._scratch = {}
        self._hash = hashlib.sha256()
        self._jacp = np.zeros((3, self.m.nv))
        self._jacr = np.zeros((3, self.m.nv))

    # ------------------------------------------------------------------------------------------------ set-up
    def _maps(self, m):
        self.nq_hand = self.nv_hand = len(N.JOINTS)
        self.sq = np.array([m.jnt_qposadr[m.joint(s.joint).id] for s in N.SERVOS])
        self.sd = np.array([m.jnt_dofadr[m.joint(s.joint).id] for s in N.SERVOS])
        self.jq = np.array([m.jnt_qposadr[m.joint(j).id] for j in N.JOINTS])
        self.jd = np.array([m.jnt_dofadr[m.joint(j).id] for j in N.JOINTS])
        if max(self.jq) >= self.nq_hand or max(self.jd) >= self.nv_hand:
            raise ValueError("the hand's joints must come first in the model")
        links = {m.body(lk).id: lk for lk in N.LINKS}
        self.link_body = {lk: b for b, lk in links.items()}
        self.hand_geoms = np.array([g for g in range(m.ngeom) if m.geom_bodyid[g] in links])
        self.mod_body = {mod: m.body(N.MODULE_LINK[mod]).id for mod in N.MODULES}
        self.pad_site = {mod: m.site(N.site_name(N.MODULE_LINK[mod], "pad")).id for mod in N.MODULES}
        self.nest_mocap = {nest: int(m.body_mocapid[m.body("nest_" + nest).id]) for nest in N.NESTS}
        self.probe_mocap = int(m.body_mocapid[m.body(cell.PROBE_BODY).id])
        self.probe_geoms = [m.geom(g).id for g in cell.PROBE_GEOMS]
        self.loop_sites = {f: (m.site(N.site_name("%s_dip" % f, "fourbar")).id,
                               m.site(N.site_name("%s_rocker" % f, "fourbar")).id) for f in N.FINGERS}

    def _kinds(self, m):
        """Per geom: 0 other, 1..4 a pad of module k-1, 5 another hand piece, 6 / 7 a nest, 8 the specimen."""
        kind = np.zeros(m.ngeom, np.int64)
        for g in self.hand_geoms:
            kind[g] = 5
        for k, mod in enumerate(N.MODULES):
            for g in self.hand_geoms:
                if m.geom_bodyid[g] == self.mod_body[mod] and m.geom_group[g] == PAD_GROUP:
                    kind[g] = 1 + k
        for k, nest in enumerate(N.NESTS):
            b = m.body("nest_" + nest).id
            kind[m.geom_bodyid == b] = 6 + k
        if self._job is not None:
            for name in (specimens.GEOM_A, specimens.GEOM_B):
                kind[m.geom(name).id] = 8
        return kind

    def _set_contact_params(self, m, mu):
        g = self.hand_geoms
        m.geom_friction[g, 0] = mu
        m.geom_friction[g, 1] = _CON["torsional_m"]
        m.geom_friction[g, 2] = _CON["rolling"]
        m.geom_solref[g] = _CON["pad_solref"]
        m.geom_solimp[g] = _CON["pad_solimp"]
        self.kind = self._kinds(m)

    def _set_mocap(self, t_s):
        for nest, mid in self.nest_mocap.items():
            p, R = self.field.nest_h(nest, t_s)
            self.d.mocap_pos[mid] = p
            self.d.mocap_quat[mid] = cell.quat_of(R)

    def _switch(self, m_new):
        d_old, d_new = self.d, mujoco.MjData(m_new)
        nq, nv = self.nq_hand, self.nv_hand
        d_new.time = d_old.time
        d_new.qpos[:nq] = d_old.qpos[:nq]
        d_new.qvel[:nv] = d_old.qvel[:nv]
        d_new.qacc_warmstart[:nv] = d_old.qacc_warmstart[:nv]
        d_new.qfrc_applied[:nv] = d_old.qfrc_applied[:nv]
        d_new.mocap_pos[:] = d_old.mocap_pos
        d_new.mocap_quat[:] = d_old.mocap_quat
        m_new.opt.gravity[:] = self.m.opt.gravity
        self.m, self.d = m_new, d_new
        self._jacp = np.zeros((3, m_new.nv))
        self._jacr = np.zeros((3, m_new.nv))
        mujoco.mj_forward(self.m, self.d)

    # ------------------------------------------------------------------------------------------------ the API
    @property
    def t_ms(self):
        return self._t

    @staticmethod
    def period_ms(reads, requested=None):
        """The effective cycle period (ms) for the read blocks (bus rule; requested defaults to the shortest)."""
        return bus.period_ms(reads, requested)

    def write(self, fields):
        """Queue a step reply's "write" dict for the next tick; return the writes that will be refused."""
        if not fields:
            return []
        if self._pending is not None:
            raise RuntimeError("one write per tick: a write is already pending")
        clean = {}
        for k, vals in fields.items():
            if k not in N.WRITE_FIELDS:
                raise ValueError("not a write field: %r" % (k,))
            if vals is None:
                continue
            if not isinstance(vals, (list, tuple)) or len(vals) != N.N_SERVOS:
                raise ValueError("%s: a list of %d entries" % (k, N.N_SERVOS))
            for v in vals:
                if v is not None and (isinstance(v, bool) or not isinstance(v, (int, np.integer))):
                    raise ValueError("%s: integers or nulls" % k)
            clean[k] = [None if v is None else int(v) for v in vals]
        refused = self.fw.refused_for(clean)
        self._pending = (clean, refused)
        return [list(x) for x in refused]

    def read(self, reads):
        """{key: [13 ints]} for every register wholly inside the read blocks, sampled at the present tick."""
        return bus.sync_read(self.fw.reg, bus.check_reads([list(b) for b in reads]))

    def tactile_zero(self, modules):
        """Re-zero the named modules at the next tick: each one's zero becomes the values it reports then."""
        self.tac.request_zero(list(modules))

    def tactile_newest(self, after_seq):
        """The newest frame arrived by the present tick with seq > after_seq: (seq, t_capture_ms, t_arrival_ms,
        {module: bytes}), or None."""
        return self.tac.newest(int(after_seq), self._t)

    def cell_rotation_in_hand(self):
        """The cell frame's axes (columns: x, y, z up) in H at the pick pose."""
        return self.taught.pick_R.copy()

    def place(self, spec, nest):
        """Place a specimen (a scenario job's "spec") in a nest at its placed pose; the physics then holds it."""
        if nest not in N.NESTS:
            raise ValueError("not a nest: %r" % (nest,))
        if self._job is not None:
            raise RuntimeError("a specimen is already placed")
        geo = specimens.geometry(spec, self.soft_limit_mm)
        pn, Rn = self.field.nest_h(nest, self._t / 1000.0)
        pos, R = specimens.placed_pose(spec, pn, Rn)
        s2 = self._spec.copy()
        specimens.add(s2, geo, pos, R)
        m_new = s2.compile()
        self._job, self._geo = dict(spec), geo
        self._switch(m_new)
        self._set_contact_params(self.m, geo["mu"])
        mujoco.mj_forward(self.m, self.d)
        m = self.m
        self.bA, self.bB = m.body(specimens.BODY_A).id, m.body(specimens.BODY_B).id
        self.gA, self.gB = m.geom(specimens.GEOM_A).id, m.geom(specimens.GEOM_B).id
        self.spring_q = m.jnt_qposadr[m.joint(specimens.SPRING).id]

    def remove(self):
        """Take the specimen out of the cell (the hand's state carries on unchanged)."""
        if self._job is None:
            return
        self._job, self._geo = None, None
        self._switch(self.m_empty)
        self._set_contact_params(self.m, _CON["nest_friction"])

    def set_base(self, kind, t0_ms, params):
        """The arm's next motion segment (names.SET_BASE_KINDS), starting at t0_ms >= t_ms."""
        self.field.set(kind, int(t0_ms), dict(params or {}), self._t)

    def advance(self, n_ms):
        """n physics steps of 1 ms, each with its firmware tick."""
        m, d, fw = self.m, self.d, self.fw
        dt = m.opt.timestep
        for _ in range(int(n_ms)):
            if self._pending is not None:
                fields, want = self._pending
                self._pending = None
                got = fw.apply(fields)
                if got != want:
                    raise RuntimeError("refused writes changed between write and apply")
            t_s = self._t / 1000.0
            self.field.prune(t_s)
            m.opt.gravity[:] = self.field.gravity_h(t_s)
            self._set_mocap(t_s)
            fw.tick(d.qpos[self.sq], self.enc.row(self._t))
            d.qfrc_applied[self.sd] = fw.substep(d.qvel[self.sd], dt)
            mujoco.mj_step(m, d)
            self._t += 1
            h = self._hash
            h.update(struct.pack("<q", self._t))
            h.update(d.qpos.tobytes())
            h.update(d.qvel.tobytes())
            h.update(fw.I_rep.tobytes())
            h.update(fw.Tw.tobytes())
            if not np.isfinite(d.qpos).all():
                raise PlantError("non-finite state at t = %d ms" % self._t)
            tac = self.tac
            if tac.pending_zero or tac.next_capture_ms() <= self._t:
                con = self._tactile_contacts()
                if tac.pending_zero:
                    tac.apply_zero(con)
                while tac.next_capture_ms() <= self._t:
                    seq, _tc, _ta, frame = tac.capture(con)
                    h.update(struct.pack("<q", seq))
                    for mod in N.MODULES:
                        h.update(frame[mod])

    def _tactile_contacts(self):
        """The pad-specimen contacts of the last step in each module's frame: {module: (pos mm, normal from the pad
        into the specimen, normal force N, force on the pad N)}."""
        out = {mod: tactile.NO_CONTACT for mod in N.MODULES}
        if self._job is None:
            return out
        g1, g2, idx, k1, k2 = self._contacts()
        sel = ((k1 == 8) & (k2 >= 1) & (k2 <= 4)) | ((k2 == 8) & (k1 >= 1) & (k1 <= 4))
        if not np.any(sel):
            return out
        m, d = self.m, self.d
        f6 = np.zeros(6)
        rows = {mod: [] for mod in N.MODULES}
        for i, ka, kb in zip(idx[sel], k1[sel], k2[sel]):
            c = d.contact[int(i)]
            mujoco.mj_contactForce(m, d, int(i), f6)
            fr = np.array(c.frame).reshape(3, 3)
            f_geom2 = fr.T @ f6[:3]                       # the force on geom2, in H
            if kb == 8:                                   # geom2 is the specimen: geom1 is the pad
                mod, nrm, fpad = N.MODULES[ka - 1], fr[0], -f_geom2
            else:
                mod, nrm, fpad = N.MODULES[kb - 1], -fr[0], f_geom2
            rows[mod].append((np.array(c.pos), nrm, float(f6[0]), fpad))
        for mod, rr in rows.items():
            if not rr:
                continue
            v = self.maps.modules[mod]
            b = self.mod_body[mod]
            Rl = d.xmat[b].reshape(3, 3)
            Rm = Rl @ v["axes_link"]
            om = d.xpos[b] + Rl @ v["origin_link_m"]
            pos = np.array([r[0] for r in rr])
            out[mod] = ((pos - om) @ Rm * 1000.0, np.array([r[1] for r in rr]) @ Rm, np.array([r[2] for r in rr]),
                        np.array([r[3] for r in rr]) @ Rm)
        return out

    # ------------------------------------------------------------------------------------------------ geometry
    def clear_for(self, spec, nest):
        """True when every hand geometry is at least the clearance (2 mm) from the specimen at its placed pose."""
        return self.clearance_m(spec, nest) >= CLEARANCE_M

    def clearance_m(self, spec, nest, *, cap=0.05):
        """The least distance (m, capped) between a hand geometry and the specimen at its placed pose in a nest."""
        geo = specimens.geometry(spec, self.soft_limit_mm)
        pn, Rn = self.field.nest_h(nest, self._t / 1000.0)
        pos, R = specimens.placed_pose(spec, pn, Rn)
        m = self.m
        key = id(m)
        if key not in self._scratch:
            self._scratch = {key: mujoco.MjData(m)}
        ds = self._scratch[key]
        ds.qpos[:] = self.d.qpos
        ds.mocap_pos[:] = self.d.mocap_pos
        ds.mocap_quat[:] = self.d.mocap_quat
        ds.mocap_pos[self.probe_mocap] = pos
        ds.mocap_quat[self.probe_mocap] = cell.quat_of(R)
        t, L, H = geo["t_m"], geo["L_m"], geo["H_m"]
        for g, x in zip(self.probe_geoms, specimens.half_offsets(geo)):
            m.geom_size[g] = (t / 2.0, L / 2.0, H / 2.0)
            m.geom_pos[g] = (x, 0.0, 0.0)
            m.geom_sameframe[g] = 0                    # the frame no longer coincides with the body's
            m.geom_rbound[g] = float(np.linalg.norm(m.geom_size[g]))
            m.geom_aabb[g] = (0.0, 0.0, 0.0, t / 2.0, L / 2.0, H / 2.0)
        mujoco.mj_kinematics(m, ds)
        best = cap
        hg = self.hand_geoms
        for g in self.probe_geoms:
            gap = (np.linalg.norm(ds.geom_xpos[hg] - ds.geom_xpos[g], axis=1) - m.geom_rbound[hg] - m.geom_rbound[g])
            for h in hg[gap < best]:
                dist = mujoco.mj_geomDistance(m, ds, int(h), int(g), best, None)
                if dist < best:
                    best = dist
        return float(best)

    def _contacts(self):
        """The contacts of the last step as (geom1, geom2, contact index) arrays, and the kinds of both geoms."""
        n = self.d.ncon
        if n == 0:
            e = np.zeros(0, np.int64)
            return e, e, e, e, e
        g1 = np.asarray(self.d.contact.geom1[:n])
        g2 = np.asarray(self.d.contact.geom2[:n])
        return g1, g2, np.arange(n), self.kind[g1], self.kind[g2]

    def _nest_of(self, pmid):
        px, py = cell.pocket_half()
        for nest, mid in self.nest_mocap.items():
            pn = self.d.mocap_pos[mid]
            Rn = cell.mat_of(self.d.mocap_quat[mid])
            loc = Rn.T @ (pmid - pn)
            if abs(loc[0]) <= px and abs(loc[1]) <= py and -cell.HALF_H <= loc[2] <= cell.HALF_H + 0.1:
                return nest, pn, Rn
        return None, None, None

    def _lowest_above(self, pn, Rn):
        z = []
        for g in (self.gA, self.gB):
            sz = self.m.geom_size[g]
            Rg = self.d.geom_xmat[g].reshape(3, 3)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for s3 in (-1, 1):
                        p = self.d.geom_xpos[g] + Rg @ (np.array([sx, sy, s3]) * sz)
                        z.append((Rn.T @ (p - pn))[2])
        return float(min(z) + cell.HALF_H)

    def _spring(self):
        x = max(0.0, float(self.d.qpos[self.spring_q]))
        return self._geo["k_n_per_m"] * x, x

    def specimen_spring(self):
        """(spring force N, travel m) of the placed specimen at the present tick, or None: a cheap read for checks
        that run every tick."""
        if self._job is None:
            return None
        return self._spring()

    def measure(self):
        """The grader's measurements (schemas/measure.json)."""
        fw = self.fw
        out = {"t_ms": self._t, "specimen": None,
               "servos": {"present_temperature_c": [int(x) for x in fw.reg["present_temperature"]],
                          "winding_c": [float(x) for x in fw.Tw], "hardware_error": [int(x) for x in fw.hw],
                          "torque_on": [bool(x) for x in fw.torque], "duty": [abs(float(x)) for x in fw.duty]}}
        if self._job is None:
            return out
        d = self.d
        pmid = 0.5 * (d.xpos[self.bA] + d.xpos[self.bB])
        g1, g2, _i, k1, k2 = self._contacts()
        sp = (k1 == 8) | (k2 == 8)
        other = np.where(k1 == 8, k2, k1)[sp]
        pad = {mod: bool(np.any(other == 1 + k)) for k, mod in enumerate(N.MODULES)}
        nest, pn, Rn = self._nest_of(pmid)
        force, travel = self._spring()
        out["specimen"] = {"nest": nest, "pos_m": [float(x) for x in pmid],
                           "quat_wxyz": [float(x) for x in d.xquat[self.bA]],
                           "spring_force_n": float(force), "spring_travel_m": float(travel),
                           "nest_contact": bool(np.any((other == 6) | (other == 7))),
                           "lowest_above_nest_m": None if nest is None else self._lowest_above(pn, Rn),
                           "pad_contact": pad, "other_hand_contact": bool(np.any(other == 5))}
        return out

    def _coupling(self, f):
        """d(dip)/d(pip) of a finger's four-bar at the present posture (from the loop constraint's Jacobians)."""
        m, d = self.m, self.d
        s1, s2 = self.loop_sites[f]
        J1, J2 = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, J1, None, s1)
        mujoco.mj_jacSite(m, d, J2, None, s2)
        jd = {a: m.jnt_dofadr[m.joint("%s %s" % (f, a)).id] for a in ("pip", "dip", "rocker")}
        A = np.column_stack([J1[:, jd["dip"]] - J2[:, jd["dip"]], -(J2[:, jd["rocker"]] - J1[:, jd["rocker"]])])
        b = -(J1[:, jd["pip"]] - J2[:, jd["pip"]])
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        return float(x[0]), jd

    def truth(self):
        """The truth (schemas/truth.json) for the truth script, the ceilings and the corner check only."""
        m, d, fw = self.m, self.d, self.fw
        t_s = self._t / 1000.0
        sp = None
        contacts = []
        cent = {mod: [np.zeros(3), 0.0] for mod in N.MODULES}
        if self._job is not None:
            force, travel = self._spring()
            sp = {"com_m": [float(x) for x in d.subtree_com[self.bA]],
                  "quat_wxyz": [float(x) for x in d.xquat[self.bA]], "mass_kg": float(self._geo["mass_kg"]),
                  "mu": float(self._geo["mu"]), "gamma_m": float(_CON["torsional_m"]),
                  "squeeze_axis": [float(x) for x in d.xmat[self.bA].reshape(3, 3)[:, 0]],
                  "spring_force_n": float(force), "spring_travel_m": float(travel)}
            g1, g2, idx, k1, k2 = self._contacts()
            f6 = np.zeros(6)
            for a, b, i, ka, kb in zip(g1, g2, idx, k1, k2):
                if not ((ka == 8 and 1 <= kb <= 4) or (kb == 8 and 1 <= ka <= 4)):
                    continue
                c = d.contact[int(i)]
                mujoco.mj_contactForce(m, d, int(i), f6)
                fr = np.array(c.frame).reshape(3, 3)
                if kb == 8:                       # geom2 is the specimen: the frame's normal points pad -> specimen
                    mod = N.MODULES[ka - 1]
                    frame, fv = fr, [f6[0], f6[1], f6[2]]
                else:                             # flip to keep the normal pad -> specimen, right-handed
                    mod = N.MODULES[kb - 1]
                    frame = np.array([-fr[0], -fr[1], fr[2]])
                    fv = [f6[0], f6[1], -f6[2]]
                pos = np.array(c.pos)
                contacts.append({"module": mod, "pos_m": [float(x) for x in pos],
                                 "normal": [float(x) for x in frame[0]],
                                 "frame": [[float(x) for x in r] for r in frame],
                                 "force_n": [float(x) for x in fv]})
                cent[mod][0] += f6[0] * pos
                cent[mod][1] += f6[0]
        jac = {}
        for mod in N.MODULES:
            p, w = cent[mod]
            point = p / w if w > 1e-9 else d.site_xpos[self.pad_site[mod]].copy()
            jp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, None, point, self.mod_body[mod])
            J = jp[:, self.sd].copy()
            if mod in N.FINGERS:
                c, jd = self._coupling(mod)
                k = N.SERVO_ID["%s pip" % mod]
                J[:, k] += jp[:, jd["dip"]] * c
            jac[mod] = [[float(x) for x in row] for row in J]
        return {"t_ms": self._t, "field": {"g_hand_mps2": [float(x) for x in self.field.gravity_h(t_s)]},
                "specimen": sp, "contacts": contacts, "jac": jac,
                "joints": {"names": list(N.JOINTS), "q_rad": [float(x) for x in d.qpos[self.jq]],
                           "qd_rad_s": [float(x) for x in d.qvel[self.jd]]},
                "servos": {"winding_c": [float(x) for x in fw.Tw], "case_c": [float(x) for x in fw.Tc],
                           "i_motor_a": [float(x) for x in fw.I_m], "i_input_ma": [float(x) for x in fw.input_ma()],
                           "duty": [abs(float(x)) for x in fw.duty], "demand_ma": [float(x) for x in fw.I_ref],
                           "bias_nm": [float(x) for x in d.qfrc_bias[self.sd]]},
                "unit": copy.deepcopy(self.unit)}

    def trace_digest(self):
        """sha256 of the state trace so far (every tick's positions, velocities, readings and winding temperatures)."""
        return self._hash.copy().hexdigest()
