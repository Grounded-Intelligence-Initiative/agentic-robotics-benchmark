"""The privileged reference: a truth script that grips every carry job and declines every decline job.

It runs inside the grader only when the privileged token matches; it reads the plant's truth (true contacts, the
specimen's mass and friction, the pad Jacobians, the servos' winding temperatures and the unit's constants) and
commands the hand through the same bus as any controller. Its logic, per job:
- open: the taught posture of the job's width plus 16 mm; the ring finger parked;
- close: along the taught family, fast to 3 mm before the width, then 4 mm/s, until the true spring force reaches
  0.3 N (the touch);
- squeeze and hold, the reference (the true beliefs): one wanted squeeze per class or size code, F_REF =
  REF_LS_FACTOR (1.3) x the simulator's least squeeze of the job's code (read from world/data/least_squeeze.json,
  never typed in), capped at 0.8 x the ceiling (shells), at the squeeze of 0.8 x the travel limit at the pack's own
  stiffness (soft packs) and at the squeeze whose predicted duty is REF_DUTY_CAP (0.90) on the most loaded servo of
  the grasp (evaluated every cycle on the present Jacobians, field and winding temperature); reached by the squeeze
  ramp from 0.3 N over 0.3 s (no step) and held; these caps limit the wanted squeeze (the true-force target), not
  the command;
- squeeze and hold, the open-loop form (the variants' base): the wanted squeeze is the least thumb force that holds
  the specimen on the true contacts (hold_lp) times 1.3 plus half the specimen's weight along the squeeze axis,
  taken over the transfer's field 0.40 s ahead (so it is raised before each acceleration peak), ramped from 0.3 N
  over 0.3 s, and the COMMANDED squeeze is capped at 0.8 x the ceiling (shells) or the force of 0.8 x the travel limit
  (soft packs) (this form feeds no true force back);
- in both: the pad forces along the squeeze axis (thumb against the two fingers, equal shares), the specimen's weight
  in thirds and the hand's own gravity are mapped to servo torques by the pad Jacobians and converted to a position
  offset by the believed current law (true: motor current = torque / kt; supply-input demand = standby + R(T) I^2 /
  Vs; offset = demand / (u_P P / 128) counts), each servo's torque capped at the torque of a predicted duty of 0.95;
- the force feedback (the reference): from the touch to the release request the commanded squeeze also closes on
  the true spring force (an integral of its shortfall against the wanted squeeze, FORCE_KI per s; the wanted
  squeeze's changes pass in full), so the wanted squeeze is on the specimen however it moved in the nest; the
  command is then limited on the servos by the torque cap of predicted duty 0.95 (and by the integral's own bound,
  CMD_LIMIT_FACTOR x the wanted squeeze + CMD_LIMIT_OFFSET_N), not by the code's cap;
- the grasped flag 0.4 s after the touch; release: the squeeze ramped to zero over 0.2 s, then open at 40 mm/s;
- a decline job: the decline flag at the first cycle of "present", the hand stays open (the class-typical object
  belief never declines: it grips every job as a carry).
Two forms: "reference" (the true beliefs: the class squeeze and the force feedback) and "open_loop" (the base of the
single-fault variants: the plan converted to position offsets with no force fed back; with the true beliefs it is
"the base", a run of its own). Single-fault variants change one belief of REFERENCE_BELIEFS (variants.json: law,
loop_scale, object, touch, heat, close_speed) on the open-loop form. The touch-as-force belief keeps its loop on the
touch reading in every phase.
The script keeps a record for the maintainers (nothing is commanded from it): per carry job its squeeze (the class
squeeze's terms and which one bound; per cell phase the true spring force against the wanted squeeze), and per unit
the hottest winding at the end of each shift and the servos' duty sampled at every cycle (the largest, the time and
the longest run above DUTY_RECORD_LEVEL, the overload counter's longest run).

Grip is the per-job logic on a truth frame (any plant that gives the frame can run it); TruthScript is its adapter
to the acceptance loop (setup / command).
"""
import math

import numpy as np

from world import hold_lp

REFERENCE_BELIEFS = {"law": "true", "loop_scale": "true", "object": "true", "touch": "none", "heat": "aware",
                     "close_speed": "slow"}
BELIEF_VALUES = {"law": ("true", "proportional"), "loop_scale": ("true", "makers_128"),
                 "object": ("true", "class_typical"), "touch": ("none", "sum_is_force"),
                 "heat": ("aware", "ignored"), "close_speed": ("slow", "fast_1radps")}

# the reference's own settings (its design, not properties of the hand)
PERIOD_MS = 5
READS = [[132, 4]]
P_GAIN = 1600
D_GAIN = 200
GOAL_CURRENT_MA = {"brittle": 600.0, "soft": 600.0, "full_load": 910.0}
GOAL_CURRENT_HOT_MA = 910.0
CLOSE_SPEED_M_S = 0.004
FAST_SPEED_M_S = 0.020
FAST_ZONE_M = 0.003
OPEN_EXTRA_M = 0.016
F_TOUCH_N = 0.3
SETTLE_S = 0.3                  # the slice's settle before the close (the slice reproduction runs with it)
RAMP_S = 0.3
GRASP_DELAY_S = 0.6             # the slice's lift 0.6 s after the touch
CELL_SETTLE_S = 0.1             # in the cell: the hand waits at the open pre-shape, so the close starts sooner
CELL_GRASP_DELAY_S = 0.4        # in the cell: the grasped flag 0.4 s after the touch (the squeeze ramp is 0.3 s)
T_LEAD_S = 0.40
MARGIN = 1.3
DEMAND_DT_S = 0.02
DEMAND_REFRESH_S = 0.5
RELEASE_RAMP_S = 0.2
OPEN_RATE_M_S = 0.04
CAP_FRACTION = 0.8
DUTY_CAP = 0.95
CMD_LIMIT_FACTOR, CMD_LIMIT_OFFSET_N = 3.0, 5.0
# the force feedback on the true spring force: its integral gain (the slice's ff P1600 runs with force feedback, and
# the least-squeeze searches behind the printed ceilings, use the same 5 per s) and the plan mark where it stops
# ("t_end": every phase);
# None = no feedback (the open-loop logic of the slice's rows without force feedback: the variants' base)
FORCE_KI = 5.0
CORRECT_UNTIL = "t_end"
CORRECTION_ENDS = ("t_end",)
# the forms of the script: the reference (the class squeeze and the feedback) and the open-loop base of the variants
FORMS = ("reference", "open_loop")
FORM_GRIP = {"reference": {"correct_until": "t_end", "class_squeeze": True},
             "open_loop": {"correct_until": None, "class_squeeze": False}}
# the reference's squeeze of a carry job: this factor x the simulator's least squeeze of the job's code, capped (as
# well as by the code's cap) at the squeeze whose predicted duty on the most loaded grasp servo is REF_DUTY_CAP
REF_LS_FACTOR = 1.3
REF_DUTY_CAP = 0.90
# the record's duty level (the time and the longest run above it are recorded; nothing is commanded from it)
DUTY_RECORD_LEVEL = 0.90
# the variants' settings
LINEAR_A_PER_NM = 0.69 / 0.72                 # "current proportional to torque" at the curve's stall point
GC_OVER_COUNTS = 60.0
TOUCH_KI = 5.0
HOT_FACTOR, HOT_CAP_FRACTION = 2.0, 0.85
FAST_PIP_RAD_S = 1.0
TYPICAL_MU = {"matte": 0.475, "rough": 0.625}
TYPICAL_SOFT_K_N_PER_MM = 3.16
COUNT_RAD = 2.0 * math.pi / 4096.0
GRASP_DIGITS = (("thumb", 1.0), ("index", -0.5), ("middle", -0.5))


def merge_beliefs(beliefs=None):
    """The reference's beliefs with the given ones replaced (each key and value checked)."""
    b = dict(REFERENCE_BELIEFS)
    for k, v in (beliefs or {}).items():
        if k not in BELIEF_VALUES or v not in BELIEF_VALUES[k]:
            raise ValueError("unknown belief %r = %r" % (k, v))
        b[k] = v
    return b


class Law:
    """The current law the script uses to turn a held torque into a current (per servo arrays): the true one
    (supply-input current, the loop regulating the reading with the standby current), or a belief."""

    def __init__(self, kt, R, Vs, I0_a, u_P, raw=True):
        self.kt = np.asarray(kt, float)
        self.R = np.asarray(R, float)
        self.Vs = np.asarray(Vs, float)
        self.I0 = np.asarray(I0_a, float)
        self.u_P = float(u_P)
        self.raw = bool(raw)

    def current_ma(self, tau_abs, belief="true"):
        """mA to demand for a held torque |tau| (N m) at a stall."""
        if belief == "proportional":
            return tau_abs * LINEAR_A_PER_NM * 1e3
        Im = tau_abs / self.kt
        sup = self.R * Im * Im / self.Vs * 1e3
        if self.raw:
            return self.I0 * 1e3 * np.minimum(1.0, tau_abs / 0.01) + sup
        return sup

    def kd_ma_per_count(self, P, belief="true"):
        """mA of demand per count of position error."""
        if belief == "makers_128":
            return P / 128.0
        return self.u_P * P / 128.0

    def tau_at_duty(self, duty):
        """The stall torque at a predicted duty (motor voltage R I_m over the supply)."""
        return self.kt * duty * self.Vs / self.R


class Grip:
    """One job's grip, one call per host cycle (see the module docstring).

    q_of_w(w) -> servo angles (rad) of the taught family at width w (m); lo, hi: the servos' limits (rad);
    thumb: boolean mask of the thumb servos; pip: index of the servo whose rate sets the fast close's speed;
    plan_at_touch(t_touch) -> a plan with .g_hand_at(t) (the field in the hand frame, m/s^2) and .marks
    ("t_transfer0", "t_release", "t_end"; "t_release" may be inf when the release is requested by the caller).
    """

    def __init__(self, *, beliefs, cls, finish, width_m, mass_kg, mu, gamma_m, k_n_per_mm, ceiling, q_of_w, lo,
                 hi, thumb, pip, plan_at_touch, host_dt=PERIOD_MS / 1000.0, P=P_GAIN, ff=True, typical=None,
                 w_top=None, open_delta=None, const_squeeze=None, ki_f=None, settle_s=SETTLE_S, follow_spring=False,
                 correct_until=CORRECT_UNTIL, ref_ls_n=None):
        self.b = merge_beliefs(beliefs)
        # the feedback on the believed squeeze force from the touch to this plan mark (None: none)
        if correct_until is not None and correct_until not in CORRECTION_ENDS:
            raise ValueError("unknown end of the correction %r" % (correct_until,))
        self.correct_until = correct_until
        # the class squeeze of the reference: the least squeeze of the job's code (N), or None (the plan's squeeze)
        self.ref_ls = None
        if ref_ls_n is not None:
            if const_squeeze is not None:
                raise ValueError("a constant squeeze and the class squeeze exclude each other")
            if not float(ref_ls_n) > 0.0:
                raise ValueError("the class squeeze needs a positive least squeeze")
            self.ref_ls = float(ref_ls_n)
        self.ref_rec = None if self.ref_ls is None else {"cycles": 0, "bound_cycles": {}, "terms_min_n": {},
                                                         "wanted_min_n": None, "wanted_max_n": None}
        self.cls = cls
        self.q_family = q_of_w
        # the closing path: from the open pre-shape (the family's top plus open_delta) along the family, the extra
        # opening fading out linearly before the slow zone; without them, the family from width + OPEN_EXTRA_M
        self.w_top = None if w_top is None else float(w_top)
        self.delta = None if open_delta is None else np.asarray(open_delta, float)
        self.const_squeeze = None if const_squeeze is None else float(const_squeeze)
        self.settle_s = float(settle_s)
        # the base posture follows the specimen's spring: the commanded squeeze compresses it by F / k past the touch,
        # so the offsets are taken from the posture of that narrower width (the slice did not: its shells barely give)
        self.follow_spring = bool(follow_spring)
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.thumb = np.asarray(thumb, bool)
        self.n = len(self.lo)
        self.plan_at_touch = plan_at_touch
        self.hdt = float(host_dt)
        self.P = float(P)
        self.ff = bool(ff)
        if self.b["object"] == "class_typical":
            t = typical or {}
            mass_kg = t["mass_kg"]
            mu = TYPICAL_MU[finish]
            if cls == "soft":
                k_n_per_mm = TYPICAL_SOFT_K_N_PER_MM
        self.mass, self.mu, self.gam, self.k = float(mass_kg), float(mu), float(gamma_m), float(k_n_per_mm)
        self.mh = self.mass / 2.0
        self.w_rest = float(width_m)
        self.w_open = self.w_rest + OPEN_EXTRA_M if self.w_top is None else max(self.w_top, self.w_rest + FAST_ZONE_M)
        self.w_cmd = self.w_open
        self.w_touch = self.w_rest
        hot = self.b["heat"] == "ignored"
        self.gc = GOAL_CURRENT_HOT_MA if hot else GOAL_CURRENT_MA[cls]
        self.ki_f = TOUCH_KI if self.b["touch"] == "sum_is_force" else 0.0
        if ki_f is not None:
            self.ki_f = float(ki_f)
        frac = HOT_CAP_FRACTION if hot else CAP_FRACTION
        if self.const_squeeze is not None:
            self.cap = None
        elif cls == "brittle":
            self.cap = frac * float(ceiling)
        elif cls == "soft":
            self.cap = frac * float(ceiling) * self.k          # ceiling = the travel limit in mm
        else:
            self.cap = None
        # the cap on the commanded squeeze: the plan's forms cap the command at the code's cap; the class squeeze
        # carries the code's cap in its wanted squeeze (the true-force target), so its command is not capped by it
        self.command_cap = self.cap if self.ref_ls is None else None
        if self.b["close_speed"] == "fast_1radps":
            dq = (self.q_family(self.w_rest + 0.001)[pip] - self.q_family(self.w_rest - 0.001)[pip]) / 0.002
            self.close_speed = FAST_PIP_RAD_S / abs(dq)
        else:
            self.close_speed = CLOSE_SPEED_M_S
        self.phase = "settle"
        self.plan = None
        self.dem = None
        self.dem_re = False
        self.F_des = 0.0
        self.F_cmd = 0.0
        self.F_law = 0.0
        self.t_touch = None
        self.t_rel = None
        self.F_rel0 = 0.0
        self.release_requested = False
        self.peak_demand = None

    def q_of_w(self, w):
        """The posture of the closing path at width w (m)."""
        q = self.q_family(w)
        if self.delta is None:
            return q
        span = self.w_open - self.w_rest - FAST_ZONE_M
        lam = 1.0 if span <= 0 else min(1.0, max(0.0, (w - self.w_rest - FAST_ZONE_M) / span))
        return q + lam * self.delta

    # ------------------------------------------------------------------------------------------- the demand
    def demand_table(self, t0, fr):
        P = fr["contacts"]
        com = np.asarray(fr["com"], float)
        u = np.asarray(fr["u"], float)
        r = [np.asarray(P["thumb"], float) - com, np.asarray(P["index"], float) - com,
             np.asarray(P["middle"], float) - com]
        ts = np.arange(t0, self.plan.marks["t_end"] + 0.5, DEMAND_DT_S)
        vals = np.zeros(len(ts))
        cache = {}
        for k, tt in enumerate(ts):
            ge = self.plan.g_hand_at(tt)
            key = tuple(np.round(ge, 2))
            if key not in cache:
                gu = float(ge @ u)
                NT = hold_lp.least_thumb_force(r[0], r[1], r[2], u, self.mass * ge, self.mu, self.gam)[0]
                if not math.isfinite(NT):          # no squeeze holds on these contacts: the two-face bound
                    NT = self.mass * float(np.linalg.norm(ge)) / (2.0 * self.mu)
                cache[key] = max(F_TOUCH_N, MARGIN * NT + self.mh * gu)
            vals[k] = cache[key]
        return ts, vals

    def _demand_now(self, t, fr):
        if self.const_squeeze is not None:
            return self.const_squeeze
        if self.ref_ls is not None:
            return self._class_squeeze(fr)
        if self.dem is None or (not self.dem_re and t >= self.plan.marks["t_transfer0"] - DEMAND_REFRESH_S):
            self.dem = self.demand_table(t, fr)
            self.dem_re = t >= self.plan.marks["t_transfer0"] - DEMAND_REFRESH_S
            pk = float(self.dem[1].max())
            self.peak_demand = pk if self.peak_demand is None else max(self.peak_demand, pk)
        ts, vals = self.dem
        k0 = int(np.searchsorted(ts, t))
        k1 = int(np.searchsorted(ts, t + T_LEAD_S))
        L = float(vals[k0:max(k1, k0 + 1)].max()) if k0 < len(ts) else float(vals[-1])
        if self.b["heat"] == "ignored":
            L = HOT_FACTOR * self.peak_demand
        if self.cap is not None:
            L = min(L, self.cap)
        return L

    # ---------------------------------------------------------------------------------------------- one cycle
    def request_release(self):
        self.release_requested = True

    @property
    def grasped(self):
        return self.t_touch is not None

    def touched_force(self, fr):
        """The force the script believes is the squeeze: the true spring force, or the touch reading."""
        if self.b["touch"] == "sum_is_force":
            return float(fr.get("touch_sum_n") or 0.0)
        return float(fr["spring_force_n"])

    def correcting(self, t):
        """Is the squeeze corrected on the force at job time t? From the touch until the plan's correction mark."""
        if self.correct_until is None or self.t_touch is None or self.plan is None:
            return False
        return t < self.plan.marks[self.correct_until] - 1e-9

    @staticmethod
    def _both_sides(fr):
        """The squeeze has formed: the thumb and a finger touch the specimen (a one-sided push of the specimen across
        its nest can raise the spring force too). A frame without contact flags counts as touching."""
        pads = fr.get("pads")
        if pads is None:
            return True
        return "thumb" in pads and ("index" in pads or "middle" in pads)

    # ------------------------------------------------------------------------------------- the class squeeze
    def duty_squeeze(self, fr, duty=DUTY_CAP):
        """The squeeze whose predicted duty is `duty` on the most loaded servo of the grasp: the servo torques are
        affine in the squeeze (the pad forces along the axis, plus the fed-forward weight and bias in the hold), each
        capped at the torque of that duty."""
        u = np.asarray(fr["u"], float)
        g_h = np.asarray(fr["g_hand"], float)
        a = np.zeros(len(np.asarray(fr["bias"], float)))
        b = np.zeros_like(a)
        for dg, sh in GRASP_DIGITS:
            J = np.asarray(fr["jac"][dg], float)
            a += J.T @ (sh * u)
            if self.ff:
                b -= J.T @ (self.mass * g_h / 3.0)
        if self.ff:
            b += np.asarray(fr["bias"], float)
        sel = fr.get("sel")
        if sel is not None:
            a, b = a[sel], b[sel]
        cap = np.asarray(fr["law"].tau_at_duty(duty), float)
        m = np.abs(a) > 1e-6 * max(1e-12, float(np.abs(a).max()))
        if not m.any():
            return float("inf")
        return max(0.0, float(((cap[m] - np.sign(a[m]) * b[m]) / np.abs(a[m])).min()))

    def _class_squeeze(self, fr):
        """The reference's wanted squeeze this cycle: REF_LS_FACTOR x the code's least squeeze, capped at the code's
        cap (0.8 x a shell's ceiling; for a soft pack the squeeze of 0.8 x its travel limit at its own stiffness) and
        at the squeeze whose predicted duty is REF_DUTY_CAP; which term bound is recorded."""
        terms = {"ls": REF_LS_FACTOR * self.ref_ls}
        if self.cap is not None:
            terms["ceiling" if self.cls == "brittle" else "travel"] = self.cap
        terms["duty"] = self.duty_squeeze(fr, REF_DUTY_CAP)
        bound, F = "ls", terms["ls"]
        for k in ("ceiling", "travel", "duty"):
            if k in terms and terms[k] < F:
                bound, F = k, terms[k]
        r = self.ref_rec
        r["cycles"] += 1
        r["bound_cycles"][bound] = r["bound_cycles"].get(bound, 0) + 1
        for k, v in terms.items():
            if math.isfinite(v):
                r["terms_min_n"][k] = v if k not in r["terms_min_n"] else min(r["terms_min_n"][k], v)
        r["wanted_min_n"] = F if r["wanted_min_n"] is None else min(r["wanted_min_n"], F)
        r["wanted_max_n"] = F if r["wanted_max_n"] is None else max(r["wanted_max_n"], F)
        return F

    def squeeze_record(self):
        """The class squeeze's record of the job (None for the plan's squeeze or a constant one)."""
        if self.ref_ls is None:
            return None
        r = self.ref_rec

        def n4(x):
            return None if x is None else round(float(x), 4)
        return {"ls_n": self.ref_ls, "f_ref_n": n4(REF_LS_FACTOR * self.ref_ls), "cap_n": n4(self.cap),
                "command_cap_n": n4(self.command_cap), "duty_cap": REF_DUTY_CAP, "cycles": r["cycles"],
                "bound_cycles": dict(r["bound_cycles"]),
                "terms_min_n": {k: n4(v) for k, v in r["terms_min_n"].items()},
                "wanted_min_n": n4(r["wanted_min_n"]), "wanted_max_n": n4(r["wanted_max_n"])}

    def step(self, t, fr):
        """One host cycle at time t (s) on the truth frame fr. Returns {"goal_rad", "goal_ma", "phase"}."""
        Fs = self.touched_force(fr)
        hdt = self.hdt
        if self.phase == "settle":
            if t >= self.settle_s - 1e-9 or fr.get("settled"):
                self.phase = "close"
        elif self.phase == "close":
            sp = max(FAST_SPEED_M_S, self.close_speed) if self.w_cmd > self.w_rest + FAST_ZONE_M else self.close_speed
            self.w_cmd -= sp * hdt
            if Fs > F_TOUCH_N and self._both_sides(fr):
                self.phase = "squeeze"
                self.t_touch = t
                self.w_touch = self.w_cmd
                self.plan = self.plan_at_touch(t)
                self.F_cmd = self.F_law = Fs
        if self.phase in ("squeeze", "hold"):
            Ldem = self._demand_now(t, fr)
            if self.phase == "squeeze":
                ramp = min(1.0, (t - self.t_touch) / RAMP_S)
                Fd_new = F_TOUCH_N + ramp * (Ldem - F_TOUCH_N)
                if ramp >= 1.0:
                    self.phase = "hold"
            else:
                Fd_new = Ldem
            if self.release_requested or t >= self.plan.marks["t_release"]:
                self.phase = "release"
                self.t_rel = t
                self.F_rel0 = self.F_cmd
            else:
                ki =max(self.ki_f, FORCE_KI) if self.correcting(t) else self.ki_f
                self.F_law += (Fd_new - self.F_des) + ki * (Fd_new - Fs) * hdt
                self.F_law = min(max(self.F_law, 0.0), CMD_LIMIT_FACTOR * Fd_new + CMD_LIMIT_OFFSET_N)
                if self.command_cap is not None:
                    self.F_law = min(self.F_law, self.command_cap)
                self.F_cmd = self.F_law
                self.F_des = Fd_new
        if self.phase == "release":
            fr_ = max(0.0, 1.0 - (t - self.t_rel) / RELEASE_RAMP_S)
            self.F_cmd = self.F_rel0 * fr_
            if fr_ <= 0.0:
                self.w_touch = min(self.w_open, self.w_touch + OPEN_RATE_M_S * hdt)
        goal_ma = np.full(self.n, self.gc)
        if self.phase in ("settle", "close"):
            goal = np.clip(self.q_of_w(self.w_cmd), self.lo, self.hi)
        else:
            goal, goal_ma = self._squeeze_goal(fr, goal_ma)
        return {"goal_rad": goal, "goal_ma": goal_ma, "phase": self.phase}

    def _w_base(self):
        """The width whose posture the squeeze's offsets start from."""
        if not self.follow_spring or self.phase not in ("squeeze", "hold") or self.k <= 0.0:
            return self.w_touch
        return self.w_touch - max(0.0, self.F_cmd - F_TOUCH_N) / (self.k * 1000.0)

    def _base(self):
        """The posture the squeeze's offsets start from (servo angles, rad): the closing path at the base width."""
        return self.q_of_w(self._w_base())

    def _squeeze_goal(self, fr, goal_ma):
        u_act = np.asarray(fr["u"], float)
        jac = fr["jac"]
        bias = np.asarray(fr["bias"], float)
        tau = np.zeros(len(bias))
        g_h = np.asarray(fr["g_hand"], float)
        hold = self.phase == "hold"
        for dg, sh in GRASP_DIGITS:
            J = np.asarray(jac[dg], float)
            tau += J.T @ (sh * self.F_cmd * u_act)
            if self.ff and hold:
                tau -= J.T @ (self.mass * g_h / 3.0)
        if self.ff and hold:
            tau += bias
        sel = fr.get("sel")
        ts_ = tau[sel] if sel is not None else tau
        law = fr["law"]
        cap = law.tau_at_duty(DUTY_CAP)
        ts_ = np.sign(ts_) * np.minimum(np.abs(ts_), cap)
        lb = "proportional" if self.b["law"] == "proportional" else "true"
        I = law.current_ma(np.abs(ts_), lb)
        kd = law.kd_ma_per_count(self.P, self.b["loop_scale"])
        base = self._base()
        u_ctrl = base + np.sign(ts_) * I / kd * COUNT_RAD
        if lb == "proportional" and self.phase in ("squeeze", "hold"):
            Igc = np.minimum(I, 910.0)
            over = (1.5 * Igc / law.kd_ma_per_count(self.P, "true") + GC_OVER_COUNTS) * COUNT_RAD
            u_ctrl = np.where(self.thumb, base + np.sign(ts_) * over, u_ctrl)
            goal_ma = np.where(self.thumb, Igc, goal_ma)
        return np.clip(u_ctrl, self.lo, self.hi), goal_ma


# ---------------------------------------------------------------------------------------------------------------
# The adapter to the acceptance loop
# ---------------------------------------------------------------------------------------------------------------
def _rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _minjerk(t, T):
    if t <= 0:
        return 0.0, 0.0
    if t >= T:
        return 1.0, 0.0
    s = t / T
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, (60 * s - 180 * s ** 2 + 120 * s ** 3) / (T * T)


def _quat_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CellPlan:
    """The arm's printed motion from the test lift on (job clock, s), as the field in the hand frame.
    R0: the hand frame's axes in the cell frame at the pick pose (cell = R0 @ hand)."""

    def __init__(self, t_lift0, profile, track_scale, R0):
        from contract import constants as C
        mo = C.MOTION
        self.R0 = np.asarray(R0, float)
        self.scale = float(track_scale)
        self.moves, self.tilts = [], []
        t = float(t_lift0)
        self.moves.append((t, mo["test_lift"]["rise_s"], np.array([0, 0, mo["test_lift"]["rise_mm"] / 1000.0])))
        t += C.TIMELINE["test_lift_s"]
        t_tr0 = t
        for st in C.transfer_steps(profile):
            if st["kind"] == "move":
                self.moves.append((t_tr0 + st["t0_s"], st["dur_s"], np.asarray(st["delta_m"], float)))
            elif st["kind"] == "tilt":
                self.tilts.append((t_tr0 + st["t0_s"], st["dur_s"], math.radians(st["deg"]),
                                   np.asarray(st["axis"], float)))
        t_tr1 = t_tr0 + C.transfer_duration_s(profile)
        sd = mo["set_down"]
        self.moves.append((t_tr1, sd["s"], np.array([0, 0, -sd["drop_mm"] / 1000.0])))
        t_set = t_tr1 + sd["s"]
        self.ramp = mo["tilt_ramp_s"]
        self.marks = {"t_lift0": float(t_lift0), "t_transfer0": t_tr0, "t_transfer1": t_tr1, "t_set": t_set,
                      "t_release": float("inf"), "t_end": t_set + C.TIMELINE["release_window_s"]}

    def at(self, t):
        a = np.zeros(3)
        R = np.eye(3)
        for t0, T, d in self.moves:
            _f, dd = _minjerk(t - t0, T)
            a += self.scale * dd * d
        for t0, T, ang, axis in self.tilts:
            s = min(max(t - t0, 0.0), T)
            tr = self.ramp
            vmax = ang / (T - tr)
            if s < tr:
                th = 0.5 * vmax * s * s / tr
            elif s > T - tr:
                th = ang - 0.5 * vmax * (T - s) ** 2 / tr
            else:
                th = 0.5 * vmax * tr + vmax * (s - tr)
            R = _rot(axis, th) @ R
        return a, R

    def g_hand_at(self, t):
        a, R = self.at(t)
        return (R @ self.R0).T @ (np.array([0.0, 0.0, -9.81]) - a)


class Taught:
    """The taught grasp family on the 13 servos: postures (rad, servo id order) at a list of widths (m), the
    ring's parked posture included in every row; the servos' limits (rad)."""

    def __init__(self, widths_m, rows_rad, lo, hi, open_rad=None):
        self.w = np.asarray(widths_m, float)
        self.x = np.asarray(rows_rad, float)
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        order = np.argsort(self.w)
        self.w, self.x = self.w[order], self.x[order]
        self.open = None if open_rad is None else np.asarray(open_rad, float)

    def q_of_w(self, w):
        w = min(max(w, self.w[0]), self.w[-1])
        return np.array([np.interp(w, self.w, self.x[:, k]) for k in range(self.x.shape[1])])

    @property
    def w_top(self):
        return float(self.w[-1])

    def open_delta(self):
        """The open pre-shape less the family's widest posture (None without a pre-shape)."""
        return None if self.open is None else self.open - self.q_of_w(self.w_top)

    @classmethod
    def from_session(cls, session):
        """The plant's own taught grasp: the family, the open pre-shape and the servos' mounted limits."""
        from contract import names as N
        t = session.taught
        m = session.m
        lo = [float(m.jnt_range[m.joint(s.joint).id][0]) for s in N.SERVOS]
        hi = [float(m.jnt_range[m.joint(s.joint).id][1]) for s in N.SERVOS]
        return cls(t.widths_m, t.q, lo, hi, open_rad=t.open)

    @classmethod
    def load(cls, path=None):
        import json
        import os
        p = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "taught_grasp.json")
        with open(p, "r", encoding="utf-8") as fh:
            d = json.load(fh)
        return cls(d["widths_m"], d["rows_rad"], d["lo_rad"], d["hi_rad"])


# the plant's constants of the current law: the "law" block of plant/truth_constants.json
LAW_KEYS = {"kt": "kt_nm_per_a", "R25": "r25_ohm", "alpha": "copper_per_k", "T_ref": "copper_ref_c", "u_P": "u_p"}


def law_from_constants(law, tr, unit):
    """The true current law at the servos' present winding temperature from a "law" block."""
    missing = [v for v in LAW_KEYS.values() if v not in law]
    if missing:
        raise KeyError("the law block lacks %s (the reference's current law)" % ", ".join(missing))
    servos = sorted(unit["servos"], key=lambda s: s["id"])
    kt = np.array([float(law[LAW_KEYS["kt"]]) * (1.0 + float(s["dev"])) for s in servos])
    Tw = np.asarray(tr["servos"]["winding_c"], float)
    R = float(law[LAW_KEYS["R25"]]) * (1.0 + float(law[LAW_KEYS["alpha"]]) * (Tw - float(law[LAW_KEYS["T_ref"]])))
    I0 = np.array([float(s["i0_ma"]) for s in servos]) * 1e-3
    raw = law.get("loop_offset", "raw") == "raw"
    return Law(kt, R, np.full(len(servos), float(unit["vs"])), I0, float(law[LAW_KEYS["u_P"]]), raw=raw)


_LAW_BLOCK = {}


def plant_law(tr, unit):
    """The true current law from the plant's truth constants."""
    if "law" not in _LAW_BLOCK:
        import importlib.util
        import json
        import os
        spec = importlib.util.find_spec("plant")
        p = os.path.join(list(spec.submodule_search_locations)[0], "truth_constants.json")
        with open(p, "r", encoding="utf-8") as fh:
            _LAW_BLOCK["law"] = json.load(fh)["law"]
    return law_from_constants(_LAW_BLOCK["law"], tr, unit)


class _DutyRecord:
    """The record of the servos' |duty| sampled at every cycle of the script (for the record only): the largest value
    and where, the cycles above DUTY_RECORD_LEVEL per servo, the longest run above it and where it started, and the
    largest value of the firmware's own overload counter (its run at the latch level, in s)."""

    def __init__(self, n):
        self.cycles = 0
        self.max, self.max_at = 0.0, None
        self.above = np.zeros(n, np.int64)
        self.run = np.zeros(n, np.int64)
        self.run_at = [None] * n
        self.longest, self.longest_at = 0, None
        self.latch_max_s = 0.0

    def add(self, d, sat_max, where):
        self.cycles += 1
        k = int(np.argmax(d))
        if float(d[k]) > self.max:
            self.max = float(d[k])
            self.max_at = {"servo": k, "job": where["job"], "phase": where["phase"], "t_ms": where["t_ms"]}
        for i in range(len(d)):
            if d[i] > DUTY_RECORD_LEVEL:
                if self.run[i] == 0:
                    self.run_at[i] = {"servo": i, "job": where["job"], "phase": where["phase"],
                                      "t_ms_start": where["t_ms"]}
                self.run[i] += 1
                self.above[i] += 1
                if self.run[i] > self.longest:
                    self.longest, self.longest_at = int(self.run[i]), dict(self.run_at[i])
            else:
                self.run[i] = 0
        if sat_max is not None:
            self.latch_max_s = max(self.latch_max_s, sat_max)

    def summary(self):
        return {"cycles": self.cycles, "max_abs": round(self.max, 4), "max_at": self.max_at,
                "above_ms_by_servo": [int(c) * PERIOD_MS for c in self.above],
                "longest_above_ms": int(self.longest) * PERIOD_MS, "longest_above_at": self.longest_at,
                "latch_run_max_ms": round(1000.0 * self.latch_max_s, 3)}


class TruthScript:
    """The reference in the acceptance loop: setup() -> setup reply; command(obs, session, view) -> step reply.

    It reads only session.truth() and the read-only SupervisorView (never writes either). The unit's true home
    offsets turn joint angles into Goal Position counts. law_of(truth, unit) gives the true current law of the
    servos at their present winding temperature (the plant's constants); taught gives the family."""

    def __init__(self, scenario, beliefs=None, *, taught=None, law_of=None, tc=None, grip_kw=None, form=None,
                 least_squeeze=None):
        from contract import names as N
        self.grip_kw = dict(grip_kw or {})
        self.N = N
        self.scenario = scenario
        b = dict(beliefs or {})
        form = b.pop("form", None) if form is None else form          # the base's run names its form in its beliefs
        b.pop("form", None)
        self.beliefs = merge_beliefs(b)
        if form is None:
            form = "reference" if self.beliefs == REFERENCE_BELIEFS else "open_loop"
        if form not in FORMS:
            raise ValueError("unknown form %r" % (form,))
        if form == "reference" and self.beliefs != REFERENCE_BELIEFS:
            raise ValueError("a single-fault variant runs on the open-loop form, never with the reference's squeeze")
        self.form = form
        self._ls = least_squeeze
        self._grips = {}
        self._squeeze = {}
        self._heat = {}
        self._duty = None
        self.unit = scenario["unit"]
        self.home = np.array([s["home_true"] for s in sorted(self.unit["servos"], key=lambda s: s["id"])], float)
        self.taught = taught
        self.law_of = law_of or plant_law
        self.tc = tc
        self.thumb = np.array([s.digit == "thumb" for s in N.SERVOS])
        self.pip = N.SERVO_ID["index pip"]
        self._started = False
        self._job = None
        self._grip = None
        self._t0 = None
        self._plan_fixed = False
        self._touch_sum = 0.0
        self._R0 = None

    # ------------------------------------------------------------------------------------------------ setup
    def setup(self):
        return {"period_ms": PERIOD_MS, "reads": [list(r) for r in READS]}

    # ------------------------------------------------------------------------------------------------ helpers
    def _counts(self, q_rad):
        return [int(round(h + q / COUNT_RAD)) for h, q in zip(self.home, q_rad)]

    def _open(self, w_mm=None):
        """The open posture: the taught pre-shape when the cell has one, else the family 16 mm wider."""
        if self.taught.open is not None:
            return np.clip(self.taught.open, self.taught.lo, self.taught.hi)
        w = (self.taught.w[-1] if w_mm is None else float(w_mm) / 1000.0) + OPEN_EXTRA_M
        return np.clip(self.taught.q_of_w(w), self.taught.lo, self.taught.hi)

    def _bias(self, session, tr):
        """The hand's own bias torque at the servo joints (gravity in the arm's field and the velocity terms)."""
        b = tr["servos"].get("bias_nm")
        if b is not None:
            return np.asarray(b, float)
        d, sd = getattr(session, "d", None), getattr(session, "sd", None)
        if d is not None and sd is not None:
            return np.asarray(d.qfrc_bias, float)[np.asarray(sd)].copy()
        return np.zeros(self.N.N_SERVOS)

    def _typical(self, cls):
        from world import wconst
        w = self.tc if self.tc is not None else wconst.load()
        codes = self.N.CODES[cls]
        lo = min(w["carry_kg"][c][0] for c in codes)
        hi = max(w["carry_kg"][c][1] for c in codes)
        return {"mass_kg": 0.5 * (lo + hi)}

    def _touch_sum_n(self, obs):
        """The thumb module's summed Fz read as plain unsigned bytes in 0.1 N steps (the touch-as-force belief)."""
        tac = obs.get("tactile") if isinstance(obs, dict) else None
        if tac and tac.get("modules", {}).get("thumb"):
            b = bytes.fromhex(tac["modules"]["thumb"])
            self._touch_sum = 0.1 * sum(b[6 + 3 * k + 2] for k in range((len(b) - 6) // 3))
        return self._touch_sum

    def _cell_rotation(self, session, tr, spec):
        """R0 (cell = R0 @ hand): from the plant when it offers it, else from the placed specimen's frame (x the
        squeeze axis, z up at the pick) less the placement yaw."""
        f = getattr(session, "cell_rotation_in_hand", None)
        if callable(f):
            return np.asarray(f(), float).T
        fld = getattr(session, "field", None)
        if fld is not None and getattr(fld, "R_wh0", None) is not None:
            return np.asarray(fld.R_wh0, float).copy()
        R_spec = _quat_to_R(tr["specimen"]["quat_wxyz"])
        R_nest = R_spec @ _rot((0, 0, 1), -math.radians(float(spec["yaw_deg"])))
        return R_nest.T

    def _frame(self, tr, obs, spec, bias=None):
        sp = tr["specimen"]
        u = np.asarray(sp["squeeze_axis"], float)
        com = np.asarray(sp["com_m"], float)
        acc = {m: [np.zeros(3), 0.0] for m in ("thumb", "index", "middle")}
        for c in tr["contacts"]:
            if c["module"] in acc:
                fn = float(c["force_n"][0])
                acc[c["module"]][0] += fn * np.asarray(c["pos_m"], float)
                acc[c["module"]][1] += fn
        half = float(spec["w_mm"]) / 2000.0
        face = {"thumb": com - half * u, "index": com + half * u, "middle": com + half * u}
        contacts = {m: (p / f if f > 1e-6 else face[m]) for m, (p, f) in acc.items()}
        pads = sorted(m for m, (_p, f) in acc.items() if f > 1e-6)
        if bias is None:
            bias = np.asarray(tr["servos"].get("bias_nm", np.zeros(self.N.N_SERVOS)), float)
        return {"spring_force_n": float(sp["spring_force_n"]), "touch_sum_n": self._touch_sum_n(obs),
                "contacts": contacts, "com": com, "u": u, "g_hand": np.asarray(tr["field"]["g_hand_mps2"], float),
                "jac": {m: np.asarray(tr["jac"][m], float) for m in ("thumb", "index", "middle")}, "bias": bias,
                "sel": None, "law": self.law_of(tr, self.unit), "pads": pads}

    def least_squeeze(self):
        """The simulator's least squeezes by class or size code: world/data/least_squeeze.json (or the ones given)."""
        if self._ls is None:
            import json
            import os
            p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "least_squeeze.json")
            with open(p, "r", encoding="utf-8") as fh:
                self._ls = {k: float(v) for k, v in json.load(fh)["least_squeeze_n"].items()}
        return self._ls

    def _new_grip(self, view, session, tr):
        spec = view.spec
        cls = spec["class"]
        sp = tr["specimen"]
        self._R0 = self._cell_rotation(session, tr, spec)
        prof, track = view.ticket["profile"], self._job_track(view.job_id)
        R0 = self._R0
        delta = self.taught.open_delta()
        kw = {"correct_until": FORM_GRIP[self.form]["correct_until"]}
        if FORM_GRIP[self.form]["class_squeeze"] and "const_squeeze" not in self.grip_kw:
            kw["ref_ls_n"] = self.least_squeeze()[spec["code"]]
        kw.update(self.grip_kw)
        return Grip(beliefs=self.beliefs, cls=cls, finish=view.ticket["finish"], width_m=float(spec["w_mm"]) / 1000.0,
                    mass_kg=sp["mass_kg"], mu=sp["mu"], gamma_m=sp["gamma_m"], k_n_per_mm=spec["k_n_per_mm"],
                    ceiling=view.ceiling, q_of_w=self.taught.q_of_w, lo=self.taught.lo, hi=self.taught.hi,
                    thumb=self.thumb, pip=self.pip,
                    plan_at_touch=lambda tt: CellPlan(tt + CELL_GRASP_DELAY_S, prof, track, R0),
                    settle_s=CELL_SETTLE_S, follow_spring=True,
                    typical=self._typical(cls) if self.beliefs["object"] == "class_typical" else None,
                    w_top=None if delta is None else self.taught.w_top, open_delta=delta, **kw)

    def _job_track(self, job_id):
        for sh in self.scenario["shifts"]:
            for j in sh:
                if j["id"] == job_id:
                    return float(j["track_scale"])
        raise KeyError(job_id)

    def _declines(self, view):
        """Does the script decline this job? With the true object it declines what the job's label declines (the
        decline segment's masses); a class-typical object is a carry mass, so that belief never declines and grips
        every job as a carry."""
        return view.label == "decline" and self.beliefs["object"] == "true"

    # ------------------------------------------------------------------------------------------- the record
    def _note_servos(self, session, view):
        """The record of the servos at this cycle (nothing is commanded from it): the hottest winding (and Present
        Temperature register) at the first cycle after each shift (the pause, the end), at each shift's last cycle and
        over the unit; the |duty| of every servo sampled at every cycle of the script, over all cycles and over the
        cycles of a job (from its "await_clear" to its "rest"; the hand-over's first move to the open posture is in
        "all" only): its largest value and where, the time above DUTY_RECORD_LEVEL per servo, the longest run above
        it, and the firmware's own overload counter's largest value."""
        fw = getattr(session, "fw", None)
        if fw is None or getattr(fw, "Tw", None) is None:
            return
        tw = float(np.max(np.asarray(fw.Tw, float)))
        reg = getattr(fw, "reg", None)
        pt = None
        if isinstance(reg, dict) and "present_temperature" in reg:
            pt = int(np.max(np.asarray(reg["present_temperature"])))
        h = self._heat
        h["unit_max_winding_c"] = max(h.get("unit_max_winding_c", tw), tw)
        if view is not None:
            row = {"t_ms": int(view.t_ms), "phase": view.phase, "max_winding_c": round(tw, 3), "max_register_c": pt}
            if view.shift is not None:
                h["shift%d_last" % int(view.shift)] = row
            end_key = {"pause": "shift1_end", "end": "shift2_end"}.get(view.phase)
            if end_key is not None and end_key not in h:
                h[end_key] = dict(row)
        duty = getattr(fw, "duty", None)
        if duty is None:
            return
        d = np.abs(np.asarray(duty, float))
        sat = getattr(fw, "sat_time", None)
        sat_max = None if sat is None else float(np.max(np.asarray(sat, float)))
        if self._duty is None:
            self._duty = {"all": _DutyRecord(len(d)), "jobs": _DutyRecord(len(d))}
        where = {"job": None if view is None else view.job_id, "phase": None if view is None else view.phase,
                 "t_ms": None if view is None else int(view.t_ms)}
        self._duty["all"].add(d, sat_max, where)
        if where["job"] is not None:
            self._duty["jobs"].add(d, sat_max, where)

    def _note_squeeze(self, job_id, cell_phase, g, spring_n):
        """Per carry job and cell phase, from the touch to the release request: the true spring force against the
        wanted squeeze (least, largest, last) and the largest commanded squeeze."""
        p = self._squeeze.setdefault(job_id, {}).setdefault(cell_phase, {
            "cycles": 0, "spring_min_n": spring_n, "spring_max_n": spring_n, "wanted_min_n": g.F_des,
            "wanted_max_n": g.F_des, "command_max_n": g.F_cmd})
        p["cycles"] += 1
        p["spring_min_n"], p["spring_max_n"] = min(p["spring_min_n"], spring_n), max(p["spring_max_n"], spring_n)
        p["wanted_min_n"], p["wanted_max_n"] = min(p["wanted_min_n"], g.F_des), max(p["wanted_max_n"], g.F_des)
        p["command_max_n"] = max(p["command_max_n"], g.F_cmd)
        p["spring_last_n"], p["wanted_last_n"] = spring_n, g.F_des

    def report(self):
        """The script's own record: its form, each carry job's squeeze, the heat and the duty it saw."""
        def n4(x):
            return round(float(x), 4) if isinstance(x, float) else x
        squeeze = {}
        for j, g in self._grips.items():
            phases = {ph: {k: n4(v) for k, v in st.items()} for ph, st in self._squeeze.get(j, {}).items()}
            squeeze[j] = {"class_squeeze": g.squeeze_record(), "phases": phases}
        duty = None
        if self._duty is not None:
            duty = {"sampled_every_ms": PERIOD_MS, "level": DUTY_RECORD_LEVEL,
                    "all": self._duty["all"].summary(), "jobs": self._duty["jobs"].summary()}
        return {"form": self.form, "squeeze": squeeze, "heat": dict(self._heat), "duty": duty}

    # ------------------------------------------------------------------------------------------------ a cycle
    def command(self, obs, session, view):
        n = self.N.N_SERVOS
        w = {}
        self._note_servos(session, view)
        if self.taught is None:
            self.taught = Taught.from_session(session)
        if not self._started:
            self._started = True
            w.update({"position_p_gain": [P_GAIN] * n, "position_d_gain": [D_GAIN] * n,
                      "goal_current": [int(GOAL_CURRENT_MA["brittle"])] * n,
                      "goal_position": self._counts(self._open()), "torque_enable": [1] * n})
            return {"write": w, "grasped": False, "decline": False}
        reply = {"write": w, "grasped": False, "decline": False}
        if view.job_id != self._job:
            self._job, self._grip, self._t0, self._plan_fixed = view.job_id, None, None, False
        ph = view.phase
        active = ("present", "test_lift", "transfer", "set_down", "release", "lower")
        if self._declines(view) and ph in active:
            reply["decline"] = ph == "present"
            w["goal_position"] = self._counts(self._open(view.spec["w_mm"]))
            return reply
        if view.label in ("carry", "decline") and ph in active:
            tr = session.truth()
            if tr.get("specimen") is None:
                w["goal_position"] = self._counts(self._open(view.spec["w_mm"]))
                return reply
            if self._grip is None:
                self._grip = self._new_grip(view, session, tr)
                self._t0 = view.phase_t0_ms
                if view.label == "carry":
                    self._grips[view.job_id] = self._grip
            g = self._grip
            if ph in ("release", "lower"):
                g.request_release()
            t = (view.t_ms - self._t0) / 1000.0
            if view.plan_ms and not self._plan_fixed and g.plan is not None and "test_lift" in view.plan_ms:
                t_lift0 = (view.plan_ms["test_lift"][0] - self._t0) / 1000.0
                g.plan = CellPlan(t_lift0, view.ticket["profile"], self._job_track(view.job_id), self._R0)
                g.dem, g.dem_re = None, False
                self._plan_fixed = True
            out = g.step(t, self._frame(tr, obs, view.spec, self._bias(session, tr)))
            if view.label == "carry" and g.phase in ("squeeze", "hold"):
                self._note_squeeze(view.job_id, ph, g, float(tr["specimen"]["spring_force_n"]))
            w["goal_position"] = self._counts(out["goal_rad"])
            w["goal_current"] = [int(round(x)) for x in out["goal_ma"]]
            reply["grasped"] = bool(g.t_touch is not None and t >= g.t_touch + CELL_GRASP_DELAY_S - 1e-9)
            return reply
        w["goal_position"] = self._counts(self._open(view.spec["w_mm"] if view.spec else None))
        return reply
