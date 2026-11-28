"""The servo firmware and electrical stage of the hand's thirteen bus servos.

One ServoBank holds all servos, vectorised. Per millisecond the session calls, in this order:
  apply(fields)    the host's writes of one step reply, in the contract's order (torque off, operating mode, the RAM
                   fields by ascending address, torque on); returns the refused writes;
  tick(q, k)       the firmware tick: encoder, Present-register estimators, profile, the mode's controller, demand;
  substep(w, dt)   the electrical stage over one physics step: current loop, duty, the reading, heat, protective
                   stops; returns the joint torques.

Current law. In the current-based modes the position loop's output, the Goal Current limit and the Present Current
reading all act on the supply-input current: the loop regulates the reading (standby included), so the motor current
I_m satisfies |d I_m| = |demand| - I0 with duty d = (R I_m + ke w) / Vs. The reading is sign(d) (I0 + |d I_m|),
filtered. Torque = kt (1 + dev) I_m. In the voltage modes (position, extended position, velocity, PWM) the
controller's output is a duty and the motor current follows the voltage balance.
Heat. Two nodes: the winding (heat enters, its copper sets R) and the case (to ambient); the temperature register
reads the winding side in 1 deg C steps. Overheating (register above Temperature Limit) and overload (full output
held too long) set their Hardware Error bits and, as the Shutdown mask says, turn torque off for good (no reboot).
Every hidden number comes from truth_constants.json; the register table and its units from the contract.
"""
import math

import numpy as np

from contract import constants as C
from contract import names as N
from plant import TRUTH

COUNTS_PER_REV = C.SERVO["resolution_counts_per_rev"]
COUNT = 2.0 * math.pi / COUNTS_PER_REV                               # rad per count
VEL_UNIT = C.SERVO["velocity_rpm_per_unit"] * 2.0 * math.pi / 60.0     # rad/s per velocity unit
ACC_UNIT = C.SERVO["profile_acceleration_rev_per_min2_per_unit"] * 2.0 * math.pi / 3600.0   # rad/s^2 per unit
PWM_FULL = C.SERVO["pwm_limit_full"]
DIV = C.SERVO["gain_divisors"]
ERR = N.HARDWARE_ERROR_BITS
TICK_S = 1.0 / C.FIRMWARE["loop_hz"]

# control-table entries of the EEPROM that the record does not carry: the vendor's printed initial values
VENDOR_EEPROM_DEFAULTS = dict(N.EEPROM_DEFAULTS)
CURRENT_MODES = (0, 5)
POSITION_MODES = (3, 4, 5)
VOLTAGE_MODES = (1, 3, 4, 16)

_LAW, _HEAT, _LATCH, _FW = TRUTH["law"], TRUTH["heat"], TRUTH["latch"], TRUTH["firmware"]


def _ram_initial():
    out = {}
    for r in N.REGISTERS:
        if r.area == "RAM":
            out[r.key] = 0 if r.initial is None else int(r.initial)
    return out


class ServoBank:
    """The thirteen servos of one hand unit (unit truth: schemas/unit_truth.json)."""

    def __init__(self, unit, q0=None, *, heat=True, overload=True):
        servos = sorted(unit["servos"], key=lambda s: s["id"])
        n = len(servos)
        if [s["id"] for s in servos] != list(range(n)):
            raise ValueError("servo ids must be 0..n-1")
        self.n = n
        self.heat_on = bool(heat)
        self.overload_on = bool(overload)        # the session always runs with both protections on
        f = np.float64
        self.Vs = np.full(n, float(unit["vs"]), f)
        self.I0 = np.array([float(s["i0_ma"]) for s in servos], f) * 1e-3          # A
        self.home = np.array([int(s["home_true"]) for s in servos], np.int64)
        self.kt = _LAW["kt_nm_per_a"] * (1.0 + np.array([float(s["dev"]) for s in servos], f))
        self.ke = np.full(n, _LAW["ke_v_s_per_rad"], f)
        self.R25 = np.full(n, _LAW["r25_ohm"], f)
        self.alpha = _LAW["copper_per_k"]
        self.Tref = _LAW["copper_ref_c"]
        self.uP, self.uD, self.uI = _LAW["u_p"], _LAW["u_d"], _LAW["u_i"]
        self.cur_tau = _LAW["current_loop_tau_s"]
        self.sense_tau = _LAW["register_filter_s"]
        self.sigma = _LAW["encoder_sigma_counts"]
        # heat member (fixed, the same on every servo)
        h = _HEAT
        self.T_amb, self.R_wc, self.C_w = h["t_amb_c"], h["r_wc_k_per_w"], h["c_w_j_per_k"]
        self.R_ca, self.C_c, self.beta = h["r_ca_k_per_w"], h["c_c_j_per_k"], h["beta"]
        t0 = float(unit["t_start_c"])
        self.Tw = np.full(n, t0, f)
        self.Tc = np.full(n, t0, f)
        self.ol_duty, self.ol_s = _LATCH["overload_duty"], _LATCH["overload_s"]
        # EEPROM (the record's settings, then the vendor's defaults for the rest)
        self.ee = {k: np.full(n, int(v), np.int64) for k, v in C.UNIT_EEPROM.items()}
        for k, v in VENDOR_EEPROM_DEFAULTS.items():
            self.ee[k] = np.full(n, int(v), np.int64)
        # RAM register file (raw integer units)
        self.reg = {k: np.full(n, v, np.int64) for k, v in _ram_initial().items()}
        self.reg["goal_pwm"][:] = self.ee["pwm_limit"]
        self.reg["goal_current"][:] = self.ee["current_limit"]
        q0 = np.zeros(n) if q0 is None else np.asarray(q0, f)
        c0 = self.home + np.round(q0 / COUNT).astype(np.int64)
        self.reg["goal_position"][:] = c0                  # the encoder's reading at power-up
        self.reg["present_position"][:] = c0
        self.reg["position_trajectory"][:] = c0
        self.reg["present_input_voltage"][:] = np.round(self.Vs * 10).astype(np.int64)
        self.reg["present_temperature"][:] = np.round(self.Tw).astype(np.int64)
        self.torque = np.zeros(n, bool)
        self.hw = np.zeros(n, np.int64)
        # estimators and loops
        self.Nr = max(1, int(round(_LAW["velocity_window_s"] / TICK_S)))
        self.hist = np.repeat(c0[:, None], self.Nr + 1, axis=1)
        self.hk = 0
        self.traj = c0.astype(f)
        self.traj_v = np.zeros(n, f)                       # counts/s
        self.integ = np.zeros(n, f)
        self.vinteg = np.zeros(n, f)
        self.prof_type = np.zeros(n, np.int64)             # Moving Status bits 4-5
        self.I_ref = np.zeros(n, f)                        # mA: the current family's demand after the limit
        self.d_ref = np.zeros(n, f)                        # duty demand of the voltage family
        self.I_m = np.zeros(n, f)                          # A
        self.duty = np.zeros(n, f)
        self.I_rep = self.I0 * 1e3                         # mA: the filtered reading (standby at power-up)
        self.reg["present_current"][:] = np.round(self.I_rep).astype(np.int64)
        self.sat_time = np.zeros(n, f)
        self.energy_J = np.zeros(n, f)
        self.int_clip = _FW["integrator_clip_counts"]
        self.in_pos = _FW["in_position_counts"]
        self.follow_err = _FW["following_error_counts"]
        self._masks()

    def _masks(self):
        """Per-servo masks of the operating mode (recomputed when a mode write is applied)."""
        m = self.ee["operating_mode"]
        self.m_pos = np.isin(m, POSITION_MODES)
        self.m_volt = np.isin(m, VOLTAGE_MODES)
        self.m5, self.m0, self.m1, self.m3, self.m34 = m == 5, m == 0, m == 1, m == 3, np.isin(m, (3, 4))

    # ------------------------------------------------------------------------------------------------ host side
    def mode(self):
        return self.ee["operating_mode"]

    def refused_for(self, fields):
        """The writes of one step reply that would be refused if applied now: [[field, servo], ...]."""
        torque_after_off = self.torque.copy()
        te = fields.get("torque_enable")
        if te is not None:
            for i, v in enumerate(te):
                if v == 0:
                    torque_after_off[i] = False
        out = []
        mode = fields.get("operating_mode")
        if mode is not None:
            for i, v in enumerate(mode):
                if v is not None and (torque_after_off[i] or int(v) not in N.OPERATING_MODES):
                    out.append(["operating_mode", i])
        if te is not None:
            for i, v in enumerate(te):
                if v == 1 and self.hw[i] != 0:
                    out.append(["torque_enable", i])
        return out

    def apply(self, fields):
        """Apply one step reply's writes in the contract's order; return the refused writes."""
        n = self.n
        for k, vals in fields.items():
            if k not in N.WRITE_FIELDS:
                raise ValueError("not a write field: %r" % (k,))
            if vals is not None and len(vals) != n:
                raise ValueError("%s: %d entries, not %d" % (k, len(vals), n))
        refused = self.refused_for(fields)
        bad = {(f, i) for f, i in refused}
        te = fields.get("torque_enable")
        if te is not None:                                                    # (1) torque off
            for i, v in enumerate(te):
                if v == 0:
                    self.torque[i] = False
                    self.reg["torque_enable"][i] = 0
        mode = fields.get("operating_mode")
        if mode is not None:                                                  # (2) operating mode
            for i, v in enumerate(mode):
                if v is None or ("operating_mode", i) in bad:
                    continue
                self.ee["operating_mode"][i] = int(v)
                for key, rv in C.MODE_CHANGE_RESET.items():
                    self.reg[key][i] = self.ee[rv][i] if isinstance(rv, str) else int(rv)
                self._restart_trajectory(i)
            self._masks()
        for key in sorted(N.WRITE_FIELDS[2:], key=lambda k: N.REGISTER[k].address):   # (3) RAM fields
            vals = fields.get(key)
            if vals is None:
                continue
            for i, v in enumerate(vals):
                if v is None:
                    continue
                v = int(v)
                if key == "goal_current":
                    lim = int(self.ee["current_limit"][i])
                    v = max(-lim, min(lim, v))
                self.reg[key][i] = v
                if key in ("goal_position", "profile_velocity", "profile_acceleration"):
                    self._set_profile_type(i)
        if te is not None:                                                    # (4) torque on
            for i, v in enumerate(te):
                if v == 1 and ("torque_enable", i) not in bad:
                    if not self.torque[i]:
                        self._restart_trajectory(i)
                        self.integ[i] = 0.0
                        self.vinteg[i] = 0.0
                    self.torque[i] = True
                    self.reg["torque_enable"][i] = 1
        return refused

    def _restart_trajectory(self, i):
        self.traj[i] = float(self.reg["present_position"][i])
        self.traj_v[i] = 0.0

    def _set_profile_type(self, i):
        pv, pa = int(self.reg["profile_velocity"][i]), int(self.reg["profile_acceleration"][i])
        if pv == 0:
            self.prof_type[i] = 0
        elif pa == 0:
            self.prof_type[i] = 1
        else:
            v = pv * VEL_UNIT / COUNT
            a = pa * ACC_UNIT / COUNT
            dist = abs(float(self.reg["goal_position"][i]) - self.traj[i])
            self.prof_type[i] = 2 if dist < v * v / a else 3

    # ------------------------------------------------------------------------------------------------ the tick
    def _profile(self):
        T = TICK_S
        goal = self.reg["goal_position"].astype(np.float64)
        if self.m3.any():
            lo, hi = self.ee["min_position_limit"], self.ee["max_position_limit"]
            goal = np.where(self.m3, np.clip(goal, lo, hi), goal)
        pos_mode = self.m_pos & self.torque
        present = self.reg["present_position"].astype(np.float64)
        if not (self.reg["profile_velocity"] > 0).any():
            self.traj = np.where(pos_mode, goal, present)
            self.traj_v = np.zeros(self.n)
            return goal
        pv = self.reg["profile_velocity"].astype(np.float64) * VEL_UNIT / COUNT
        pa = self.reg["profile_acceleration"].astype(np.float64) * ACC_UNIT / COUNT
        dist = goal - self.traj
        s = np.sign(dist)
        v = self.traj_v
        noacc = pa <= 0
        a_s = np.where(noacc, 1.0, pa)
        vmax = np.where(pv > 0, pv, 1e12)
        stop = v * v / (2.0 * a_s)
        sv = s * v
        brake = (sv >= 0) & (stop >= np.abs(dist))
        vn = np.where(sv < 0, v + s * a_s * T, np.where(brake, v - s * a_s * T, v + s * a_s * T))
        vn = np.where(brake & (s * vn < 0), 0.0, vn)
        vn = np.where(noacc, s * np.minimum(vmax, np.abs(dist) / T), vn)
        vn = np.clip(vn, -vmax, vmax)
        lim = np.abs(dist) / T
        vn = np.where((s * vn > 0) & (np.abs(vn) > lim), s * lim, vn)
        traj = self.traj + vn * T
        done = (np.abs(goal - traj) < 0.5) & (np.abs(vn) <= a_s * T + 1e-9)
        traj = np.where(done, goal, traj)
        vn = np.where(done, 0.0, vn)
        flat = pv <= 0
        traj = np.where(flat, goal, traj)
        vn = np.where(flat, 0.0, vn)
        # a servo whose torque is off, or that is not in a position mode, keeps its trajectory at its position
        self.traj = np.where(pos_mode, traj, present)
        self.traj_v = np.where(pos_mode, vn, 0.0)
        return goal

    def tick(self, q, noise):
        """Firmware tick. q: true joint angles (rad, the servo's sense); noise: 13 standard-normal draws."""
        reg = self.reg
        c = self.home + np.round(np.asarray(q) / COUNT + self.sigma * np.asarray(noise)).astype(np.int64)
        c_prev = self.hist[:, self.hk].copy()
        self.hk = (self.hk + 1) % (self.Nr + 1)
        c_old = self.hist[:, self.hk].copy()                   # the reading Nr ticks ago
        self.hist[:, self.hk] = c
        reg["present_position"][:] = c
        goal = self._profile()
        vd = (c - c_prev) / TICK_S                             # counts/s
        vr = (c - c_old) / (self.Nr * TICK_S)
        reg["present_velocity"][:] = np.round(vr * COUNT / VEL_UNIT).astype(np.int64)
        e = np.round(self.traj) - c
        ve = (self.traj_v - vd) * 1e-3
        self.integ = np.clip(self.integ + e, -self.int_clip, self.int_clip)
        P = reg["position_p_gain"] / DIV["position_p_gain"]
        D = reg["position_d_gain"] / DIV["position_d_gain"]
        I = reg["position_i_gain"] / DIV["position_i_gain"]
        pid = self.uP * P * e + self.uD * D * ve + self.uI * I * self.integ
        on = self.torque
        gc = np.abs(reg["goal_current"]).astype(np.float64)
        cl = self.ee["current_limit"].astype(np.float64)
        # the current family: current-based position (limit = |Goal Current|), current control (Goal Current)
        i5 = np.clip(pid, -gc, gc)
        i0 = np.clip(reg["goal_current"].astype(np.float64), -cl, cl)
        self.I_ref = np.where(on & self.m5, i5, np.where(on & self.m0, i0, 0.0))
        # the voltage family: duty demands, limited by Goal PWM
        gp = np.minimum(np.abs(reg["goal_pwm"]), self.ee["pwm_limit"]).astype(np.float64)
        ev = reg["goal_velocity"] - vd * COUNT / VEL_UNIT
        self.vinteg = np.where(on & self.m1, self.vinteg + ev, 0.0)
        pwm_v = self.uP * (reg["velocity_p_gain"] / DIV["velocity_p_gain"] * ev
                           + reg["velocity_i_gain"] / DIV["velocity_i_gain"] * self.vinteg)
        pwm = np.where(self.m34, pid, np.where(self.m1, pwm_v, reg["goal_pwm"].astype(np.float64)))
        self.d_ref = np.where(on & self.m_volt, np.clip(pwm, -gp, gp) / PWM_FULL, 0.0)
        # registers sampled this tick
        reg["present_current"][:] = np.round(self.I_rep).astype(np.int64)
        reg["present_pwm"][:] = np.round(self.duty * PWM_FULL).astype(np.int64)
        reg["present_temperature"][:] = np.round(self.Tc + self.beta * (self.Tw - self.Tc)).astype(np.int64)
        reg["position_trajectory"][:] = np.round(self.traj).astype(np.int64)
        reg["velocity_trajectory"][:] = np.round(self.traj_v * COUNT / VEL_UNIT).astype(np.int64)
        ongoing = (np.abs(goal - self.traj) > 0) | (self.traj_v != 0)
        ongoing &= self.m_pos & on
        moving = (np.abs(reg["present_velocity"]) > self.ee["moving_threshold"]) | ongoing
        reg["moving"][:] = moving.astype(np.int64)
        inpos = np.abs(goal - c) <= self.in_pos
        follow = np.abs(np.round(self.traj) - c) > self.follow_err
        reg["moving_status"][:] = ((self.prof_type << 4) | (follow.astype(np.int64) << 3)
                                   | (ongoing.astype(np.int64) << 1) | inpos.astype(np.int64))
        hot = reg["present_temperature"] > self.ee["temperature_limit"]
        if np.any(hot):
            self._fault(hot, ERR["overheating"])

    def _fault(self, mask, bit):
        self.hw = np.where(mask, self.hw | bit, self.hw)
        trip = mask & ((self.ee["shutdown"] & bit) != 0)
        self.torque = np.where(trip, False, self.torque)
        self.reg["hardware_error_status"][:] = self.hw
        self.reg["torque_enable"][:] = self.torque.astype(np.int64)

    # ------------------------------------------------------------------------------------------------ electrical
    def R(self):
        return self.R25 * (1.0 + self.alpha * (self.Tw - self.Tref))

    def substep(self, w, dt):
        """w: true joint rates (rad/s); dt: physics step (s). Returns the servo torques (N m)."""
        w = np.asarray(w, np.float64)
        R = self.R()
        Vs, ke = self.Vs, self.ke
        # the current family: |d I_m| = |demand| - I0 (the loop regulates the reading, standby included)
        Iref = self.I_ref * 1e-3
        mag = np.maximum(np.abs(Iref) - self.I0, 0.0)
        s = np.where(Iref >= 0, 1.0, -1.0)
        kw = s * ke * w
        x = (-kw + np.sqrt(np.maximum(kw * kw + 4.0 * R * Vs * mag, 0.0))) / (2.0 * R)
        i_cur = s * np.where(mag > 0, x, 0.0)
        # the voltage family: the duty sets the motor voltage
        i_volt = (self.d_ref * Vs - ke * w) / R
        Istar = np.where(self.m_volt, i_volt, i_cur)
        a = 1.0 - math.exp(-dt / self.cur_tau)
        Im = self.I_m + a * (Istar - self.I_m)
        Im = np.clip(Im, (-Vs - ke * w) / R, (Vs - ke * w) / R)
        Im = np.where(self.torque, Im, 0.0)
        self.I_m = Im
        d = np.where(self.torque, (R * Im + ke * w) / Vs, 0.0)
        self.duty = d
        sgn = np.where(d >= 0, 1.0, -1.0)
        Is = sgn * (self.I0 + np.abs(d * Im)) * 1e3
        b = dt / (self.sense_tau + dt)
        self.I_rep = self.I_rep + b * (Is - self.I_rep)
        Pw = Vs * self.I0 + R * Im * Im + np.abs((ke - self.kt) * w * Im)
        self.energy_J += Pw * dt
        if self.heat_on:
            self.heat_step(Pw, dt)
        sat = (np.abs(d) >= self.ol_duty) & self.torque
        self.sat_time = np.where(sat, self.sat_time + dt, 0.0)
        ol = self.sat_time >= self.ol_s - 1e-12
        if self.overload_on and np.any(ol):
            self._fault(ol, ERR["overload"])
            self.sat_time = np.where(ol, 0.0, self.sat_time)
        return self.kt * Im

    def heat_step(self, P, dt):
        """Advance the two heat nodes by dt (s) with heat input P (W) entering the winding."""
        q = (self.Tw - self.Tc) / self.R_wc
        self.Tw = self.Tw + dt * (P - q) / self.C_w
        self.Tc = self.Tc + dt * (q - (self.Tc - self.T_amb) / self.R_ca) / self.C_c

    def register_temperature(self):
        return np.round(self.Tc + self.beta * (self.Tw - self.Tc)).astype(np.int64)

    # ------------------------------------------------------------------------------------------------ views
    def winding_c(self):
        return self.Tw.copy()

    def case_c(self):
        return self.Tc.copy()

    def input_ma(self):
        """The physical supply-input current (mA), unfiltered."""
        sgn = np.where(self.duty >= 0, 1.0, -1.0)
        return sgn * (self.I0 + np.abs(self.duty * self.I_m)) * 1e3

    def set_temperatures(self, winding_c, case_c):
        self.Tw = np.asarray(winding_c, np.float64).copy()
        self.Tc = np.asarray(case_c, np.float64).copy()
        self.reg["present_temperature"][:] = np.round(self.Tc + self.beta * (self.Tw - self.Tc)).astype(np.int64)
