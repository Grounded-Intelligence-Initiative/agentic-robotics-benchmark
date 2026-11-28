"""hidden_unit: the photographed unit as the grader drives it (grader only; never shipped to the agent).

The unit (tests/gt/unit_state.json): per servo i the count H_i at the true joint zero (0 <= H_i < 4096), the true
direction s_i, the true travel [a_i, b_i] (rad), the packed posture p_i and the index n_i of the count frame the
partner's power-up leaves (n_i = floor((H_i + c s_i p_i) / 4096), c = 4096 / (2 pi)). After that power-up the count of
servo i at true angle q is K_i(q) = H_i + c s_i q - 4096 n_i, and the true angle at a count k is
Q_i(k) = (k - H_i + 4096 n_i) / (c s_i).

    unit = HiddenUnit(truth, unit_state)          # truth: kin.HandModel of the unit (tests/gt/robot.xml)
    q = unit.angles_at_counts(k)                  # Q(k), 13 angles (rad, motor order)
    k = unit.counts_at_angles(q)                  # K(q), real counts
    ex = unit.execute({5: 3901, 6: 3700})         # servo id -> goal count; the others hold the packed posture
    ends = unit.travel_ends_error(calib, ranges)  # per joint (err at the low end, err at the high end), degrees

Execution of a goal count: t_i = Q_i(goal_i); more than HARD_STOP_DEG beyond a true end is a hard stop (the request
scores 0); otherwise the servo stops at e_i = clip(t_i, a_i, b_i). The DIP follows the unit's own four-bar; motion
profiles, dynamics and stiffness are not modelled (a servo reaches its target exactly).
"""
import math

import numpy as np

try:
    from . import contract as C
except ImportError:  # loaded as a plain module beside contract.py
    import contract as C

CPR = C.COUNTS_PER_TURN
CPRAD = CPR / (2.0 * math.pi)                   # counts per radian
D = 2.0 * math.pi / CPR                         # radians per count
HARD_STOP_DEG = 2.0


class Exec(object):
    """One executed command: q_exec (the 13 driven angles of the unit, rad), target (the involved servos' target
    angles, rad), overshoot_deg (per involved servo, how far its target lies beyond the true travel, >= 0) and
    hard_stop (a target more than HARD_STOP_DEG beyond a true end)."""

    __slots__ = ("q_exec", "target", "overshoot_deg", "hard_stop")

    def __init__(self, q_exec, target, overshoot_deg, hard_stop):
        self.q_exec = q_exec
        self.target = target
        self.overshoot_deg = overshoot_deg
        self.hard_stop = hard_stop


class HiddenUnit(object):
    def __init__(self, truth, unit_state, hard_stop_deg=HARD_STOP_DEG):
        if unit_state.get("schema") != "hd04-unit/1" or int(unit_state.get("cpr", 0)) != CPR:
            raise ValueError("unit_state: unknown schema")
        servos = sorted(unit_state["servos"], key=lambda s: int(s["motor_id"]))
        if [int(s["motor_id"]) for s in servos] != list(C.MOTOR_IDS):
            raise ValueError("unit_state: servo ids are not 0 .. 12")
        if [s["joint"] for s in servos] != list(C.MOTOR_ORDER):
            raise ValueError("unit_state: joints are not in motor order")
        self.truth = truth
        self.H = np.array([float(s["H_counts"]) for s in servos])
        self.s = np.array([float(s["s"]) for s in servos])
        self.n = np.array([int(s["n_partner"]) for s in servos], dtype=np.int64)
        self.travel = np.array([[float(x) for x in s["travel_rad"]] for s in servos])
        self.dip_travel = {j: tuple(float(x) for x in unit_state["dip_travel_rad"][j]) for j in C.DIP_JOINTS}
        self.packed = np.array([float(unit_state["packed_posture_rad"][j]) for j in C.MOTOR_ORDER])
        self.hard_stop_rad = math.radians(float(hard_stop_deg))
        if np.any(np.abs(self.s) != 1.0) or np.any(self.travel[:, 0] >= self.travel[:, 1]):
            raise ValueError("unit_state: a direction is not +-1 or a travel is empty")
        self._collider = None

    # ------------------------------------------------------------------------------------------------ the channel
    def angles_at_counts(self, k):
        """Q(k): the unit's true angles (rad, motor order) at 13 counts (the partner's frame)."""
        k = np.asarray(k, dtype=np.float64).reshape(len(C.MOTOR_IDS))
        return (k - self.H + CPR * self.n.astype(np.float64)) / (CPRAD * self.s)

    def counts_at_angles(self, q):
        """K(q): the counts (real, not rounded) at 13 true angles (rad, motor order)."""
        q = np.asarray(q, dtype=np.float64).reshape(len(C.MOTOR_IDS))
        return self.H + CPRAD * self.s * q - CPR * self.n.astype(np.float64)

    def angle_at_count(self, i, k):
        return (float(k) - self.H[i] + CPR * float(self.n[i])) / (CPRAD * self.s[i])

    def packed_posture(self):
        return {j: float(self.packed[i]) for i, j in enumerate(C.MOTOR_ORDER)}

    def execute(self, goal):
        """Execute goal counts {servo id: int} on the unit after its power-up in the packed posture."""
        q = self.packed.copy()
        target, over, hard = {}, {}, False
        for mid in sorted(goal):
            i = int(mid)
            t = self.angle_at_count(i, int(goal[mid]))
            a, b = self.travel[i]
            beyond = max(0.0, a - t, t - b)
            target[C.MOTOR_ORDER[i]] = t
            over[C.MOTOR_ORDER[i]] = math.degrees(beyond)
            if beyond > self.hard_stop_rad:
                hard = True
            q[i] = min(max(t, a), b)
        q_exec = {j: float(q[i]) for i, j in enumerate(C.MOTOR_ORDER)}
        return Exec(q_exec, target, over, hard)

    def solve(self, q_exec):
        """The unit's kinematic state at 13 driven angles (its DIPs on its own four-bar)."""
        return self.truth.solve(q_exec)

    # ------------------------------------------------------------------------------------------------ travel ends
    def driven_end_errors(self, calib, ranges):
        """Per driven joint the signed differences (low end, high end) between the submitted range ends, converted
        to the unit's true angles through the submitted calibration, and the true ends (rad). The two converted ends
        are sorted, so a reversed direction is compared end to end."""
        out = {}
        for i, j in enumerate(C.MOTOR_ORDER):
            lo, hi = ranges[j]
            kap = [(float(calib.sign[i]) * float(x) + float(calib.home[i])) / D for x in (lo, hi)]
            ends = sorted(self.angle_at_count(i, x) for x in kap)
            a, b = self.travel[i]
            out[j] = (ends[0] - a, ends[1] - b)
        return out

    def dip_end_errors(self, ranges, dip_sense):
        """Per DIP the signed differences (low end, high end) between the submitted DIP range read in the true sense
        (sense +1: [lo, hi]; -1: [-hi, -lo]) and the unit's true DIP ends (rad)."""
        out = {}
        for f in C.FINGERS:
            j = C.DIP_OF_FINGER[f]
            lo, hi = ranges[j]
            if int(dip_sense.get(f, 1)) < 0:
                lo, hi = -hi, -lo
            a, b = self.dip_travel[j]
            out[j] = (lo - a, hi - b)
        return out

    def travel_ends_error(self, calib, ranges, dip_sense=None, modulo_turn=False):
        """Per joint (13 driven + 3 DIP) the absolute errors (deg) at its two ends. modulo_turn: the driven ends'
        differences are first taken modulo one turn (360 deg)."""
        out = {}
        for j, (dl, dh) in self.driven_end_errors(calib, ranges).items():
            vals = []
            for x in (dl, dh):
                deg = math.degrees(x)
                if modulo_turn:
                    deg = (deg + 180.0) % 360.0 - 180.0
                vals.append(abs(deg))
            out[j] = tuple(vals)
        for j, (dl, dh) in self.dip_end_errors(ranges, dip_sense or {}).items():
            out[j] = (abs(math.degrees(dl)), abs(math.degrees(dh)))
        return out
