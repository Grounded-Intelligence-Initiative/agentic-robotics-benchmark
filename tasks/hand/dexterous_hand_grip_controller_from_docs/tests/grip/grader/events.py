"""What ends a shift and what loses a job, read tick by tick from the plant's measurements (CONTRACT.md section 11).

Each monitor takes the specimen block or the servo block of plant measure() (contract/schemas/measure.json) once per
1 ms tick and reports at most one outcome, always as a contract.names.LOSS_KINDS string:

    Crush(klass, ceiling_n, travel_limit_mm)   brittle: spring force above the code's ceiling for 10 ms or more;
                                               soft: spring travel above its limit for 10 ms or more (any time the
                                               specimen exists); "crush_impact" when the run starts within 150 ms
                                               of the job's first pad contact, else "crush_static"
    Transfer(t0, specimen)                     during the transfer: "drop_slip" (more than 20 mm from the pose at
                                               the start of the transfer) or "drop_contact" (no pad touches it for
                                               50 ms); keeps the largest slip (mm) and rotation (deg) in the hand
    Shutdown()                                 a servo whose torque its overheat ("shutdown_overheat") or overload
                                               ("shutdown_overload") protection turned off
    Release(t0, specimen, up)                  the 1.0 s after a release request: 50 ms without pad contact, and the
                                               horizontal motion on the nest (3 mm or more is a push)
    lifted(specimen), rests_declined(specimen), seated_on_place(specimen)
                                               seated: inside the place nest's pocket with the lowest point at most
                                               1.0 mm above its plate (geometric: no contact flag is asked)
"""
import math

from contract import constants as C
from contract import names as N

OVERHEAT = N.HARDWARE_ERROR_BITS["overheating"]
OVERLOAD = N.HARDWARE_ERROR_BITS["overload"]
SC = C.SCORE


def _assert_kind(kind):
    if kind not in N.LOSS_KINDS:
        raise AssertionError("not a loss kind: %r" % (kind,))
    return kind


def rot_deg(q0, q1):
    """The rotation angle (deg) between two unit quaternions [w, x, y, z]."""
    d = abs(sum(a * b for a, b in zip(q0, q1)))
    n0 = math.sqrt(sum(a * a for a in q0))
    n1 = math.sqrt(sum(b * b for b in q1))
    c = min(1.0, d / (n0 * n1)) if n0 > 0 and n1 > 0 else 1.0
    return math.degrees(2.0 * math.acos(c))


def dist_mm(p0, p1):
    return 1000.0 * math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))


def horizontal_mm(p0, p1, up):
    d = [b - a for a, b in zip(p0, p1)]
    n = math.sqrt(sum(u * u for u in up))
    u = [x / n for x in up]
    dot = sum(a * b for a, b in zip(d, u))
    h = [a - dot * b for a, b in zip(d, u)]
    return 1000.0 * math.sqrt(sum(x * x for x in h))


def any_pad(sp):
    return bool(sp) and any(bool(v) for v in sp["pad_contact"].values())


class Crush(object):
    def __init__(self, klass, ceiling_n=None, travel_limit_mm=None):
        self.klass = klass
        self.ceiling_n = ceiling_n
        self.travel_limit_mm = travel_limit_mm
        self.run_start = None

    def over(self, sp):
        if self.klass == "brittle":
            return self.ceiling_n is not None and float(sp["spring_force_n"]) > float(self.ceiling_n)
        if self.klass == "soft":
            return self.travel_limit_mm is not None and \
                1000.0 * float(sp["spring_travel_m"]) > float(self.travel_limit_mm)
        return False

    def update(self, t, sp, first_contact_ms):
        """t: the tick (ms); sp: the specimen block or None. Returns None or (kind, detail)."""
        if sp is None or not self.over(sp):
            self.run_start = None
            return None
        if self.run_start is None:
            self.run_start = int(t)
        if int(t) - self.run_start + 1 >= SC["crush_run_ms"]:
            impact = first_contact_ms is not None and \
                abs(self.run_start - int(first_contact_ms)) <= SC["crush_impact_window_ms"]
            kind = _assert_kind("crush_impact" if impact else "crush_static")
            return kind, {"run_start_ms": self.run_start, "force_n": round(float(sp["spring_force_n"]), 4),
                          "travel_mm": round(1000.0 * float(sp["spring_travel_m"]), 4)}
        return None


class Transfer(object):
    def __init__(self, t0, sp0):
        self.t0 = int(t0)
        self.p0 = list(sp0["pos_m"])
        self.q0 = list(sp0["quat_wxyz"])
        self.slip_mm = 0.0
        self.rot_deg = 0.0
        self.no_contact = 0

    def update(self, t, sp):
        if sp is None:
            return _assert_kind("drop_contact"), {"why": "the specimen is gone"}
        s = dist_mm(self.p0, sp["pos_m"])
        r = rot_deg(self.q0, sp["quat_wxyz"])
        self.slip_mm = max(self.slip_mm, s)
        self.rot_deg = max(self.rot_deg, r)
        if s > SC["drop_slip_mm"]:
            return _assert_kind("drop_slip"), {"slip_mm": round(s, 3)}
        self.no_contact = 0 if any_pad(sp) else self.no_contact + 1
        if self.no_contact >= SC["drop_no_contact_ms"]:
            return _assert_kind("drop_contact"), {"no_contact_ms": self.no_contact}
        return None


class Shutdown(object):
    def __init__(self):
        self.seen = [0] * N.N_SERVOS

    def update(self, servos):
        hw = servos["hardware_error"]
        on = servos["torque_on"]
        for i in range(N.N_SERVOS):
            new = int(hw[i]) & (OVERHEAT | OVERLOAD) & ~self.seen[i]
            if new and not on[i]:
                self.seen[i] |= new
                kind = "shutdown_overheat" if new & OVERHEAT else "shutdown_overload"
                return _assert_kind(kind), {"servo": i, "hardware_error": int(hw[i])}
        return None


class Release(object):
    def __init__(self, t0, sp0, up):
        self.t0 = int(t0)
        self.p0 = list(sp0["pos_m"]) if sp0 else None
        self.up = list(up)
        self.no_contact = 0 if sp0 is None or any_pad(sp0) else 1
        self.free = self.no_contact >= SC["release_no_contact_ms"]
        self.horiz_mm = 0.0

    def update(self, t, sp):
        if int(t) - self.t0 > int(round(1000 * SC["release_window_s"])) or sp is None or self.p0 is None:
            return
        self.no_contact = 0 if any_pad(sp) else self.no_contact + 1
        if self.no_contact >= SC["release_no_contact_ms"]:
            self.free = True
        self.horiz_mm = max(self.horiz_mm, horizontal_mm(self.p0, sp["pos_m"], self.up))

    @property
    def pushed(self):
        return self.horiz_mm >= SC["release_push_mm"]


def lifted(sp):
    """At the end of the test lift: not touching the pick nest and the lowest point 4 mm or more above it."""
    if sp is None:
        return False
    if sp.get("nest_contact") and sp.get("nest") == "pick":
        return False
    low = sp.get("lowest_above_nest_m")
    return low is not None and 1000.0 * float(low) >= SC["lifted_min_above_nest_mm"] - 1e-9


def rests_declined(sp):
    """1.0 s after the release request of a declined job: in the pick nest, no pad touching it."""
    return sp is not None and sp.get("nest") == "pick" and bool(sp.get("nest_contact")) and not any_pad(sp)


def seated_on_place(sp):
    """At the release request of a carried job: inside the place nest's pocket and the lowest point at most 1.0 mm
    above its plate."""
    if sp is None or sp.get("nest") != "place":
        return False
    low = sp.get("lowest_above_nest_m")
    return low is not None and 1000.0 * float(low) <= SC["seated_max_above_nest_mm"] + 1e-9
