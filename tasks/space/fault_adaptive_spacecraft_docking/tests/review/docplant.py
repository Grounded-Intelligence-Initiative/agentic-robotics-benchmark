"""The docking plant written from the agent's page alone (a maintainer's tool; the grader never runs it).

`DocPlant` is the model that the agent's page, environment/setup/payload/given/model.md, states: transcribed section
by section in the page's notation, every formula evaluated as the page writes it (left to right, `hypot` for a length,
the builtin `sum` of the six duties). Nothing in it comes from anywhere but the page: not from tests/env.py, the
grader or the reference (it imports nothing of the task). PAGE_SHA256 is the page it was written from; the repository
test fails as soon as the page changes, until this file has been checked against the new page and the pin moved.

What it proves, stepped beside tests/env.py by doc_vs_env.py (the same draws, the same raw noise numbers, the same
commands, == on every state and observation field after every step): that the page states env.py's model completely
and exactly, so a simulator built from the page alone computes the grader's numbers to the last bit.

The interface leaves open what the page leaves open, the order in which random numbers are drawn:
  * draw_episode(src) is the page's table (What an episode draws): it asks `src` for each draw by name, with the page's
    distribution, and returns the episode's draws under the page's symbols. PageRandom(random.Random(...)) draws them
    with the library's own algorithms; doc_vs_env.py answers them with the numbers env.py drew.
  * DocPlant(draws) is the episode at its start. observe(noise) is the observation at the current step count: it asks
    noise(name) for each unit uniform it needs (the dropout test, the camera's six, the IMU's three: NOISE) and scales
    it to the page's range. step(action) takes one step and says whether the episode has ended; quality() is the
    episode's quality.

Run from any checkout; it has no command line (doc_vs_env.py, and the repository test, import it).
"""

import hashlib
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(os.path.dirname(os.path.dirname(HERE)), "environment", "setup", "payload", "given", "model.md")
PAGE_SHA256 = "797b65ea7ba059a47f26538814b6eb761b1378c922ad570e595067c8bde38c5a"


def page_sha256(path=PAGE):
    """sha256 of the page as it is now (compare PAGE_SHA256, the page this plant was written from)."""
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


# ------------------------------------------------------------------------------------------------ Notation
def wrap(a):
    """wrap(a) = ((a + pi) mod 2 pi) - pi."""
    return ((a + math.pi) % (2 * math.pi)) - math.pi


def clamp(x, lo, hi):
    """x limited to [lo, hi]."""
    return lo if x < lo else (hi if x > hi else x)


def R(a, x, y):
    """R(a)(x, y) = (cos(a) x - sin(a) y, sin(a) x + cos(a) y)."""
    return math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y


def length(x, y):
    """|(x, y)|, which the grader computes as hypot(x, y)."""
    return math.hypot(x, y)


def U(u, a, b):
    """U(a, b) from a unit uniform u in [0, 1)."""
    return a + (b - a) * u


# ------------------------------------------------------------------------------------------------ Fixed constants
DT = 0.12                       # step
STEPS = 400                     # at most 400 steps
BODY = 0.82                     # target body: a disc of radius 0.82 m
PORT_MEAN = 1.14                # docking port: mean radius 1.14 m
REGION = 6.2                    # operating region: radius 6.2 m
DAMP_V, DAMP_W = 0.998, 0.996   # damping, every step: velocity x 0.998, yaw rate x 0.996
BURN = 0.00034                  # fuel: each step burns 0.00034 * (u_0 + ... + u_5)
ARM_BELOW, FIRE_ABOVE = 0.12, 0.78  # relay: armed by a duty below 0.12 (moderate), fired by one above 0.78 (severe)
STRIDE_ABOVE = 1.35             # rotor stride: 2 slots when u_0 + ... + u_5 > 1.35, otherwise 1
ONSET, STRENGTH = 0.65, 0.82    # plume: onset gap 0.65 m, strength 0.82
SWIRL = (0.92, 0.98, 1.04, 1.10, 1.16, 1.20)  # plume: swirl magnitudes
DOCK = (0.085, 0.060, 0.020, 0.015)  # docking: position, velocity, yaw, yaw-rate error bounds
HOLD = 30                       # ... for 30 consecutive steps

# The six nominal nozzle geometries (ax, ay in m/s^2, yaw acceleration in rad/s^2)
NOMINAL = (
    (0.235, 0, 0.132),
    (0.235, 0, -0.132),
    (-0.235, 0, -0.132),
    (-0.235, 0, 0.132),
    (0, 0.205, 0.148),
    (0, -0.205, -0.148),
)
REMOUNT_DELTAS = (-0.79, -0.43, 0.43, 0.79)  # Q = R(delta) diag(1, m)
DROPOUT = 0.035                 # The observation: not fresh with probability 0.035 outside the blackouts

# The observation's unit uniforms, by the name observe() asks for them (each field's noise; the dropout test)
NOISE = ("dropout", "camera x", "camera y", "camera theta", "camera port x", "camera port y", "camera phi", "imu x",
         "imu y", "imu yaw")


# ------------------------------------------------------------------------------------------------ What an episode draws
def draw_episode(src):
    """The page's table, row by row: each draw asked of `src` by name with its distribution (src.U(name, a, b) on
    [a, b), src.integer(name, a, b) an integer a..b, src.sign(name) +1 or -1, src.permutation(name) of 0..5,
    src.other(name, n) uniform among the five nozzles other than n, src.one_of(name, options)), and the dependences the
    table states. Returns the draws under the page's symbols; DocPlant computes everything else."""
    d = {}
    d["phi0"] = src.U("phi0", -math.pi, math.pi)
    d["slew1"] = src.sign("slew1 sign") * src.U("slew1", 0.060, 0.125)          # +-U(0.060, 0.125)
    d["bearing"] = d["phi0"] + src.U("bearing", -1.45, 1.45)                    # p: bearing phi0 + U(-1.45, 1.45),
    d["range"] = src.U("range", 2.85, 3.65)                                     #    range U(2.85, 3.65)
    d["v"] = (src.U("vx", -0.10, 0.10), src.U("vy", -0.10, 0.10))
    d["theta"] = wrap(d["phi0"] + src.U("theta", -1.1, 1.1))
    d["omega"] = src.U("omega", -0.11, 0.11)
    d["G"] = src.permutation("G")
    d["eff"] = [src.U(("eff", n), 0.70, 1.28) for n in range(6)]
    d["inst"] = [src.U(("inst", n), -0.18, 0.18) for n in range(6)]
    d["arm"] = [src.U(("arm", n), 0.82, 1.18) for n in range(6)]
    d["bias"] = (src.U("bias ax", -0.010, 0.010), src.U("bias ay", -0.010, 0.010), src.U("bias yaw", -0.007, 0.007))
    d["K1"] = src.integer("K1", 58, 103)
    d["O1"], d["s1"], d["d1"] = src.permutation("O1"), src.integer("s1", 1, 5), src.sign("d1")
    d["a1"] = src.integer("a1", 0, 5)
    d["b1"] = src.other("b1", d["a1"])
    d["S1"], d["M1"] = src.U("S1", 0.35, 0.50), src.U("M1", 0.55, 0.70)
    d["e1"] = [src.U(("e1", n), 0.65, 1.60) for n in range(6)]
    d["m1"] = [src.U(("m1", i), 0.145 + 0.02 * i, 0.155 + 0.02 * i) for i in range(6)]
    d["E"] = max(d["K1"] + 25, src.integer("E", 108, 125))
    d["D"] = src.integer("D", 148, 165)
    d["rho"] = src.U("rho", 1.18, 1.30)
    O1, O2 = d["O1"], src.permutation("O2")
    if O2 == O1:  # (one that comes out equal to O1 is replaced by O1 rotated by one place)
        O2 = [O1[1], O1[2], O1[3], O1[4], O1[5], O1[0]]
    d["O2"], d["s2"], d["d2"] = O2, src.integer("s2", 1, 5), src.sign("d2")
    d["a2"] = src.integer("a2", 0, 5)
    d["b2"] = src.other("b2", d["a2"])
    d["S2"], d["M2"] = src.U("S2", 0.45, 0.60), src.U("M2", 0.65, 0.80)
    d["e2"] = [src.U(("e2", n), 0.65, 1.60) for n in range(6)]
    d["m2"] = [src.U(("m2", i), 0.235 + 0.025 * i, 0.245 + 0.025 * i) for i in range(6)]
    speed = src.U("slew2", 0.020, 0.045)
    d["slew2"] = -speed if d["slew1"] > 0 else speed                            # its sign opposite to slew1's
    d["alpha"], d["mu"], d["eta"] = src.U("alpha", -math.pi, math.pi), src.sign("mu"), src.sign("eta")
    d["delta"], d["m"] = src.one_of("Q", [(delta, m) for delta in REMOUNT_DELTAS for m in (1, -1)])
    d["eta2"] = src.sign("eta2")
    d["d"] = src.integer("d", 1, 3)
    d["c1"], d["l1"] = src.integer("c1", 48, 81), src.integer("l1", 7, 13)
    d["c2"], d["l2"] = src.integer("c2", 112, 157), src.integer("l2", 10, 18)
    d["P"] = src.sign("P")
    d["A"], d["omega_b"] = src.U("A", 0.10, 0.16), src.U("omega_b", 0.18, 0.28)
    d["psi0"] = src.U("psi0", -math.pi, math.pi)
    d["chi1"], d["chi2"] = src.sign("chi1"), src.sign("chi2")
    return d


class PageRandom(object):
    """The page's draws and noise from a random.Random, with the library's algorithms (uniform, randint, choice,
    shuffle): a rehearsal's episodes, and the page's side of doc_vs_env.py's comparison of the table with env.py."""

    def __init__(self, rng):
        self.rng = rng

    def U(self, name, a, b):
        return self.rng.uniform(a, b)

    def integer(self, name, a, b):
        return self.rng.randint(a, b)

    def sign(self, name):
        return self.rng.choice((1, -1))

    def permutation(self, name):
        values = list(range(6))
        self.rng.shuffle(values)
        return values

    def other(self, name, n):
        return self.rng.choice([k for k in range(6) if k != n])

    def one_of(self, name, options):
        return self.rng.choice(list(options))

    def noise(self, name):
        return self.rng.random()


class Malformed(ValueError):
    """An action the page calls malformed (One step, 1): the episode ends there and scores 0."""


# ------------------------------------------------------------------------------------------------ the plant
class DocPlant(object):
    """One episode of the page's model, from its draws (draw_episode)."""

    def __init__(self, draws):
        d = self.dr = dict(draws)
        # The start (What an episode draws; Fixed constants: the fuel)
        self.k = 0
        self.x = d["range"] * math.cos(d["bearing"])
        self.y = d["range"] * math.sin(d["bearing"])
        self.vx, self.vy = d["v"]
        self.theta, self.omega = d["theta"], d["omega"]
        self.fuel = 1.0
        # The thrusters: nozzle n's far-field column c_n from its nominal geometry G[n]
        self.c = []
        for n in range(6):
            gx, gy, gt = NOMINAL[d["G"][n]]
            eff, inst, arm = d["eff"][n], d["inst"][n], d["arm"][n]
            self.c.append((eff * (math.cos(inst) * gx - math.sin(inst) * gy),
                           eff * (math.sin(inst) * gx + math.cos(inst) * gy),
                           eff * gt * arm))
        self.bias = tuple(d["bias"])
        # The bus: nothing is installed before the first event; the relay is armed at the start
        self.E = d["E"]
        self.L = 30 + ((d["K1"] + d["D"]) % 13)
        self.first = False          # the first event has fired
        self.rekeyed = False        # the re-key has fired, in step kr
        self.kr = None
        self.O = self.h = self.dir = None
        self.a = self.S = self.b = self.M = None
        self.e = self.m = None
        self.z = [0.0] * 6
        self.armed = True
        self.rev1 = self.rev2 = 0   # reversals of the first and of the second rotor
        # The target, the boom and the port
        self.phi, self.slew, self.slew2 = d["phi0"], d["slew1"], d["slew2"]
        self.psi = d["psi0"]
        # The plume near the target: the swirl cells
        self.sw = [d["P"] * (1 if n % 2 == 0 else -1) * SWIRL[n] for n in range(6)]
        self.cell = [0.0] * 6
        self.T = [0.24 + 0.028 * n for n in range(6)]
        # The observation
        delta, m = d["delta"], d["m"]
        self.Q = ((math.cos(delta), -math.sin(delta) * m), (math.sin(delta), math.cos(delta) * m))
        self.B = ((d["c1"], d["c1"] + d["l1"]), (d["c2"], d["c2"] + d["l2"]))
        self.chi = d["chi1"]
        self.acc_body = (0.0, 0.0, 0.0)   # the IMU's next reading (zero for the observation at k = 0)
        self.records = []                 # the camera's records, one per step count
        self.last_fresh = None            # (stamp, fields 0 to 7) of the last fresh sample
        self._record()
        # Docking, the end and the quality
        self.hold = 0
        gx, gy = self.port()
        self.initial_error = length(self.x - gx, self.y - gy)
        self.best_error = self.initial_error
        self.errors = None
        self.end = None   # "docked", "collision", "escape", "empty tank" or "timeout" once the episode has ended

    # ---- The target, the boom and the port
    def r_port(self):
        return PORT_MEAN + self.dr["A"] * math.sin(self.psi)

    def port(self):
        """g = r_port * (cos(phi), sin(phi))."""
        r = self.r_port()
        return r * math.cos(self.phi), r * math.sin(self.phi)

    def port_velocity(self):
        """v_g = A * omega_b * cos(psi) * (cos(phi), sin(phi)) + slew * (-g_y, g_x)."""
        gx, gy = self.port()
        radial = self.dr["A"] * self.dr["omega_b"] * math.cos(self.psi)
        return radial * math.cos(self.phi) + self.slew * -gy, radial * math.sin(self.phi) + self.slew * gx

    # ---- The bus
    def drives(self, i):
        """The physical nozzle slot i drives: O[(i + h) mod 6] from the first event on, nozzle i before."""
        return self.O[(i + self.h) % 6] if self.first else i

    def slot_of(self, n):
        return next(i for i in range(6) if self.drives(i) == n)

    def _install_first(self):
        d = self.dr
        self.first = True
        self.O, self.dir, self.h = list(d["O1"]), d["d1"], (d["d1"] * d["s1"]) % 6
        self.a, self.S, self.b, self.M = d["a1"], d["S1"], d["b1"], d["M1"]
        self.e, self.m = list(d["e1"]), list(d["m1"])
        self.z = [0.0] * 6

    def _install_rekey(self, k):
        d = self.dr
        self.rekeyed, self.kr = True, k
        self.O, self.dir, self.h = list(d["O2"]), d["d2"], (d["d2"] * d["s2"]) % 6
        self.armed = True
        self.a, self.S, self.b, self.M = d["a2"], d["S2"], d["b2"], d["M2"]
        self.e, self.m = list(d["e2"]), list(d["m2"])
        self.z = [0.0] * 6
        self.slew = self.slew2
        self.chi = d["chi2"]

    # ---- The plume near the target
    def near_field(self, n):
        """Nozzle n's near-field column, with its field F and coupling C_n, from the state before the step."""
        cx, cy, ct = self.c[n]
        r = length(self.x, self.y)
        F = clamp((ONSET - (r - BODY)) / (ONSET - (self.r_port() - BODY)), 0.0, 1.0)
        tx = -(math.cos(self.theta) * self.x + math.sin(self.theta) * self.y) / r
        ty = (math.sin(self.theta) * self.x - math.cos(self.theta) * self.y) / r
        al = max(0.0, -(cx * tx + cy * ty) / length(cx, cy))
        C = 0.35 + 0.65 * al ** 4
        loss = STRENGTH * F * C
        sw = self.sw[n]
        return ((1 - loss) * cx - sw * loss * cy, (1 - loss) * cy + sw * loss * cx, (1 - loss) * ct), F, C

    # ---- One step
    def step(self, action):
        """Step k (One step, 1 to 11). True when the episode has ended (Docking, the end and the quality)."""
        # 1. The command
        if not isinstance(action, list) or len(action) != 6:
            raise Malformed("not a list of six entries")
        u = []
        for entry in action:
            try:
                value = float(entry)
            except (TypeError, ValueError, OverflowError):
                raise Malformed("an entry that cannot be read as a number")
            if value != value:
                raise Malformed("an entry that is NaN")
            u.append(clamp(value, 0.0, 1.0))
        if self.fuel <= 0:
            u = [0.0] * 6
        k = self.k
        # 2. The events: the re-key's test (p and g as they stand before the step) and its installation, then the
        #    first event's
        gx, gy = self.port()
        if not self.rekeyed and k >= self.E and (length(self.x - gx, self.y - gy) <= self.dr["rho"]
                                                 or k >= self.dr["D"]):
            self._install_rekey(k)
        if not self.first and k == self.dr["K1"]:
            self._install_first()
        # 3. The relay (with the routing before this step's advance) and the valves, from the first event's step on
        reverse = False
        if self.first:
            i_a, i_b = self.slot_of(self.a), self.slot_of(self.b)
            if u[i_b] < ARM_BELOW:
                self.armed = True
            if u[i_a] > FIRE_ABOVE and self.armed:
                reverse, self.armed = True, False
            self.z = [self.m[i] * self.z[i] + (1 - self.m[i]) * u[i] for i in range(6)]
        else:
            self.z = list(u)
        # 4. The body acceleration: from the bias, slot i's term scale_n * z_i ** e_n * (near-field column of n)
        acc_x, acc_y, acc_yaw = self.bias
        load = [None] * 6   # nozzle n: (F, C_n, z ** e_n of the slot that drives it)
        for i in range(6):
            n = self.drives(i)
            if self.first:
                scale = self.S if n == self.a else (self.M if n == self.b else 1)
                e = self.e[n]
            else:
                scale, e = 1, 1
            realized = self.z[i] ** e
            column, F, C = self.near_field(n)
            term = scale * realized
            acc_x = acc_x + term * column[0]
            acc_y = acc_y + term * column[1]
            acc_yaw = acc_yaw + term * column[2]
            load[n] = (F, C, realized)
        # 5. The swirl cells take this step's loads
        for n in range(6):
            F, C, realized = load[n]
            self.cell[n] = self.cell[n] + 0.12 * (F * C * realized)
            if self.cell[n] >= self.T[n]:
                self.cell[n] = self.cell[n] - self.T[n]
                self.sw[n] = -self.sw[n]
        # 6. The IMU's next reading
        self.acc_body = (acc_x, acc_y, acc_yaw)
        # 7. The motion, rotating with theta as it was before this update
        wx, wy = R(self.theta, acc_x, acc_y)
        self.vx = DAMP_V * (self.vx + DT * wx)
        self.vy = DAMP_V * (self.vy + DT * wy)
        self.x = self.x + DT * self.vx
        self.y = self.y + DT * self.vy
        self.omega = DAMP_W * (self.omega + DT * acc_yaw)
        self.theta = wrap(self.theta + DT * self.omega)
        # 8. The target and the boom
        self.phi = wrap(self.phi + DT * self.slew)
        self.psi = wrap(self.psi + DT * self.dr["omega_b"])
        # 9. The fuel
        self.fuel = max(0.0, self.fuel - BURN * sum(u))
        # 10. The rotor: its reversal, then its advance
        if self.first:
            if reverse:
                self.dir = -self.dir
                if self.rekeyed:
                    self.rev2 += 1
                else:
                    self.rev1 += 1
            stride = 2 if sum(u) > STRIDE_ABOVE else 1
            self.h = (self.h + stride * self.dir) % 6
        # 11. k <- k + 1, the camera's record; the docking test and the end
        self.k = k + 1
        self._record()
        gx, gy = self.port()
        vgx, vgy = self.port_velocity()
        pos = length(self.x - gx, self.y - gy)
        vel = length(self.vx - vgx, self.vy - vgy)
        yaw = abs(wrap(self.theta - self.phi))
        rate = abs(self.omega - self.slew)
        self.errors = (pos, vel, yaw, rate)
        docked = (self.rekeyed and self.rev1 >= 1 and self.rev2 >= 1
                  and pos < DOCK[0] and vel < DOCK[1] and yaw < DOCK[2] and rate < DOCK[3])
        self.hold = self.hold + 1 if docked else 0
        self.best_error = min(self.best_error, pos)
        r = length(self.x, self.y)
        if self.hold >= HOLD:
            self.end = "docked"
        elif r < BODY:
            self.end = "collision"
        elif r > REGION:
            self.end = "escape"
        elif self.fuel <= 0:
            self.end = "empty tank"
        elif self.k >= STEPS:
            self.end = "timeout"
        return self.end is not None

    def quality(self):
        """Docked: 0.94 + 0.04 fuel + 0.02 (1 - k / 400); a collision or an escape: 0; otherwise 0.03 x the progress."""
        if self.end == "docked":
            return 0.94 + 0.04 * self.fuel + 0.02 * (1 - self.k / 400)
        if self.end in ("collision", "escape"):
            return 0.0
        return 0.03 * clamp((self.initial_error - self.best_error) / self.initial_error, 0.0, 1.0)

    # ---- The observation
    def _record(self):
        """The camera records the step count (the stamp) with p, theta, g and phi as they then are."""
        gx, gy = self.port()
        self.records.append((self.k, self.x, self.y, self.theta, gx, gy, self.phi))

    def blackout(self):
        """k in B1 or B2, or, once the re-key has fired in step kr, in [kr, kr + L)."""
        k = self.k
        if any(a <= k < b for a, b in self.B):
            return True
        return self.rekeyed and self.kr <= k < self.kr + self.L

    def observe(self, noise):
        """The sixteen fields of the observation at step count k; noise(name) gives each unit uniform (NOISE)."""
        k = self.k
        if k == 0:
            fresh = True
        elif self.blackout():
            fresh = False
        else:
            fresh = not noise("dropout") < DROPOUT
        if fresh:
            stamp = max(0, k - self.dr["d"])
            rec_k, x, y, theta, gx, gy, phi = self.records[stamp]
            assert rec_k == stamp
            yaw = theta + U(noise("camera theta"), -0.008, 0.008)
            port_yaw = phi + U(noise("camera phi"), -0.006, 0.006)
            fields = [x + U(noise("camera x"), -0.012, 0.012), y + U(noise("camera y"), -0.012, 0.012),
                      math.cos(yaw), math.sin(yaw),
                      gx + U(noise("camera port x"), -0.007, 0.007), gy + U(noise("camera port y"), -0.007, 0.007),
                      math.cos(port_yaw), math.sin(port_yaw)]
            self.last_fresh = (stamp, fields)
        else:
            stamp, fields = self.last_fresh
        bx, by, byaw = self.acc_body
        f8, f9 = R(self.dr["alpha"], bx, self.dr["mu"] * by)
        f8 = f8 + U(noise("imu x"), -0.006, 0.006)
        f9 = f9 + U(noise("imu y"), -0.006, 0.006)
        f10 = self.dr["eta"] * byaw + U(noise("imu yaw"), -0.004, 0.004)
        if self.rekeyed:  # from the observation that follows the re-key's step on: Q and eta2
            Q = self.Q
            f8, f9 = Q[0][0] * f8 + Q[0][1] * f9, Q[1][0] * f8 + Q[1][1] * f9
            f10 = self.dr["eta2"] * f10
        encoder = PORT_MEAN + self.chi * (self.r_port() - PORT_MEAN)
        return list(fields) + [f8, f9, f10, self.fuel, float(k - stamp), 1.0 if fresh else 0.0, k / 400, encoder]
