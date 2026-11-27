"""The hidden plant: a family of cable-driven surgical wrists (instrument model ND-8W) on a
drive unit. Every instrument has its own cable ratios, routing coupling, motor zero offsets,
servo lags and lost motion, drawn within the datasheet's tolerances; all share one geometry
(the dVRK Large Needle Driver's meshes, tests/instrument_model/).

This file is the ground truth the grader simulates, and the model behind every recording the
agent gets (tests/gen/make_payload.py, and the calibration recordings the grader makes of
each hidden instrument). It lives in tests/, which is absent from the sandbox while the agent
works; the agent sees only recordings and the datasheet.

Physics, per 1 ms substep, with the motor command held for one 5 ms command period:

* Motor servo: each drive motor follows its command as a first-order lag with time constant
  tau_i, rate-limited to VMAX rad/s. The encoders read the motor angle.
* Cables: the pitch joint's input is u_p = cp * (q_p - q0_p). Each jaw's cables run over the
  pitch pulleys, so a jaw's input also moves with the ACTUAL pitch angle:
  u_j = c_j * (q_j - q0_j) + k_j * theta_p.
* Lost motion: every joint angle is a Prandtl-Ishlinskii superposition of play (backlash)
  operators of its input, theta = sum_i a_i * play(u; r_i) with sum a_i = 1 (r_0 = 0, the
  elastic part). Cable friction on the pitch pulleys grows with the wrap angle (capstan
  effect), so a jaw's play half-widths scale with exp(mu_j * |theta_p|).

A calibration recording starts settled (the drive dithers before it): all play operators
centred on their input. Nothing settles the wrist after that: the grader runs the requested
episodes back to back from the state the recording left (simulate(start=...)), as the
datasheet says.

There is no jaw contact model. The jaws never meet in a calibration recording: case.py
re-engages an instrument (new motor zero offsets, engagement_offsets) until the stock
firmware's calibration motion keeps them apart, as the datasheet states. The requested
trajectories keep the grip positive, so only commands far off the request could close the
jaws through each other in a scored episode.
"""

import math

import numpy as np

DT = 0.005            # command period, s (200 Hz)
SUB = 5               # physics substeps per command period (1 ms)
VMAX = 30.0           # motor rate limit, rad/s
MOTOR_LIMIT = 8.0     # |motor command| beyond this is clipped by the drive, rad

# --- the datasheet's nominal values and tolerances -------------------------------------- #
# The nominal cable relations and the pitch-to-yaw distance are those of the da Vinci Large
# Needle Driver as the open-source da Vinci Research Kit models it
# (jhu-dvrk/sawIntuitiveResearchKit, share/tool/LARGE_NEEDLE_DRIVER_400006.json, from the dVRK
# user guide): joint = A * actuator with pitch = 1.0186 a_p, yaw = -0.8306 a_p +
# 0.6089 (a_1 + a_2), grip = 1.2177 (a_2 - a_1). Per jaw that is a cable ratio of 1.2177 and a
# coupling of -0.8306 / 1.0186 = -0.8154 per radian of pitch; the DH offset between the pitch
# and yaw axes is 9.1 mm. The jaw length, 9.65 mm from the yaw axis to the jaw's tip, is that of
# the instrument's meshes (dVRK tip_006, tests/instrument_model/): every instrument shares them.
NOM_CP, NOM_CJ, NOM_KJ = 1.0186, 1.2177, -0.8154
NOM_L1, NOM_L2 = 9.1, 9.65
OFFSET_MAX = 0.3      # motor zero offset after an engagement, rad


class Instrument(object):
    """One instrument's hidden parameters."""

    def __init__(self, q0, cp, cj, kj, tau, pi_w, pi_r, mu, l1, l2):
        self.q0 = np.asarray(q0, dtype=float)
        self.cp = float(cp)
        self.cj = np.asarray(cj, dtype=float)
        self.kj = np.asarray(kj, dtype=float)
        self.tau = np.asarray(tau, dtype=float)
        self.pi_w = tuple(np.asarray(w, dtype=float) for w in pi_w)
        self.pi_r = tuple(np.asarray(r, dtype=float) for r in pi_r)
        self.mu = np.asarray(mu, dtype=float)
        self.l1 = float(l1)
        self.l2 = float(l2)

    def as_dict(self):
        return dict(q0=self.q0.tolist(), cp=self.cp, cj=self.cj.tolist(), kj=self.kj.tolist(),
                    tau=self.tau.tolist(), pi_w=[w.tolist() for w in self.pi_w],
                    pi_r=[r.tolist() for r in self.pi_r], mu=self.mu.tolist(), l1=self.l1,
                    l2=self.l2)


def engagement_offsets(rng):
    """Motor zero offsets of one engagement of an instrument on the drive (rad)."""
    return rng.uniform(-OFFSET_MAX, OFFSET_MAX, 3)


def random_instrument(rng):
    """An instrument drawn within the datasheet's tolerances (and the physics' plausible
    range for what the datasheet does not specify). Its motor zero offsets are those of a
    first engagement; case.calibration_recording re-engages it when the stock firmware would
    close its jaws onto each other."""
    def weights():
        w0 = rng.uniform(0.22, 0.45)
        rest = rng.dirichlet([3.0, 3.0]) * (1.0 - w0)
        return np.array([w0, rest[0], rest[1]])

    return Instrument(
        q0=engagement_offsets(rng),
        cp=NOM_CP * (1.0 + rng.uniform(-0.06, 0.06)),
        cj=NOM_CJ * (1.0 + rng.uniform(-0.06, 0.06, 2)),
        kj=NOM_KJ * (1.0 + rng.uniform(-0.15, 0.15, 2)),
        tau=0.020 * (1.0 + rng.uniform(-0.3, 0.3, 3)),
        pi_w=(weights(), weights(), weights()),
        pi_r=(np.array([0.0, rng.uniform(0.007, 0.016), rng.uniform(0.024, 0.044)]),
              np.array([0.0, rng.uniform(0.011, 0.024), rng.uniform(0.034, 0.060)]),
              np.array([0.0, rng.uniform(0.011, 0.024), rng.uniform(0.034, 0.060)])),
        mu=rng.uniform(0.55, 1.15, 2),
        l1=NOM_L1,
        l2=NOM_L2,
    )


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def jaw_tips(theta_p, theta_1, theta_2, l1, l2):
    """Tip positions (mm, instrument frame) of both jaws, vectorised over time.

    The pitch axis is x; the clevis points along +z at zero pitch; the yaw axis is the
    pitched y axis at distance l1; a jaw at angle theta points along (sin theta, 0, cos theta)
    in the pitched frame.
    """
    tp = np.asarray(theta_p, dtype=float)
    out = []
    for tj in (theta_1, theta_2):
        tj = np.asarray(tj, dtype=float)
        x = l2 * np.sin(tj)
        z = l1 + l2 * np.cos(tj)
        # rotate (x, 0, z) about x by theta_p
        out.append(np.stack([x, -np.sin(tp) * z, np.cos(tp) * z], axis=-1))
    return out[0], out[1]


class Wrist(object):
    """An instrument on the drive. `scale_r` and `scale_tau` perturb the play widths and the
    servo lags of one run (run-to-run friction and temperature variation)."""

    def __init__(self, inst, scale_r=(1.0, 1.0, 1.0), scale_tau=(1.0, 1.0, 1.0)):
        self.i = inst
        self.r = [inst.pi_r[k] * float(scale_r[k]) for k in range(3)]
        self.tau = inst.tau * np.asarray(scale_tau, dtype=float)
        self.alpha = 1.0 - np.exp(-(DT / SUB) / self.tau)
        self.q = np.zeros(3)
        self.y = [np.zeros(3) for _ in range(3)]

    def _pitch_angle(self):
        return float(np.dot(self.i.pi_w[0], self.y[0]))

    def settle(self, q):
        """Motors at rest at q, every play operator centred on its input (the dither)."""
        self.q = np.array(q, dtype=float)
        self.y[0][:] = self.i.cp * (self.q[0] - self.i.q0[0])
        tp = self._pitch_angle()
        for j in (0, 1):
            self.y[1 + j][:] = self.i.cj[j] * (self.q[1 + j] - self.i.q0[1 + j]) + self.i.kj[j] * tp

    def state(self):
        return dict(q=self.q.tolist(), y=[v.tolist() for v in self.y])

    def load(self, st):
        self.q = np.array(st["q"], dtype=float)
        self.y = [np.array(v, dtype=float) for v in st["y"]]

    def joints(self):
        return tuple(float(np.dot(self.i.pi_w[k], self.y[k])) for k in range(3))

    def substep(self, cmd):
        dq = (cmd - self.q) * self.alpha
        lim = VMAX * DT / SUB
        self.q = self.q + np.clip(dq, -lim, lim)
        up = self.i.cp * (self.q[0] - self.i.q0[0])
        r = self.r[0]
        self.y[0] = np.minimum(np.maximum(self.y[0], up - r), up + r)
        tp = self._pitch_angle()
        atp = abs(tp)
        for j in (0, 1):
            uj = self.i.cj[j] * (self.q[1 + j] - self.i.q0[1 + j]) + self.i.kj[j] * tp
            rj = self.r[1 + j] * math.exp(self.i.mu[j] * atp)
            self.y[1 + j] = np.minimum(np.maximum(self.y[1 + j], uj - rj), uj + rj)


def simulate(inst, commands, scale_r=(1.0, 1.0, 1.0), scale_tau=(1.0, 1.0, 1.0),
             record_sub=False, start=None, return_state=False):
    """Run one command sequence (N x 3 motor commands, one per 5 ms period) on `inst`.

    The wrist starts settled with its motors at commands[0], or in the state `start` (a
    Wrist.state() from the run before) with the drive stepping to commands[0]. Returns the
    joint angles at the start of every period (N x 3; row k is the state at t_k = k*DT,
    produced by commands 0..k-1) and the motor angles at the same instants. With `record_sub`
    it also returns the 1 ms joint and motor trajectories ((N-1)*SUB + 1 rows); with
    `return_state` last of all the state the run ends in.
    """
    cmds = np.asarray(commands, dtype=float)
    cmds = np.clip(cmds, -MOTOR_LIMIT, MOTOR_LIMIT)
    n = cmds.shape[0]
    w = Wrist(inst, scale_r, scale_tau)
    if start is None:
        w.settle(cmds[0])
    else:
        w.load(start)
    joints = np.empty((n, 3))
    motors = np.empty((n, 3))
    joints[0] = w.joints()
    motors[0] = w.q
    sub_j = [joints[0].copy()] if record_sub else None
    sub_q = [motors[0].copy()] if record_sub else None
    for k in range(n - 1):
        c = cmds[k]
        for _ in range(SUB):
            w.substep(c)
            if record_sub:
                sub_j.append(w.joints())
                sub_q.append(w.q.copy())
        joints[k + 1] = w.joints()
        motors[k + 1] = w.q
    # the drive holds the last command until the next run (half a second at least)
    for _ in range(100 * SUB):
        w.substep(cmds[-1])
    out = (joints, motors)
    if record_sub:
        out = out + (np.array(sub_j), np.array(sub_q))
    if return_state:
        out = out + (w.state(),)
    return out


def nominal_commands(joints):
    """Motor commands the drive's stock firmware sends for a joint trajectory: the
    datasheet's nominal cable ratios, no offsets, no lag or lost-motion compensation."""
    j = np.asarray(joints, dtype=float)
    q = np.empty_like(j)
    q[:, 0] = j[:, 0] / NOM_CP
    q[:, 1] = (j[:, 1] - NOM_KJ * j[:, 0]) / NOM_CJ
    q[:, 2] = (j[:, 2] - NOM_KJ * j[:, 0]) / NOM_CJ
    return q


def pyg_to_joints(pyg):
    """(pitch, yaw, grip) -> (pitch, jaw 1, jaw 2): jaw 1 = yaw + grip/2, jaw 2 = yaw - grip/2."""
    a = np.asarray(pyg, dtype=float)
    return np.stack([a[:, 0], a[:, 1] + 0.5 * a[:, 2], a[:, 1] - 0.5 * a[:, 2]], axis=-1)
