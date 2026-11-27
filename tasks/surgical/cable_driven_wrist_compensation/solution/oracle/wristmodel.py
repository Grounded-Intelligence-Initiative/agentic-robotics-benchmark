"""The reference method's model: a servo lag per motor, and a
Prandtl-Ishlinskii lost-motion model per joint whose play widths grow with |pitch| (capstan
friction). Shared by fit.py (identification from the bench logs) and compensate.py (the
inverse used at run time, episode after episode from the state the one before left). numpy
only."""

import math

import numpy as np

DT = 0.005
# play-width grid (motor radians) for the lost-motion models
RHO = np.array([0.004, 0.008, 0.013, 0.020, 0.029, 0.040, 0.054, 0.072])


def play_series(x, rho, scale=None):
    """Play operators of input series x (n,) for every width in rho (m,), started centred.
    `scale` (n,) multiplies the widths per sample. Returns (n, m)."""
    n = len(x)
    out = np.empty((n, len(rho)))
    y = np.full(len(rho), x[0])
    for k in range(n):
        r = rho if scale is None else rho * scale[k]
        y = np.minimum(np.maximum(y, x[k] - r), x[k] + r)
        out[k] = y
    return out


def play_series_multi(x, rho, scales):
    """Like play_series for several width-scale series at once: scales (c, n) -> (c, n, m)."""
    c, n = scales.shape
    out = np.empty((c, n, len(rho)))
    y = np.repeat(np.full(len(rho), x[0])[None, :], c, axis=0)
    for k in range(n):
        r = rho[None, :] * scales[:, k][:, None]
        y = np.minimum(np.maximum(y, x[:, k][:, None] - r), x[:, k][:, None] + r)
        out[:, k, :] = y
    return out


def pi_inverse_step(y, r, w, target):
    """Input u that makes sum_i w_i clip(y_i, u - r_i, u + r_i) equal `target`, given the
    play states y; returns (u, new states). The map is nondecreasing and piecewise linear
    with breakpoints y_i +- r_i, so the inverse is exact."""
    bps = np.sort(np.concatenate([y - r, y + r]))
    vals = np.array([np.dot(w, np.minimum(np.maximum(y, b - r), b + r)) for b in bps])
    total = float(w.sum())
    if target <= vals[0]:
        u = bps[0] - (vals[0] - target) / total
    elif target >= vals[-1]:
        u = bps[-1] + (target - vals[-1]) / total
    else:
        i = int(np.searchsorted(vals, target))
        i = min(max(i, 1), len(vals) - 1)
        span = vals[i] - vals[i - 1]
        u = bps[i] if span <= 0 else bps[i - 1] + (target - vals[i - 1]) * (bps[i] - bps[i - 1]) / span
    return u, np.minimum(np.maximum(y, u - r), u + r)


class JointModel(object):
    """theta = b + a0 * v + sum_m w_m * play(v; rho_m * exp(mu * |pitch|)),
    v = q + kappa * pitch (kappa = 0, mu = 0 for the pitch joint itself)."""

    def __init__(self, b, a0, w, rho=None, kappa=0.0, mu=0.0):
        self.b, self.a0, self.kappa, self.mu = float(b), float(a0), float(kappa), float(mu)
        self.w = np.asarray(w, dtype=float)
        self.rho = np.asarray(RHO if rho is None else rho, dtype=float)
        # a0 is a play of width 0
        self.W = np.concatenate([[self.a0], self.w])
        self.R0 = np.concatenate([[0.0], self.rho])

    def as_dict(self):
        return dict(b=self.b, a0=self.a0, w=self.w.tolist(), rho=self.rho.tolist(),
                    kappa=self.kappa, mu=self.mu)

    @classmethod
    def from_dict(cls, d):
        return cls(d["b"], d["a0"], d["w"], d["rho"], d.get("kappa", 0.0), d.get("mu", 0.0))

    def end_state(self, v, pitch):
        """Play states (the elastic part first) after the input series v (started settled)
        with the wrist pitch series `pitch`."""
        P = play_series(v, self.rho, np.exp(self.mu * np.abs(pitch)))
        return np.concatenate([[v[-1]], P[-1]])

    def settle(self, y, v, pitch):
        """The play states once the input has moved on to v and come to rest there."""
        r = self.R0 * math.exp(self.mu * abs(pitch))
        return np.minimum(np.maximum(y, v - r), v + r)

    def invert(self, theta, pitch, state=None):
        """Motor angles q (n,) that make the joint follow theta (n,) while the wrist pitch
        follows `pitch` (n,), starting settled, or from `state` (play states, input) that the
        run before left. Returns q and the state the run leaves (the drive holding its last
        command)."""
        n = len(theta)
        q = np.empty(n)
        if state is None:
            v = (theta[0] - self.b) / float(self.W.sum())
            y = np.full(len(self.W), v)
        else:
            # the step to the first command, from where the run before left the cables
            v, y = pi_inverse_step(np.asarray(state[0], dtype=float),
                                   self.R0 * math.exp(self.mu * abs(pitch[0])), self.W,
                                   theta[0] - self.b)
        q[0] = v - self.kappa * pitch[0]
        for k in range(1, n):
            r = self.R0 * math.exp(self.mu * abs(pitch[k]))
            v, y = pi_inverse_step(y, r, self.W, theta[k] - self.b)
            q[k] = v - self.kappa * pitch[k]
        return q, (y, v)


def lag_inverse(q_target, tau):
    """Commands (held for DT) that make a first-order servo with time constant tau land on
    q_target at every period boundary; the first command equals the start position."""
    a = math.exp(-DT / tau)
    c = np.empty_like(q_target)
    c[0] = q_target[0]
    c[1:-1] = (q_target[2:] - a * q_target[1:-1]) / (1.0 - a)
    c[-1] = q_target[-1]
    return c
