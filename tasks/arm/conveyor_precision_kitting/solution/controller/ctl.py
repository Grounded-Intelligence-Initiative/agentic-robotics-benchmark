"""Controller framework shared by the privileged expert, the reference and the naive baseline.

A controller is a generator script that yields Intents; `Runner.act(obs)` turns the current Intent and the
observation into the 7-number action. A new step index t advances the script once; a repeated t (the grader's
probe) recomputes the action from the observation it carries with the same Intent, without advancing anything.
"""
import math
from collections import deque

import numpy as np

MAX_DPOS = 0.03
MAX_DYAW = 0.15
HOME_YAW = math.pi / 2           # the hand's yaw at the home pose
YAW_LO, YAW_HI = HOME_YAW - math.radians(160), HOME_YAW + math.radians(160)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Intent(object):
    __slots__ = ("goal", "vel", "yaw", "grip", "cams", "turn", "fn", "max_step", "gain")

    def __init__(self, goal=None, vel=None, yaw=None, grip=None, cams=0, turn=False, fn=None, max_step=MAX_DPOS,
                 gain=1.0):
        self.goal = None if goal is None else np.asarray(goal, float)
        self.vel = None if vel is None else np.asarray(vel, float)
        self.yaw = yaw
        self.grip = grip
        self.cams = int(cams)
        self.turn = bool(turn)
        self.fn = fn
        self.max_step = max_step
        self.gain = gain


class Runner(object):
    def __init__(self, script_factory, latency=2, comp=True, naive_gain=0.5):
        self.latency = latency
        # the arm's per-step response, displacement d[k+1] = a d[k] + b u (u = the command executed in that step);
        # nominal values until identify() measures the shift's own (latency, a, b)
        self.arm_a, self.arm_b = 0.05, 0.62
        self.stiffness = 1.0           # the shift's tracking stiffness relative to nominal (estimated from a, b)
        self.drift = np.zeros(3)       # the hand's drift per step under a zero command (gravity compensation error)
        self.comp = comp
        self.naive_gain = naive_gain
        self.script_factory = script_factory
        self.reset()

    def reset(self):
        self.pending = deque([np.zeros(3)] * self.latency)
        self.pending_yaw = deque([0.0] * self.latency)
        self.vel = np.zeros(3)
        self.integ = np.zeros(3)
        self.last_t = None
        self.last_obs = None
        self.prev_ee = None
        self.yaw_unwrapped = None
        self.grip = -1.0
        self.intent = Intent()
        self.last_dp = np.zeros(3)
        self.last_dyaw = 0.0
        self.script = None
        self.last_goal = None
        self.hist = []                 # (t, ee, dp sent for t) for identification

    # the hand's yaw, unwrapped by continuity
    def _yaw(self, obs):
        y = float(obs["ee_yaw"])
        if self.yaw_unwrapped is None:
            return y
        return self.yaw_unwrapped + wrap(y - self.yaw_unwrapped)

    def act(self, obs):
        t = int(obs["t"])
        if t != self.last_t:
            # commit the previous step
            if self.last_t is not None:
                if len(self.hist) < 400:
                    self.hist.append((self.last_t, self.prev_ee.copy(), np.asarray(self.last_dp, float).copy()))
                self.pending.append(self.last_dp)
                self.pending.popleft()
                self.pending_yaw.append(self.last_dyaw)
                self.pending_yaw.popleft()
            ee = np.asarray(obs["ee_pos"], float)
            if self.prev_ee is not None:
                self.vel = ee - self.prev_ee
            self.prev_ee = ee
            self.yaw_unwrapped = self._yaw(obs)
            self.last_t = t
            self.obs = obs
            if self.script is None:
                self.script = self.script_factory(self)
                self.intent = next(self.script)
            else:
                try:
                    self.intent = self.script.send(obs)
                except StopIteration:
                    self.intent = Intent()
            it = self.intent
            if it.goal is not None and it.fn is None and self.comp:
                if it.vel is None and self.last_goal is not None and np.linalg.norm(it.goal - self.last_goal) > 0.004:
                    self.integ[:] = 0.0
                ki = (0.25 if it.vel is not None else 0.12) * (2.0 / max(2, int(self.latency))) ** 2
                err = it.goal - ee
                if it.vel is not None:
                    err[2] = 0.0 if abs(err[2]) > 0.004 else err[2]
                    self.integ = np.clip(self.integ + ki * err, -0.006, 0.006)
                elif np.linalg.norm(err) < 0.006:
                    self.integ = np.clip(self.integ + ki * err, -0.006, 0.006)
                self.last_goal = it.goal.copy()
            elif it.fn is not None:
                self.integ *= 0.6   # a raw command: let the tracker's hold term fade out instead of dropping it
            else:
                self.integ[:] = 0.0
                self.last_goal = None
        return self._action(obs)

    def predicted_ee(self, obs):
        p = np.asarray(obs["ee_pos"], float).copy()
        dv = self.vel.copy()
        for u in self.pending:          # oldest first: the commands still to be executed before ours
            dv = self.arm_a * dv + self.arm_b * np.asarray(u, float) + self.drift
            p = p + dv
        return p

    def identify(self, t0, t1):
        """Fit (latency, a, b) from the recorded steps t0..t1 (an excitation with known commands): for each
        candidate latency L, d[k+1] = a d[k] + b u[k-L] by least squares on all three axes; the best residual wins."""
        H = {t: (ee, dp) for t, ee, dp in self.hist}
        best = None
        for L in (2, 3, 4):
            X, Y = [], []
            for k in range(t0 + L + 1, t1):
                if not all(j in H for j in (k - 1, k, k + 1, k - L)):
                    continue
                d0 = H[k][0] - H[k - 1][0]
                d1 = H[k + 1][0] - H[k][0]
                u = H[k - L][1]
                for ax in range(3):
                    X.append([d0[ax], u[ax]])
                    Y.append(d1[ax])
            if len(Y) < 12:
                continue
            X, Y = np.array(X), np.array(Y)
            coef, *_ = np.linalg.lstsq(X, Y, rcond=None)
            res = float(np.sqrt(np.mean((X @ coef - Y) ** 2)))
            if best is None or res < best[0]:
                best = (res, L, float(coef[0]), float(coef[1]))
        if best is None:
            return None
        res, L, a, b = best
        if not (0.2 < b < 1.2 and -0.5 < a < 0.8):
            return None
        hist_dp = [dp for _t, _ee, dp in self.hist[-L:]]
        while len(hist_dp) < L:
            hist_dp.insert(0, np.zeros(3))
        self.latency, self.arm_a, self.arm_b = L, a, b
        # stiffness relative to nominal, from the fitted response (regression over many shifts of the cell's
        # documented range; rms error about 0.04)
        self.stiffness = float(np.clip(1.064 - 4.686 * a + 0.811 * b, 0.5, 1.5))
        self.pending = deque([np.asarray(u, float) for u in hist_dp])
        self.pending_yaw = deque([0.0] * L)
        return best

    def _action(self, obs):
        it = self.intent
        ee = np.asarray(obs["ee_pos"], float)
        if it.fn is not None:
            dp = np.asarray(it.fn(obs, self), float) + (self.integ if self.comp else 0.0)
            if self.comp:
                dp = dp - self.drift / self.arm_b        # what a zero command would drift (gravity compensation)
        elif it.goal is None:
            dp = np.zeros(3)
        elif self.comp:
            gv = np.zeros(3) if it.vel is None else it.vel
            target = it.goal + gv * (self.latency + 1)
            # a longer command delay leaves less phase margin: lower feedback gain (the model's prediction does the rest)
            # a longer command delay leaves less phase margin: a little less feedback (bench: sub-mm tracking of a
            # belt-speed goal at every latency once settled)
            kl = {2: 1.0, 3: 0.9}.get(int(self.latency), 0.8)
            dp = it.gain * kl * (target - self.predicted_ee(obs)) + self.integ
            if it.vel is not None:
                dp = dp + gv * (1.0 - self.arm_a) / self.arm_b
        else:
            dp = self.naive_gain * (it.goal - ee)
        n = float(np.linalg.norm(dp))
        if n > it.max_step:
            dp = dp * it.max_step / n
        dyaw = 0.0
        if it.yaw is not None:
            cur = self._yaw(obs)
            if self.comp:
                cur = cur + sum(self.pending_yaw)
            dyaw = float(np.clip(it.yaw - cur, -MAX_DYAW, MAX_DYAW)) * (1.0 if self.comp else self.naive_gain * 2)
        if it.grip is not None:
            self.grip = float(it.grip)
        self.last_dp = dp
        self.last_dyaw = dyaw
        return [float(dp[0]), float(dp[1]), float(dp[2]), float(dyaw), float(self.grip), it.cams,
                1.0 if it.turn else 0.0]
