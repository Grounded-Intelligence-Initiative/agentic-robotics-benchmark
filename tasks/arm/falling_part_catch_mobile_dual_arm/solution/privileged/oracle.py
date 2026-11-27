"""Privileged reference controller: reads the cell's ground truth, never the cameras.

Track and belt alike: the right gripper waits open over the part's lane, fingers either side of it, approach along the
surface normal; it closes as the part's centre arrives, lifts, carries the part over the tote, opens, and goes back.
All geometry is world frame; joint targets come from the URDF kinematics in kin.py (base frame = base_link)."""
from __future__ import annotations
import math
import numpy as np
import kin

HZ = 30
MAX_STEP = 0.07            # rad per control step, joint target slew
PICK_SIDE = 0.385      # where the right arm takes parts best: metres to the right of the base (the lane of the tuning shift)
SL = {"base": slice(0, 3), "trunk": slice(3, 7), "arm_left": slice(7, 14), "gl": 14, "arm_right": slice(15, 22), "gr": 22}
PRO = {"arm_left": slice(3, 10), "arm_right": slice(28, 35), "trunk": slice(53, 57)}


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Oracle:
    def __init__(self, s_catch=0.32, lead_s=0.10, clear=0.012, s_min=0.10, settle=3):
        self.arm = kin.Arm("right"); self.s_catch, self.lead, self.clear, self.s_min, self.settle = s_catch, lead_s, clear, s_min, settle; self.t_close = 0
        self.tool = None; self.state = "init"; self.t_state = 0; self.target_q = None; self.log = []
        self.q_pref = np.r_[0.3, -0.6, -0.3, 0.0, 0.0, -0.3, 0.0, -1.2, 0.0, 0.0, 0.0]; self.diag = {}; self.grip = 1.0; self.event = -1; self.fk_err = None; self.reach = 0.045   # fingertip beyond the finger links' origin

    # ---- frames
    def _base(self, tr):
        Rb = rot(tr["base_quat"]); return Rb, np.array(tr["base_pos"])

    def _q(self, proprio):
        return np.r_[proprio[PRO["trunk"]], proprio[PRO["arm_right"]]]

    def _calibrate(self, tr, proprio):
        """The URDF model and the simulator's links differ by a fixed offset (6 cm measured), so the controller servoes
        on the measured gripper pose from truth; the model only supplies the Jacobian. Here: the constant rotation
        between the simulator's eef link and the URDF gripper link."""
        Rb, pb = self._base(tr); q = self._q(proprio)
        p, R, _ = self.arm.fk(q); pw = Rb @ p + pb
        self.fk_err = float(np.linalg.norm(pw - np.array(tr["eef_right_pos"])))
        self.R_off = (Rb @ R).T @ rot(tr["eef_right_quat"])
        self.tool = np.array([0.0, 0.0, -0.125])
        Rg0 = rot(tr["eef_right_quat"]) @ self.R_off.T
        if tr.get("finger_low_right") is not None and Rg0[2, 2] > 0.97:      # gripper pointing straight down at start
            self.reach = float(np.clip(tr["fingers_right"][2] - tr["finger_low_right"], 0.02, 0.12))
        self.log.append({"reach": round(self.reach, 4), "fk_vs_eef_m": round(self.fk_err, 4), "offset_world": (np.array(tr["eef_right_pos"]) - pw).round(4).tolist()})

    def _measured(self, tr):
        """Gripper rotation (URDF gripper-link convention) and tool point (between the fingertips), world frame."""
        Rg = rot(tr["eef_right_quat"]) @ self.R_off.T
        return np.array(tr["fingers_right"]) - Rg[:, 2] * self.reach, Rg

    def _servo(self, tr, q_now, p_world, R_world, gain=0.6):
        """Resolved-rate step toward the pose: measured error, model Jacobian. Returns new joint targets and the error."""
        Rb, _ = self._base(tr); tp, Rm = self._measured(tr)
        flip = R_world @ np.diag([-1.0, -1.0, 1.0])       # the gripper is symmetric: turned half a turn about its approach axis it grasps the same
        if np.trace(flip @ Rm.T) > np.trace(R_world @ Rm.T): R_world = flip
        e_p = Rb.T @ (np.asarray(p_world) - tp); Re = (Rb.T @ R_world) @ (Rb.T @ Rm).T
        e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        J, _, _ = self.arm.jacobian(q_now, self.tool); free = np.ones(11, bool); free[3] = False
        A = np.r_[J[:3], 0.3 * J[3:]][:, free]; err = np.r_[np.clip(e_p, -0.15, 0.15), 0.3 * e_r]
        Ap = A.T @ np.linalg.inv(A @ A.T + 0.04 ** 2 * np.eye(6)); dq = Ap @ err
        # Null-space pull toward a comfortable posture, so the local servo does not creep into joint limits.
        dq = dq + (np.eye(A.shape[1]) - np.linalg.pinv(A, rcond=1e-3) @ A) @ (0.08 * (self.q_pref[free] - q_now[free]))   # exact projector: no leak into the task
        q = q_now.copy(); q[free] = np.clip(q_now[free] + np.clip(gain * dq, -MAX_STEP, MAX_STEP), self.arm.lo[free], self.arm.hi[free])
        return q, float(np.linalg.norm(e_p))

    def _solve(self, tr, q_now, p_world, R_world):
        """Global IK (model frame); used only for large moves. Seeds, in order: postures that solved an earlier pose of this
        shift, two fixed ones, the present posture; if none of them reaches the pose, a fixed set of scattered seeds. A
        local solve from the posture the arm is left in over the tote often ended 0.5 m from the pose and the arm then
        stood there for the rest of the shift (job 62771971, seed 4418: every rod after the first lost)."""
        Rb, pb = self._base(tr); p = Rb.T @ (np.asarray(p_world) - pb); R = Rb.T @ R_world
        free = np.ones(11, bool); free[3] = False; best = None; good = getattr(self, "_good", [])
        fixed = [np.r_[0.3, -0.6, -0.3, 0, 0, 0, 0, -1.2, 0, 0, 0], np.r_[0.6, -1.2, -0.6, 0, 0, 0, 0, -1.0, 0, 0, 0], np.asarray(q_now, float)]
        g = np.random.default_rng(7); scattered = [self.arm.lo + (self.arm.hi - self.arm.lo) * g.uniform(0.15, 0.85, 11) for _ in range(6)]
        import time as _t; _t0 = _t.time()
        for k, seed in enumerate(list(good) + fixed + scattered):
            if k >= len(good) + len(fixed) and best is not None and best[2] < 0.01: break       # scattered seeds only when nothing reached it
            q, err = self.arm.ik(np.clip(seed, self.arm.lo, self.arm.hi), p, R, tool=self.tool, free=free, iters=150)
            margin = float(np.minimum(q - self.arm.lo, self.arm.hi - q)[4:].min())          # stay off the arm's joint limits
            cost = err * 50 + (0.5 if margin < 0.05 else 0.0) + 0.01 * float(np.abs(q - q_now).max())
            if best is None or cost < best[0]: best = (cost, q, err)
            if err < 0.003 and margin >= 0.05 and k < len(good): break                          # a remembered posture does it
        self.solve_s = getattr(self, 'solve_s', 0.0) + _t.time() - _t0; self.solves = getattr(self, 'solves', 0) + 1
        if best[2] < 0.005:
            self._good = ([best[1].copy()] + [x for x in good if float(np.abs(x - best[1]).max()) > 0.2])[:6]
        return best[1], best[2]

    def log_has_event(self):
        return bool(self.log) and self.log[-1].get("event") == self.event

    def _go(self, name):
        if name != self.state: self.state, self.t_state = name, 0

    # ---- one control step
    def act(self, obs, tr):
        proprio = np.asarray(obs["proprio"], float); q_now = self._q(proprio)
        if self.tool is None: self._calibrate(tr, proprio); self.target_q = q_now.copy(); self.home_left = proprio[PRO["arm_left"]].copy()
        self.t_state += 1
        fwd, left, up = np.array(tr["fwd"]), np.array(tr["left"]), np.array(tr["up_chute"])
        low = np.array(tr["low_end"]); belt = tr["station"] == "belt"
        along_axis = fwd if belt else up
        n = np.cross(along_axis, left); n = n / np.linalg.norm(n) * (1 if np.cross(along_axis, left)[2] > 0 else -1)
        Rg = np.stack([np.cross(left, n), left, n], axis=1)          # gripper x, y (opening, across the lane), z (away from surface)
        if tr["phase"] in ("staged", "live") and tr["event"] != self.event:
            self.event = tr["event"]; self._go("wait"); self.grip = 1.0; self.catch_err = None; self.stall = 0; self._gap = 9.0; self.reopen_until = 0
        if self.state in ("init",): self._go("wait")
        want = None
        # "rest": this part has been dealt with (set down, or closed on nothing at the end). Its event may run on for a moment
        # (a part settling in the tote is still live), and following it again sent the hand into the tote, closed on
        # nothing and left the arm away from the feed point when the next part came (job 62707239: every part after the
        # second placed one). The hand goes back to the waiting pose and stays there until the next part is staged.
        if (self.state == "wait" and tr["phase"] not in ("staged", "live")) or self.state == "rest":
            self.grip = 1.0
            want = low + along_axis * self.s_catch + left * getattr(self, "last_lane", 0.0) + n * (self.clear + 0.03 + (0.10 if self.state == "rest" else 0.0))
        if self.state in ("wait", "follow") and tr["phase"] in ("staged", "live"):
            # Tracking grasp: wait at s_max; once the part passes, travel down the lane with it (target = the part's own
            # position, led by the servo lag) so that the fingers close on a part that is at rest relative to them.
            pos = np.array(tr["pos"]); box = tr.get("aabb")
            if box is not None: pos = 0.5 * (np.array(box[0]) + np.array(box[1]))       # geometric centre, not the mesh origin (mug, drill)
            lane = float((pos - low) @ left); self.last_lane = lane
            s = float((pos - low) @ along_axis); v = -float(np.array(tr["vel"]) @ along_axis)
            Rp = rot(tr["quat"]); width = 2.0 * float(np.abs(Rp.T @ left) @ (np.array(tr["size"]) / 2.0)); self.width = width
            # The arm follows a moving target with a steady lag (joint controller, step limits): 2 to 4 cm at these speeds,
            # which is the whole alignment tolerance. An integral term on the measured along-lane error removes it.
            if self.state == "follow":
                lag = float((pos - self._measured(tr)[0]) @ along_axis)                 # negative: the hand trails the part
                self.i_along = float(np.clip(getattr(self, "i_along", 0.0) + 0.12 * lag, -0.08, 0.08))
            else: self.i_along = 0.5 * getattr(self, "i_along", 0.0)
            s_ref = float(np.clip(s - max(v, 0.0) * self.lead + self.i_along, self.s_min, self.s_catch))
            height = 2.0 * float(np.abs(Rp.T @ n) @ (np.array(tr["size"]) / 2.0))        # the part's extent off the surface
            length = 2.0 * float(np.abs(Rp.T @ along_axis) @ (np.array(tr["size"]) / 2.0))
            # Extents from the item's own box and its orientation: the world-aligned bounding box overstated the width
            # of every part (job 62685458), so it is used for the centre only.
            # Which way to hold it: across the lane if it fits between the fingers, else along the lane, else by the rim
            # (open containers), else leave it alone: fingers set down on a part too wide for them pinned it (jam).
            # Parts lie at any yaw (they turn as they slide; on the belt they are set down at random). Hold a part across its
            # own narrow horizontal axis: take the two body axes nearest the surface plane, decide which carries which
            # size by comparing with the measured bounding box, and turn the gripper's opening axis onto the narrow one.
            sz = np.array(tr["size"], float); hor = np.argsort(np.abs(Rp.T @ n))[:2]; u = [Rp[:, k] - n * float(Rp[:, k] @ n) for k in hor]; u = [v_ / (np.linalg.norm(v_) + 1e-9) for v_ in u]
            dims = [float(sz[hor[0]]), float(sz[hor[1]])]
            if box is not None:
                ext = np.array(box[1]) - np.array(box[0]); m_lat = float(np.abs(left) @ ext); m_al = float(np.abs(along_axis) @ ext)
                def miss(d): return abs(abs(u[0] @ left) * d[0] + abs(u[1] @ left) * d[1] - m_lat) + abs(abs(u[0] @ along_axis) * d[0] + abs(u[1] @ along_axis) * d[1] - m_al)
                if miss(dims[::-1]) < miss(dims): dims = dims[::-1]
            k = int(np.argmin(dims)); g_axis = u[k]; width = dims[k]; length = dims[1 - k]
            mode = "rim" if tr["item"] == "024_bowl" else "skip" if width > 0.118 else "narrow"
            if mode == "narrow": Rg = np.stack([np.cross(g_axis, n), g_axis, n], axis=1)
            yaw_off = float(np.degrees(np.arctan2(abs(g_axis @ along_axis), abs(g_axis @ left))))
            if mode == "rim": pos = pos + left * (0.5 * width - 0.004); lane = float((pos - low) @ left); width = 0.01
            low_tip = max(self.clear, height - (0.03 if mode == "rim" else 0.065))           # fingertips as low as the fingers' length allows
            # Hover clear of the part's top while waiting and synchronising (a low gripper stopped tall parts: jam),
            # then come down onto it while travelling with it.
            down = self.state == "follow" and self.t_state >= self.settle and mode != "skip"
            self.tip_h = low_tip if down else height + 0.04
            want = low + along_axis * s_ref + left * lane + n * self.tip_h
            if tr["phase"] == "live" and self.state == "wait" and s - max(v, 0.0) * self.lead <= self.s_catch: self._go("follow")
            if self.state == "follow":
                ts_close = max(1, int(round((0.127 - width) / 0.5 * HZ)))                 # steps the fingers need to reach the part
                tip_now = float((self._measured(tr)[0] - low) @ n)
                off = pos - self._measured(tr)[0]; aligned = abs(float(off @ along_axis)) < 0.03 and abs(float(off @ left)) < 0.025
                _, Rm_ = self._measured(tr)
                self.dbg = {"s": round(s, 3), "al": round(float(off @ along_axis), 3), "lat": round(float(off @ left), 3), "tip": round(tip_now, 3), "low_tip": round(low_tip, 3), "h": round(height, 3),
                            "yaw_err": round(float(np.degrees(np.arccos(min(1.0, abs(float(Rm_[:, 1] @ g_axis)))))), 1) if mode == "narrow" else None, "mode": mode, "down": bool(down), "i": round(self.i_along, 3)}
                d = self.diag.setdefault(self.event, {"item": tr["item"], "follow_steps": 0, "min_along": 9.0, "min_lat": 9.0, "min_tip_above": 9.0, "v": round(v, 2), "s_start": round(s, 2), "mode": mode, "w": round(width, 3), "h": round(height, 3), "yaw": round(yaw_off)})
                d["follow_steps"] += 1; d["min_along"] = round(min(d["min_along"], abs(float(off @ along_axis))), 3); d["min_lat"] = round(min(d["min_lat"], abs(float(off @ left))), 3)
                d["min_tip_above"] = round(min(d["min_tip_above"], tip_now - low_tip), 3); d["s_end"] = round(s, 2)
                # Low enough: at the planned fingertip height, or resting as low as the surface lets the fingers go (they
                # stop a centimetre or two above a conveyor that does not give) while still well below the part's top.
                hist = getattr(self, "_tips", []) if self.t_state > 1 else []; hist = (hist + [tip_now])[-8:]; self._tips = hist
                settled = down and len(hist) == 8 and max(hist) - min(hist) < 0.002 and tip_now <= max(height - 0.012, low_tip + 0.012)
                if self.grip > 0 and self.t_state >= self.settle and self.t_state >= getattr(self, "reopen_until", 0) and (tip_now <= low_tip + 0.012 or settled) and aligned and mode != "skip":
                    self.grip = -1.0; self.t_close = self.t_state
                    self.log.append({"event": self.event, "item": tr["item"], "close_s": round(s, 3), "v": round(v, 3), "lane": round(lane, 3), "width": round(width, 3), "height": round(height, 3),
                                     "ik_err": self.catch_err, "tool_minus_part": (self._measured(tr)[0] - pos).round(3).tolist()})
                # Lift only once the fingers have stopped on the part (the gap no longer shrinks): lifting while they
                # still close let the part slip out (seen 2026-11-25). A gap at its minimum means nothing was caught.
                gap = float(tr.get("finger_gap_right", 0.0)); stalled = self.grip < 0 and abs(gap - getattr(self, "_gap", 9.0)) < 0.0015
                self.stall = getattr(self, "stall", 0) + 1 if stalled else 0; self._gap = gap
                att = tr.get("attached_right")
                # Lift only once the simulator has attached the part to the hand (both fingers on it for 0.3 s): parts lifted
                # on finger pressure alone were lost or thrown on the way to the tote (anchor run of 2026-11-25).
                if self.grip < 0 and att: self.log[-1]["grip_gap"] = round(gap, 3); self.log[-1]["attached"] = True; self._go("lift")
                elif self.grip < 0 and att is None and self.stall >= 5 and gap > 0.0295: self.log[-1]["grip_gap"] = round(gap, 3); self._go("lift")
                elif self.grip < 0 and ((self.stall >= 3 and gap <= 0.0295) or self.t_state >= self.t_close + (24 if att is not None else 16)):
                    # Closed on nothing: open again and keep travelling with the part for another try (parts are slow).
                    self.log[-1]["retries"] = self.log[-1].get("retries", 0) + 1
                    if s_ref > self.s_min + 0.03 and self.log[-1]["retries"] <= 3: self.grip = 1.0; self.reopen_until = self.t_state + 7; self.stall = 0; self._gap = 9.0
                    else: self.log[-1]["grip_gap"] = round(gap, 3); self._go("lift" if gap > 0.0295 else "rest")
                if s_ref <= self.s_min + 1e-6 and self.grip > 0 and not self.log_has_event(): self.log.append({"event": self.event, "item": tr["item"], "close_s": round(s, 3), "v": round(v, 3), "width": round(width, 3), "last_chance": True})
                if s_ref <= self.s_min + 1e-6 and self.grip > 0 and mode != "skip": self.grip = -1.0; self.t_close = self.t_state
        elif self.state == "lift":
            want = np.array(tr["fingers_right"]); want = np.array([want[0], want[1], max(want[2], low[2] + 0.22)])
            Rg = np.eye(3) @ self._yaw_R(tr)
            if self.t_state > 22: self._go("carry")
        elif self.state == "carry":
            tote = np.array(tr["tote"]); want = tote + np.array([0, 0, 0.36]); Rg = self._yaw_R(tr)
            near = float(np.linalg.norm((np.array(tr["fingers_right"]) - want)[:2])) < 0.06
            if (near and self.t_state > 8) or self.t_state > 70: self.grip = 1.0; self._go("drop")
        elif self.state == "drop":
            tote = np.array(tr["tote"]); want = tote + np.array([0, 0, 0.36]); Rg = self._yaw_R(tr)
            if self.t_state > 12: self._go("rest")
        if want is not None:
            slow = self.state in ("lift", "carry")                       # B. gentle with a part in the fingers
            tp, _ = self._measured(tr); far = float(np.linalg.norm(np.asarray(want) - tp))
            # The local servo is the proven controller. When the hand is far away (it got stuck after some events),
            # first return in joint space to the posture in which the waiting pose was last reached, then servo.
            q_ready = getattr(self, "q_ready", None)
            if far > 0.25 and q_ready is not None and self.state in ("wait", "rest"):
                self.target_q = q_now + np.clip(q_ready - q_now, -MAX_STEP, MAX_STEP); err = far
            else:
                self.target_q, err = self._servo(tr, q_now, want, Rg, gain=0.35 if slow else 0.6)
            if self.state == "wait" and err < 0.008: self.q_ready = q_now.copy()
            if self.state in ("wait", "follow"): self.catch_err = round(float(err), 4)
        row = np.zeros(23); row[SL["trunk"]] = self.target_q[:4]; row[SL["arm_right"]] = self.target_q[4:]
        # The robot is mobile, and the right arm takes parts well only in the lane it was tuned on (0.385 m to the right of
        # the base: scores fell from 0.40 to 0 as a shift's lane moved 0.2 m to the left, anchor run of 2026-11-25). So the
        # base shifts sideways until the part's lane is at that offset while it waits, stands still while the hand is on a
        # part, and returns home to reach the tote.
        Rb, pb = self._base(tr)
        if not hasattr(self, "base_home"): self.base_home = pb.copy(); self.base_yaw0 = math.atan2(Rb[1, 0], Rb[0, 0]); self.base_goal = pb[:2].copy()
        if self.state == "wait" and tr.get("phase") in ("staged", "live") and tr.get("pos") is not None:
            side = np.array([-math.sin(self.base_yaw0), math.cos(self.base_yaw0)])                # the base's own left
            shift = float((np.array(tr["pos"])[:2] - self.base_home[:2]) @ side) + PICK_SIDE
            self.base_goal = self.base_home[:2] + side * float(np.clip(shift, -0.25, 0.25))
        elif self.state in ("carry", "drop"): self.base_goal = self.base_home[:2].copy()
        if self.state not in ("follow", "lift"):
            e = Rb[:2, :2].T @ (self.base_goal - pb[:2]); yaw = math.atan2(Rb[1, 0], Rb[0, 0]) - self.base_yaw0
            row[SL["base"]] = [float(np.clip(4.0 * e[0], -0.5, 0.5)), float(np.clip(4.0 * e[1], -0.5, 0.5)), float(np.clip(-3.0 * yaw, -0.5, 0.5))]
        row[SL["arm_left"]] = self.home_left; row[SL["gl"]] = 1.0; row[SL["gr"]] = self.grip
        return row

    def _yaw_R(self, tr):
        """Gripper pointing straight down, opening axis along the base's y."""
        Rb, _ = self._base(tr); return Rb


class DropOracle(Oracle):
    """Overhead drop feeder: the right gripper waits palm-up under the hanging part, fingers open either side of the
    part's narrow horizontal axis, low enough that the part has fallen the required distance before it reaches the
    fingertips. The part lands between the fingers; they close as it arrives; once the simulator has attached it the
    hand carries it over the tote, tips over and opens."""
    G = 9.81

    def __init__(self, depth=0.07, late=0, margin=0.03, give=0.0, **kw):
        super().__init__(**kw); self.depth, self.late, self.margin, self.give = depth, late, margin, give   # give: m/s the hand yields downward as the part lands    # finger depth below the tips; steps to close late; fall margin
        self.q_goal = None; self.goal_key = None; self.t_live = 0; self.catch_z = None

    def _palm(self, g_axis, tilt=0.0, about=None):
        """Gripper rotation with the fingertips pointing up (tilt 0); tilt (rad) swings them about the horizontal axis `about`."""
        z = np.array([0.0, 0.0, -1.0]); y = np.asarray(g_axis, float); y = y - z * float(y @ z); y = y / np.linalg.norm(y)
        R = np.stack([np.cross(y, z), y, z], axis=1)
        if tilt:
            a = np.asarray(about, float); a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = (np.eye(3) + math.sin(tilt) * K + (1 - math.cos(tilt)) * K @ K) @ R
        return R

    def _move(self, tr, q_now, want, Rg, key, gain=0.6):
        """Servo when near; when the pose is far (position or more than a quarter turn), go in joint space to a global solution."""
        tp, Rm = self._measured(tr); far = float(np.linalg.norm(np.asarray(want) - tp))
        flip = Rg @ np.diag([-1.0, -1.0, 1.0]); ang = min(math.acos(float(np.clip((np.trace(R @ Rm.T) - 1) / 2, -1, 1))) for R in (Rg, flip))
        if self.goal_key != key: self.goal_key, self.q_goal = key, None
        ck = (key[0], tuple(np.round(np.asarray(want, float), 2)), tuple(np.round(np.asarray(Rg, float).ravel(), 2)))
        if (far > 0.20 or ang > 1.0) and self.q_goal is None and ck in getattr(self, "_ik_cache", {}): self.q_goal = self._ik_cache[ck].copy()
        if (far > 0.20 or ang > 1.0) and self.q_goal is None:
            # A global solve takes a few tenths of a second, and the cell does not wait: poses that come back (the waiting
            # pose, the pose over the tote) are solved once and remembered.
            best = None
            for R in (Rg, flip):
                q, err = self._solve(tr, q_now, want, R)
                cost = err * 5 + 0.02 * float(np.abs(q - q_now).max())
                if best is None or cost < best[0]: best = (cost, q, err)
            self.q_goal = best[1]
            if not hasattr(self, "_ik_cache"): self._ik_cache = {}
            if best[2] < 0.01: self._ik_cache[ck] = best[1].copy()          # never remember a posture that does not reach the pose
        if self.q_goal is not None and (far > 0.06 or ang > 0.35) and float(np.abs(self.q_goal - q_now).max()) > 0.03:
            # Ramp the commanded posture, not the measured one: a target one small step ahead of where a joint is does
            # not move a joint that carries load (the torso bent over the tote, the elbow at its stop): the arm stood
            # 0.5 m from its pose for the rest of the shift (job 62773923, seed 4418). The command may lead the joints by
            # up to 0.4 rad.
            base = np.asarray(self.target_q, float) if getattr(self, "target_q", None) is not None and float(np.abs(np.asarray(self.target_q) - q_now).max()) < 0.4 else q_now
            return base + np.clip(self.q_goal - base, -MAX_STEP, MAX_STEP), far
        self.q_goal = None if far < 0.06 and ang < 0.35 else self.q_goal
        return self._servo(tr, q_now, want, Rg, gain=gain)

    def act(self, obs, tr):
        proprio = np.asarray(obs["proprio"], float); q_now = self._q(proprio)
        if self.tool is None: self._calibrate(tr, proprio); self.target_q = q_now.copy(); self.home_left = proprio[PRO["arm_left"]].copy()
        self.t_state += 1
        left = np.array(tr["left"]); feeder = np.array(tr["low_end"]); tote = np.array(tr["tote"])
        if tr["phase"] in ("staged", "live") and tr["event"] != self.event:
            self.event = tr["event"]; self._go("under"); self.grip = 1.0; self.t_live = 0; self.plan = None; self.stall = 0; self._gap = 9.0; self.late_run = 0
        if self.state == "init": self._go("under")
        want, Rg, key, gain = None, None, None, 0.6
        if self.state == "under" and tr["phase"] not in ("staged", "live"):
            want = np.array([feeder[0], feeder[1], feeder[2] - 0.55]); Rg = self._palm(getattr(self, "g_last", left)); key = ("idle",)
        if self.state in ("under", "close") and tr["phase"] in ("staged", "live"):
            pos = np.array(tr["pos"]); box = tr.get("aabb"); sz = np.array(tr["size"], float); Rp = rot(tr["quat"])
            if self.plan is None or tr["phase"] == "staged":
                # Decided while the part hangs (and refreshed every step until it is let go: the bounding box read on the
                # first step after a part is staged can be stale, which put the hand 0.145 m off for one item in job
                # 62703591): the catch point and how to hold. The part falls without turning.
                first = self.plan is None
                c = 0.5 * (np.array(box[0]) + np.array(box[1])) if box is not None else pos
                bottom = float(box[0][2]) if box is not None else float(pos[2] - sz[2] / 2)
                up = np.array([0.0, 0.0, 1.0]); hor = np.argsort(np.abs(Rp.T @ up))[:2]; k = int(hor[np.argmin(sz[hor])])
                g = Rp[:, k] - up * float(Rp[:, k] @ up); g = g / (np.linalg.norm(g) + 1e-9); width = float(sz[k])
                tip_z = bottom - float(tr.get("min_fall", 0.25)) - self.margin              # fingertips: first contact comes after the required fall
                self.plan = {"c": c, "g": g, "width": width, "bottom": bottom, "tip_z": tip_z, "skip": width > 0.118}
                self.g_last = g
                rec = {"event": self.event, "item": tr["item"], "width": round(width, 3), "tip_z": round(tip_z, 3), "bottom": round(bottom, 3), "skip": bool(width > 0.118), "c_minus_pos": (c - pos).round(3).tolist()}
                if first: self.log.append(rec)
                else: self.log[-1].update(rec)
            P = self.plan; Rg = self._palm(P["g"]); want = np.array([P["c"][0], P["c"][1], P["tip_z"]]); key = ("under", self.event)
            if tr["phase"] == "live":
                self.t_live += 1
                # Landing on the palm: bottom of the part reaches tip_z - depth. Fingers need ts_close steps to meet it.
                t_land = math.sqrt(2.0 * max(P["bottom"] - (P["tip_z"] - self.depth), 0.01) / self.G) * HZ
                # Close only once the part is coming in between the fingers: its underside will be below the fingertips on
                # the next step. Closing earlier (to have the fingers meet it on landing) narrowed the gap to the part's own
                # width before it arrived, and it landed on the fingertips and slid off (job 62700202: 10 of 16 parts).
                # A part that is in rests on the palm for the fifth of a second the fingers then need.
                bottom_now = float(box[0][2]) if box is not None else float(pos[2] - sz[2] / 2); vz = float(tr["vel"][2])
                self.late_run = getattr(self, "late_run", 0) + 1 if bottom_now + vz / HZ <= P["tip_z"] else 0
                if self.state == "under" and not P["skip"] and self.late_run > self.late:
                    self.grip = -1.0; self.t_close = self.t_state; self._go("close")
                    tp, _ = self._measured(tr); self.log[-1].update(close_at=self.t_live, t_land=round(t_land, 1), tip_err=(tp - want).round(3).tolist(), part_above_tip=round(float(tr["aabb"][0][2] - tp[2]), 3))
            if tr["phase"] == "live" and self.give > 0:
                # Yield with the landing: from two steps before it, the hand goes down at `give` for a fifth of a second,
                # which takes that much off the speed the part meets the palm with and keeps it from bouncing out.
                t_land = math.sqrt(2.0 * max(P["bottom"] - (P["tip_z"] - self.depth), 0.01) / self.G) * HZ
                want = want - np.array([0.0, 0.0, self.give * float(np.clip(self.t_live - (t_land - 2), 0, 6)) / HZ])
            if self.state == "close":
                gap = float(tr.get("finger_gap_right", 0.0)); self.stall = self.stall + 1 if abs(gap - self._gap) < 0.0015 else 0; self._gap = gap
                att = tr.get("attached_right")
                if att: self.log[-1]["attached"] = True; self.log[-1]["grip_gap"] = round(float(tr.get("finger_gap_right", 0.0)), 3); self._go("carry")
                elif att is None and self.stall >= 8 and gap > 0.0295: self.log[-1]["grip_gap"] = round(gap, 3); self._go("carry")
                elif self.t_state > 30 and self.stall >= 8 and gap > 0.0295: self.log[-1]["grip_gap"] = round(gap, 3); self.log[-1]["attached"] = False; self._go("carry")   # fingers stopped on something: carry it on friction
                elif self.t_state > 30: self.log[-1]["grip_gap"] = round(float(tr.get("finger_gap_right", 0.0)), 3); self.grip = 1.0; self._go("rest")
        elif self.state == "carry":
            # Palm-up over the tote, to the tote's near side so that tipping the fingers outward puts the part above it.
            want = tote + np.array([0.0, 0.10, 0.34]); Rg = self._palm(getattr(self, "g_last", left)); key = ("carry", self.event); gain = 0.35
            tp, _ = self._measured(tr)
            if (float(np.linalg.norm((tp - want)[:2])) < 0.05 and self.t_state > 10) or self.t_state > 90: self._go("tip")
        elif self.state == "tip":
            # Swing the fingertips over toward the tote (about the horizontal axis square to the way there), then open.
            away = tote[:2] - np.array(tr["base_pos"])[:2]; away = np.r_[away / (np.linalg.norm(away) + 1e-9), 0.0]; about = np.cross(np.array([0.0, 0.0, 1.0]), away)
            ang = min(2.0, 0.06 * self.t_state)
            want = tote + np.array([0.0, 0.10, 0.34]) + away * 0.10 * (ang / 2.0); Rg = self._palm(getattr(self, "g_last", left), tilt=ang, about=about); key = None; gain = 0.5
            if self.t_state > 40: self.grip = 1.0
            if self.t_state > 58: self._go("rest")
        elif self.state == "rest":                    # this part is done with: back under the feeder, hand open, until the next one is staged
            want = np.array([feeder[0], feeder[1], feeder[2] - 0.55]); Rg = self._palm(getattr(self, "g_last", left)); key = ("idle",)
        if want is not None:
            if key is None: self.target_q, err = self._servo(tr, q_now, want, Rg, gain=gain)
            else: self.target_q, err = self._move(tr, q_now, want, Rg, key, gain=gain)
            self.catch_err = round(float(err), 4)
        row = np.zeros(23); row[SL["trunk"]] = self.target_q[:4]; row[SL["arm_right"]] = self.target_q[4:]
        row[SL["arm_left"]] = self.home_left; row[SL["gl"]] = 1.0; row[SL["gr"]] = self.grip
        return row


class RodOracle(DropOracle):
    """Truth-only rod catcher with bounded Cartesian motion.

    Keep the torso and elbow on a comfortable branch, catch at a fixed safe height,
    turn the rod level over the tote, release above its rim, and withdraw vertically.
    Every reply uses a single small Jacobian solve; there is no global IK search.
    """

    def __init__(self, inset=0.035, below=0.31, grab=0.6, lead=0.0, **kw):
        super().__init__(**kw); self.inset, self.below, self.grab, self.lead_steps = inset, below, grab, lead

    def _side(self, a):
        """Gripper rotation with the fingertips pointing along the horizontal direction a and the opening axis level."""
        a = np.array([a[0], a[1], 0.0]); a = a / np.linalg.norm(a); z = -a; y = np.cross(np.array([0.0, 0.0, 1.0]), a)
        return np.stack([np.cross(y, z), y, z], axis=1), a

    def _calibrate(self, tr, proprio):
        super()._calibrate(tr, proprio)
        self.q_pref = np.r_[0.8, -1.2, -0.4, 0.0, -0.2, -0.6, 0.0, -1.2, 0.0, -0.2, 0.0]
        # Keep the torso out of the folded placement posture. A little yaw lets
        # the right hand reach the leftmost rack slots without saturating its shoulder.
        self.arm.lo[:4] = [0.40, -1.6, -0.75, -0.35]
        self.arm.hi[:4] = [1.00, -0.5, 0.10, 0.35]
        self.arm.lo[4] = -1.8
        self.arm.hi[4] = 0.8
        self.arm.lo[5] = -1.8
        self.arm.hi[7] = -0.12
        # The simulator eef and URDF gripper origins differ. Calibrate the actual
        # fingertip tool vector instead of using the old approximate 0.125 m offset.
        Rb, pb = self._base(tr)
        p, R, _ = self.arm.fk(self._q(proprio))
        tp, _ = self._measured(tr)
        self.tool = R.T @ (Rb.T @ (tp - pb) - p)

    def _move(self, tr, q_now, want, Rg, key, gain=0.6):
        return self._servo(tr, q_now, want, Rg, gain)

    def _servo(self, tr, q_now, p_world, R_world, gain=0.6):
        """Resolved-rate step toward the pose: measured error, model Jacobian. Returns new joint targets and the error."""
        Rb, _ = self._base(tr); tp, Rm = self._measured(tr)
        # The carried rod must clear even the longest hanging neighbour. Bound its
        # top while its conservative horizontal box overlaps the rack footprint.
        # Lower before sweeping sideways; rotation then reduces the rod's height.
        if self.state == "carry" and tr.get("phase") == "live" and tr.get("aabb") is not None:
            box_lo, box_hi = np.asarray(tr["aabb"], float)
            centre = (box_lo + box_hi) * 0.5
            half = (box_hi - box_lo) * 0.5
            feeder = np.asarray(tr["low_end"], float)
            fwd, left = np.asarray(tr["fwd"], float), np.asarray(tr["left"], float)
            near_rack = (abs(float((centre - feeder) @ fwd)) < 0.13 + float(np.abs(fwd) @ half)
                         and abs(float((centre - feeder) @ left)) < 0.28 + float(np.abs(left) @ half))
            if near_rack:
                height_room = float(feeder[2] - 0.52 - box_hi[2])
                p_world = np.asarray(p_world, float).copy()
                p_world[2] = min(p_world[2], tp[2] + (2.0 if height_room < 0 else 1.0) * height_room)
        flip = R_world @ np.diag([-1.0, -1.0, 1.0])       # the gripper is symmetric: turned half a turn about its approach axis it grasps the same
        if np.trace(flip @ Rm.T) > np.trace(R_world @ Rm.T): R_world = flip
        e_p = Rb.T @ (np.asarray(p_world) - tp); Re = (Rb.T @ R_world) @ (Rb.T @ Rm).T
        e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        J, _, _ = self.arm.jacobian(q_now, self.tool); free = np.ones(11, bool)
        A = np.r_[J[:3], 0.3 * J[3:]][:, free]; err = np.r_[np.clip(e_p, -0.15, 0.15), 0.3 * e_r]
        Ap = A.T @ np.linalg.inv(A @ A.T + 0.04 ** 2 * np.eye(6)); dq = Ap @ err
        # Null-space pull toward a comfortable posture, so the local servo does not creep into joint limits.
        dq = dq + (np.eye(A.shape[1]) - np.linalg.pinv(A, rcond=1e-3) @ A) @ (0.08 * (self.q_pref[free] - q_now[free]))   # exact projector: no leak into the task
        q = q_now.copy(); q[free] = np.clip(q_now[free] + np.clip(gain * dq, -MAX_STEP, MAX_STEP), self.arm.lo[free], self.arm.hi[free])
        return q, float(np.linalg.norm(e_p))

    def act(self, obs, tr):
        # Repeated observations must not advance timers or the commanded motion.
        if getattr(self, "_last_step", None) == tr["step"]:
            return self._last_row.copy()
        proprio = np.asarray(obs["proprio"], float); q_now = self._q(proprio)
        if self.tool is None: self._calibrate(tr, proprio); self.target_q = q_now.copy(); self.home_left = proprio[PRO["arm_left"]].copy()
        self.t_state += 1
        Rb, pb = self._base(tr); feeder = np.array(tr["low_end"]); tote = np.array(tr["tote"])
        Rg, a = self._side(Rb[:, 0])                                   # reach forward: fingertips away from the robot
        if tr["phase"] in ("staged", "live") and tr["event"] != self.event:
            self.event = tr["event"]; self._go("under"); self.grip = 1.0; self.plan = None; self.stall = 0; self._gap = 9.0
        if self.state == "init": self._go("rest")
        if self.state in ("under", "close") and tr["phase"] not in ("staged", "live"): self._go("rest")     # the part's event ended under the hand
        want, key, gain = None, None, 0.6
        if self.state == "rest" or (self.state == "under" and tr["phase"] not in ("staged", "live")):
            self.grip = 1.0; want = np.array([feeder[0], feeder[1], feeder[2] - 0.70]) + a * self.inset; key = ("idle",)
        elif self.state in ("under", "close"):
            pos = np.array(tr["pos"]); box = tr.get("aabb"); L = float(tr["size"][2]); r = float(tr["size"][0]) / 2
            if self.plan is None or tr["phase"] == "staged":
                first = self.plan is None; c = 0.5 * (np.array(box[0]) + np.array(box[1])) if box is not None else pos
                below = self.below        # the fingers wait this far below the hanging rod's middle; its lower end passes between them untouched
                self.plan = {"c": c, "z": float(c[2] - below), "fall_c": below + (self.grab - 0.5) * L, "L": L, "r": r}
                rec = {"event": self.event, "item": tr["item"], "r": round(r, 4), "L": round(L, 3), "grip_z": round(self.plan["z"], 3)}
                if first: self.log.append(rec)
                else: self.log[-1].update(rec)
            P = self.plan; want = np.array([P["c"][0], P["c"][1], P["z"]]) + a * self.inset; key = ("under", self.event)
            gap = float(tr.get("finger_gap_right", 0.127))
            if tr["phase"] == "live" and self.state == "under":
                # Close so that the fingers arrive on the sleeve when the point `grab` of the way up the rod is level with them.
                fallen = float(P["c"][2] - (0.5 * (box[0][2] + box[1][2]) if box is not None else pos[2])); v = max(0.0, -float(tr["vel"][2]))
                d = P["fall_c"] - fallen; t_rem = ((-v + math.sqrt(max(v * v + 2 * self.G * d, 0.0))) / self.G * HZ) if d > 0 else 0.0
                ts_close = max(0.0, gap - (2 * P["r"] + 0.02)) / (0.5 / HZ)
                if t_rem <= ts_close + self.lead_steps:
                    self.grip = -1.0; self._go("close"); tp, _ = self._measured(tr)
                    self.log[-1].update(fallen_at_cmd=round(fallen, 3), t_rem=round(t_rem, 1), ts_close=round(ts_close, 1), tip_err=(tp - want).round(3).tolist())
            elif self.state == "close":
                self.stall = self.stall + 1 if abs(gap - self._gap) < 0.0015 else 0; self._gap = gap; att = tr.get("attached_right")
                if self.t_state == 1 or "fallen_at_stop" not in self.log[-1]:
                    if self.stall >= 2 or att: self.log[-1]["fallen_at_stop"] = round(float(P["c"][2] - pos[2]), 3); self.log[-1]["grip_gap"] = round(gap, 3)
                if att and self.t_state > 6: self.log[-1]["attached"] = True; self._go("carry")
                elif self.stall >= 10 and gap > 0.0295 and self.t_state > 12: self.log[-1]["attached"] = bool(att); self._go("carry")
                elif self.t_state > 14 and gap <= 0.0295: self._go("rest")
                elif self.t_state > 40: self._go("rest")
        elif self.state in ("carry", "lower", "drop", "back"):
            # Lay the rod down in the tote rather than stand it there: the hand turns fingertips-down over the tote, which
            # brings the rod level, along the tote's diagonal (the longest rod is as long as the tote's long side); it comes
            # down until the rod is above the tote's rim, opens, and leaves straight up. A rod set down upright
            # toppled over the tote's rim or was swept out by the leaving hand (jobs 62711211 and 62711231).
            (ta, tb, _h) = (0.42, 0.32, 0.16); d = np.array([ta, tb, 0.0]) / math.hypot(ta, tb); up = np.array([0.0, 0.0, 1.0])
            y = np.cross(up, d); Rg = np.stack([np.cross(y, up), y, up], axis=1); gain = 0.35
            tp, Rm = self._measured(tr); live = tr["phase"] == "live" and tr.get("aabb") is not None
            c = 0.5 * (np.array(tr["aabb"][0]) + np.array(tr["aabb"][1])) if live else tp
            r = float(tr["size"][0]) / 2 if live else 0.02
            if self.state == "carry":
                want = tote + np.array([0.0, 0.0, 0.37]); key = ("carry", self.event); level = abs(float(Rm[2, 2])) > 0.96
                if (float(np.linalg.norm((tp - want)[:2])) < 0.05 and level and self.t_state > 12) or self.t_state > 130: self._go("lower")
            elif self.state == "lower":
                goal_c = tote + np.array([0.0, 0.0, r + 0.19]); want = tp + np.clip(goal_c - c, -0.06, 0.06); key = None
                if (float(np.linalg.norm(goal_c - c)) < 0.02 and self.t_state > 6) or self.t_state > 45: self._hold = tp.copy(); self._go("drop")
            elif self.state == "drop":
                self.grip = 1.0; want = self._hold; key = None
                if self.t_state > 12: self._go("back")
            else:
                self.grip = 1.0; want = self._hold + np.array([0.0, 0.0, 0.26 * min(1.0, self.t_state / 14.0)]); key = None; gain = 0.5
                if self.t_state > 16: self._go("rest")
        if want is not None:
            self.target_q, err = (self._move(tr, q_now, want, Rg, key, gain=gain) if key is not None else self._servo(tr, q_now, want, Rg, gain=gain)); self.catch_err = round(float(err), 4)
        row = np.zeros(23); row[SL["trunk"]] = self.target_q[:4]; row[SL["arm_right"]] = self.target_q[4:]
        row[SL["arm_left"]] = self.home_left; row[SL["gl"]] = 1.0; row[SL["gr"]] = self.grip
        self._last_step = tr["step"]
        self._last_row = row.copy()
        return row
# Optional local diagnostics; never reads any additional observation channel.
import os as _os
if _os.environ.get('ROD_TRACE'):
    import json as _json
    import time as _time
    _act = RodOracle.act
    def _traced_act(self, obs, tr):
        start = _time.process_time()
        row = _act(self, obs, tr)
        if not hasattr(self, '_trace_file'):
            self._trace_file = open(_os.environ['ROD_TRACE'], 'w', buffering=1)
        self._trace_file.write(_json.dumps(dict(tr=tr, q=self._q(obs['proprio']).tolist(), target=self.target_q.tolist(), state=self.state, ts=self.t_state, err=self.catch_err, goal=None if self.q_goal is None else self.q_goal.tolist(), log=self.log[-1] if self.log else {}, cpu=_time.process_time()-start))+'\n')
        return row
    RodOracle.act = _traced_act
