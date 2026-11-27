"""Privileged reference controller: reads the cell's ground truth, never the cameras.

Track and belt alike: the right gripper waits open over the part's lane, fingers either side of it, approach along the
surface normal; it closes as the part's centre arrives, lifts, carries the part over the tote, opens, and goes back.
All geometry is world frame; joint targets come from the URDF kinematics in kin.py (base frame = base_link)."""
from __future__ import annotations
import math, json, os
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
import numpy as np
import kin
from motion import PoseIK

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
        self.shapes={k:np.asarray(v) for k,v in json.load(open(os.path.join(os.path.dirname(__file__),"shapes.json"))).items()}
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
        self.tool = np.array([0., 0., -.03689-self.reach])
        self.log.append({"reach": round(self.reach, 4), "fk_vs_eef_m": round(self.fk_err, 4), "offset_world": (np.array(tr["eef_right_pos"]) - pw).round(4).tolist()})

    def _measured(self, tr):
        """Gripper rotation (URDF gripper-link convention) and tool point (between the fingertips), world frame."""
        Rg = rot(tr["eef_right_quat"]) @ self.R_off.T
        return np.array(tr["fingers_right"]) - Rg[:, 2] * self.reach, Rg

    def _servo(self, tr, q_now, p_world, R_world, gain=0.6):
        """Resolved-rate step toward the pose: measured error, model Jacobian. Returns new joint targets and the error."""
        Rb,pb=self._base(tr);tp,Rm=self._measured(tr)
        if not hasattr(self,'planner'): self.planner=PoseIK(self.arm,self.tool)
        slow=self.state in ('lift','join','carry','lower')
        error=np.asarray(p_world)-tp
        if slow:
            velocity=error*5.
            velocity*=min(1.,.58/(np.linalg.norm(velocity)+1e-9))
            old=getattr(self,'carry_velocity',np.zeros(3));dv=velocity-old
            velocity=old+dv*min(1.,.055/(np.linalg.norm(dv)+1e-9));self.carry_velocity=velocity
            target=tp+velocity/30.
        else:
            self.carry_velocity=np.asarray(tr.get('vel',[0.,0.,0.])) if self.state=='follow' else np.zeros(3)
            target=np.asarray(p_world)
        if self.state not in ('wait','rest','init'):
            flip=R_world@np.diag([-1.,-1.,1.])
            if np.trace(flip@Rm.T)>np.trace(R_world@Rm.T):R_world=flip
            Re=(Rb.T@R_world)@(Rb.T@Rm).T
            if slow:
                er=Rotation.from_matrix(Re).as_rotvec();er*=min(1.,.035/(np.linalg.norm(er)+1e-9))
            else:er=.5*np.array([Re[2,1]-Re[1,2],Re[0,2]-Re[2,0],Re[1,0]-Re[0,1]])
            J,_,_=self.arm.jacobian(q_now,self.tool)
            A=np.r_[J[:3],.3*J[3:]]
            ep=Rb.T@(target-tp)
            if not slow:ep=np.clip(ep,-.12,.12)*.6
            err=np.r_[ep,(.3 if slow else .18)*er]
            pref=np.array([.4,-.45,-.4,-.3,-.1,-1.1,.2,-.9,-.35,.4,.9]) if slow else getattr(self,'q_ready',self.planner.pref)
            aa=np.r_[A,.025*np.eye(11)];bb=np.r_[err,.025*.025*(pref-q_now)]
            dlo=np.minimum(np.maximum(self.arm.lo+.07-q_now,-.075),.074)
            dhi=np.maximum(np.minimum(self.arm.hi-.07-q_now,.075),-.074)
            dq=lsq_linear(aa,bb,bounds=(dlo,dhi),method='bvls',max_iter=12,tol=1e-6).x
            q=q_now+dq
            self.planner_state=self.state
            return q,float(np.linalg.norm(error))
        # Correct the small fixed model mismatch using measured hand pose.
        pm,Rk,_=self.arm.fk(q_now,self.tool)
        target_p=pm+Rb.T@(target-tp)
        target_R=Rk@Rm.T@R_world
        flip=target_R@np.diag([-1.,-1.,1.])
        ref_R=self.arm.fk(self.planner.pref,self.tool)[1]
        if np.trace(flip@ref_R.T)>np.trace(target_R@ref_R.T):target_R=flip
        if self.state in ('wait','rest') and getattr(self,'planner_state',None) not in ('wait','rest'):
            self.planner.plan=self.planner.pref.copy()
        plan=self.planner.solve(q_now,target_p,target_R)
        self.planner_state=self.state
        # Direct joint targets retain the branch rather than integrating Jacobian drift.
        delta=plan-q_now
        limit=.09 if not slow else .055
        q=q_now+np.clip(delta,-limit,limit)
        return q,float(np.linalg.norm(error))

    def _solve(self, tr, q_now, p_world, R_world):
        """Global IK (model frame) from a few seeds; used only for large moves."""
        Rb, pb = self._base(tr); p = Rb.T @ (np.asarray(p_world) - pb); R = Rb.T @ R_world
        free = np.ones(11, bool); free[3] = True; best = None
        for seed in (np.r_[0.3, -0.6, -0.3, 0, 0, 0, 0, -1.2, 0, 0, 0], np.r_[0.6, -1.2, -0.6, 0, 0, 0, 0, -1.0, 0, 0, 0], q_now):
            q, err = self.arm.ik(np.clip(seed, self.arm.lo, self.arm.hi), p, R, tool=self.tool, free=free, iters=150)
            margin = float(np.minimum(q - self.arm.lo, self.arm.hi - q)[4:].min())          # stay off the arm's joint limits
            cost = err * 50 + (0.5 if margin < 0.05 else 0.0)
            if best is None or cost < best[0]: best = (cost, q, err)
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
            self.event = tr["event"]; self._go("wait"); self.grip = 1.0; self.catch_err = None; self.stall = 0; self._gap = 9.0; self.reopen_until = 0; self.descent_height=float("inf");self.descent_started=False
        if self.state in ("init",): self._go("wait")
        if tr["phase"] in ("gap", "done"): self._go("rest")
        want = None
        # "rest": this part has been dealt with (set down in the tote, or closed on nothing at the end). Its event runs on for
        # a moment (the part has to come to rest in the tote), and a hand that went back to "wait" took the part in the tote
        # for one coming down the track, chased it, and was still away when the next part arrived (found by the conveyor
        # session, 2026-11-25). So it waits above the lane, open, until the next event starts.
        if (self.state == "wait" and tr["phase"] not in ("staged", "live")) or self.state == "rest":
            if self.state == "rest": self.grip = 1.0
            want = low + along_axis * self.s_catch + left * tr.get("lane",getattr(self, "last_lane", 0.0)) + n * (self.clear + 0.03 + (0.10 if self.state == "rest" else 0.0))
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
            s_ref=self.s_catch
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
            if tr["item"] in ("011_banana", "025_mug"):
                local=np.array([0.,0.,-.020]) if tr["item"]=="011_banana" else np.array([-.015,0.,0.])
                pos=np.array(tr["pos"])+Rp@local;lane=float((pos-low)@left);s=float((pos-low)@along_axis)
                s_ref=float(np.clip(s-max(v,0.)*self.lead+getattr(self,"i_along",0.),self.s_min,self.s_catch))
            if tr["item"] == "037_scissors":
                pos=np.array(tr["pos"])+Rp@np.array([-.025,.056,0.])
                lane=float((pos-low)@left);s=float((pos-low)@along_axis);width=.080
                g_axis=Rp[:,1]-n*float(Rp[:,1]@n);g_axis/=np.linalg.norm(g_axis)
                s_ref=float(np.clip(s-max(v,0.)*self.lead+getattr(self,"i_along",0.),self.s_min,self.s_catch))
            if "grasp_local" in tr:
                pos=np.array(tr["pos"])+Rp@np.array(tr["grasp_local"])
                g_axis=Rp@np.array(tr["grasp_axis"]);g_axis-=n*(g_axis@n);g_axis/=np.linalg.norm(g_axis)
                width=tr["grasp_width"];lane=float((pos-low)@left);s=float((pos-low)@along_axis)
                s_ref=float(np.clip(s-max(v,0.)*self.lead+getattr(self,"i_along",0.),self.s_min,self.s_catch))
            if tr['item'] in ('002_master_chef_can','005_tomato_soup_can','007_tuna_fish_can','040_large_marker'):
                axis=Rp[:,1]-n*float(Rp[:,1]@n);axis/=np.linalg.norm(axis)+1e-9
                candidates=[]
                for ga in (axis,np.cross(n,axis)):
                    projection=self.shapes[tr['item']]@(Rp.T@ga);w=float(projection.max()-projection.min())
                    if w<.118:candidates.append((1.-abs(ga@left)+.2*w,ga,w))
                if candidates:_,g_axis,width=min(candidates,key=lambda x:x[0])
            mode = "rim" if tr["item"] == "024_bowl" else "skip" if width > 0.118 else "narrow"
            if mode == "narrow": Rg = np.stack([np.cross(g_axis, n), g_axis, n], axis=1)
            yaw_off = float(np.degrees(np.arctan2(abs(g_axis @ along_axis), abs(g_axis @ left))))
            if mode == "rim":
                axis=Rp[:,1]
                if abs(axis@n)>.70:
                    radial=left-axis*(axis@left);radial/=np.linalg.norm(radial)+1e-9
                    g_axis=radial-n*(radial@n)
                else:
                    radial=n-axis*(axis@n);radial/=np.linalg.norm(radial)+1e-9
                    g_axis=axis-n*(axis@n)
                pos=np.asarray(tr['pos'])-.023*axis+.073*radial
                g_axis/=np.linalg.norm(g_axis)+1e-9
                Rg=np.stack([np.cross(g_axis,n),g_axis,n],axis=1);width=.012
            lane=float((pos-low)@left);s=float((pos-low)@along_axis)
            if self.state=="follow" and s<=self.s_catch:
                lag=float((pos-self._measured(tr)[0])@along_axis)
                self.i_along=float(np.clip(getattr(self,"i_along",0.)+.12*lag,-.08,.08))
            else:self.i_along=0.
            s_ref=float(np.clip(s-max(v,0.)*self.lead+getattr(self,"i_along",0.),self.s_min,self.s_catch))
            verts=self.shapes.get(tr['item'])
            if verts is not None:
                proj=verts@(Rp.T@n);height=float(proj.max()-proj.min())
            low_tip = max(.006 if tr["item"] in ("040_large_marker", "037_scissors") else self.clear, height - (0.03 if mode == "rim" else 0.035))           # fingertips as low as the fingers' length allows
            if mode=="rim":low_tip=max(.006,float((pos-low)@n)-.022)
            # Hover clear of the part's top while waiting and synchronising (a low gripper stopped tall parts: jam),
            # then come down onto it while travelling with it.
            a=max(0.,tr.get("accel_roll" if tr["item"] in ("005_tomato_soup_can", "007_tuna_fish_can") else "accel_slide",0.))
            distance=max(0.,s-self.s_catch)
            eta=(2*distance/(max(v,0.)+math.sqrt(max(v,0.)**2+2*a*distance)+1e-9))
            if tr["phase"]=="staged":eta=10.
            down = self.state=="follow" and mode != "skip"
            tp,Rm=self._measured(tr);off=pos-tp
            lateral_tol=.010 if width>.08 else .020
            ready=abs(float(off@along_axis))<.035 and abs(float(off@left))<lateral_tol and abs(float(Rm[:,1]@g_axis))>.94
            predicted_ready=abs(float((tp-low)@along_axis)-self.s_catch)<.025 and abs(float(off@left))<lateral_tol and abs(float(Rm[:,1]@g_axis))>.96
            if down and (ready or (eta<.20 and predicted_ready)):self.descent_started=True
            if down and self.descent_started:
                self.descent_height=max(low_tip,min(self.descent_height,height+.04)-.018)
                self.tip_h=self.descent_height
            else:self.tip_h=height+.04
            if self.grip<0 and hasattr(self,"close_R"):
                Rg=self.close_R;self.tip_h=self.close_tip_h
            want = low + along_axis * s_ref + left * lane + n * self.tip_h
            if tr["phase"] == "live" and self.state == "wait" and eta <= .32: self._go("follow")
            if self.state == "follow":
                ts_close = max(1, int(round((0.127 - width) / 0.5 * HZ)))                 # steps the fingers need to reach the part
                tip_now = float((self._measured(tr)[0] - low) @ n)
                off = pos - self._measured(tr)[0]; aligned = abs(float(off @ along_axis)) < 0.03 and abs(float(off @ left)) < 0.025
                if mode=='narrow' and length>.14 and abs(float(g_axis@along_axis))<.4 and any(tr.get('finger_contact_right',[])):
                    aligned = aligned or (abs(float(off@along_axis))<.075 and abs(float(off@left))<.018)
                d = self.diag.setdefault(self.event, {"item": tr["item"], "follow_steps": 0, "min_along": 9.0, "min_lat": 9.0, "min_tip_above": 9.0, "v": round(v, 2), "s_start": round(s, 2), "mode": mode, "w": round(width, 3), "h": round(height, 3), "yaw": round(yaw_off)})
                d["follow_steps"] += 1; d["min_along"] = round(min(d["min_along"], abs(float(off @ along_axis))), 3); d["min_lat"] = round(min(d["min_lat"], abs(float(off @ left))), 3)
                d["min_tip_above"] = round(min(d["min_tip_above"], tip_now - low_tip), 3); d["s_end"] = round(s, 2)
                if self.grip > 0 and self.t_state >= self.settle and self.t_state >= getattr(self, "reopen_until", 0) and (tip_now <= max(low_tip + 0.012,height-.015) or all(tr.get("finger_contact_right",[False,False]))) and aligned and mode != "skip":
                    self.grip = -1.0; self.t_close = self.t_state; self.close_R=Rg.copy();self.close_tip_h=max(self.tip_h,tip_now-.005) if all(tr.get("finger_contact_right",[False,False])) else self.tip_h
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
                    else: self.log[-1]["grip_gap"] = round(gap, 3); self._go("rest")
                if s_ref <= self.s_min + 1e-6 and self.grip > 0 and not self.log_has_event(): self.log.append({"event": self.event, "item": tr["item"], "close_s": round(s, 3), "v": round(v, 3), "width": round(width, 3), "last_chance": True})
                if s_ref <= self.s_min + 1e-6 and self.grip > 0 and mode != "skip": self.grip = -1.0; self.t_close = self.t_state; self.close_R=Rg.copy();self.close_tip_h=self.tip_h
        elif self.state == "lift":
            if self.t_state == 1:
                tp, self.hold_R = self._measured(tr)
                self.lift_target = tp + np.array([0., 0., .16])
                Rp=rot(tr['quat']);self.grasp_rel_R=Rp.T@self.hold_R
                iz=int(np.argmax(np.abs(Rp[2,:])));axis=Rp[:,iz];goalaxis=np.array([0.,0.,1. if axis[2]>0 else -1.])
                correction=Rotation.align_vectors([goalaxis],[axis])[0].as_matrix()
                # Keep the wrist approach nearly vertical; do not demand a different resting face in flight.
                candidate=correction@Rp
                self.flat_R=candidate if (candidate@self.grasp_rel_R)[2,2]>.92 else Rp.copy()

            tp,_=self._measured(tr)
            want = self.lift_target; Rg = self.hold_R
            if self.t_state > 12 and (np.linalg.norm(tp-self.lift_target)<.045 or self.t_state>24): self._go("join" if ("grasp_local" in tr and not tr.get("no_join")) else "carry"); self.carry_settle = 0
        elif self.state == "join":
            tp,_=self._measured(tr);part=np.asarray(tr["pos"])
            goal=np.array([self.base_home[0]+.60,self.base_home[1]-.015,low[2]+.32])
            want=tp+goal-part;Rg=self.hold_R
            if tr.get("attached_left") or (self.t_state>12 and all(tr.get("finger_contact_left",[False,False]))) or self.t_state>150:self._go("carry");self.carry_settle=0
        elif self.state == "carry":
            tote = np.array(tr["tote"]); tp, _ = self._measured(tr)
            if tr.get("pos") is not None:
                part = np.array(tr["pos"])
                clear_chute=(part[0]<low[0]+.06 or float((part-low)@left)<-.30)
                carry_h=tr.get('drop_height',.20) if clear_chute else max(.28,low[2]+.16-tote[2]);want = tp + tote + np.array([0, 0, carry_h]) - part
                near = np.linalg.norm((part-tote)[:2]) < .030 and abs(part[2]-tote[2]-carry_h)<.035
                stable = np.linalg.norm(tr["vel"]) < .15
            else: want = tote + np.array([0,0,.30]); near = False; stable = False
            Rg = self.flat_R@self.grasp_rel_R
            self.hold_R=Rg
            self.carry_settle = self.carry_settle + 1 if near and stable else 0
            origin=np.asarray(tr.get('tote_origin',tr['tote']));rel=part-origin
            supported=abs(rel[0])<.185 and abs(rel[1])<.13 and .03<rel[2]<.245 and float((part-low)@left)<-.305 and np.linalg.norm(tr['vel'])<.07
            self.supported_steps=getattr(self,'supported_steps',0)+1 if supported else 0
            if self.supported_steps>=7:
                self.drop_target=tp.copy();self.grip=1.;self._go('drop')
            # 2026-11-26: a carry that does not settle on its exact slot within 2.5 s goes on to lowering anyway once
            # the part is over the tote's inside (traces of job 62723811: carry ran to its 150-step limit and lowering to
            # its 90-step limit for about half of the parts, 8 s in all, and the part's time ran out with it in the tote).
            over=abs(rel[0])<.17 and abs(rel[1])<.12
            elif_go=(self.carry_settle >= 2 and self.t_state > 12) or (over and self.t_state>75) or self.t_state > 110
            if self.supported_steps<7 and elif_go:
                self._go("lower");self.lower_stable=0;self.lower_hist=[]
        elif self.state == "lower":
            tp, _ = self._measured(tr); part=np.array(tr["pos"]); tote=np.array(tr["tote"])
            h=tr.get("drop_height",.105 if tr.get("item")=="002_master_chef_can" else .23)
            want=tp+tote+np.array([0.,0.,h])-part; Rg=self.hold_R
            stable=np.linalg.norm((part-tote)[:2])<.03 and h-.02<part[2]-tote[2]<min(.255,h+.07) and np.linalg.norm(tr["vel"])<.08
            self.lower_stable=self.lower_stable+1 if stable else 0
            # Let go as soon as the part has stopped coming down over the tote's inside, not only at the planned height: the hand
            # often cannot get that low (parts already in the tote), and waiting out the limit cost the part its time.
            origin=np.asarray(tr.get('tote_origin',tr['tote']));rel=part-origin;hist=(getattr(self,'lower_hist',[])+[float(part[2])])[-8:];self.lower_hist=hist
            stalled=len(hist)==8 and max(hist)-min(hist)<.004 and abs(rel[0])<.18 and abs(rel[1])<.13 and rel[2]<.30
            if self.lower_stable>=4 or stalled or self.t_state>40:
                self.drop_target=tp.copy();self.grip=1.;self._go("drop")
        elif self.state == "drop":
            want = self.drop_target + np.array([0.,0.,.14 if self.t_state > 6 else 0.]); Rg = self.hold_R
            if tr.get("item") == "002_master_chef_can":
                want=self.drop_target+self.hold_R[:,0]*min(.20,max(0,self.t_state-12)*.008)
                if self.t_state>40 and not any(tr.get("finger_contact_right",[])):self._go("rest")
            elif self.t_state > 20: self._go("rest")
        self.want=want;self.want_R=Rg
        if want is not None:
            slow = self.state in ("lift", "join", "carry", "lower")                       # B. gentle with a part in the fingers
            tp, _ = self._measured(tr); far = float(np.linalg.norm(np.asarray(want) - tp))
            # The local servo is the proven controller. When the hand is far away (it got stuck after some events),
            # first return in joint space to the posture in which the waiting pose was last reached, then servo.
            if self.state in ("wait", "rest") and far > .25 and tp[2] < low[2] + .20:
                want = np.array([tp[0], tp[1], low[2] + .27])
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
        if self.state in ("wait","rest"):
            side = np.array([-math.sin(self.base_yaw0), math.cos(self.base_yaw0)])                # the base's own left
            lane_pos=np.array(tr["low_end"])+np.array(tr["left"])*tr["lane"]+np.array(tr["up_chute"])*self.s_catch
            shift = float((lane_pos[:2] - self.base_home[:2]) @ side) + PICK_SIDE
            self.base_goal = self.base_home[:2] + side * float(np.clip(shift, -0.25, 0.25)) + np.array([.15,0.])
        elif self.state in ("join", "carry", "lower", "drop"): self.base_goal = self.base_home[:2].copy()
        if self.state not in ("follow", "lift"):
            e = Rb[:2, :2].T @ (self.base_goal - pb[:2]); yaw = math.atan2(Rb[1, 0], Rb[0, 0]) - self.base_yaw0
            speed=.2 if self.state in ("carry","lower","drop") else .65
            row[SL["base"]] = [float(np.clip(4.0 * e[0], -speed, speed)), float(np.clip(4.0 * e[1], -speed, speed)), float(np.clip(-3.0 * yaw, -.5, .5))]
        row[SL["arm_left"]] = self.home_left; row[SL["gl"]] = 1.0; row[SL["gr"]] = self.grip
        if tr.get('item')=='040_large_marker' and self.grip<0 and not tr.get('attached_right'):
            row[SL['gr']]=-.65
        return row

    def _yaw_R(self, tr):
        """Gripper pointing straight down, opening axis along the base's y."""
        Rb, _ = self._base(tr); return Rb
