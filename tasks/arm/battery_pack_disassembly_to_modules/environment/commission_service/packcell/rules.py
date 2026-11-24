"""The rules of the cell that are not left to the physics engine (NumPy only, no simulator needed).

A fastener is a rule-based constraint (the design's stated simplification, tests/README.md): the nutrunner turns it
when the right bit is on its head, the axes are aligned, the bit is pressed on hard enough and enough loosening
turns are applied; a wrong bit or a bad alignment wears the head until it is stripped. A part is held until its
fasteners are out (or, for the plugs, until it is pulled hard enough along its axis), then it is an ordinary rigid
body. The gripper holds a part when it closes on one of the part's grip features. Two events end a pack: a tool
touching a live busbar before the pack is isolated, and a module falling. The credit is read from where the bodies
are at the end.

The cell (`cell.py`) owns the bodies; it calls in here once per control tick with poses and forces read from the
physics engine, and applies what comes back (free a part, attach it to the gripper, hand a fastener to the socket).
"""

import math

import numpy as np

from . import series as S

UP = np.array([0.0, 0.0, 1.0])


def angle_deg(a, b):
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Rules(object):
    def __init__(self, pack):
        self.pack = pack
        self.T = S.pack_pose(pack)
        self.n = self.T[:3, 2].copy()                    # the pack's z axis (every fastener's axis) in the world
        self.fast = {}
        for f in pack["fasteners"]:
            head = S.HEADS[f["head"]]
            self.fast[f["id"]] = dict(
                f, status="missing" if f["missing"] else "seated", turned=0.0, wear=0.0, stripped=False, broken=False,
                top=(self.T[:3, :3] @ (np.array(f["pos"]) + [0, 0, head["height"]])) + self.T[:3, 3])
        self.part = {}
        for p in pack["parts"]:
            seat = np.eye(4)
            seat[:3, :3] = self.T[:3, :3]
            seat[:3, 3] = (self.T[:3, :3] @ np.array(p["pos"])) + self.T[:3, 3]
            self.part[p["id"]] = dict(p, locked=True, grasped=False, damaged=False, seat=seat, left_seat=False,
                                      dropped=False)
        self.tool, self.bit, self.captured = "nutrunner", "S10", None
        self.width = S.TOOLS["gripper"]["open_max"]
        self.grasp = None
        self.msd_out_at = None
        self.ended = None
        self.events = []
        self.t = 0.0
        self.angle = 0.0               # the spindle's angle (rad, signed, since the cell started)
        # live zones (pack frame boxes) around every busbar with its bolts and posts
        self.hv = []
        for p in pack["parts"]:
            if p["kind"] == "busbar":
                hx, hy = (p["length"] / 2, 0.011) if p["along"] == "x" else (0.011, p["length"] / 2)
                m = S.HV_MARGIN
                self.hv.append((p["id"], np.array([p["pos"][0] - hx - m, p["pos"][1] - hy - m, S.Z["post_top"] - 0.012]),
                                np.array([p["pos"][0] + hx + m, p["pos"][1] + hy + m, S.Z["busbar_seat"] + 0.008 + m])))

    def log(self, kind, **detail):
        self.events.append(dict(t=round(self.t, 3), kind=kind, **detail))

    # ------------------------------------------------------------------ isolation and the live busbars
    def isolated(self):
        return self.msd_out_at is not None and self.t - self.msd_out_at >= S.ISOLATION_WAIT_S

    def hv_check(self, points):
        """`points`: world points of the mounted tool that can touch (the bit's tip; the fingertips). Ends the pack
        when one of them is inside a live zone before isolation."""
        if self.ended or self.isolated():
            return
        Ti = np.linalg.inv(self.T)
        for pt in points:
            q = (Ti[:3, :3] @ np.asarray(pt)) + Ti[:3, 3]
            for bid, lo, hi in self.hv:
                if np.all(q >= lo) and np.all(q <= hi):
                    self.ended = "live_contact"
                    self.log("live_contact", busbar=bid)
                    return

    # ------------------------------------------------------------------ the nutrunner on a fastener
    def under_tip(self, tip, axis):
        """The seated fastener the bit's tip is on, with the lateral offset, the gap and the tilt; or None."""
        best = None
        for fid, f in self.fast.items():
            if f["status"] != "seated":
                continue
            d = tip - f["top"]
            gap = float(np.dot(d, self.n))
            if abs(gap) > 0.004:
                continue
            lat = float(np.linalg.norm(d - gap * self.n))
            if lat < 0.008 and (best is None or lat < best[1]):
                best = (fid, lat, gap)
        if best is None:
            return None
        return best + (angle_deg(-np.asarray(axis), self.n),)

    def grade(self, f, lat, gap, tilt, force):
        """How the mounted bit sits on fastener f: "full", "partial" or "none"."""
        E = S.ENGAGE
        if gap > E["seat_gap"] or force < E["seat_force"] or self.captured is not None:
            return "none"
        kind = S.HEADS[f["head"]]["kind"]
        if f["stripped"]:
            x = E["extractor"]
            ok = self.bit == "EX" and lat <= x["lateral"] and tilt <= E["tilt_full_deg"] and force >= x["force"]
            return "full" if ok else "none"
        how = "loose" if self.bit == "EX" else S.BIT_ON_HEAD.get((self.bit, f["head"]))
        if how is None or lat > E[kind]["partial"] or tilt > E["tilt_partial_deg"]:
            return "none"
        if how == "fit" and lat <= E[kind]["full"] and tilt <= E["tilt_full_deg"]:
            return "full"
        return "partial"

    def spindle(self, tip, axis, force, speed, limit, dt, harness_axis=None):
        """One tick of the spindle. `speed` is the commanded speed (rad/s; negative loosens), `limit` the torque
        limit (N m), `force` the axial force on the bit (N). Returns (torque, speed): what the nutrunner measures.
        A fastener that comes free goes into the socket (`self.captured`)."""
        E = S.ENGAGE
        if self.tool != "nutrunner" or abs(speed) < 1e-9:
            return 0.0, 0.0
        if harness_axis is not None and force > E["seat_force"]:
            # the bit turning on the harness chafes it through
            pts = harness_axis
            for a, b in zip(pts[:-1], pts[1:]):
                ab = b - a
                u = max(0.0, min(1.0, float(np.dot(tip - a, ab) / np.dot(ab, ab))))
                if np.linalg.norm(tip - (a + u * ab)) < S.HARNESS_R + 0.004:
                    if not self.part["harness"]["damaged"]:
                        self.part["harness"]["damaged"] = True
                        self.log("harness_damaged")
                    break
        hit = self.under_tip(tip, axis)
        free = (E["free_torque"], speed)
        if hit is None:
            self.angle += speed * dt
            return free
        fid, lat, gap, tilt = hit
        f = self.fast[fid]
        g = self.grade(f, lat, gap, tilt, force)
        if g == "none":
            self.angle += speed * dt
            return free
        th = S.THREADS[f["thread"]]
        running = E["running_fraction"] * th["tighten"]
        need = running if f["broken"] else f["breakaway"]
        rev = abs(speed) * dt / (2 * math.pi)
        if g == "partial":
            f["wear"] += E["wear_per_rev_partial"] * rev
            self.angle += speed * dt
            torque = min(limit, 0.4 * need * (0.6 + 0.4 * math.sin(6 * self.angle)))
            return self._wear(f, torque, speed)
        ex = self.bit == "EX"
        if speed < 0:                                   # loosening
            if ex:
                need *= E["extractor"]["torque_factor"]
            if limit < need:
                return limit, 0.0                       # stalled at the limit
            if S.HEADS[f["head"]]["kind"] == "torx" and not ex and force < E["torx_force_base"] + E["torx_force_per_Nm"] * need:
                f["wear"] += E["wear_per_rev_camout"] * rev
                self.angle += speed * dt
                return self._wear(f, min(limit, 0.3 * need * (0.6 + 0.4 * math.sin(6 * self.angle))), speed)
            k = E["extractor"]["speed_factor"] if ex else 1.0
            f["turned"] += abs(speed) * dt * k
            self.angle += speed * dt * k
            if math.degrees(f["turned"]) >= E["breakaway_deg"]:
                f["broken"] = True
            if f["turned"] >= th["turns"] * 2 * math.pi:
                f["status"] = "captured"
                self.captured = fid
                self.log("fastener_out", id=fid, stripped=f["stripped"])
            return need, speed * k
        # tightening: a fastener that was backed out goes in again; a seated one stalls the spindle at once
        if f["turned"] > 0:
            f["turned"] = max(0.0, f["turned"] - speed * dt)
            self.angle += speed * dt
            if f["turned"] == 0.0:
                f["broken"] = False
            return running, speed
        if limit > th["head_limit"]:
            f["wear"] += E["wear_per_s_overtorque"] * dt
            return self._wear(f, limit, 0.0)
        return limit, 0.0

    def _wear(self, f, torque, speed):
        if f["wear"] >= 1.0 and not f["stripped"]:
            f["stripped"] = True
            self.log("fastener_stripped", id=f["id"])
        return torque, speed

    def eject(self):
        """Let the fastener in the socket go; returns its id (the cell turns it into a falling body) or None."""
        fid, self.captured = self.captured, None
        if fid is not None:
            self.fast[fid]["status"] = "free"
        return fid

    # ------------------------------------------------------------------ tool and bit exchange
    def at_dock(self, flange_T, speed, name):
        tol = S.CELL["dock_tol"]
        d = np.linalg.norm(flange_T[:3, 3] - np.array(S.CELL["docks"][name]))
        return (d <= tol["pos"] and angle_deg(flange_T[:3, 2], -UP) <= tol["angle_deg"]
                and angle_deg(flange_T[:3, 0], [1.0, 0.0, 0.0]) <= tol["angle_deg"] + 2.0 and speed <= tol["speed"])

    def at_bit_station(self, tip, axis, speed):
        tol = S.CELL["dock_tol"]
        return (np.linalg.norm(tip - np.array(S.CELL["bit_station"])) <= tol["pos"]
                and angle_deg(axis, -UP) <= tol["angle_deg"] and speed <= tol["speed"])

    # ------------------------------------------------------------------ the gripper
    def grip_points(self, tcp_T, width=None):
        """The fingertips' ends and the point between them (world)."""
        w = self.width if width is None else width
        g = S.TOOLS["gripper"]
        c = np.array(g["close_axis"])
        return [tcp_T[:3, 3], (tcp_T[:3, :3] @ (c * w / 2 + [0, 0, 0.015])) + tcp_T[:3, 3],
                (tcp_T[:3, :3] @ (-c * w / 2 + [0, 0, 0.015])) + tcp_T[:3, 3]]

    def _candidate(self, tcp_T, poses, width):
        """The part whose grip feature lies between the fingers: (pid, feature width) or None."""
        g = S.TOOLS["gripper"]
        c_tool = tcp_T[:3, :3] @ np.array(g["close_axis"])
        z_tool, x_tool = tcp_T[:3, 2], np.cross(c_tool, tcp_T[:3, 2])
        best = None
        for pid, p in self.part.items():
            if p["grasped"] or pid not in poses:
                continue
            Tp = poses[pid]
            if angle_deg(z_tool, -Tp[:3, 2]) > 20.0:
                continue
            feats = []
            if p["kind"] == "harness":
                pts = [(Tp[:3, :3] @ (np.array(q) - np.array(p["pos"]))) + Tp[:3, 3] for q in p["path"]]
                for a, b in zip(pts[:-1], pts[1:]):
                    ab = b - a
                    u = max(0.0, min(1.0, float(np.dot(tcp_T[:3, 3] - a, ab) / np.dot(ab, ab))))
                    feats.append((a + u * ab, np.cross(ab / np.linalg.norm(ab), UP), 2 * S.HARNESS_R, 25.0))
            else:
                for gr in p["grasps"]:
                    feats.append(((Tp[:3, :3] @ np.array(gr["pos"])) + Tp[:3, 3], Tp[:3, :3] @ np.array(gr["close"]), gr["width"], 20.0))
            for pos, close, fw, tol in feats:
                a = angle_deg(c_tool, close)
                if min(a, 180.0 - a) > tol or fw > width + 1e-6:
                    continue
                d = pos - tcp_T[:3, 3]
                dc, dz, dx = abs(float(np.dot(d, c_tool))), abs(float(np.dot(d, z_tool))), abs(float(np.dot(d, x_tool)))
                if dc <= (width - fw) / 2 + 0.004 and dz <= 0.012 and dx <= 0.012:
                    score = dc + dz + dx
                    if best is None or score < best[2]:
                        best = (pid, fw, score)
        return best

    def gripper(self, tcp_T, target, dt, poses):
        """One tick of the gripper toward opening `target` (m). Returns ("attach", pid), ("detach", pid) or None."""
        g = S.TOOLS["gripper"]
        if self.tool != "gripper":
            return None
        target = max(0.0, min(g["open_max"], float(target)))
        step = g["speed"] * dt
        if self.grasp is not None:
            if target > self.width + 0.001:              # opening lets go
                pid, self.grasp = self.grasp, None
                self.part[pid]["grasped"] = False
                self.width = min(target, self.width + step)
                self.log("release", id=pid)
                return ("detach", pid)
            return None
        if target >= self.width:
            self.width = min(target, self.width + step)
            return None
        new = max(target, self.width - step)
        cand = self._candidate(tcp_T, poses, self.width)
        if cand is not None and new <= cand[1]:
            pid, fw, _ = cand
            self.width = fw
            self.grasp = pid
            p = self.part[pid]
            p["grasped"] = True
            if p["lock"]["type"] == "free":
                p["locked"] = False
            self.log("grasp", id=pid)
            return ("attach", pid)
        self.width = new
        return None

    # ------------------------------------------------------------------ what holds a part
    def unlock(self, moved, f_ext):
        """Free the parts that nothing holds any more; returns their ids. `moved`: ids of the parts that are more
        than 0.02 m from their seats; `f_ext`: the external force on the tool (world, N): a plug comes out when
        the gripper pulls it along its axis harder than its latch holds."""
        out = []
        for pid in moved:
            p = self.part[pid]
            if not p["left_seat"]:
                p["left_seat"] = True
                if p["kind"] == "msd" and self.msd_out_at is None:
                    self.msd_out_at = self.t
                    self.log("msd_out")
        for pid, p in self.part.items():
            if not p["locked"]:
                continue
            lock = p["lock"]
            if lock["type"] == "fasteners":
                free = all(self.fast[i]["status"] != "seated" for i in lock["ids"])
            elif lock["type"] == "parts":
                free = all(self.part[i]["left_seat"] for i in lock["ids"] if i in self.part)
            elif lock["type"] == "pull":
                free = p["grasped"] and float(np.dot(f_ext, self.n)) < -lock["force"]
            else:
                free = False
            if free:
                p["locked"] = False
                out.append(pid)
                self.log("freed", id=pid)
        return out

    def drop_check(self, velocities):
        """`velocities`: {module id: vertical speed (m/s)} of the modules. A free module that falls ends the pack."""
        if self.ended:
            return
        for pid, vz in velocities.items():
            p = self.part[pid]
            if not p["locked"] and not p["grasped"] and vz < -0.7:
                p["dropped"] = p["damaged"] = True
                self.ended = "module_dropped"
                self.log("module_dropped", id=pid)
                return

    # ------------------------------------------------------------------ the credit
    @staticmethod
    def _in_bin(name, pos):
        bn = S.CELL["bins"][name]
        return (abs(pos[0] - bn["center"][0]) <= bn["size"][0] / 2 and abs(pos[1] - bn["center"][1]) <= bn["size"][1] / 2
                and S.TABLE_Z - 0.01 <= pos[2] <= S.TABLE_Z + bn["wall_h"] + 0.03)

    def placed(self, item, T):
        """Whether a part or fastener at world pose T is in the place it belongs."""
        pos, dest = T[:3, 3], item["dest"]
        tilt = angle_deg(T[:3, 2], UP)
        if dest == "fasteners":
            return self._in_bin("fasteners_left", pos) or self._in_bin("fasteners_right", pos)
        if dest in S.CELL["bins"]:
            return self._in_bin(dest, pos)
        if dest == "lid_stand":
            c = S.CELL["lid_stand"]["center"]
            return (abs(pos[0] - c[0]) <= 0.10 and abs(pos[1] - c[1]) <= 0.10 and tilt <= 5.0
                    and abs(pos[2] - (S.TABLE_Z + S.LID["t"] / 2)) <= 0.03)
        if dest == "rack":
            rk = S.CELL["rack"]
            bottom = pos[2] - S.MODULE["size"][2] / 2 - S.TABLE_Z
            return any(abs(pos[0] - c[0]) <= rk["bay_size"][0] / 2 and abs(pos[1] - c[1]) <= rk["bay_size"][1] / 2
                       for c in rk["bay_centers"]) and tilt <= 8.0 and -0.005 <= bottom <= 0.015
        return False

    def credit(self, part_poses, fastener_poses):
        """The pack's credit: the value of what lies undamaged where it belongs over the value of everything in the
        pack. Returns (credit, detail)."""
        total = earned = 0.0
        detail = {}
        for pid, p in self.part.items():
            total += p["value"]
            ok = (pid in part_poses and not p["damaged"] and not p["grasped"] and not p["locked"]
                  and self.placed(p, part_poses[pid]))
            detail[pid] = bool(ok)
            earned += p["value"] if ok else 0.0
        for fid, f in self.fast.items():
            if f["missing"]:
                continue
            total += f["value"]
            ok = f["status"] == "free" and not f["stripped"] and fid in fastener_poses and self.placed(f, fastener_poses[fid])
            detail[fid] = bool(ok)
            earned += f["value"] if ok else 0.0
        return (earned / total if total > 0 else 0.0), detail

    # ------------------------------------------------------------------ the privileged view (reference runs only)
    def truth(self, part_poses):
        fs = {fid: {"top": [round(float(v), 5) for v in f["top"]], "head": f["head"], "thread": f["thread"],
                    "status": f["status"], "stripped": f["stripped"], "corroded": f["corroded"], "group": f["group"],
                    "holds": f["holds"]} for fid, f in self.fast.items()}
        ps = {}
        for pid, p in self.part.items():
            Tp = part_poses.get(pid)
            if Tp is None:
                continue
            item = {"kind": p["kind"], "locked": p["locked"], "grasped": p["grasped"], "left_seat": p["left_seat"],
                    "pos": [round(float(v), 5) for v in Tp[:3, 3]],
                    "R": [[round(float(v), 6) for v in row] for row in Tp[:3, :3]], "grasps": p["grasps"],
                    "lock": p["lock"]}
            if p["kind"] == "harness":
                item["path"] = [[round(float(v), 5) for v in (Tp[:3, :3] @ (np.array(q) - np.array(p["pos"]))) + Tp[:3, 3]]
                                for q in p["path"]]
                item["covers"] = p["covers"]
            ps[pid] = item
        return {"variant": self.pack["variant"], "pack_T": [[round(float(v), 6) for v in row] for row in self.T],
                "fasteners": fs, "parts": ps, "isolated": self.isolated(), "msd_out_at": self.msd_out_at,
                "captured": self.captured}
