"""What is not left to the physics engine: what the cutter-gripper cuts and holds, what ends a truss's worth, and
what a row section's yield is. NumPy only; no simulator needed to read or test it (`tests/review/test_rules.py`).

The cell (`cell.py`) hands the rules the bodies' poses from the physics engine every control tick and carries out
what they decide (a joint removed where something was cut, a joint to the tool where the pad holds something).

Members. Everything the blade can meet is a straight member with a radius: a plant's stem segments, a truss's
peduncle (the cut section, from the stem to the first fruit's node) and its three rachis segments, a leaf's petiole.
A member's body frame has its origin at the member's proximal end and z along it.

The cutter-gripper (farm.TOOL; tool frame: origin at the tool centre point in the mouth's centre, z out of the mouth,
the jaws closing along y). A member is in the mouth at a plane x = X when its axis crosses that plane between the
jaws and between the throat and the tips. As the jaws close they meet the thickest member at the blade's plane when
the opening equals its diameter: with a closing force of the row's toughness times the member's cross-section or more
the blade cuts it through and the jaws go on; with less they stall there, and the opening reads the diameter. The pad
holds every member at its own plane that the jaws have closed down to, for as long as they stay closed on it with
`pad_force` or more; a peduncle that the pad presses with more than `crush` times its own cutting force is crushed.

- A cut peduncle frees its truss. The truss stays in the tool when the pad is on the truss's side of the cut;
  otherwise it falls.
- A cut rachis parts the truss (damage). A cut petiole drops its leaf. A cut stem loses the plant: none of its
  trusses counts.
- A fruit whose speed changes by more than `impact_dv` within one control tick is bruised (a fall, a hard landing,
  a blow); a fruit the robot presses with more than `squeeze_force` is bruised.
- At the end a ready truss counts when it was cut free, is whole, unbruised and uncrushed, its plant stands, the tool
  does not hold it, every fruit of it lies inside a crate, and it neither lies on another cut truss nor has one lying
  on it (a fruit of the one resting on a fruit of the other). The row's yield is the count over the row's ready
  trusses.
"""

import math

import numpy as np

from . import farm as F


def member_table(row):
    """Every member of a row: id -> {kind, r, len, owner ...}. The id is the body's id in the scene."""
    out = {}
    for p in row["plants"]:
        pts = [np.array(q) for q in p["points"]]
        for i in range(len(pts) - 1):
            out["%s.s%d" % (p["id"], i)] = {"kind": "stem", "plant": p["id"], "index": i, "r": p["stem_r"],
                                             "len": float(np.linalg.norm(pts[i + 1] - pts[i]))}
        for t in p["trusses"]:
            a, b = np.array(t["peduncle"]["a"]), np.array(t["peduncle"]["b"])
            out[t["id"] + ".p"] = {"kind": "peduncle", "plant": p["id"], "truss": t["id"], "r": t["peduncle"]["r"],
                                   "len": float(np.linalg.norm(b - a))}
            for k, s in enumerate(t["rachis"]):
                out["%s.r%d" % (t["id"], k)] = {"kind": "rachis", "plant": p["id"], "truss": t["id"], "index": k, "r": t["rachis_r"],
                                                "len": float(np.linalg.norm(np.array(s["b"]) - np.array(s["a"])))}
        for lf in p["leaves"]:
            a, b = np.array(lf["petiole"]["a"]), np.array(lf["petiole"]["b"])
            out[lf["id"]] = {"kind": "petiole", "plant": p["id"], "leaf": lf["id"], "r": lf["petiole"]["r"],
                             "len": float(np.linalg.norm(b - a))}
    return out


def fruit_local(truss):
    """Each fruit's centre in the frame of the rachis segment's body it grows on: [(segment index, xyz, r)]."""
    out = []
    for f in truss["fruits"]:
        s = truss["rachis"][f["segment"]]
        R = frame_z(np.array(s["b"]) - np.array(s["a"]))
        out.append((f["segment"], R.T @ (np.array(f["center"]) - np.array(s["a"])), f["r"]))
    return out


def frame_z(d):
    """A rotation whose z axis is along d (the convention every member's body frame is built with)."""
    z = np.asarray(d, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.cross([0.0, 1.0, 0.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x = x / np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def member_ends(row):
    """Every member's two ends at rest (W): id -> (proximal end, distal end)."""
    out = {}
    for p in row["plants"]:
        pts = [np.array(q, dtype=float) for q in p["points"]]
        for i in range(len(pts) - 1):
            out["%s.s%d" % (p["id"], i)] = (pts[i], pts[i + 1])
        for t in p["trusses"]:
            out[t["id"] + ".p"] = (np.array(t["peduncle"]["a"]), np.array(t["peduncle"]["b"]))
            for k, s in enumerate(t["rachis"]):
                out["%s.r%d" % (t["id"], k)] = (np.array(s["a"]), np.array(s["b"]))
        for lf in p["leaves"]:
            out[lf["id"]] = (np.array(lf["petiole"]["a"]), np.array(lf["petiole"]["b"]))
    return out


def rest_poses(row):
    """Every member's body frame at rest (W, 4x4): what the scene is built with."""
    out = {}
    for mid, (a, b) in member_ends(row).items():
        T = np.eye(4)
        T[:3, :3] = frame_z(b - a)
        T[:3, 3] = a
        out[mid] = T
    return out


def crossing(T_tool, T_body, length, X):
    """Where a member's axis crosses the tool's plane x = X: (t along the member in 0..1, y, z in the tool frame), or
    None when it does not cross."""
    Ti = np.linalg.inv(T_tool)
    pa = Ti[:3, :3] @ T_body[:3, 3] + Ti[:3, 3]
    pb = Ti[:3, :3] @ (T_body[:3, 3] + length * T_body[:3, 2]) + Ti[:3, 3]
    da, db = pa[0] - X, pb[0] - X
    if da * db > 0.0 or abs(da - db) < 1e-12:
        return None
    t = da / (da - db)
    p = pa + t * (pb - pa)
    return float(t), float(p[1]), float(p[2])


class Rules(object):
    def __init__(self, row):
        self.row = row
        self.t = 0.0
        self.events = []
        self.ended = None
        self.member = member_table(row)
        for m in self.member.values():
            m.update(cut=False, need=F.cut_force(row, m["r"]) * (F.CUT["stem_factor"] if m["kind"] == "stem" else 1.0))
        self.truss, self.plant, self.leaf = {}, {}, {}
        for p in row["plants"]:
            self.plant[p["id"]] = {"lost": False}
            for t in p["trusses"]:
                self.truss[t["id"]] = {"plant": p["id"], "ready": bool(t["ready"]), "status": "on_plant", "bruised": False,
                                       "squeezed": False, "crushed": False, "parted": False, "cut_t": None,
                                       "fruits": fruit_local(t), "dv_max": 0.0, "press_max": 0.0, "pad_ratio": 0.0}
            for lf in p["leaves"]:
                self.leaf[lf["id"]] = {"plant": p["id"], "cut": False}
        # the gripper
        self.opening = float(F.TOOL["open_max"])
        self.target = float(F.TOOL["open_max"])
        self.limit = 0.0
        self.force = 0.0             # what the jaws press with now (the reading, without its noise)
        self.held = []               # ids of the members the pad holds
        self.stalled = False
        self.margins = {"cut_y_mm": 0.0, "cut_z_mm": 0.0, "cut_force_ratio": 0.0}

    def log(self, kind, **kw):
        self.events.append(dict(kind=kind, t=round(self.t, 2), **kw))

    # ------------------------------------------------------------------ the mouth
    def in_mouth(self, T_tool, poses, X, opening, uncut=False):
        """The members whose axes cross the plane x = X between the jaws (at `opening`) and between the mouth's
        throat and tips: [(id, t, y, z)]. With `uncut`, only members that have not been cut (the blade's)."""
        z0, z1 = F.TOOL["mouth_z"][0] - F.TOOL["tcp_z"], F.TOOL["mouth_z"][1] - F.TOOL["tcp_z"]
        out = []
        p_tool = T_tool[:3, 3]
        for mid, m in self.member.items():
            if uncut and m["cut"]:
                continue                                # what was cut once is not cut again
            T = poses.get(mid)
            if T is None or np.linalg.norm(T[:3, 3] - p_tool) > m["len"] + 0.08:
                continue
            c = crossing(T_tool, T, m["len"], X)
            if c is None:
                continue
            t, y, z = c
            if abs(y) <= opening / 2.0 + 1e-9 and z0 <= z <= z1:
                out.append((mid, t, y, z))
        return out

    # ------------------------------------------------------------------ one control tick of the gripper
    def gripper(self, T_tool, poses, target, limit, dt):
        """Move the jaws toward the opening `target` (m) with the closing force `limit` (N) for one control tick.
        Returns what happened: [("cut", member id), ("hold", member id), ("release", member id)]; the cell removes a
        cut member's joint to what it grew on and keeps a held member's body on the tool."""
        tool = F.TOOL
        target = min(tool["open_max"], max(0.0, float(target)))
        limit = min(tool["force_max"], max(0.0, float(limit)))
        self.target, self.limit = target, limit
        out, captured, stalled = [], [], False
        w0 = self.opening
        if target < w0 - 1e-9:
            w1 = max(target, w0 - tool["jaw_speed"] * dt)
            blade = self.in_mouth(T_tool, poses, tool["blade_x"], w0, uncut=True)
            while True:
                meet = [(2.0 * self.member[mid]["r"], mid, t, y, z) for mid, t, y, z in blade
                        if not self.member[mid]["cut"] and 2.0 * self.member[mid]["r"] >= w1 - 1e-9]
                if not meet:
                    break
                d, mid, t, y, z = max(meet)
                m = self.member[mid]
                if limit >= m["need"]:
                    m["cut"] = True
                    self.margins["cut_y_mm"] = max(self.margins["cut_y_mm"], abs(y) * 1000.0)
                    self.margins["cut_z_mm"] = max(self.margins["cut_z_mm"], abs(z) * 1000.0)
                    if m["kind"] == "peduncle":
                        self.margins["cut_force_ratio"] = max(self.margins["cut_force_ratio"], limit / m["need"])
                    self._cut(mid, t, T_tool, poses)
                    out.append(("cut", mid))
                else:
                    w1, stalled = min(w0, d), True      # the jaws stall on it: the opening reads its diameter
                    break
            # the pad meets every member at its plane that the jaws have now closed down to
            for mid, t, y, z in self.in_mouth(T_tool, poses, tool["clamp_x"], w0):
                if mid not in self.held and self.member[mid]["kind"] != "stem" and 2.0 * self.member[mid]["r"] >= w1 - 5e-4:
                    captured.append(mid)
            self.opening = w1
        elif target > w0 + 1e-9:
            self.opening = min(target, w0 + tool["jaw_speed"] * dt)
        self.stalled = stalled
        self.force = limit if (stalled or (self.opening <= 1e-6 and target <= 1e-6)) else 0.0
        now = []
        for mid in self.held + captured:
            m = self.member[mid]
            if self.opening > 2.0 * m["r"] + 5e-4 or limit < tool["pad_force"]:
                continue                                # the jaws have opened past it, or the pad does not press
            if m["kind"] == "peduncle" and self.truss[m["truss"]].get("stub_side"):
                continue                                # the pad holds the stub on the plant, not the truss
            now.append(mid)
            if m["kind"] == "peduncle":
                tr = self.truss[m["truss"]]
                tr["pad_ratio"] = max(tr["pad_ratio"], limit / m["need"])
                if limit > F.CUT["crush"] * m["need"] and not tr["crushed"]:
                    tr["crushed"] = True
                    self.log("crushed", truss=m["truss"], force=round(limit, 1), cut_force=round(m["need"], 1))
        out += [("release", mid) for mid in self.held if mid not in now]
        out += [("hold", mid) for mid in now if mid not in self.held]
        self.held = now
        for tid, tr in self.truss.items():
            if tr["status"] != "on_plant":
                tr["status"] = "held" if any(self.member[mid].get("truss") == tid for mid in now) else "free"
        return out

    def _cut(self, mid, t, T_tool, poses):
        """A member is cut through at the fraction t of its length."""
        m = self.member[mid]
        if m["kind"] == "peduncle":
            tr = self.truss[m["truss"]]
            tr["status"], tr["cut_t"] = "free", self.t
            c = crossing(T_tool, poses[mid], m["len"], F.TOOL["clamp_x"])
            tr["stub_side"] = bool(c is None or c[0] < t)           # the pad sits on the plant's side of the cut
            self.log("cut", member=mid, truss=m["truss"], at=round(t, 3), force=round(self.limit, 1), need=round(m["need"], 1),
                     pad_on_truss=not tr["stub_side"])
        elif m["kind"] == "rachis":
            self.truss[m["truss"]]["parted"] = True
            self.log("parted", member=mid, truss=m["truss"])
        elif m["kind"] == "petiole":
            self.leaf[m["leaf"]]["cut"] = True
            self.log("leaf_cut", leaf=m["leaf"])
        elif m["kind"] == "stem":
            if not self.plant[m["plant"]]["lost"]:
                self.plant[m["plant"]]["lost"] = True
                self.log("plant_lost", plant=m["plant"], member=mid)

    # ------------------------------------------------------------------ what bruises a fruit
    def impacts(self, dv):
        """`dv`: {rachis body id: the change of its speed over the last control tick (m/s)}."""
        lim = F.DAMAGE["impact_dv"]
        for bid, v in dv.items():
            tr = self.truss[self.member[bid]["truss"]]
            tr["dv_max"] = max(tr["dv_max"], float(v))
            if v > lim and not tr["bruised"]:
                tr["bruised"] = True
                self.log("bruised", truss=self.member[bid]["truss"], dv=round(float(v), 3))

    def presses(self, force):
        """`force`: {rachis body id: the force the robot presses it with (N)}."""
        lim = F.DAMAGE["squeeze_force"]
        for bid, f in force.items():
            tr = self.truss[self.member[bid]["truss"]]
            tr["press_max"] = max(tr["press_max"], float(f))
            if f > lim and not tr["squeezed"]:
                tr["squeezed"] = True
                self.log("squeezed", truss=self.member[bid]["truss"], force=round(float(f), 1))

    # ------------------------------------------------------------------ the yield
    def fruit_centres(self, tid, poses):
        """A truss's fruit centres in W: [(xyz, r)]."""
        out = []
        for seg, local, r in self.truss[tid]["fruits"]:
            T = poses["%s.r%d" % (tid, seg)]
            out.append((T[:3, :3] @ local + T[:3, 3], r))
        return out

    @staticmethod
    def crate_of(fruits, y_cart):
        """The crate every fruit of a truss lies in (name), or None. `fruits` in W; the crates ride on the cart."""
        fz, wh = F.CART["crate_floor_z"], F.CART["crate_wall_h"]
        for name, cr in F.CART["crates"].items():
            (cx, cy), (sx, sy) = cr["center"], cr["size"]
            if all(abs(c[0] - cx) <= sx / 2 and abs(c[1] - y_cart - cy) <= sy / 2 and fz - 0.005 <= c[2] - r and c[2] <= fz + wh
                   for c, r in fruits):
                return name
        return None

    @staticmethod
    def lies_on(upper, lower):
        """True when a fruit of `upper` rests on a fruit of `lower` (two trusses' fruit centres in W): the two touch
        (their centres within `stack_touch` times the sum of their radii) and the upper one's centre is higher by
        more than `stack_dz` times that sum."""
        kz, kt = F.DAMAGE["stack_dz"], F.DAMAGE["stack_touch"]
        for c1, r1 in upper:
            for c2, r2 in lower:
                if c1[2] - c2[2] > kz * (r1 + r2) and float(np.linalg.norm(np.asarray(c1) - np.asarray(c2))) < kt * (r1 + r2):
                    return True
        return False

    def credit(self, poses, y_cart):
        """(yield, detail): the row's ready trusses that count over the row's ready trusses."""
        where, fruits = {}, {}
        for tid, tr in self.truss.items():
            if tr["status"] in ("free", "held"):
                fruits[tid] = self.fruit_centres(tid, poses)
                where[tid] = self.crate_of(fruits[tid], y_cart)
        # a truss that lies on another damages both, wherever the upper one's other fruits are
        stacked = set()
        for a in fruits:
            for b in fruits:
                if a != b and self.lies_on(fruits[a], fruits[b]):
                    stacked.update((a, b))
        detail, n, good = {}, 0, 0
        for tid, tr in self.truss.items():
            why = None
            if self.plant[tr["plant"]]["lost"]:
                why = "plant_lost"
            elif tr["status"] == "on_plant":
                why = "not_cut"
            elif tr["status"] == "held":
                why = "in_the_tool"
            elif tr["parted"]:
                why = "parted"
            elif tr["crushed"]:
                why = "crushed"
            elif tr["bruised"]:
                why = "bruised"
            elif tr["squeezed"]:
                why = "squeezed"
            elif where.get(tid) is None:
                why = "not_in_a_crate"
            elif tid in stacked:
                why = "stacked"
            detail[tid] = {"ready": tr["ready"], "counts": bool(tr["ready"] and why is None), "why": why,
                           "crate": where.get(tid), "dv_max": round(tr["dv_max"], 3), "press_max": round(tr["press_max"], 2),
                           "pad_ratio": round(tr["pad_ratio"], 3)}
            if tr["ready"]:
                n += 1
                good += int(why is None)
        return (good / float(n) if n else 0.0), detail

    # ------------------------------------------------------------------ the privileged view (the reference only)
    def truth(self, poses, y_cart):
        """Where everything is, in the cart frame C (the grader adds it to the reference's observations only)."""
        off = np.array([0.0, y_cart, 0.0])

        def seg(mid):
            T, L = poses[mid], self.member[mid]["len"]
            a = T[:3, 3] - off
            return [round(float(v), 5) for v in a], [round(float(v), 5) for v in a + L * T[:3, 2]]
        trusses = {}
        for p in self.row["plants"]:
            for t in p["trusses"]:
                tid, tr = t["id"], self.truss[t["id"]]
                a, b = seg(tid + ".p")
                trusses[tid] = {"plant": p["id"], "ready": tr["ready"], "status": tr["status"], "peduncle": [a, b],
                                "r": t["peduncle"]["r"], "need": round(self.member[tid + ".p"]["need"], 3),
                                "fruits": [[round(float(v), 5) for v in c - off] + [r] for c, r in self.fruit_centres(tid, poses)],
                                "damaged": bool(tr["bruised"] or tr["squeezed"] or tr["crushed"] or tr["parted"])}
        plants = {}
        for p in self.row["plants"]:
            pts = [seg("%s.s%d" % (p["id"], i)) for i in range(len(p["points"]) - 1)]
            leaves = []
            for lf in p["leaves"]:
                if self.leaf[lf["id"]]["cut"]:
                    continue
                T = poses[lf["id"]]
                a, b = seg(lf["id"])
                Lp = self.member[lf["id"]]["len"]
                # the blade: from the petiole's end along its own direction (body frame, fixed at build)
                d_local = frame_z(np.array(lf["petiole"]["b"]) - np.array(lf["petiole"]["a"])).T @ np.array(lf["blade"]["dir"])
                d = T[:3, :3] @ d_local
                e = T[:3, 3] + Lp * T[:3, 2] - off
                leaves.append({"id": lf["id"], "petiole": [a, b], "blade": [[round(float(v), 5) for v in e],
                               [round(float(v), 5) for v in e + lf["blade"]["length"] * d]], "width": lf["blade"]["width"]})
            plants[p["id"]] = {"lost": self.plant[p["id"]]["lost"], "stem": [pts[0][0]] + [q[1] for q in pts], "stem_r": p["stem_r"],
                               "leaves": leaves}
        return {"trusses": trusses, "plants": plants, "cart_y": round(float(y_cart), 5), "held": list(self.held),
                "opening": round(self.opening, 6), "toughness": self.row["mechanics"]["toughness"]}
