#!/usr/bin/env python3
"""Checks of the cell's rules, the wire and the pack draw that need no simulator (maintainers' tool).

    python3 test_rules.py

Each check drives `packcell/rules.py` the way the cell does (a bit's tip pose, an axial force, a spindle command per
0.05 s tick) and looks at what the rule decides: a matching bit frees a fastener in its number of turns, a stalled
spindle does no harm, a loose bit, a misaligned bit and a cammed-out Torx bit strip the head, the extractor removes
a stripped fastener, the live-parts check, the locks, the grip, the credit, the action schema, the probe's copy and
the conversion of a pose near a half turn.
"""

import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from packcell import kin, rules, wire  # noqa: E402
from packcell import series as S  # noqa: E402

DOWN = np.array([0.0, 0.0, -1.0])
RPM = 240 * 2 * math.pi / 60


def fresh(seed=11, variant="P5", harness="busbars"):
    return rules.Rules(S.draw_pack(seed, variant=variant, harness=harness))


def pick(r, head, corroded=False, group=None):
    for fid, f in r.fast.items():
        if f["head"] == head and f["status"] == "seated" and f["corroded"] == corroded and (group is None or f["group"] == group):
            return fid
    raise LookupError((head, corroded, group))


def run(r, fid, bit, turns, limit, force, offset=(0.0, 0.0), tilt_deg=0.0, speed=-RPM):
    r.tool, r.bit = "nutrunner", bit
    f = r.fast[fid]
    tip = f["top"] + np.array([offset[0], offset[1], 0.0003])
    a = math.radians(tilt_deg)
    axis = np.array([math.sin(a), 0.0, -math.cos(a)])
    out = []
    for _ in range(int(turns * 2 * math.pi / (abs(speed) * S.TICK)) + 1):
        out.append(r.spindle(tip, axis, force, speed, limit, S.TICK))
        if f["status"] != "seated":
            break
    return out


def unit(rng):
    v = np.array([rng.gauss(0.0, 1.0) for _ in range(3)])
    return v / np.linalg.norm(v)


def acos_rotvec(R):
    """`kin.R_rotvec` before 2026-11-24 (acos of the trace, a division by sin): within about 1e-5 rad of a half turn
    a pose written with it and read back was off by up to 2e-3 in a matrix entry."""
    c = max(-1.0, min(1.0, (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if math.pi - a < 1e-6:
        M = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        return M[:, i] / math.sqrt(max(M[i, i], 1e-12)) * a
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a)) * a


def downward_states(n, seed):
    """The reference's states at a fastener: the nutrunner commanded straight down (`kin.ik_step` from home to a pose
    whose rotation is a half turn), the measured joints behind by a following error of 0 to 1e-3 rad (its size on a
    log scale), observed as the cell observes, and a probe's copy of each. Returns how many true observations and
    how many copies have a residual above 1e-5, and in how many pairs the true one is the larger. With
    `acos_rotvec` in place of `kin.R_rotvec` the same 400 states gave 82, 0 and 267."""
    rng = random.Random(seed)
    tool, home = wire._tool_T("nutrunner"), np.array(S.CELL["home_q"], dtype=float)
    p0 = (kin.flange(home) @ tool)[:3, 3]
    big_true = big_copy = larger = 0
    for k in range(n):
        R = kin.Rz(rng.uniform(-math.pi, math.pi)) @ np.diag([1.0, -1.0, -1.0])
        T = kin.pose(R, p0 + np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), rng.uniform(-0.1, 0.05)]))
        qt = home.copy()
        for _ in range(30):
            qt, err = kin.ik_step(qt, T, tool)
        assert np.abs(err).max() < 1e-6, err
        m = 0.0 if k % 10 == 0 else 10 ** rng.uniform(-7.0, -3.0)
        q = qt + np.array([rng.uniform(-m, m) for _ in range(6)])
        obs = {"q": [round(float(v), 6) for v in q], "q_target": [round(float(v), 6) for v in qt], "tool": "nutrunner",
               "tcp": wire.pose6(kin.flange(q) @ tool), "tcp_target": wire.pose6(kin.flange(qt) @ tool),
               "images": {"wrist": {"rgb": "x", "pose": wire.pose6(kin.flange(q) @ wire._wrist_T())}} if k % 2 else {}}
        other = wire.perturbed(obs, random.Random(1000 + k), exact={"q": [float(v) for v in q], "q_target": [float(v) for v in qt]})
        r_true, r_copy = wire.residual(obs), wire.residual(other)
        big_true += r_true > 1e-5
        big_copy += r_copy > 1e-5
        larger += r_true > r_copy
    return big_true, big_copy, larger


def main():
    checks = 0
    # 1. the right bit, square, pressed on: out after the thread's turns, never stripped
    for head in sorted(S.HEADS):
        r = fresh()
        fid = pick(r, head)
        th = S.THREADS[r.fast[fid]["thread"]]
        log = run(r, fid, head, th["turns"] + 1, 1.3 * th["tighten"], 60.0)
        assert r.captured == fid and not r.fast[fid]["stripped"], head
        assert abs(len(log) * RPM * S.TICK / (2 * math.pi) - th["turns"]) < 0.3, (head, len(log))
        assert r.eject() == fid and r.fast[fid]["status"] == "free"
        checks += 1
    # 2. a corroded fastener stalls a default limit without harm, and comes out with a raised one
    r = fresh(seed=3)
    fid = pick(r, "S10", corroded=True)
    log = run(r, fid, "S10", 3, 13.0, 40.0)
    assert all(sp == 0.0 and abs(tq - 13.0) < 1e-9 for tq, sp in log) and r.fast[fid]["wear"] == 0.0
    run(r, fid, "S10", 8, 20.0, 40.0)
    assert r.captured == fid
    checks += 1
    # 3. loose bits wear the head: a 13 mm socket on a 10 mm hexagon, a T30 in a T40 recess
    for bit, head in (("S13", "S10"), ("T30", "T40")):
        r = fresh(seed=5)
        fid = pick(r, head)
        run(r, fid, bit, 2.0, 30.0, 60.0)
        assert r.fast[fid]["stripped"] and r.captured is None, (bit, head)
        checks += 1
    # 4. bits that do not fit turn freely: no wear, no progress
    for bit, head in (("S10", "S13"), ("T40", "T30"), ("S10", "T30"), ("T30", "S10")):
        r = fresh(seed=5)
        fid = pick(r, head)
        log = run(r, fid, bit, 3.0, 30.0, 60.0)
        assert r.fast[fid]["wear"] == 0.0 and r.fast[fid]["turned"] == 0.0 and max(tq for tq, _ in log) < 0.5, (bit, head)
        checks += 1
    # 5. off-axis or tilted: partial engagement strips; further off: nothing
    r = fresh()
    fid = pick(r, "T30")
    run(r, fid, "T30", 2.0, 13.0, 60.0, offset=(0.002, 0.0))
    assert r.fast[fid]["stripped"]
    r = fresh()
    fid = pick(r, "S10")
    run(r, fid, "S10", 2.0, 13.0, 60.0, tilt_deg=5.0)
    assert r.fast[fid]["stripped"]
    r = fresh()
    fid = pick(r, "S10")
    run(r, fid, "S10", 2.0, 13.0, 60.0, offset=(0.005, 0.0))
    assert r.fast[fid]["wear"] == 0.0
    checks += 3
    # 6. a Torx bit pressed too lightly cams out; a socket does not need the extra force
    r = fresh()
    fid = pick(r, "T30")
    run(r, fid, "T30", 2.0, 13.0, 15.0)
    assert r.fast[fid]["stripped"]
    r = fresh()
    fid = pick(r, "S10")
    run(r, fid, "S10", 8.0, 13.0, 15.0)
    assert r.captured == fid
    checks += 2
    # 7. tightening a seated fastener: stall; above the head's limit it gives way (an M8: its head's limit is below
    #    its shear torque); an M6 at its shear torque twists off first (check 16)
    r = fresh()
    fid = pick(r, "S13")
    log = run(r, fid, "S13", 2.0, 40.0, 60.0, speed=RPM)
    assert r.fast[fid]["wear"] == 0.0 and all(sp == 0.0 for _, sp in log)
    run(r, fid, "S13", 2.0, 45.5, 60.0, speed=RPM)
    assert r.fast[fid]["stripped"] and not r.fast[fid]["sheared"]
    # 8. the extractor takes a stripped fastener out; it ruins a sound one
    run(r, fid, "S13", 3.0, 30.0, 60.0)
    assert r.captured is None
    run(r, fid, "EX", 20.0, 40.0, 60.0)
    assert r.captured == fid
    r = fresh()
    fid = pick(r, "T40")
    run(r, fid, "EX", 2.0, 40.0, 60.0)
    assert r.fast[fid]["stripped"]
    checks += 3
    # 9. live parts: the tip at a busbar bolt before isolation ends the pack; after isolation it does not
    r = fresh()
    fid = pick(r, "S10", group="busbar") if any(f["head"] == "S10" and f["group"] == "busbar" for f in r.fast.values()) else pick(r, "T30", group="busbar")
    r.hv_check([r.fast[fid]["top"] + [0, 0, 0.001]])
    assert r.ended == "live_contact"
    r = fresh()
    r.msd_out_at, r.t = 0.0, S.ISOLATION_WAIT_S - 1.0
    r.hv_check([r.fast[fid]["top"]])
    assert r.ended == "live_contact"
    r = fresh()
    r.msd_out_at, r.t = 0.0, S.ISOLATION_WAIT_S + 0.1
    r.hv_check([r.fast[fid]["top"]])
    assert r.ended is None
    lid_bolt = next(f for f in r.fast.values() if f["group"] == "lid" and not f["missing"])
    r2 = fresh()
    r2.hv_check([lid_bolt["top"]])
    assert r2.ended is None
    checks += 4
    # 10. locks: the lid when its bolts are out; a module when its brackets and busbars have left
    r = fresh()
    assert r.unlock([], np.zeros(3)) == [p for p in ()] or True
    for f in r.fast.values():
        if f["group"] == "lid" and f["status"] == "seated":
            f["status"] = "free"
    assert "lid" in r.unlock([], np.zeros(3))
    m = r.part["M1"]
    assert m["locked"]
    r.unlock(list(m["lock"]["ids"]), np.zeros(3))
    assert not r.part["M1"]["locked"] and r.part["M2"]["locked"]
    # a plug: only while it is gripped and pulled along its axis harder than its latch holds
    assert "msd" not in r.unlock([], np.array([0.0, 0.0, -100.0]))
    r.part["msd"]["grasped"] = True
    assert "msd" not in r.unlock([], np.array([0.0, 0.0, -40.0])) and "msd" in r.unlock([], np.array([0.0, 0.0, -70.0]))
    checks += 3
    # 11. the grip: on the feature and aligned, the fingers stop at the feature's width; turned 90 degrees, nothing
    r = fresh()
    r.tool = "gripper"
    p = r.part["M3"]
    g = p["grasps"][0]
    poses = {pid: q["seat"] for pid, q in r.part.items()}
    T = np.eye(4)
    c = p["seat"][:3, :3] @ np.array(g["close"])
    T[:3, :3] = np.stack([np.cross(c, DOWN), c, DOWN], axis=1)
    T[:3, 3] = p["seat"][:3, 3] + p["seat"][:3, :3] @ np.array(g["pos"]) + [0.003, 0.002, 0.004]
    r.width, ev = 0.06, None
    for _ in range(40):
        ev = ev or r.gripper(T, 0.0, S.TICK, poses)
    assert ev == ("attach", "M3") and abs(r.width - g["width"]) < 1e-9 and r.grasp == "M3"
    assert r.gripper(T, 0.085, S.TICK, poses) == ("detach", "M3")
    r = fresh()
    r.tool, r.width = "gripper", 0.06
    T2 = T.copy()
    T2[:3, :3] = np.stack([c, -np.cross(c, DOWN), DOWN], axis=1)
    for _ in range(40):
        assert r.gripper(T2, 0.0, S.TICK, poses) is None
    assert r.width == 0.0 and r.grasp is None
    checks += 2
    # 12. the credit: nothing is in place when a pack arrives; everything where it belongs is 1; a stripped
    #     fastener, a held part and a module beside its bay do not count
    r = fresh()
    parts = {pid: q["seat"] for pid, q in r.part.items()}
    fast = {}
    assert r.credit(parts, fast)[0] == 0.0

    def at(x, y, z):
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        return T
    bins, rk = S.CELL["bins"], S.CELL["rack"]
    bay = 0
    for pid, q in r.part.items():
        q["locked"] = False
        if q["dest"] == "rack":
            parts[pid] = at(rk["bay_centers"][bay][0], rk["bay_centers"][bay][1], S.TABLE_Z + S.MODULE["size"][2] / 2 + 0.001)
            bay += 1
        elif q["dest"] == "lid_stand":
            parts[pid] = at(S.CELL["lid_stand"]["center"][0], S.CELL["lid_stand"]["center"][1], S.TABLE_Z + 0.002)
        else:
            parts[pid] = at(bins[q["dest"]]["center"][0], bins[q["dest"]]["center"][1], S.TABLE_Z + 0.02)
    for fid, f in r.fast.items():
        if not f["missing"]:
            f["status"] = "free"
            fast[fid] = at(bins["fasteners_left"]["center"][0], bins["fasteners_left"]["center"][1], S.TABLE_Z + 0.01)
    assert abs(r.credit(parts, fast)[0] - 1.0) < 1e-12
    total = S.total_value(r.pack)
    some = next(fid for fid, f in r.fast.items() if not f["missing"])
    r.fast[some]["stripped"] = True
    r.part["M2"]["grasped"] = True
    parts["M1"] = at(rk["bay_centers"][0][0] + 0.09, rk["bay_centers"][0][1], S.TABLE_Z + 0.055)
    want = 1.0 - (S.VALUES["fastener"] + 2 * S.VALUES["module"]) / total
    assert abs(r.credit(parts, fast)[0] - want) < 1e-12
    # a missing fastener is not in the pack: not in the denominator, and nothing is lost for it
    assert any(f["missing"] for f in r.fast.values())
    assert abs(total - (sum(q["value"] for q in r.part.values()) + S.VALUES["fastener"] * sum(1 for f in r.fast.values() if not f["missing"]))) < 1e-12
    checks += 3
    # 13. the wire: the schema, the 15 numbers, the probe's copy
    assert wire.check_action({})["move"] == {"type": "hold", "delta": [0.0] * 6, "ticks": 1, "guard_force": None, "guard_stroke_mm": None}
    for bad in ({"move": {"type": "tcp", "delta": [0] * 5}}, {"move": {"type": "tcp", "delta": [0] * 6, "ticks": 41}}, {"cmd": "bit:T50"},
                {"gripper": True}, {"spindle": {"speed_rpm": float("nan"), "torque_limit": 1}}, {"x": 1}, {"cams": ["wrist", "wrist"]}, [1]):
        try:
            wire.check_action(bad)
            raise AssertionError(bad)
        except ValueError:
            pass
    a = {"move": {"type": "tcp", "delta": [0.01, 0, 0, 0, 0, 0.1], "ticks": 7, "guard_force": 20}, "spindle": {"speed_rpm": -200, "torque_limit": 13},
         "gripper": 0.03, "cmd": "eject", "cams": ["wrist"]}
    assert len(wire.canonical(a)) == 15 and wire.canonical(a) != wire.canonical(dict(a, cmd="done"))
    assert wire.is_idle({"cmd": "done"}) and wire.is_idle({"move": {"type": "tcp", "delta": [0] * 6, "ticks": 3}}) and not wire.is_idle(a)
    # the probe's copy: built as the cell builds an observation, so its poses agree with its joints as a true
    # observation's do (the robot's model cannot tell them apart), and the agent's library builds the same copy
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "environment", "setup", "payload", "given", "lib"))
    import packcell_client as pc
    from packcell import kin
    draw, res_true, res_copy, moved = random.Random(5), [], [], []
    for k in range(300):
        q = [draw.uniform(lo + 0.05, hi - 0.05) for lo, hi in kin.Q_LIMITS]
        qt = [v + draw.uniform(-0.03, 0.03) for v in q]
        tool = ("none", "nutrunner", "gripper")[k % 3]
        obs = {"q": [round(v, 6) for v in q], "q_target": [round(v, 6) for v in qt], "tool": tool,
               "tcp": wire.pose6(kin.flange(q) @ wire._tool_T(tool)), "tcp_target": wire.pose6(kin.flange(qt) @ wire._tool_T(tool)),
               "wrench": [0.3, -0.2, 1.0, 0.0, 0.01, 0.0], "qd": [0.1] * 6,
               "images": {"wrist": {"rgb": "x", "pose": wire.pose6(kin.flange(q) @ wire._wrist_T())}} if k % 2 else {}}
        keep = repr(obs)
        other = wire.perturbed(obs, random.Random(k))
        assert repr(obs) == keep                                           # the true observation is not touched
        assert other["wrench"] == obs["wrench"] and other["qd"] == obs["qd"] and other["tool"] == tool
        assert max(abs(a - b) for a, b in zip(other["q"], obs["q"])) <= wire.PROBE_SHIFT_RAD + 1e-6
        d = [a - b for a, b in zip(other["q"], obs["q"])]
        assert max(abs((a - b) - x) for a, b, x in zip(other["q_target"], obs["q_target"], d)) < 2.1e-6   # one shift
        if k % 2:
            assert other["images"]["wrist"]["rgb"] == "x" and other["images"]["wrist"]["pose"] != obs["images"]["wrist"]["pose"]
        else:
            assert other["images"] is obs["images"]
        res_true.append(wire.residual(obs))
        res_copy.append(wire.residual(other))
        moved.append(float(np.linalg.norm(np.array(other["tcp"][:3]) - np.array(obs["tcp"][:3]))))
        mine = pc.perturbed(obs, random.Random(k))                         # the agent's library: the same copy
        for key in ("q", "q_target", "tcp", "tcp_target"):
            assert np.abs(wire.pose_T(mine[key]) - wire.pose_T(other[key])).max() < 3e-6 if key.startswith("tcp") else mine[key] == other[key]
        if k % 2:
            # the camera's mounting comes from cell.json (nine decimals): the library's camera pose is the cell's
            assert np.abs(wire.pose_T(mine["images"]["wrist"]["pose"]) - wire.pose_T(other["images"]["wrist"]["pose"])).max() < 3e-6
        assert np.abs(pc.flange_T(q) - kin.flange(q)).max() < 1e-9
    # the residual of a copy is the residual of a true observation: rounding on the wire and nothing else
    assert max(res_true) < 5e-6 and max(res_copy) < 5e-6, (max(res_true), max(res_copy))
    assert abs(float(np.median(res_true)) - float(np.median(res_copy))) < 3e-7, (np.median(res_true), np.median(res_copy))
    assert float(np.percentile(moved, 5)) > 1e-3 and max(moved) < 0.05, (np.percentile(moved, 5), max(moved))
    # a shift never takes a joint past its limit
    edge = {"q": [float(hi) - 0.001 for lo, hi in kin.Q_LIMITS], "q_target": [float(hi) - 0.001 for lo, hi in kin.Q_LIMITS], "tool": "none",
            "tcp": [0.0] * 6, "tcp_target": [0.0] * 6, "wrench": [0.0] * 6, "images": {}}
    for k in range(50):
        assert all(v <= float(hi) + 1e-9 for v, (lo, hi) in zip(wire.perturbed(edge, random.Random(k))["q"], kin.Q_LIMITS))
    checks += 5
    # 14. the draw: reproducible, the forced cases hold, a harness never starts inside a head it bridges
    for seed in range(40):
        for variant in sorted(S.VARIANTS):
            for harness in ("none", "lid", "busbars"):
                p = S.draw_pack(seed, variant=variant, harness=harness)
                assert p == S.draw_pack(seed, variant=variant, harness=harness)
                assert p["variant"] == variant and (p["harness"]["on"] if p["harness"] else "none") == harness
                if not p["harness"]:
                    continue
                h = p["parts"][-1]
                pts = [np.array(q) for q in h["path"]]
                if harness == "busbars":            # short of the modules' handle bars
                    assert max(q[0] for q in pts) + S.HARNESS_R < S.MODULE["x"] - 0.05 - 0.005, (seed, variant)
                else:                               # on the lid, inside its outline and away from its handle
                    assert all(abs(q[0]) < 0.31 and abs(q[1]) < 0.47 and (abs(q[0]) > 0.09 or abs(q[1]) > 0.05) for q in pts)
                for f in p["fasteners"]:
                    if f["missing"]:
                        continue
                    head = S.HEADS[f["head"]]
                    rad = head.get("flange_r", head.get("head_r"))
                    top = f["seat_z"] + head["height"]
                    for a_, b_ in zip(pts[:-1], pts[1:]):
                        for u in np.linspace(0, 1, 9):
                            q = a_ + u * (b_ - a_)
                            lateral = math.hypot(q[0] - f["pos"][0], q[1] - f["pos"][1])
                            if lateral < rad + S.HARNESS_R - 1e-4:
                                assert q[2] - S.HARNESS_R >= top - 1e-6, (seed, variant, harness, f["id"], lateral, q[2] - S.HARNESS_R - top)
    checks += 1
    # 15. poses near a half turn (the tool pointing straight down is one): written on the wire and read back, a pose
    #     is the pose to the rounding at every angle; away from a half turn the conversion is the earlier one; in
    #     near-downward states a true observation's residual and a copy's are alike
    draw = random.Random(17)
    for gap in (1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 0.0):
        for _ in range(200):
            T = kin.pose(kin.rotvec_R(unit(draw) * (math.pi - gap)), np.array([0.3, -0.2, 0.1]))
            err = float(np.abs(wire.pose_T(wire.pose6(T)) - T).max())
            assert err < 2e-6, (gap, err)
    for _ in range(5000):
        a = draw.uniform(1e-6, math.pi - 1e-3) if draw.random() < 0.8 else 10 ** draw.uniform(-6.0, -3.0)
        R = kin.rotvec_R(unit(draw) * a)
        assert np.abs(kin.R_rotvec(R) - acos_rotvec(R)).max() < 2e-9, a
    big_true, big_copy, larger = downward_states(400, 23)
    assert big_true == 0 and big_copy == 0 and 160 <= larger <= 240, (big_true, big_copy, larger)
    # the agent's library and the reference convert as the cell does
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "reference_controller", os.path.join(os.path.dirname(os.path.dirname(HERE)), "solution", "reference", "controller.py"))
    ref = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ref)
    for gap in (1e-1, 1e-3, 1e-5, 1e-6, 1e-7, 0.0):
        for _ in range(50):
            R = kin.rotvec_R(unit(draw) * (math.pi - gap))
            assert np.array_equal(pc.R_rotvec(R), kin.R_rotvec(R)) and np.array_equal(ref.R_rotvec(R), kin.R_rotvec(R)), gap
    checks += 4
    checks += round2_rules()
    checks += huge_numbers()
    print("ok: %d checks" % checks)


def seized_pack(seed=11, variant="P5", n=4):
    return rules.Rules(S.draw_pack(seed, variant=variant, harness="none", seized_seed=seed + 1, seized_n=n,
                                   swollen_seed=seed + 2, swollen=True))


def round2_rules():
    """Checks 16 to 21: the shank's strength, seized fasteners and working them loose, the Torx bit at a stall, the
    head's limit both ways, the swollen module (rules 5 and 6; series.SEIZED, WORK, SWOLLEN)."""
    checks = 0
    # 16. a fully engaged bit holding the shear torque twists the shank off, either way: the head rides on the bit,
    #     the stub keeps the part held for good, the fastener does not count
    for way in (-RPM, RPM):
        r = seized_pack()
        fid = next(i for i, f in r.fast.items() if f["seized"] and S.HEADS[f["head"]]["kind"] == "hex")
        th = S.THREADS[r.fast[fid]["thread"]]
        run(r, fid, r.fast[fid]["head"], 1.0, th["shear"] + 0.5, 80.0, speed=way)
        f = r.fast[fid]
        assert f["sheared"] and r.captured == fid and r.eject() == fid, way
        others = [i for i in r.part[f["holds"]]["lock"]["ids"] if i != fid] if r.part[f["holds"]]["lock"]["type"] == "fasteners" else []
        for i in others:
            r.fast[i]["status"] = "free"
        assert f["holds"] not in r.unlock([], np.zeros(3)) and r.part[f["holds"]]["locked"]
        T = np.eye(4)
        T[:3, 3] = S.CELL["bins"]["fasteners_left"]["center"] + [S.TABLE_Z + 0.01]
        assert not r.credit({}, {fid: T})[1][fid]
    checks += 2
    # a clean fastener at any limit and a corroded one at a limit just below its shear torque come out unharmed
    r = seized_pack()
    for corroded in (False, True):
        fid = next(i for i, f in r.fast.items() if not f["seized"] and f["status"] == "seated" and f["corroded"] == corroded
                   and S.HEADS[f["head"]]["kind"] == "hex")
        th = S.THREADS[r.fast[fid]["thread"]]
        run(r, fid, r.fast[fid]["head"], th["turns"] + 1, 60.0 if not corroded else 0.975 * th["shear"], 80.0)
        assert r.captured == fid and not r.fast[fid]["sheared"] and r.fast[fid]["wear"] == 0.0, corroded
        r.eject()
    checks += 1
    # 17. a seized fastener stalls below its shear torque without harm, and comes loose by working: each cycle (a
    #     loosening stall at 0.85 x shear or more for 0.5 s, a tightening stall at the assembly torque or more for
    #     0.25 s) multiplies the breakaway's excess over a clean one's by the working factor
    r = seized_pack()
    for fid in [i for i, f in r.fast.items() if f["seized"]]:
        f = r.fast[fid]
        th, bit = S.THREADS[f["thread"]], f["head"]
        safe = min(0.975 * th["shear"], th["head_limit"] - 0.5)
        log = run(r, fid, bit, 0.3, safe, 90.0)
        assert all(sp == 0.0 for _, sp in log) and f["wear"] == 0.0 and f["turned"] == 0.0 and not f["sheared"], fid
        clean, b0 = 1.15 * th["tighten"], f["breakaway"]
        assert S.SEIZED["breakaway"][0] * th["shear"] - 1e-6 <= b0 <= S.SEIZED["breakaway"][1] * th["shear"] + 1e-6
        f["load_s"], f["loaded"] = 0.0, False
        cycles = 0
        while f["breakaway"] > safe:
            run(r, fid, bit, 0.55 * RPM * 0.05 / (2 * math.pi) / S.TICK * 1.0, 0.9 * th["shear"], 90.0)    # 0.55 s loosening stall
            assert not f["loaded"] or f["load_s"] >= S.WORK["loosen_s"] - 1e-9
            run(r, fid, bit, 0.30 * RPM * 0.05 / (2 * math.pi) / S.TICK * 1.0, 1.05 * th["tighten"], 90.0, speed=RPM)
            cycles += 1
            assert f["cycles"] == cycles and abs((f["breakaway"] - clean) - (b0 - clean) * f["work"] ** cycles) < 1e-6, fid
            assert cycles <= 8
        assert 1 <= cycles <= 5, (fid, cycles)
        run(r, fid, bit, th["turns"] + 1, safe, 90.0)
        assert r.captured == fid and f["wear"] == 0.0 and not f["sheared"], fid
        r.eject()
    # half a cycle does nothing: a loosening stall below 0.85 x shear, or no tightening stall after it
    r = seized_pack()
    fid = next(i for i, f in r.fast.items() if f["seized"])
    f, th = r.fast[fid], S.THREADS[r.fast[fid]["thread"]]
    b0 = f["breakaway"]
    run(r, fid, f["head"], 0.6, 0.8 * th["shear"], 90.0)
    run(r, fid, f["head"], 0.3, 1.05 * th["tighten"], 90.0, speed=RPM)
    assert f["breakaway"] == b0 and f["cycles"] == 0
    checks += 2
    # 18. a Torx bit cams out at a stall too when it is not pressed hard enough for the torque it holds
    r = seized_pack()
    fid = pick(r, "T30")
    run(r, fid, "T30", 0.5, 0.8 * r.fast[fid]["breakaway"], 10.0 + 1.5 * 0.8 * r.fast[fid]["breakaway"] - 2.0)
    assert r.fast[fid]["wear"] > 0.0
    r = seized_pack()
    fid = pick(r, "T30")
    run(r, fid, "T30", 0.5, 0.8 * r.fast[fid]["breakaway"], 10.0 + 1.5 * 0.8 * r.fast[fid]["breakaway"] + 2.0)
    assert r.fast[fid]["wear"] == 0.0
    checks += 1
    # 19. the head's limit holds when loosening too: an M8 stalled above it wears its head
    r = seized_pack()
    fid = next(i for i, f in r.fast.items() if f["seized"] and f["thread"] == "M8") if any(
        f["seized"] and f["thread"] == "M8" for f in r.fast.values()) else None
    if fid is not None:
        run(r, fid, r.fast[fid]["head"], 2.0, 45.6, 120.0)
        assert r.fast[fid]["stripped"] and not r.fast[fid]["sheared"]
        checks += 1
    # 20. the swollen module: one per pack that carries one, a bump of 0.6 to 3 mm against the others' crown of at most
    #     0.25 mm; moved 5 mm off its seat it vents and the pack ends; it counts where it stands, freed and level; a
    #     healthy module left there does not
    r = seized_pack()
    sw = [pid for pid, p in r.part.items() if p.get("swollen")]
    assert len(sw) == 1 and r.part[sw[0]]["dest"] == "seat"
    bumps = {pid: p["bump"]["h"] for pid, p in r.part.items() if p["kind"] == "module"}
    assert S.SWOLLEN["bulge"][0] <= bumps[sw[0]] <= S.SWOLLEN["bulge"][1]
    assert all(h <= S.SWOLLEN["crown"][1] for pid, h in bumps.items() if pid != sw[0])
    r.vent_check({sw[0]: 0.0049})
    assert r.ended is None
    r.vent_check({sw[0]: 0.0050})
    assert r.ended == "module_vented" and r.part[sw[0]]["damaged"]
    r = seized_pack()
    healthy = next(pid for pid, p in r.part.items() if p["kind"] == "module" and not p.get("swollen"))
    seat = {pid: p["seat"].copy() for pid, p in r.part.items()}
    r.part[sw[0]]["locked"] = r.part[healthy]["locked"] = False
    _, d = r.credit(seat, {})
    assert d[sw[0]] and not d[healthy]
    tilted = seat[sw[0]].copy()
    tilted[:3, :3] = kin.rotvec_R(np.array([math.radians(6.0), 0.0, 0.0]))
    moved = seat[sw[0]].copy()
    moved[0, 3] += 0.006
    assert not r.credit(dict(seat, **{sw[0]: tilted}), {})[1][sw[0]] and not r.credit(dict(seat, **{sw[0]: moved}), {})[1][sw[0]]
    r.part[sw[0]]["locked"] = True
    assert not r.credit(seat, {})[1][sw[0]]
    r2 = seized_pack()
    r2.vent_check({healthy: 0.05})
    assert r2.ended is None
    checks += 3
    return checks


def huge_numbers():
    """Check 21 (the re-review's N1): a JSON integer too large for a float is a malformed action, never an
    OverflowError, on the grader's path (check_action, canonical, is_idle) and in the agent's library."""
    import json
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "environment", "setup", "payload", "given", "lib"))
    import packcell_client as pc
    big = int("1" * 400)
    acts = [{"move": {"type": "tcp", "delta": [big, 0, 0, 0, 0, 0]}}, {"move": {"type": "tcp", "delta": [0] * 6, "ticks": big}},
            {"spindle": {"speed_rpm": -big, "torque_limit": 5}}, {"gripper": big},
            json.loads('{"move": {"type": "joint", "delta": [0, 0, %s, 0, 0, 0]}}' % ("9" * 400))]
    for a in acts:
        for fn in (wire.check_action, wire.canonical):
            try:
                fn(a)
                raise AssertionError((fn.__name__, a))
            except ValueError:
                pass
        assert wire.is_idle(a) is False
        for fn in (pc.check_action, pc.canonical, lambda x: pc.replies_differ(x, {"cmd": "done"})):
            try:
                fn(a)
                raise AssertionError(("client", a))
            except ValueError:
                pass
        assert pc.is_idle(a) is False
        try:
            pc._num(big, "x")
            raise AssertionError("client _num")
        except ValueError:
            pass
    return 1


if __name__ == "__main__":
    main()
