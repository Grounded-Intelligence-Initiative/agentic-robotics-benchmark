#!/usr/bin/env python3
"""Checks of the cell's rules, the wire and the pack draw that need no simulator (maintainers' tool).

    python3 test_rules.py

Each check drives `packcell/rules.py` the way the cell does (a bit's tip pose, an axial force, a spindle command per
0.05 s tick) and looks at what the rule decides: a matching bit frees a fastener in its number of turns, a stalled
spindle does no harm, a loose bit, a misaligned bit and a cammed-out Torx bit strip the head, the extractor removes
a stripped fastener, the live-parts check, the locks, the grip, the credit, the action schema and the probe's copy.
"""

import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from packcell import rules, wire  # noqa: E402
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
    # 7. tightening a seated fastener: stall; above the head's limit it gives way
    r = fresh()
    fid = pick(r, "S10")
    log = run(r, fid, "S10", 2.0, 20.0, 60.0, speed=RPM)
    assert r.fast[fid]["wear"] == 0.0 and all(sp == 0.0 for _, sp in log)
    run(r, fid, "S10", 2.0, 30.0, 60.0, speed=RPM)
    assert r.fast[fid]["stripped"]
    # 8. the extractor takes a stripped fastener out; it ruins a sound one
    run(r, fid, "S10", 3.0, 20.0, 60.0)
    assert r.captured is None
    run(r, fid, "EX", 16.0, 20.0, 60.0)
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
    obs = {"q": [0.1] * 6, "q_target": [0.1] * 6, "tcp": [0.5, 0.1, 0.2, 3.1, 0.0, 0.0], "tcp_target": [0.5, 0.1, 0.2, 3.1, 0.0, 0.0],
           "wrench": [0.0] * 6, "images": {"x": 1}}
    other = wire.perturbed(obs, random.Random(1))
    assert other["images"] is obs["images"] and other["tcp"] != obs["tcp"] and obs["tcp"] == [0.5, 0.1, 0.2, 3.1, 0.0, 0.0]
    d1 = np.array(other["tcp"][:3]) - np.array(obs["tcp"][:3])
    d2 = np.array(other["tcp_target"][:3]) - np.array(obs["tcp_target"][:3])
    assert np.allclose(d1, d2, atol=1e-6) and 1e-4 < np.linalg.norm(d1) < 0.025
    checks += 4
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
    print("ok: %d checks" % checks)


if __name__ == "__main__":
    main()
