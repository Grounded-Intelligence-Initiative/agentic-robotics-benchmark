#!/usr/bin/env python3
"""Checks of the cell's rules, the wire and the row draw that need no simulator (maintainers' tool).

    python3 test_rules.py

Each check drives `rowcell/rules.py` the way the cell does (the tool's pose, every member's pose, a gripper command
per 0.05 s tick) and looks at what the rule decides: the jaws stall on a peduncle at its diameter, cut it with its
cutting force and hold the truss when the pad is on the truss's side, crush it above the crush ratio, cut a stem and
lose the plant, part a rachis, drop a leaf; what bruises a fruit; what counts at the end (in a crate, whole, in one
layer); the row draw's promises; the action schema and the probe's copy.
"""

import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from rowcell import farm as F  # noqa: E402
from rowcell import kin, rules, wire  # noqa: E402

OPEN = F.TOOL["open_max"]


def fresh(seed=3, cultivar="Rossa"):
    row = F.draw_row(seed, cultivar, light_seed=seed, mech_seed=seed)
    return row, rules.Rules(row), rules.rest_poses(row)


def truss_of(row, tid):
    return [t for p in row["plants"] for t in p["trusses"] if t["id"] == tid][0]


def tool_at(row, tid, frac=None, alpha=0.0, flip=False, shift=(0.0, 0.0, 0.0)):
    """The tool frame (W) with its centre point on the truss's peduncle at `frac` of its length, x along the
    peduncle toward the stem (away from it with `flip`), displaced by `shift` in the tool frame."""
    t = truss_of(row, tid)
    R = F.approach_axes(t, alpha)
    if flip:
        R = R @ np.diag([-1.0, -1.0, 1.0])
    a, b = np.array(t["peduncle"]["a"]), np.array(t["peduncle"]["b"])
    p = a + (F.cut_fraction(t) if frac is None else frac) * (b - a)
    return kin.pose(R, p + R @ np.array(shift))


def close(r, T, poses, force, ticks=30, target=0.0):
    out = []
    for _ in range(ticks):
        out += r.gripper(T, poses, target, force, F.TICK)
    return out


def lay(row, r, poses, tid, crate="front", lane=1, dz=0.0, dy=0.0):
    """Poses with truss `tid` lying in a crate lane (a rigid turn of its rest shape: the rachis along the lane)."""
    t = truss_of(row, tid)
    slot = [s for s in F.crate_slots() if s["crate"] == crate][lane]
    a = np.array(t["peduncle"]["b"])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])        # down -> -y... the truss lies along y
    ids = [tid + ".p"] + ["%s.r%d" % (tid, k) for k in range(3)]
    out = dict(poses)
    rest = rules.rest_poses(row)
    for mid in ids:
        T = rest[mid].copy()
        T[:3, :3] = Rx @ T[:3, :3]
        T[:3, 3] = Rx @ (T[:3, 3] - a)
        out[mid] = T
    fr = [(Tc, rr) for Tc, rr in ((out["%s.r%d" % (tid, s)][:3, :3] @ loc + out["%s.r%d" % (tid, s)][:3, 3], rr) for s, loc, rr in r.truss[tid]["fruits"])]
    lo = min(c[2] - rr for c, rr in fr)
    cy = 0.5 * (min(c[1] for c, _ in fr) + max(c[1] for c, _ in fr))
    cx = 0.5 * (min(c[0] for c, _ in fr) + max(c[0] for c, _ in fr))
    move = np.array([slot["x"] - cx, 0.5 * (slot["y0"] + slot["y1"]) - cy + dy, F.CART["crate_floor_z"] - lo + dz])
    for mid in ids:
        out[mid] = out[mid].copy()
        out[mid][:3, 3] += move
    return out


def harvested(r, tid):
    r.member[tid + ".p"]["cut"] = True
    r.truss[tid]["status"] = "free"


def main():
    n = 0
    # ------------------------------------------------------------------ 1. the row draw's promises
    for seed in range(1, 21):
        for cultivar in ("Rossa", "Ambra"):
            row = F.draw_row(seed, cultivar, light_seed=seed + 100, mech_seed=seed + 200)
            again = F.draw_row(seed, cultivar, light_seed=seed + 100, mech_seed=seed + 200)
            assert row == again, "a row is a function of its seeds"
            c = F.CULTIVARS[cultivar]
            ts = [t for p in row["plants"] for t in p["trusses"]]
            assert len(row["plants"]) == F.ROW["plants"] and len(ts) == F.ROW["plants"] * F.ROW["trusses"]
            assert F.ROW["ready"][0] <= len(row["ready"]) <= F.ROW["ready"][1], (seed, row["ready"])
            for p in row["plants"]:
                flags = [t["ready"] for t in p["trusses"]]
                assert flags == sorted(flags, reverse=True), "on a plant the lowest trusses ripen first"
                for t in p["trusses"]:
                    assert t["ready"] == F.is_ready(cultivar, t["stages"])
                    assert c["fruits"][0] <= len(t["fruits"]) <= c["fruits"][1]
                    assert c["peduncle_len"][0] <= t["peduncle"]["len"] <= c["peduncle_len"][1]
                    assert c["peduncle_r"][0] <= t["peduncle"]["r"] <= c["peduncle_r"][1]
                    for f in t["fruits"]:
                        assert abs(f["ripeness"] - f["stage"]) <= 0.35 + 1e-9 and 1 <= f["stage"] <= 6
                        want = np.array(F.chart_color(cultivar, f["ripeness"]))
                        assert np.all(np.abs(np.array(f["albedo"]) / want - 1.0) <= 0.051), "a fruit's colour is its chart colour within 5 %"
                    if t["ready"]:
                        assert F.clear_alphas(t, row["plants"]), "every ready truss has a clear straight approach"
            m = row["mechanics"]
            assert F.CUT["toughness"][0] <= m["toughness"] <= F.CUT["toughness"][1] and F.ROW["slip"][0] <= m["slip"] <= F.ROW["slip"][1]
            assert c["stem_k"][0] <= m["stem_k"] <= c["stem_k"][1] and c["string_k"][0] <= m["string_k"] <= c["string_k"][1]
            lt = row["light"]
            assert F.ROW["light_gain"][0] <= lt["gain"] <= F.ROW["light_gain"][1] and F.ROW["light_g"][0] <= lt["rgb"][1] <= F.ROW["light_g"][1]
            n += 1
    # readiness is the cultivar's rule
    assert F.is_ready("Rossa", [6, 5, 4, 4, 3]) and not F.is_ready("Rossa", [6, 5, 4, 3, 3])
    assert F.is_ready("Ambra", [6, 5, 5, 4, 3]) and not F.is_ready("Ambra", [6, 5, 4, 4, 3])
    print("1. row draws: %d rows keep the rule's promises" % n)

    # ------------------------------------------------------------------ 2. the caliper, the cut, the hold
    row, r, poses = fresh()
    tid = row["ready"][0]
    pm = r.member[tid + ".p"]
    T = tool_at(row, tid)
    ev = close(r, T, poses, 4.0)
    assert abs(r.opening - 2 * pm["r"]) < 1e-9 and r.stalled and r.force == 4.0, "the jaws stall on the peduncle at its diameter"
    assert ev == [("hold", tid + ".p")] and r.truss[tid]["status"] == "on_plant" and not pm["cut"]
    ev = close(r, T, poses, pm["need"] * 0.98)
    assert not pm["cut"] and ev == [], "a force short of the cutting force cuts nothing"
    ev = close(r, T, poses, pm["need"] * 1.05)
    assert ("cut", tid + ".p") in ev and pm["cut"] and r.truss[tid]["status"] == "held" and r.opening == 0.0
    assert not r.truss[tid]["crushed"] and r.held == [tid + ".p"]
    ev = close(r, T, poses, 0.0, ticks=20, target=OPEN)
    assert ("release", tid + ".p") in ev and r.truss[tid]["status"] == "free" and r.opening == OPEN
    print("2. caliper %.2f mm, cut at %.1f N, held, released" % (2000 * pm["r"], pm["need"]))

    # the pad on the plant's side of the cut: the truss falls
    row, r, poses = fresh()
    T = tool_at(row, tid, flip=True)
    ev = close(r, T, poses, pm["need"] * 1.2)
    assert ("cut", tid + ".p") in ev and r.truss[tid]["status"] == "free" and r.held == [], "the pad holds the stub, the truss falls"
    # the crush
    row, r, poses = fresh()
    T = tool_at(row, tid)
    close(r, T, poses, pm["need"] * F.CUT["crush"] * 1.01)
    assert r.truss[tid]["crushed"] and r.truss[tid]["status"] == "held"
    row, r, poses = fresh()
    close(r, T, poses, pm["need"] * F.CUT["crush"] * 0.99)
    assert not r.truss[tid]["crushed"]
    close(r, T, poses, pm["need"] * F.CUT["crush"] * 1.01, ticks=2)
    assert r.truss[tid]["crushed"], "a held peduncle pressed harder later is crushed too"
    # a pad that does not press lets go
    row, r, poses = fresh()
    close(r, T, poses, pm["need"] * 1.2)
    ev = close(r, T, poses, F.TOOL["pad_force"] * 0.5, ticks=2)
    assert ("release", tid + ".p") in ev and r.truss[tid]["status"] == "free"
    print("3. stub side, crush at %.2f times the cutting force, a slack pad lets go" % F.CUT["crush"])

    # ------------------------------------------------------------------ 4. the mouth's extent
    row, r, poses = fresh()
    half = OPEN / 2
    z0, z1 = F.TOOL["mouth_z"][0] - F.TOOL["tcp_z"], F.TOOL["mouth_z"][1] - F.TOOL["tcp_z"]
    # (the tool moved by s along its axis has the peduncle at -s in its own frame)
    for shift, inside in (((0, half - 0.0005, 0), True), ((0, half + 0.0005, 0), False), ((0, 0, -z0 - 0.0005), True), ((0, 0, -z0 + 0.0005), False),
                          ((0, 0, -z1 + 0.0005), True), ((0, 0, -z1 - 0.0005), False)):
        T = tool_at(row, tid, shift=shift)
        got = [m for m, _t, _y, _z in r.in_mouth(T, poses, F.TOOL["blade_x"], OPEN)]
        assert (tid + ".p" in got) == inside, (shift, inside, got)
    # a member that comes into a closed mouth is neither cut nor held
    row, r, poses = fresh()
    far = tool_at(row, tid, shift=(0, 0, -0.10))
    close(r, far, poses, 500.0)
    assert r.opening == 0.0
    ev = close(r, tool_at(row, tid), poses, 500.0, ticks=5)
    assert ev == [] and not r.member[tid + ".p"]["cut"] and r.held == []
    print("4. the mouth: +-%.0f mm between the jaws, %.0f mm deep; a closed mouth takes nothing" % (half * 1000, (z1 - z0) * 1000))

    # ------------------------------------------------------------------ 5. what else the blade meets
    row, r, poses = fresh()
    t = truss_of(row, tid)
    p = [q for q in row["plants"] if q["id"] == t["plant"]][0]
    # the stem, taken for a peduncle: tool x along the stem at the truss's node
    seg = [i for i in range(len(p["points"]) - 1) if p["points"][i][2] <= t["node"][2] <= p["points"][i + 1][2]][0]
    sa, sb = np.array(p["points"][seg]), np.array(p["points"][seg + 1])
    x = (sb - sa) / np.linalg.norm(sb - sa)
    z = np.cross(x, [0.0, 1.0, 0.0])
    z /= np.linalg.norm(z)
    Ts = kin.pose(np.stack([x, np.cross(z, x), z], axis=1), sa + 0.5 * (sb - sa))
    sm = r.member["%s.s%d" % (p["id"], seg)]
    close(r, Ts, poses, sm["need"] * 0.9)
    assert abs(r.opening - 2 * sm["r"]) < 1e-9 and not r.plant[p["id"]]["lost"], "short of its cutting force the jaws stall on a stem"
    assert r.held == [], "a stem is not held"
    close(r, Ts, poses, sm["need"] * 1.05)
    assert r.plant[p["id"]]["lost"] and sm["cut"]
    for q in p["trusses"]:
        harvested(r, q["id"])
    credit, detail = r.credit(lay(row, r, poses, p["trusses"][0]["id"]), 0.0)
    assert all(detail[q["id"]]["why"] == "plant_lost" for q in p["trusses"]), "a lost plant's trusses do not count"
    assert sm["need"] > 2.5 * max(r.member[q["id"] + ".p"]["need"] for q in p["trusses"]), "a stem takes far more force than a peduncle"
    # a rachis segment
    row, r, poses = fresh()
    ra, rb = np.array(t["rachis"][1]["a"]), np.array(t["rachis"][1]["b"])
    x = (ra - rb) / np.linalg.norm(ra - rb)
    z = np.cross(x, [0.0, 1.0, 0.0])
    z /= np.linalg.norm(z)
    Tr = kin.pose(np.stack([x, np.cross(z, x), z], axis=1), ra + 0.12 * (rb - ra))
    ev = close(r, Tr, poses, 400.0)
    assert ("cut", tid + ".r1") in ev and r.truss[tid]["parted"] and r.truss[tid]["status"] == "on_plant"
    # a petiole
    row, r, poses = fresh()
    lf = [lf for q in row["plants"] for lf in q["leaves"]][0]
    la, lb = np.array(lf["petiole"]["a"]), np.array(lf["petiole"]["b"])
    x = (la - lb) / np.linalg.norm(la - lb)
    z = np.cross(x, [0.0, 1.0, 0.0])
    z /= np.linalg.norm(z)
    Tl = kin.pose(np.stack([x, np.cross(z, x), z], axis=1), la + 0.5 * (lb - la))
    ev = close(r, Tl, poses, 400.0)
    assert ("cut", lf["id"]) in ev and r.leaf[lf["id"]]["cut"] and all(not q["lost"] for q in r.plant.values())
    print("5. a stem stalls the jaws at %.1f mm and is cut at %.0f N (the plant is lost); a rachis parts; a petiole drops its leaf" % (2000 * sm["r"], sm["need"]))

    # ------------------------------------------------------------------ 6. what bruises
    row, r, poses = fresh()
    r.impacts({tid + ".r1": F.DAMAGE["impact_dv"] * 0.99})
    assert not r.truss[tid]["bruised"]
    r.impacts({tid + ".r1": F.DAMAGE["impact_dv"] * 1.01})
    assert r.truss[tid]["bruised"]
    other = row["ready"][1]
    r.presses({other + ".r0": F.DAMAGE["squeeze_force"] * 0.99})
    assert not r.truss[other]["squeezed"]
    r.presses({other + ".r0": F.DAMAGE["squeeze_force"] * 1.01})
    assert r.truss[other]["squeezed"]
    print("6. bruised above %.1f m/s in a tick, squeezed above %.0f N" % (F.DAMAGE["impact_dv"], F.DAMAGE["squeeze_force"]))

    # ------------------------------------------------------------------ 7. the yield
    row, r, poses = fresh()
    ready = row["ready"]
    credit, detail = r.credit(poses, 0.0)
    assert credit == 0.0 and all(d["why"] == "not_cut" for d in detail.values()), "nothing harvested: 0"
    laid = dict(poses)
    spots = [(c, k) for c in ("front", "rear") for k in range(3)]
    for tid_, (c, k) in zip(ready, spots):
        harvested(r, tid_)
        laid = lay(row, r, laid, tid_, c, k)
    credit, detail = r.credit(laid, 0.0)
    assert credit == 1.0 and all(detail[t_]["counts"] for t_ in ready), detail
    credit2, _ = r.credit({k: (lambda T: (T.__setitem__((1, 3), T[1, 3] + 0.7), T)[1])(v.copy()) for k, v in laid.items()}, 0.7)
    assert credit2 == 1.0, "the crates ride on the cart"
    credit3, d3 = r.credit(laid, 0.5)
    assert credit3 == 0.0 and all(d3[t_]["why"] == "not_in_a_crate" for t_ in ready)
    # an unready truss in a crate neither counts nor harms; cut, it is worth nothing
    unready = [t_["id"] for q in row["plants"] for t_ in q["trusses"] if not t_["ready"]][0]
    harvested(r, unready)
    free = spots[len(ready)]
    credit, detail = r.credit(lay(row, r, laid, unready, *free), 0.0)
    assert credit == 1.0 and not detail[unready]["counts"]
    # laid on another truss: both are damaged
    a_, b_ = ready[0], ready[1]
    c0, k0 = spots[0]
    heights = [0.03 + 0.005 * i for i in range(12)]
    hit = [dz for dz in heights if r.credit(lay(row, r, laid, b_, c0, k0, dz=dz), 0.0)[1][a_]["why"] == "stacked"]
    assert hit, "a truss laid on another is found"
    credit, detail = r.credit(lay(row, r, laid, b_, c0, k0, dz=hit[0]), 0.0)
    assert detail[a_]["why"] == "stacked" and detail[b_]["why"] == "stacked" and abs(credit - (len(ready) - 2) / len(ready)) < 1e-9
    assert any(r.credit(lay(row, r, laid, unready, c0, k0, dz=dz), 0.0)[1][a_]["why"] == "stacked" for dz in heights), \
        "an unready truss laid on a ready one damages it"
    assert r.credit(lay(row, r, laid, b_, c0, k0, dz=0.12), 0.0)[1][a_]["why"] is None, "a truss held clear above another does not lie on it"
    # side by side in neighbouring lanes is one layer
    assert r.credit(laid, 0.0)[1][a_]["why"] is None and not r.lies_on(r.fruit_centres(a_, laid), r.fruit_centres(ready[1], laid))
    # one at a time: in the tool, bruised, squeezed, crushed, parted
    for key, why in (("bruised", "bruised"), ("squeezed", "squeezed"), ("crushed", "crushed"), ("parted", "parted")):
        r.truss[a_][key] = True
        credit, detail = r.credit(laid, 0.0)
        assert detail[a_]["why"] == why and abs(credit - (len(ready) - 1) / len(ready)) < 1e-9
        r.truss[a_][key] = False
    r.truss[a_]["status"] = "held"
    assert r.credit(laid, 0.0)[1][a_]["why"] == "in_the_tool"
    r.truss[a_]["status"] = "free"
    # a fruit over the rim, a fruit outside the footprint
    assert r.credit(lay(row, r, laid, a_, c0, k0, dz=F.CART["crate_wall_h"]), 0.0)[1][a_]["why"] == "not_in_a_crate"
    assert r.credit(lay(row, r, laid, a_, c0, k0, dy=0.25), 0.0)[1][a_]["why"] == "not_in_a_crate"
    print("7. the yield: nothing 0, every ready truss laid 1.0 (%d of %d); stacked, damaged, held, outside: not counted" % (len(ready), len(ready)))

    # ------------------------------------------------------------------ 8. the wire
    good = {"move": {"type": "tcp", "delta": [0.01, 0, 0, 0, 0, 0.1], "ticks": 5, "guard_force": 20}, "cart": 0.1,
            "gripper": {"opening": 0.0, "force": 50.0}, "cams": ["wrist"]}
    full = wire.check_action(good)
    assert len(wire.canonical(good)) == 14 and full["cmd"] is None
    for bad in ({"move": {"type": "tcp", "delta": [0] * 5}}, {"move": {"type": "fly"}}, {"gripper": 0.01}, {"gripper": {"opening": 0.0}},
                {"cart": 5.0}, {"cmd": "stop"}, {"cams": ["overview", "overview"]}, {"move": {"type": "hold", "ticks": 41}},
                {"move": {"type": "joint", "delta": [10 ** 400, 0, 0, 0, 0, 0]}}, {"spindle": {}}, [], {"move": {"type": "tcp", "delta": [float("nan")] * 6}}):
        try:
            wire.check_action(bad)
        except ValueError:
            continue
        raise AssertionError("accepted a bad action: %r" % (bad,))
    assert wire.is_idle({}) and wire.is_idle({"gripper": {"opening": 0.0, "force": 5.0}}) and not wire.is_idle({"cart": 0.1})
    assert not wire.is_idle({"move": {"type": "joint", "delta": [0.1, 0, 0, 0, 0, 0]}})
    # the probe's copy agrees with its own joints as a true observation does
    rng = random.Random(5)
    larger = 0
    for _ in range(400):
        q = [rng.uniform(-2.0, 2.0) for _ in range(6)]
        qt = [v + rng.uniform(-0.01, 0.01) for v in q]
        obs = {"q": [round(v, 6) for v in q], "q_target": [round(v, 6) for v in qt], "tcp": wire.pose6(kin.flange(q) @ wire.TOOL_T),
               "tcp_target": wire.pose6(kin.flange(qt) @ wire.TOOL_T),
               "images": {"wrist": {"pose": wire.pose6(kin.flange(q) @ F.camera_mount("wrist"))}}}
        copy = wire.perturbed(obs, rng, exact={"q": q, "q_target": qt})
        assert copy["q"] != obs["q"] and max(abs(a - b) for a, b in zip(copy["q"], obs["q"])) <= wire.PROBE_SHIFT_RAD + 1e-9
        rt, rc = wire.residual(obs), wire.residual(copy)
        assert rt < 5e-6 and rc < 5e-6, (rt, rc)
        larger += rt > rc
    assert 120 < larger < 280, larger
    print("8. the wire: the schema, idle actions, the probe's copy (true residual larger in %d of 400)" % larger)
    print("all checks passed")


if __name__ == "__main__":
    main()
