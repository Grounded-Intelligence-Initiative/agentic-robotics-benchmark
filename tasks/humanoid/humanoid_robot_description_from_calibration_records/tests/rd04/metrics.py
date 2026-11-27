"""The published metrics of rd04: keys from a whitelist (another key is a task
error), numbers at 3 decimals, one band word per lock and member (ok = within full; partial; missed = at or beyond
zero). No signed or numeric lock error is published: lock_errors() returns them for maintainers' tools only.

Keys: score, M, D; gate ('pass' | 'fail'), gate_codes (the gate's reason codes, comma-separated); family_<f> for
the five families; term_<t> for the eleven terms; posture_<name> for the ten postures' factors; falls,
posture_failures, box_clamps, stop_clamps, region_clamps; lock_<x> for knee, ankle, bars, hip_yaw, hip_roll,
hip_pitch, elbows, masses, imu, soles."""
from __future__ import annotations

import math

import numpy as np

from .model import ANKLE_SLOTS, SIDES
from .util import DEG, word

FAMILIES = ("feet", "trunk", "weight", "sole", "arms")
TERM_KEYS = {"feet_pos_mm": "feet_pos", "feet_normal_deg": "feet_normal", "feet_heading_deg": "feet_heading",
             "trunk_deg": "trunk", "com_mm": "com", "sole_plane_mm": "sole_plane", "sole_fp_mm": "sole_fp",
             "arms_deg": "arms", "flat": "flat", "weight": "weight", "upright": "upright"}
COUNTS = ("falls", "posture_failures", "box_clamps", "stop_clamps", "region_clamps")
LOCKS = ("knee", "ankle", "bars", "hip_yaw", "hip_roll", "hip_pitch", "elbows", "masses", "imu", "soles")
JOINT_LOCK_SLOTS = {"knee": (3, 9), "hip_yaw": (2, 8), "hip_roll": (1, 7), "hip_pitch": (0, 6), "elbows": (15, 20)}


def whitelist(postures) -> set:
    keys = {"score", "M", "D", "gate", "gate_codes"}
    keys |= {f"family_{f}" for f in FAMILIES}
    keys |= {f"term_{v}" for v in TERM_KEYS.values()}
    keys |= {f"posture_{p['name']}" for p in postures["postures"]}
    keys |= set(COUNTS)
    keys |= {f"lock_{x}" for x in LOCKS}
    return keys


def _r3(x):
    x = float(x)
    return round(x, 3) if math.isfinite(x) else None


def link_group_masses(files) -> dict:
    """per named link the mass of the link and its jointless descendant bodies (they move with it: a mass placed on a
    child body counts as the link's)"""
    import mujoco
    from .model import BODIES
    m = files.model
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b): b for b in range(m.nbody)}
    root = {}
    for b in range(1, m.nbody):
        a = b
        while mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, a) not in BODIES:
            a = int(m.body_parentid[a])
        root[b] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, a)
    out = {n: 0.0 for n in BODIES}
    for b, r in root.items():
        out[r] += float(m.body_mass[b])
    return out


def split_offset(o, b, ankle_full: float) -> tuple:
    """(ankle, bars) deg of one foot: o = the files' zero-reading attitude minus the unit's as crated, b = the unit's
    as crated minus on the dry-fit bars (both (pitch, roll), deg). The bars part c = max(0, -o . b/|b|) is bounded by
    twice the whole bar effect, 2 |b| (a bar change applied with the reversed sign is the largest bar fault); it is
    booked as bars only when the rest |o + c b/|b|| lies inside the ankle band's full end; otherwise the whole offset
    |o| is booked under the ankle word and the foot's bars part is 0. b None or 0: (|o|, None)."""
    o = np.asarray(o, float)
    if b is None or not np.linalg.norm(b) > 1e-9:
        return float(np.linalg.norm(o)), None
    nb = float(np.linalg.norm(b))
    bh = np.asarray(b, float) / nb
    c = min(max(0.0, -float(o @ bh)), 2.0 * nb)
    rest = float(np.linalg.norm(o + c * bh))
    if rest <= ankle_full:
        return rest, c
    return float(np.linalg.norm(o)), 0.0


def lock_errors(files, hidden, ankle_full=None) -> dict:
    """numeric lock errors (deg, %; maintainers' tools only, never published): joint locks = the mean over the bus
    samples of |files' angle - true angle|, the worse side; ankle / bars from the zero-reading attitude offset
    (grader_config lock_words); masses %; imu = the trunk term's mean error; soles = (plane, footprint) mm, the worse
    foot.

    The split of a foot's offset into the two words: split_offset, with the ankle band's full end ankle_full (deg;
    default: grader_config lock_words.bands.ankle[0])."""
    from .grade import sole_errors
    if ankle_full is None:
        from .grade import load_config
        ankle_full = float(load_config()["lock_words"]["bands"]["ankle"][0])
    th_f = np.array([files.angles(r) for r in hidden.readings])
    out = {}
    for name, (a, b) in JOINT_LOCK_SLOTS.items():
        e = [float(np.mean(np.abs(th_f[:, j] - hidden.theta[:, j]))) / DEG for j in (a, b)]
        out[name] = max(e) if all(math.isfinite(x) for x in e) else float("inf")
    o_all, b_all = {}, {}
    for s in SIDES:
        a, b = ANKLE_SLOTS[s]
        ff = np.array(files.maps[s](files.home[a], files.home[b]))
        T = hidden.true
        z = T.zero_crank[s]
        from .vendor_chain import fw_pitch_roll
        ft = np.array(T.chain_now[s].fk(z[0], z[1], guess=fw_pitch_roll(T.home[a], T.home[b])))
        o_all[s] = (ff - ft) / DEG
        if hidden.dryfit_zero_attitude is not None:
            b_all[s] = (ft - np.array(hidden.dryfit_zero_attitude[s])) / DEG
    ank, bars = [], []
    for s in SIDES:
        a_, b_ = split_offset(o_all[s], b_all.get(s), ankle_full)
        ank.append(a_)
        if b_ is not None:
            bars.append(b_)
    out["ankle"] = max(ank) if all(math.isfinite(x) for x in ank) else float("inf")
    if bars:
        out["bars"] = max(bars) if all(math.isfinite(x) for x in bars) else float("inf")
    gt, gf = link_group_masses(hidden.truth), link_group_masses(files)
    out["masses"] = max(100.0 * abs(gf[b] - gt[b]) / gt[b] for b in gt)
    pl, fp = sole_errors(hidden.truth, files)
    out["soles_plane"] = max(pl); out["soles_footprint"] = max(fp)
    return out


def metrics_of(out: dict, files_raw, hidden, cfg) -> dict:
    m = {"score": _r3(out["score"]), "M": _r3(out["M"]), "D": _r3(out["D"])}
    if files_raw is None:
        m["gate"] = "fail"
        m["gate_codes"] = ",".join(out["gate"]["codes"])
        return m
    files, raw = files_raw
    m["gate"] = "pass"
    m["gate_codes"] = ""
    for f in FAMILIES:
        m[f"family_{f}"] = _r3(out["families"][f])
    for k, v in TERM_KEYS.items():
        m[f"term_{v}"] = _r3(out["terms"][k])
    for p in out["postures"]:
        m[f"posture_{p['posture']}"] = _r3(p["factor"])
    for c in COUNTS:
        m[c] = int(out["counts"][c])
    LB = cfg["lock_words"]["bands"]
    le = lock_errors(files, hidden, ankle_full=float(LB["ankle"][0]))
    for x in ("knee", "ankle", "bars", "hip_yaw", "hip_roll", "hip_pitch", "elbows", "masses"):
        if x in le:
            m[f"lock_{x}"] = word(le[x], *LB[x])
    trk = np.array(raw["trunk_deg"], float)
    m["lock_imu"] = word(float(trk.mean()) if np.all(np.isfinite(trk)) else float("inf"), *LB["imu"])
    wp = word(le["soles_plane"], *LB["soles_plane"]); wf = word(le["soles_footprint"], *LB["soles_footprint"])
    m["lock_soles"] = "ok" if (wp == "ok" and wf == "ok") else ("missed" if "missed" in (wp, wf) else "partial")
    bad = set(m) - whitelist(hidden.postures)
    if bad:
        raise RuntimeError(f"metric keys off the whitelist: {sorted(bad)}")
    return m
