#!/usr/bin/env python3
"""Can a program without the privileged view find and tell apart a pack's lid fasteners from the frames alone?
(Maintainers' tool; needs Isaac Sim and a GPU. It is not a solution of the task: it measures what the cameras carry.)

    PACKCELL_ROBOT_USD=.../ur20.usd python3 check_observability.py <dev pack index 0..3> [out_dir]

It starts a cell on a development pack with the grader's switch, only to compare with the truth afterwards. What it
does with public means only (the frames, the published calibration, the manual's numbers):

1. turns the arm out of the overhead camera's view and takes one overhead frame;
2. segments the lid by its depth and finds, on it, the blobs that are not lid-coloured and of a fastener head's
   size; each blob's centre goes through the depth image and the camera's calibration to a point in the base frame;
3. brings the wrist camera over each such point (the bit's tip 0.105 m above it puts the point on the camera's
   axis), takes a wrist frame, finds the head in it again and classifies it: a dark six-lobe recess in the middle
   (Torx) or none (hexagon); rust-coloured or not; an empty hole.

(Written for version 0.1.0, whose cameras stood exactly at the published poses and whose depth was exact; the table
it produced is in tests/README.md, section 6, the entry "04:40 to 05:50". Since 2026-11-25 the published poses are
nominal and the depth is a sensor's (section 2): this script still locates through the nominal poses, so on the
current cell its position errors include the cell's camera offsets. It is kept as the record of that measurement.)

Then it reports, against the cell's truth: how many of the lid's fasteners were found, the position error of the
overhead estimate and of the wrist estimate (mm), and the classification's hits. The tool catalog's tolerances are
2.0 mm (socket) and 1.2 mm (Torx).
"""

import base64
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

from packcell import cellproc, kin, wire  # noqa: E402
from packcell import series as S  # noqa: E402


def decode(image):
    import cv2
    rgb = cv2.imdecode(np.frombuffer(base64.b64decode(image["rgb"]), np.uint8), cv2.IMREAD_COLOR)
    dep = cv2.imdecode(np.frombuffer(base64.b64decode(image["depth"]), np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32) * 1e-4
    return rgb, dep


def back_project(K, T_cam, u, v, d):
    p = np.array([(u - K[0][2]) * d / K[0][0], (v - K[1][2]) * d / K[1][1], d, 1.0])
    return (T_cam @ p)[:3]


def blobs(mask, lo, hi):
    import cv2
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        if lo <= stats[i, cv2.CC_STAT_AREA] <= hi:
            w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            if 0.6 <= w / float(h) <= 1.7:
                out.append((float(cent[i][0]), float(cent[i][1]), int(stats[i, cv2.CC_STAT_AREA])))
    return out


def main():
    import cv2
    index = int(sys.argv[1])
    out = sys.argv[2] if len(sys.argv) > 2 else None
    if out:
        os.makedirs(out, exist_ok=True)
    with open(os.path.join(TASK, "environment", "commission_service", "dev_packs.json")) as f:
        entry = json.load(f)["packs"][index]
    pack = S.draw_pack(entry["seed"], variant=entry["variant"], harness=entry["harness"], cameras=entry.get("cameras"),
                         sealant_seed=entry.get("sealant_seed"), appearance_seed=entry.get("appearance_seed"))
    cell = cellproc.CellProcess(grader=True, stderr=open(os.devnull, "wb"))
    obs = cell.request({"cmd": "reset", "pack": pack, "config": {"frame_budget": 1000}})["obs"]

    def step(a):
        return cell.request({"cmd": "step", "action": a})["obs"]

    def goto(T, speed=0.35):
        nonlocal obs
        for _ in range(30):
            cur = wire.pose_T(obs["tcp_target"])
            d = np.concatenate([T[:3, 3] - cur[:3, 3], kin.R_rotvec(T[:3, :3] @ cur[:3, :3].T)])
            if np.linalg.norm(d[:3]) < 2e-4 and np.linalg.norm(d[3:]) < 1e-3:
                return
            n = int(min(40, max(4, math.ceil(1.5 * max(np.linalg.norm(d[:3]) / speed, np.linalg.norm(d[3:]) / 1.0) / S.TICK))))
            obs = step({"move": {"type": "tcp", "delta": [float(v) for v in d], "ticks": n}})

    # 1. the overhead frame, the arm out of the way
    for _ in range(3):
        obs = step({"move": {"type": "joint", "delta": [2.3 / 3, 0, 0, 0, 0, 0], "ticks": 40}})
    obs = step({"move": {"type": "hold", "ticks": 10}, "cams": ["overhead"]})
    rgb, dep = decode(obs["images"]["overhead"])
    c = S.CAMERAS["overhead"]
    K = S.camera_K("overhead")
    T_over = kin.pose(S.camera_R("overhead"), np.array(c["pos"]))
    # 2. the lid: the plane at the manual's lid height (the fixture's surface is published, the lid's height too)
    lid_depth = c["pos"][2] - (S.TABLE_Z + S.Z["lid_seat"])
    lid = np.abs(dep - lid_depth) < 0.012
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV).astype(np.int32)
    val, sat, hue = hsv[..., 2], hsv[..., 1], hsv[..., 0]
    lid_val = float(np.median(val[lid]))
    bright = lid & (val > lid_val + 45) & (sat < 90)                 # plated heads
    rusty = lid & (sat > 110) & (hue < 25) & (val > 60)              # rust
    dark = lid & (val < lid_val - 28)                                # empty holes
    px = c["pos"][2] - (S.TABLE_Z + S.Z["lid_seat"])                 # metres per pixel times fx
    mm_px = 1000.0 * px / K[0][0]
    head_area = math.pi * (6.5 / mm_px) ** 2
    cands = [(u, v, "head") for u, v, a in blobs(bright | rusty, 0.35 * head_area, 2.6 * head_area)]
    cands += [(u, v, "hole") for u, v, a in blobs(dark, 0.08 * head_area, 0.6 * head_area)]
    est = []
    for u, v, kind in cands:
        d = float(np.median(dep[int(v) - 1:int(v) + 2, int(u) - 1:int(u) + 2]))
        est.append((back_project(K, T_over, u, v, d), kind))
    truth = cell.request({"cmd": "truth"})["truth"]
    lidf = {fid: f for fid, f in truth["fasteners"].items() if f["group"] == "lid"}
    # 3. the wrist camera over each candidate
    R_down = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    Kw = S.camera_K("wrist")
    cw = S.CAMERAS["wrist"]
    rows = []
    q = np.array(obs["q_target"])
    for _ in range(3):
        obs = step({"move": {"type": "joint", "delta": [-2.3 / 3, 0, 0, 0, 0, 0], "ticks": 40}})
    for k, (p, kind) in enumerate(est):
        cur = wire.pose_T(obs["tcp_target"])
        goto(kin.pose(R_down, [cur[0, 3], cur[1, 3], 0.0]))
        goto(kin.pose(R_down, [p[0], p[1], 0.0]))
        goto(kin.pose(R_down, [p[0], p[1], p[2] + 0.105]))
        obs = step({"move": {"type": "hold", "ticks": 8}})
        obs = step({"cams": ["wrist"]})
        wrgb, wdep = decode(obs["images"]["wrist"])
        T_cam = wire.pose_T(obs["images"]["wrist"]["pose"])
        whsv = cv2.cvtColor(wrgb, cv2.COLOR_BGR2HSV).astype(np.int32)
        wv, ws, wh = whsv[..., 2], whsv[..., 1], whsv[..., 0]
        cy, cx = cw["height"] // 2, cw["width"] // 2
        win = np.zeros(wv.shape, bool)
        win[cy - 90:cy + 90, cx - 90:cx + 90] = True
        base = float(np.median(wv[cy - 150:cy + 150, cx - 150:cx + 150]))
        wmm = 1000.0 * float(np.median(wdep[cy - 3:cy + 4, cx - 3:cx + 4])) / Kw[0][0]
        area = math.pi * (6.0 / wmm) ** 2
        plated = win & (wv > base + 45) & (ws < 90)
        rust = win & (ws > 110) & (wh < 25) & (wv > 60)
        head = blobs(plated | rust, 0.4 * area, 4.0 * area)
        row = {"overhead": p}
        if head:
            u, v, a = min(head, key=lambda b: (b[0] - cx) ** 2 + (b[1] - cy) ** 2)
            # the head's outline, then its centre; a Torx head has a dark recess inside the outline
            mask = np.zeros(wv.shape, np.uint8)
            cv2.circle(mask, (int(round(u)), int(round(v))), int(0.9 * math.sqrt(a / math.pi)), 1, -1)
            inner = np.zeros(wv.shape, np.uint8)
            cv2.circle(inner, (int(round(u)), int(round(v))), max(2, int(0.45 * math.sqrt(a / math.pi))), 1, -1)
            recess = float(np.percentile(wv[inner > 0], 10)) < 0.45 * float(np.percentile(wv[mask > 0], 80))
            is_rust = float(np.mean(rust[mask > 0])) > 0.3
            d = float(np.median(wdep[int(v) - 2:int(v) + 3, int(u) - 2:int(u) + 3]))
            if recess:      # the recess' floor is deeper than the head's top: take the depth beside it
                ring = (mask > 0) & (inner == 0)
                d = float(np.median(wdep[ring]))
            row.update(wrist=back_project(Kw, T_cam, u, v, d), kind="torx" if recess else "hex", rust=is_rust,
                       size_mm=2 * math.sqrt(a / math.pi) * wmm)
        else:
            hole = blobs(win & (wv < base - 30), 0.05 * area, 1.2 * area)
            if hole:
                u, v, a = min(hole, key=lambda b: (b[0] - cx) ** 2 + (b[1] - cy) ** 2)
                d = float(np.median(wdep[int(v) - 6:int(v) + 7, int(u) - 6:int(u) + 7]))
                row.update(wrist=back_project(Kw, T_cam, u, v, d), kind="hole", rust=False)
        rows.append(row)
        if out:
            cv2.imwrite(os.path.join(out, "wrist_%02d.png" % k), wrgb)
    if out:
        cv2.imwrite(os.path.join(out, "overhead.png"), rgb)
    cell.close()
    # 4. against the truth
    found, e_over, e_wrist, kinds, rusts = 0, [], [], [0, 0], [0, 0]
    used = set()
    for fid, f in sorted(lidf.items()):
        top = np.array(f["top"])
        best = min(range(len(rows)), key=lambda i: np.linalg.norm(rows[i]["overhead"][:2] - top[:2])) if rows else None
        if best is None or np.linalg.norm(rows[best]["overhead"][:2] - top[:2]) > 0.012 or best in used:
            print("%s  %-4s %s  NOT FOUND" % (fid, f["head"], f["status"]))
            continue
        used.add(best)
        found += 1
        r = rows[best]
        eo = 1000 * np.linalg.norm(r["overhead"][:2] - top[:2])
        e_over.append(eo)
        want = "hole" if f["status"] == "missing" else ("torx" if f["head"].startswith("T") else "hex")
        line = "%s  %-4s %-8s overhead error %.2f mm" % (fid, f["head"], "corroded" if f["corroded"] else f["status"], eo)
        if "wrist" in r:
            ew = 1000 * np.linalg.norm(r["wrist"][:2] - top[:2])
            ez = 1000 * (r["wrist"][2] - top[2])
            if want != "hole":
                e_wrist.append(ew)
            kinds[0] += int(r["kind"] == want)
            kinds[1] += 1
            if want != "hole":
                rusts[0] += int(bool(r["rust"]) == bool(f["corroded"]))
                rusts[1] += 1
            line += "; wrist error %.2f mm (z %+.2f); seen as %s%s%s" % (ew, ez, r["kind"], " rust" if r["rust"] else "",
                                                                         "" if r["kind"] == want else "  <-- WRONG KIND")
        else:
            line += "; not seen by the wrist camera"
        print(line)
    spurious = len(rows) - len(used)
    print("lid fasteners (and holes) found: %d of %d; candidates that are none: %d" % (found, len(lidf), spurious))
    if e_over:
        print("overhead estimate: median %.2f mm, max %.2f mm" % (float(np.median(e_over)), max(e_over)))
    if e_wrist:
        print("wrist estimate:    median %.2f mm, max %.2f mm (tolerances: 2.0 mm socket, 1.2 mm Torx)" % (float(np.median(e_wrist)), max(e_wrist)))
    print("head kind right: %d of %d; rust right: %d of %d" % (kinds[0], kinds[1], rusts[0], rusts[1]))


if __name__ == "__main__":
    main()
