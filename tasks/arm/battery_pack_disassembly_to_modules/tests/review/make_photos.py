#!/usr/bin/env python3
"""Take the workshop manual's photographs with the cell's own cameras (maintainers' tool; needs Isaac Sim and a GPU).

    PACKCELL_ROBOT_USD=.../ur20.usd python3 make_photos.py <out_dir>

It builds an example pack (an HVB-P5 that is neither a graded nor a development pack: its seed is this file's),
dresses its lid bolts with every head type, clean and corroded, and one empty hole, and writes

    photo_example_lid.jpg      the overhead camera's frame, the robot turned out of the view
    photo_example_inside.jpg   the same with the lid taken away
    photo_heads.jpg            the wrist camera's view of each head from about 0.25 m, cropped and labelled

Then `make_documents.py --photos <out_dir>` copies them into the givens.
"""

import base64
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))

EXAMPLE_SEED = 5150
HEADS = [("L01", "S10", False), ("L02", "S10", True), ("L03", "T30", False), ("L04", "T30", True),
         ("L06", "S13", False), ("L07", "S13", True), ("L08", "T40", False), ("L09", "T40", True), ("L05", None, False)]


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    from isaacsim import SimulationApp
    cfg = {"headless": True, "multi_gpu": False}
    if os.environ.get("PACKCELL_GPU"):
        cfg.update(active_gpu=int(os.environ["PACKCELL_GPU"]), physics_gpu=int(os.environ["PACKCELL_GPU"]))
    kit = os.environ.get("PACKCELL_KIT_DIR")
    if kit:
        cfg["extra_args"] = ["--/app/folder/cache=%s/cache" % kit, "--/app/folder/data=%s/data" % kit, "--/app/folder/logs=%s/logs" % kit]
    app = SimulationApp(cfg)
    import cv2
    from packcell import kin, wire
    from packcell import series as S
    from packcell.cell import Cell

    pack = S.draw_pack(EXAMPLE_SEED, variant="P5", harness="none")
    by_id = {f["id"]: f for f in pack["fasteners"]}
    for fid, head, corroded in HEADS:
        f = by_id[fid]
        f.update(missing=head is None, corroded=corroded)
        if head:
            f["head"] = head
    for f in pack["fasteners"]:
        if f["group"] == "lid" and f["id"] not in [h[0] for h in HEADS]:
            f.update(missing=False, corroded=False)
    cell = Cell(app, pack, {"frame_budget": 1000, "render_subframes": 8}, robot_usd=os.environ["PACKCELL_ROBOT_USD"],
                log=lambda *a: print("[photos]", *a, flush=True))

    def act(**a):
        return cell.step(wire.check_action(a))

    def rgb(obs, name):
        return cv2.imdecode(np.frombuffer(base64.b64decode(obs["images"][name]["rgb"]), np.uint8), cv2.IMREAD_COLOR)

    def goto(T, speed=0.3):
        for _ in range(30):
            cur = wire.pose_T(cell.observe()["tcp_target"])
            d = np.concatenate([T[:3, 3] - cur[:3, 3], kin.R_rotvec(T[:3, :3] @ cur[:3, :3].T)])
            if np.linalg.norm(d[:3]) < 2e-4 and np.linalg.norm(d[3:]) < 1e-3:
                return
            n = int(min(40, max(4, math.ceil(1.5 * max(np.linalg.norm(d[:3]) / speed, np.linalg.norm(d[3:]) / 1.0) / S.TICK))))
            act(move={"type": "tcp", "delta": [float(v) for v in d], "ticks": n})

    # the heads, one by one: the bit's tip 0.105 m above the head puts the head on the wrist camera's axis
    R_down = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    tiles = []
    for fid, head, corroded in HEADS:
        top = cell.rules.fast[fid]["top"] if head else cell.rules.T[:3, :3] @ (np.array(by_id[fid]["pos"])) + cell.rules.T[:3, 3]
        cur = wire.pose_T(cell.observe()["tcp_target"])
        goto(kin.pose(R_down, [cur[0, 3], cur[1, 3], 0.0]))
        goto(kin.pose(R_down, [top[0], top[1], 0.0]))
        goto(kin.pose(R_down, top + [0, 0, 0.105]))
        act(move={"type": "hold", "ticks": 10})
        img = rgb(act(cams=["wrist"]), "wrist")
        c = S.CAMERAS["wrist"]
        u, v = c["width"] // 2, c["height"] // 2
        crop = img[v - 110:v + 110, u - 110:u + 110]
        crop = cv2.resize(crop, (330, 330), interpolation=cv2.INTER_CUBIC)
        label = "empty hole" if head is None else "%s%s" % (head, " corroded" if corroded else "")
        cv2.rectangle(crop, (0, 0), (329, 26), (255, 255, 255), -1)
        cv2.putText(crop, label, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
        tiles.append(crop)
        cv2.imwrite(os.path.join(out, "wrist_%s.png" % fid), img)
    rows = [np.hstack(tiles[0:3]), np.hstack(tiles[3:6]), np.hstack(tiles[6:9])]
    cv2.imwrite(os.path.join(out, "photo_heads.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])
    # the overhead photographs, the arm turned away
    cur = wire.pose_T(cell.observe()["tcp_target"])
    goto(kin.pose(R_down, [cur[0, 3], cur[1, 3], 0.1]))
    q = np.array(cell.observe()["q_target"])
    for _ in range(3):
        act(move={"type": "joint", "delta": [float((2.3 - q[0]) / 3), 0, 0, 0, 0, 0], "ticks": 40})
    act(move={"type": "hold", "ticks": 10})
    img = rgb(act(cams=["overhead"]), "overhead")
    cv2.imwrite(os.path.join(out, "photo_example_lid.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
    cell._set_pose(cell.index["parts"]["lid"], kin.pose(np.eye(3), np.array([-0.95, 0.0, -0.398])))
    for f in pack["fasteners"]:
        if f["group"] == "lid" and not f["missing"]:
            cell._set_pose(cell.index["fasteners"][f["id"]], kin.pose(np.eye(3), np.array([30.0, 30.0, -5.0])))
    act(move={"type": "hold", "ticks": 10})
    img = rgb(act(cams=["overhead"]), "overhead")
    cv2.imwrite(os.path.join(out, "photo_example_inside.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
    # a closer look at the inside for maintainers (not a given)
    goto(kin.pose(R_down, [0.9, 0.0, 0.1]))
    cv2.imwrite(os.path.join(out, "wrist_inside.png"), rgb(act(cams=["wrist"]), "wrist"))
    print("[photos] done:", sorted(os.listdir(out)))
    app.close()


if __name__ == "__main__":
    main()
