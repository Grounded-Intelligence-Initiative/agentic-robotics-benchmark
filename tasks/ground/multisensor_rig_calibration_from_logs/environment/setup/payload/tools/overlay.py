"""Draw a LiDAR scan over a camera frame with a given T_lidar_camera, to eyeball an extrinsic.

    python3 ~/tools/overlay.py --rig /opt/rigcal/practice/rig_a --seq seq_00 --frame 10 \
        --calibration out/calibration.yaml --out /tmp/overlay.png

Without --calibration it uses the rig's rough_prior.yaml. The scan drawn is the one whose stamp is
nearest to the frame's stamp after both are mapped to the reference clock with the calibration's
clock models (the prior has none, so raw stamps are compared). Points are coloured by range.
No motion between the two capture times is compensated: use frames taken at rest for a clean check.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rig_io  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", required=True)
    ap.add_argument("--seq", default="seq_00")
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--calibration", default=None)
    ap.add_argument("--out", default="/tmp/overlay.png")
    a = ap.parse_args()
    import cv2
    import yaml
    rig = rig_io.Rig(a.rig)
    seq = rig.sequence(a.seq)
    T = np.array(rig.prior["T_lidar_camera"], float)
    co = cd = lo = ld = 0.0
    if a.calibration:
        with open(a.calibration) as fh:
            cal = yaml.safe_load(fh)
        T = np.array(cal["T_lidar_camera"], float)
        co, cd = cal["time"]["camera"]["offset_s"], cal["time"]["camera"]["drift_ppm"] * 1e-6
        lo, ld = cal["time"]["lidar"]["offset_s"], cal["time"]["lidar"]["drift_ppm"] * 1e-6
    tc = (seq.cam_stamps[a.frame] - co) / (1 + cd)
    tl = (seq.scan_stamps - lo) / (1 + ld)
    li = int(np.argmin(np.abs(tl - tc)))
    pts, _ = seq.scan_points(li)
    R, t = T[:3, :3], T[:3, 3]
    pc = (pts - t) @ R
    m = pc[:, 2] > 0.3
    pc = pc[m]
    K = rig.K
    u = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]
    v = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]
    img = seq.image(a.frame)[:, :, ::-1].copy()
    big = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    d = np.linalg.norm(pc, axis=1)
    col = cv2.applyColorMap(np.clip(d / 15.0 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)
    for (x, y), c in zip(np.stack([u, v], 1), col[:, 0]):
        if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
            cv2.circle(big, (int(x * 3 + 1), int(y * 3 + 1)), 1, tuple(int(k) for k in c), -1)
    cv2.imwrite(a.out, big)
    print("frame %d (stamp %.4f) with scan %d (stamp %.4f) -> %s" % (a.frame, seq.cam_stamps[a.frame], li,
                                                                     seq.scan_stamps[li], a.out))


if __name__ == "__main__":
    main()
