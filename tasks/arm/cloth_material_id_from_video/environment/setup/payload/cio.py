"""cio: boilerplate I/O for cloth_material_id_from_video (no perception, tracking or identification code).

    import cio
    cams = cio.cameras()                         # list of dicts: name, K, R_w2c, t_w2c, width, height
    fab = cio.fabric_info("A")                   # size_m, grid, mass_kg, n_clips
    clip = cio.clip_info("A", 0)                 # kind, fps, n_frames, fabric_start, clamp_500hz (M, 8), arm_q (N, 7)
    frames = cio.load_video("A", 0, "cam1")      # (N, H, W, 3) uint8 RGB
    uv = cio.project(cams[0], X_world)           # (n, 2) pixels of world points (n, 3)
    cio.write_params({"A": {...}, "B": {...}, "C": {...}})
"""
import json
import math
import os

import numpy as np

DATA = os.path.join(os.path.expanduser("~"), "data")
SUBMISSION = os.path.join(os.path.expanduser("~"), "submission", "params.json")
KEYS = ("young", "poisson", "thickness", "damping", "mu")


def cameras():
    with open(os.path.join(DATA, "cameras.json")) as f:
        cams = json.load(f)["cameras"]
    for c in cams:
        c["K"] = np.array(c["K"])
        c["R_w2c"] = np.array(c["R_w2c"])
        c["t_w2c"] = np.array(c["t_w2c"])
    return cams


def fabric_dir(fid):
    return os.path.join(DATA, "fabrics", fid)


def fabric_info(fid):
    with open(os.path.join(fabric_dir(fid), "fabric.json")) as f:
        return json.load(f)


def clip_info(fid, i):
    d = os.path.join(fabric_dir(fid), "clip%02d" % i)
    with open(os.path.join(d, "clip.json")) as f:
        info = json.load(f)
    info["clamp_500hz"] = np.load(os.path.join(d, "clamp_500hz.npy"), allow_pickle=False)
    info["arm_q"] = np.load(os.path.join(d, "arm_q.npy"), allow_pickle=False)
    return info


def load_video(fid, i, cam):
    import cv2
    cap = cv2.VideoCapture(os.path.join(fabric_dir(fid), "clip%02d" % i, cam + ".mp4"))
    out = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        out.append(f[:, :, ::-1])
    cap.release()
    return np.stack(out)


def project(cam, X):
    X = np.atleast_2d(np.asarray(X, float))
    Xc = X @ cam["R_w2c"].T + cam["t_w2c"]
    uv = Xc @ cam["K"].T
    return uv[:, :2] / uv[:, 2:3]


def write_params(fabrics):
    """fabrics: {"A": {"young", "poisson", "thickness", "damping", "mu"}, ...} -> ~/submission/params.json.
    Checks that every value is a finite number (the grader clamps each to its admissible range)."""
    out = {}
    for k, e in fabrics.items():
        row = {}
        for key in KEYS:
            v = float(e[key])
            if not math.isfinite(v):
                raise ValueError("%s.%s is not finite" % (k, key))
            row[key] = v
        out[k] = row
    os.makedirs(os.path.dirname(SUBMISSION), exist_ok=True)
    with open(SUBMISSION, "w") as f:
        json.dump({"fabrics": out}, f, indent=1)
    return SUBMISSION
