"""Build-time generator of the agent's recordings for cloth_material_id_from_video.

Runs once while the task image builds, from a read-only bind mount of environment/gen/ (so neither this file nor
fabrics.json, which holds the hidden true materials, is left in any image layer). For every fabric (P, A, B, C)
and clip it simulates the clip in the true material (cphys, byte-identical to the agent's copy), solves the arm's
joint angles (carm), renders three calibrated cameras offscreen (MuJoCo, OSMesa), degrades the images like a real
camera and encodes H.264 with ffmpeg. Output tree (OUT, default /opt/task-assets/payload):

  data/cameras.json
  data/fabrics/<ID>/fabric.json                         size, grid, mass, number of clips
  data/fabrics/<ID>/clipNN/{clip.json, clamp_500hz.npy, arm_q.npy, cam0.mp4, cam1.mp4, cam2.mp4}

Everything here is deterministic: fixed seeds, fixed clip programs. Textures are procedural (no external files).
usage: python3 render_clips.py OUT [--workers N] [--only P,A] [--frames-max K]
"""
import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cphys as W  # noqa: E402

W_IMG, H_IMG = 640, 480
FOCAL = 640.0
CAMERAS = [  # name, position, look-at target (world)
    ("cam0", (0.72, -0.55, 0.62), (0.0, 0.0, 0.10)),
    ("cam1", (0.85, 0.32, 0.22), (0.20, 0.0, 0.0)),
    ("cam2", (-0.70, 0.55, 0.62), (-0.12, 0.0, 0.12)),
]


# ----------------------------------------------------------------------------------------------------------------
# cameras
# ----------------------------------------------------------------------------------------------------------------
def look_at(pos, target):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    z = target - pos
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R_w2c = np.stack([x, y, z])
    return R_w2c, -R_w2c @ pos


def cameras_json():
    cams = []
    for name, pos, tgt in CAMERAS:
        R, t = look_at(pos, tgt)
        K = [[FOCAL, 0.0, W_IMG / 2], [0.0, FOCAL, H_IMG / 2], [0.0, 0.0, 1.0]]
        cams.append(dict(name=name, width=W_IMG, height=H_IMG, K=K, dist=[0, 0, 0, 0, 0], R_w2c=R.tolist(),
                         t_w2c=t.tolist(), position_world=list(map(float, pos))))
    return {"cameras": cams, "model": "OpenCV pinhole, pixel ~ K (R_w2c X + t_w2c), no distortion; world z up, metres"}


# ----------------------------------------------------------------------------------------------------------------
# procedural textures (no external assets)
# ----------------------------------------------------------------------------------------------------------------
def _noise(rng, n, scale):
    g = rng.random((n // scale + 2, n // scale + 2))
    from scipy.ndimage import zoom
    return zoom(g, scale, order=3)[:n, :n]


def fabric_texture(kind, seed, n=512):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:n, 0:n] / n
    if kind == "plaid":            # P: red / cream plaid
        a = (np.sin(2 * np.pi * 9 * xx) > 0.55).astype(float) + 0.6 * (np.sin(2 * np.pi * 27 * xx) > 0.8)
        b = (np.sin(2 * np.pi * 5 * yy) > 0.55).astype(float) + 0.6 * (np.sin(2 * np.pi * 21 * yy) > 0.8)
        m = np.clip(0.5 * a + 0.5 * b, 0, 1)[..., None]
        img = (1 - m) * np.array([0.93, 0.88, 0.78]) + m * np.array([0.72, 0.12, 0.14])
    elif kind == "floral":         # A: light blue with scattered blots
        img = np.ones((n, n, 3)) * np.array([0.70, 0.82, 0.93])
        for _ in range(140):
            cx, cy, r = rng.random(), rng.random(), rng.uniform(0.012, 0.04)
            col = np.array([[0.95, 0.95, 0.98], [0.30, 0.40, 0.75], [0.95, 0.75, 0.30]][rng.integers(3)])
            d = np.hypot(xx - cx, yy - cy)
            img[d < r] = col
    elif kind == "twill":          # B: dark blue twill with a woven diagonal and blotches, white stitch frame
        diag = 0.5 + 0.5 * np.sin(2 * np.pi * 60 * (xx + yy))
        blot = _noise(rng, n, 32)
        v = 0.55 + 0.25 * diag + 0.35 * (blot - 0.5)
        img = v[..., None] * np.array([0.22, 0.32, 0.55])
        for _ in range(60):
            cx, cy, r = rng.random(), rng.random(), rng.uniform(0.01, 0.03)
            img[np.hypot(xx - cx, yy - cy) < r] = [0.85, 0.80, 0.55]
        e = (np.minimum(np.minimum(xx, 1 - xx), np.minimum(yy, 1 - yy)) < 0.02)
        img[e] = [0.9, 0.75, 0.35]
    else:                          # C: green / grey knit stripes with speckles
        s = (np.sin(2 * np.pi * 14 * xx) > 0).astype(float)
        img = (1 - s[..., None]) * np.array([0.35, 0.55, 0.35]) + s[..., None] * np.array([0.78, 0.78, 0.75])
        sp = rng.random((n, n)) < 0.01
        img[sp] = [0.1, 0.1, 0.1]
        for _ in range(50):
            cx, cy, r = rng.random(), rng.random(), rng.uniform(0.01, 0.025)
            img[np.hypot(xx - cx, yy - cy) < r] = [0.95, 0.55, 0.15]
    img = img * (0.9 + 0.2 * _noise(rng, n, 8)[..., None])
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def wood_texture(seed, n=512):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:n, 0:n] / n
    ring = 0.5 + 0.5 * np.sin(2 * np.pi * (25 * xx + 4 * _noise(rng, n, 64)))
    v = 0.75 + 0.2 * ring + 0.1 * (_noise(rng, n, 4) - 0.5)
    return (np.clip(v[..., None] * np.array([0.62, 0.45, 0.30]), 0, 1) * 255).astype(np.uint8)


def floor_texture(seed, n=512):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:n, 0:n] / n
    tiles = ((np.floor(xx * 8) + np.floor(yy * 8)) % 2)
    v = 0.45 + 0.08 * tiles + 0.1 * (_noise(rng, n, 16) - 0.5)
    return (np.clip(v[..., None] * np.array([0.9, 0.9, 0.95]), 0, 1) * 255).astype(np.uint8)


def png_bytes(img):
    import cv2
    ok, buf = cv2.imencode(".png", img[:, :, ::-1])
    assert ok
    return buf.tobytes()


TEXTURE_KIND = {"P": "plaid", "A": "floral", "B": "twill", "C": "knit"}


# ----------------------------------------------------------------------------------------------------------------
# clip programs (observation clips; the scored tests are different, see ctasks.py)
# ----------------------------------------------------------------------------------------------------------------
def clip_programs(rng):
    """List of (kind, fabric_start pose, clamp trajectory)."""
    out = []
    # 0, 1: lift by a corner, hang, lower partway
    for i in range(2):
        pose = (float(rng.uniform(-0.12, 0.02)), float(rng.uniform(-0.08, 0.08)), float(rng.uniform(-np.pi, np.pi)))
        p, yaw = W.corner_grasp(pose, int(rng.choice([-1, 1])), int(rng.choice([-1, 1])))
        top = np.array([p[0], p[1], 0.44])
        low = np.array([p[0] + 0.05, p[1], float(rng.uniform(0.33, 0.37))])
        keys = [(0.0, p, yaw, 0, 0), (0.2, p, yaw, 0, 1), (1.4, top, yaw, 0, 1), (3.2, top, yaw, 0, 1),
                (4.0, low, yaw, 0, 1), (5.6, low, yaw, 0, 1)]
        out.append(("lift_hang", pose, W.program_to_traj(keys)))
    # 2: shake: lift by a short edge, three quick vertical strokes, hold
    pose = (float(rng.uniform(-0.10, 0.0)), float(rng.uniform(-0.06, 0.06)), float(rng.uniform(-0.4, 0.4)))
    p, yaw = W.edge_grasp(pose, +1)
    top = np.array([p[0] + 0.05, p[1], 0.42])
    keys = [(0.0, p, yaw, 0, 0), (0.2, p, yaw, 0, 1), (1.3, top, yaw, 0, 1), (2.0, top, yaw, 0, 1)]
    t = 2.0
    for k in range(3):
        keys += [(t + 0.18, top + [0, 0, 0.05], yaw, 0, 1), (t + 0.36, top - [0, 0, 0.02], yaw, 0, 1)]
        t += 0.36
    keys += [(t + 0.2, top, yaw, 0, 1), (t + 2.2, top, yaw, 0, 1)]
    out.append(("shake", pose, W.program_to_traj(keys)))
    # 3, 4: drag along the table and let go while moving
    import ctasks as T
    for v in (float(rng.uniform(0.35, 0.5)), float(rng.uniform(0.6, 0.8))):
        pose = (float(rng.uniform(0.0, 0.08)), float(rng.uniform(-0.1, 0.1)), float(rng.uniform(-0.3, 0.3)))
        C, _, _ = T.prog_slide(dict(pose=pose, side=-1, d_release=float(rng.uniform(0.09, 0.13))), v)
        out.append(("drag_release", pose, C))
    # 5: lay over the table's front edge (the clamp keeps holding)
    test = dict(y=float(rng.uniform(-0.08, 0.08)), hang_x=float(rng.uniform(0.03, 0.05)))
    C, _ = T.prog_drape(test, float(rng.uniform(0.08, 0.14)))
    out.append(("edge_drape", (W.EDGE_X - 0.22, test["y"], 0.0), C))
    # 6: hang over the rod and let go on the far side
    pose = (float(rng.uniform(0.0, 0.06)), float(rng.uniform(-0.06, 0.06)), float(rng.uniform(-0.2, 0.2)))
    p, yaw = W.edge_grasp(pose, -1)
    up = np.array([p[0], p[1], 0.34])
    over = np.array([W.ROD_X + 0.10, p[1], 0.34])
    far = np.array([W.ROD_X - 0.06, p[1], float(rng.uniform(0.10, 0.14))])
    keys = [(0.0, p, yaw, 0, 0), (0.2, p, yaw, 0, 1), (1.2, up, yaw, 0, 1), (2.4, over, yaw, 0, 1),
            (3.6, far, yaw, 0, 1), (4.4, far, yaw, 0, 1), (4.402, far, yaw, 0, 0), (5.0, far + [0.0, 0, 0.15], yaw, 0, 0),
            (6.4, far + [0.0, 0, 0.15], yaw, 0, 0)]
    out.append(("rod_drape", pose, W.program_to_traj(keys)))
    return out


# ----------------------------------------------------------------------------------------------------------------
# render scene
# ----------------------------------------------------------------------------------------------------------------
def render_xml(material, mass, cams_xml, arm_assets, arm_body):
    xml = W.scene_xml(material, mass, texture="fabric_tex")
    extra_assets = ('<texture name="fabric_tex" type="2d" file="fabric.png"/>'
                    '<texture name="wood" type="2d" file="wood.png"/><material name="wood" texture="wood" texrepeat="2 2"/>'
                    '<texture name="floor" type="2d" file="floor.png"/><material name="floor" texture="floor" texrepeat="6 6"/>'
                    '<texture name="sky" type="skybox" builtin="gradient" rgb1="0.75 0.78 0.82" rgb2="0.35 0.36 0.4" width="256" height="256"/>'
                    + arm_assets)
    tx, ty = W.TABLE_HALF
    cx = W.EDGE_X - tx
    legs = "".join('<geom type="box" size="0.025 0.025 0.37" pos="%g %g -0.41" rgba="0.3 0.25 0.2 1" contype="0" conaffinity="0" group="1"/>'
                   % (cx + sx * (tx - 0.05), sy * (ty - 0.05)) for sx in (-1, 1) for sy in (-1, 1))
    posts = "".join('<geom type="cylinder" size="0.008 %g" pos="%g %g %g" rgba="0.6 0.6 0.62 1" contype="0" conaffinity="0" group="1"/>'
                    % (W.ROD_Z / 2, W.ROD_X, sy * 0.30, W.ROD_Z / 2) for sy in (-1, 1))
    world = ('<light pos="0.6 -0.4 1.6" dir="-0.35 0.25 -1" diffuse="0.75 0.73 0.7" specular="0.2 0.2 0.2" castshadow="true"/>'
             '<light pos="-0.8 0.8 1.4" dir="0.5 -0.5 -1" diffuse="0.35 0.37 0.42" castshadow="false"/>'
             '<geom type="plane" size="3 3 0.1" pos="0 0 -0.78" material="floor" contype="0" conaffinity="0" group="1"/>'
             '<geom type="box" size="0.02 2.5 1.2" pos="-1.6 0 0.4" rgba="0.82 0.8 0.76 1" contype="0" conaffinity="0" group="1"/>'
             + legs + posts + cams_xml + arm_body)
    xml = xml.replace('<asset>', '<asset>' + extra_assets, 1)
    xml = xml.replace('rgba="0.55 0.45 0.35 1" group="1"/>', 'material="wood" group="1"/>', 1)
    xml = xml.replace('<visual><global offwidth="1280" offheight="960"/>',
                      '<visual><global offwidth="1280" offheight="960"/><headlight ambient="0.35 0.35 0.35" diffuse="0.2 0.2 0.2"/>', 1)
    return xml.replace("</worldbody>", world + "</worldbody>", 1)


# ----------------------------------------------------------------------------------------------------------------
# camera degradation
# ----------------------------------------------------------------------------------------------------------------
def camera_look(rng):
    return dict(wb=rng.uniform(0.88, 1.12, 3), gain=rng.uniform(0.85, 1.15), blur=rng.uniform(0.6, 1.1),
                noise=rng.uniform(2.0, 4.0), vignette=rng.uniform(0.15, 0.3))


def degrade(img_sub, look, rng):
    """img_sub: list of float RGB renders within one exposure (motion blur), each (H, W, 3) in [0, 1]."""
    import cv2
    img = np.mean(img_sub, axis=0)
    img = cv2.GaussianBlur(img, (0, 0), look["blur"])
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    r2 = ((xx - w / 2) ** 2 + (yy - h / 2) ** 2) / ((w / 2) ** 2 + (h / 2) ** 2)
    img = img * (1 - look["vignette"] * r2)[..., None] * look["gain"] * look["wb"][None, None]
    img = np.clip(img, 0, 1) ** (1 / 1.05)
    img = img * 255 + rng.normal(0, look["noise"], img.shape) + rng.normal(0, 0.7 * look["noise"], img.shape) * np.sqrt(img)
    return np.clip(img, 0, 255).astype(np.uint8)


def encode(frames, path):
    p = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", "%dx%d" % (W_IMG, H_IMG), "-r", str(W.FPS), "-i", "-", "-c:v", "libx264",
                          "-preset", "medium", "-crf", "23", "-pix_fmt", "yuv420p", "-threads", "1", path],
                         stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f).tobytes())
    p.stdin.close()
    assert p.wait() == 0, path


# ----------------------------------------------------------------------------------------------------------------
def do_clip(args):
    import mujoco
    import carm
    fid, ci, kind, pose, C, material, mass, out_dir, frames_max = args
    seed = ord(fid) * 1000 + ci
    rng = np.random.default_rng(seed)
    d = os.path.join(out_dir, "data", "fabrics", fid, "clip%02d" % ci)
    os.makedirs(d, exist_ok=True)
    sim = W.Sim(material, mass)
    sim.place_flat(pose)
    rec = sim.run(C, record_every=W.STEPS_PER_FRAME // 2)       # frame states and the half-frame states between
    assert not sim.unstable, (fid, ci)
    frames, half = rec[0::2], rec[1::2]                       # half[k]: half a frame after frame k (motion blur)
    n = min(len(frames), frames_max or len(frames))
    rows = C[::W.STEPS_PER_FRAME][:n]
    a_assets, a_body = carm.arm_xml_parts()
    cams = carm.cameras_xml(os.path.join(out_dir, "data", "cameras.json"))
    xml = render_xml(material, mass, cams, a_assets, a_body)
    assets = {"fabric.png": png_bytes(fabric_texture(TEXTURE_KIND[fid], 7 + ord(fid))),
              "wood.png": png_bytes(wood_texture(11)), "floor.png": png_bytes(floor_texture(13))}
    m = mujoco.MjModel.from_xml_string(xml, assets)
    dd = mujoco.MjData(m)
    q = carm.solve_ik(m, dd, rows)
    qh = carm.solve_ik(m, dd, C[W.STEPS_PER_FRAME // 2::W.STEPS_PER_FRAME][:n], q0=q[0])
    rs = W.Sim.__new__(W.Sim)          # reuse Sim's vertex bookkeeping on the render model
    rs.m, rs.d = m, dd
    f = 0
    a, nv = m.flex_vertadr[f], m.flex_vertnum[f]
    rs.vbody = m.flex_vertbodyid[a:a + nv].copy()
    rs.vq = np.array([m.jnt_qposadr[m.body_jntadr[b]] for b in rs.vbody])
    rs.vd = np.array([m.jnt_dofadr[m.body_jntadr[b]] for b in rs.vbody])
    rs.body0 = m.body_pos[rs.vbody].copy()
    renderer = mujoco.Renderer(m, H_IMG, W_IMG)
    looks = [camera_look(rng) for _ in carm_cams()]
    out = {c: [] for c in carm_cams()}

    def draw(P, qa):
        rs.set_vertices(P)
        carm.set_arm(m, dd, qa)
        mujoco.mj_forward(m, dd)
        imgs = {}
        for c in carm_cams():
            renderer.update_scene(dd, c)
            imgs[c] = renderer.render().astype(np.float32) / 255.0
        return imgs

    for k in range(n):
        A = draw(frames[k], q[k])
        B = draw(half[k] if k < len(half) else frames[k], qh[k] if k < len(qh) else q[k])
        for i, c in enumerate(carm_cams()):
            out[c].append(degrade([A[c], B[c]], looks[i], rng))
    renderer.close()
    for c in carm_cams():
        encode(out[c], os.path.join(d, c + ".mp4"))
    np.save(os.path.join(d, "clamp_500hz.npy"), C.astype(np.float64))
    np.save(os.path.join(d, "arm_q.npy"), q.astype(np.float64))
    info = dict(kind=kind, fps=W.FPS, n_frames=int(n), physics_dt=W.DT, physics_steps=int(len(C)),
                frame_k_step="k * %d (frame 0 = physics step 0, before it)" % W.STEPS_PER_FRAME,
                fabric_start=[float(x) for x in pose],
                fabric_start_note="the fabric lies flat at rest: centre (x, y) and long-axis angle yaw (rad), +z face up")
    json.dump(info, open(os.path.join(d, "clip.json"), "w"), indent=1)
    return fid, ci, n


def carm_cams():
    return [c[0] for c in CAMERAS]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--only", default="P,A,B,C")
    ap.add_argument("--frames-max", type=int, default=0)
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "data"), exist_ok=True)
    json.dump(cameras_json(), open(os.path.join(a.out, "data", "cameras.json"), "w"), indent=1)
    fab = json.load(open(os.path.join(HERE, "fabrics.json")))["fabrics"]
    jobs = []
    for fid in a.only.split(","):
        progs = clip_programs(np.random.default_rng(1000 + ord(fid)))
        fd = os.path.join(a.out, "data", "fabrics", fid)
        os.makedirs(fd, exist_ok=True)
        json.dump(dict(id=fid, size_m=[W.SIZE_X, W.SIZE_Y], grid=[W.NX, W.NY], spacing_m=W.SPACING,
                       mass_kg=round(fab[fid]["mass_kg"], 6), n_clips=len(progs),
                       texture_face="+z (up when flat)"), open(os.path.join(fd, "fabric.json"), "w"), indent=1)
        for ci, (kind, pose, C) in enumerate(progs):
            jobs.append((fid, ci, kind, pose, C, fab[fid]["material"], fab[fid]["mass_kg"], a.out, a.frames_max))
    with ProcessPoolExecutor(a.workers) as ex:
        for fid, ci, n in ex.map(do_clip, jobs):
            print("[gen] %s clip%02d %d frames" % (fid, ci, n), flush=True)


if __name__ == "__main__":
    main()
