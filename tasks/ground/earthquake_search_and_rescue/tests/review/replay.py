"""Render a mission video from a trajectory record of the rubble cell (out/traj_<tag>.npz under /opt/arb-verify of a
kept verifier sandbox: time, qpos and eq_active samples at 4 Hz). Maintainers' review tool; nothing runs it.

usage: python3 tests/review/replay.py TRAJ.npz OUT.mp4 [--kind hidden] [--fps 24] [--every 1] [--size 1280x720]
       (needs mujoco, numpy, Pillow, ffmpeg and an OpenGL backend: MUJOCO_GL=egl on a GPU host, osmesa elsewhere)

Layout: a chase camera over the robot (ceilings and dropped slabs drawn see-through), the robot's mast camera as an
inset, a top-down cutaway map of the site with the path so far, a caption with the time. H.264 via ffmpeg (crf 30).
Host-side tool (EGL).
"""

import argparse
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))     # tests/ (rubblecell)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from rubblecell.plan import draw_plan  # noqa: E402
from rubblecell.robot import Crawler  # noqa: E402
from rubblecell.scene import build_scene  # noqa: E402

SEE_THROUGH = ("ceil", "upslab", "upwall", "upwin", "pancake", "voidslab", "hangpanel", "nbuilding", "nwin", "sign",
               "fillslab", "backwall", "ceilfill")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("out")
    ap.add_argument("--kind", default=None)
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--title", default="")
    ap.add_argument("--crf", type=int, default=30)
    a = ap.parse_args()
    z = np.load(a.traj, allow_pickle=True)
    traj = z["traj"]
    seed = int(z["seed"])
    kind = a.kind or str(z["kind"])
    W, H = [int(v) for v in a.size.split("x")]
    plan = draw_plan(seed, kind)
    sc = build_scene(plan)
    m, d = sc.model, sc.data
    Crawler(m, d).set_initial()
    nq, neq = m.nq, m.neq
    S = sc.site
    # see-through structure for the chase view
    hide = np.array([i for i in range(m.ngeom) if m.geom(i).name.startswith(SEE_THROUGH)], int)
    m.vis.map.znear = 0.02 / max(1.0, m.stat.extent)
    m.vis.map.zfar = 150.0 / max(1.0, m.stat.extent)
    r_main = mujoco.Renderer(m, H, W)
    iw, ih = W // 3, (W // 3) * 3 // 4
    r_cam = mujoco.Renderer(m, ih, iw)
    mw = W // 4
    mh = int(mw * (S.hy1 - 0.0 + 3.0 - (-6.0)) / (S.W + 6.0)) if False else int(mw * (S.D + 9.0) / (S.W + 4.0))
    r_map = mujoco.Renderer(m, max(64, mh), max(64, mw))
    chassis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (W, H), "-r",
           str(a.fps), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", str(a.crf), "-pix_fmt", "yuv420p",
           "-movflags", "+faststart", a.out]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
        small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
    except OSError:
        font = small = ImageFont.load_default()
    path = []
    cam_yaw = None
    gid0 = m.geom_group.copy()
    for k in range(0, len(traj), a.every):
        row = traj[k]
        t = float(row[0])
        d.qpos[:] = row[1:1 + nq]
        if neq:
            d.eq_active[:] = (row[1 + nq:1 + nq + neq] > 0.5).astype(np.uint8)
        mujoco.mj_forward(m, d)
        p = d.xpos[chassis].copy()
        R = d.xmat[chassis].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        cam_yaw = yaw if cam_yaw is None else cam_yaw + 0.15 * ((yaw - cam_yaw + math.pi) % (2 * math.pi) - math.pi)
        path.append(p[:2].copy())
        # chase view: structure overhead hidden
        m.geom_group[hide] = 5
        c = mujoco.MjvCamera()
        c.lookat[:] = p + np.array([0, 0, 0.2])
        inside = p[1] > 0.3
        c.distance = 3.2 if inside else 4.0
        c.azimuth = math.degrees(cam_yaw) + (20.0 if inside else 0.0)
        c.elevation = -38.0 if inside else -22.0
        r_main.update_scene(d, c)
        img = r_main.render()
        # top-down map
        cm = mujoco.MjvCamera()
        cm.lookat[:] = [S.W / 2, (S.D - 6.0) / 2, 0]
        cm.distance = max(S.W + 4, S.D + 9) * 1.15
        cm.azimuth = 90
        cm.elevation = -89.9
        r_map.update_scene(d, cm)
        mp = r_map.render()
        m.geom_group[:] = gid0
        # the robot's own view
        r_cam.update_scene(d, camera="mast_rgbd")
        ci = r_cam.render()
        frame = Image.fromarray(img)
        dr = ImageDraw.Draw(frame)
        frame.paste(Image.fromarray(ci), (W - iw - 12, 12))
        dr.rectangle([W - iw - 13, 11, W - 11, 12 + ih], outline=(255, 255, 255), width=2)
        dr.text((W - iw - 4, 16), "robot mast camera", fill=(255, 255, 255), font=small)
        # map with path
        mi = Image.fromarray(mp)
        md = ImageDraw.Draw(mi)
        mw_, mh_ = mi.size

        def to_px(xy):
            # top-down camera: x right, y up; span estimated from the camera distance and fovy
            span_y = 2 * cm.distance * math.tan(math.radians(m.vis.global_.fovy / 2))
            span_x = span_y * mw_ / mh_
            u = (xy[0] - cm.lookat[0]) / span_x * mw_ + mw_ / 2
            v = -(xy[1] - cm.lookat[1]) / span_y * mh_ + mh_ / 2
            return u, v
        if len(path) > 1:
            md.line([to_px(q) for q in path], fill=(255, 200, 0), width=2)
        u, v = to_px(p)
        md.ellipse([u - 4, v - 4, u + 4, v + 4], fill=(255, 60, 0))
        frame.paste(mi, (12, H - mh_ - 12))
        dr.rectangle([11, H - mh_ - 13, 12 + mw_, H - 11], outline=(255, 255, 255), width=2)
        txt = "%s   t = %5.1f s" % (a.title, t)
        dr.text((16, 12), txt, fill=(255, 255, 255), font=font, stroke_width=2, stroke_fill=(0, 0, 0))
        ff.stdin.write(np.asarray(frame.convert("RGB")).tobytes())
    ff.stdin.close()
    ff.wait()
    for r in (r_main, r_cam, r_map):
        try:
            r.close()
        except Exception:  # noqa: BLE001
            pass
    print("wrote", a.out, os.path.getsize(a.out) // 1024, "KiB", len(traj) // a.every, "frames")


if __name__ == "__main__":
    main()
