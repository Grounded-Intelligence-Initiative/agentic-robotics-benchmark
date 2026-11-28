"""Render a kit's picture manual: step pictures without text, each with a millimetre ruler drawn to scale.

    render_manual(kit) -> [{"name": "step_0_parts", "png_b64": ...}, ...]     (what a controller gets at t = 0)
    python3 toycar_manual.py <kit.json> <out_dir>                               (the same pictures as PNG files)

Pages (1280 x 640 PNG, two panels each):
    step_0_parts.png    the kit's parts (one row, true scale, top view) + ruler
    step_1_chassis.png  the chassis on the assembly stand (3/4 view) | the chassis from above + ruler
    step_2_axles.png    the two axles in the notches (3/4 view)       | from above + ruler
    step_3_wheels.png   the four wheels on the axle ends (3/4 view)   | from the front + ruler
    step_4_cabin.png    the cabin on top (3/4 view)                   | from the side + ruler
    step_5_done.png     the finished car on the table (3/4 view)      | from above + ruler
In each 3/4 view the parts added in that step are in full colour and the earlier ones are faded.
"""

import json
import math
import os
import sys

import numpy as np

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import toycar_env as E  # noqa: E402

PANEL = 640
BG = np.array([250, 250, 246], np.uint8)


def assembly_pose(kit):
    """Each kit part's pose in the chassis frame (chassis bottom-face centre at the origin, length along x)."""
    c, a, w, b = kit["chassis"], kit["axle"], kit["wheel"], kit["cabin"]
    zc = c["base_t"] + a["dia"] / 2
    I = np.eye(3)
    Rw = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)          # wheel axis (local z) along y
    poses = [("chassis", c, np.zeros(3), I, 1)]
    poses += [("axle", a, np.array([s * c["notch_x"], 0, zc]), I, 2) for s in (1, -1)]
    for s in (1, -1):
        for side in (1, -1):
            poses.append(("wheel", w, np.array([s * c["notch_x"], side * (a["length"] / 2 - w["thick"] / 2), zc]),
                          Rw, 3))
    poses.append(("cabin", b, np.array([0.0, 0, c["base_t"] + c["block_h"]]), I, 4))
    return poses


def faded(rgb, f):
    return tuple(float(v) * (1 - f) + 0.92 * f for v in rgb)


def scene_xml(parts, stand, cams, floor=True, shadows=True):
    """parts: list of (type, dims, pos, R, rgb)."""
    bodies = []
    for i, (typ, d, pos, R, rgb) in enumerate(parts):
        q = E.quat_from_mat(R)
        bodies.append('<body name="p%d" pos="%s" quat="%s">%s</body>' % (
            i, " ".join("%.6f" % v for v in pos), " ".join("%.6f" % v for v in q),
            E.part_geoms(typ, d, rgb, "p%d_" % i)))
    st = ""
    if stand:
        h = E.STAND_H
        st = ('<geom type="box" pos="0 0 {pz}" size="0.012 0.012 {ph}" rgba="0.62 0.63 0.65 1"/>'
              '<geom type="box" pos="0 0 {fz}" size="0.045 0.045 0.004" rgba="0.20 0.20 0.22 1"/>'
              '<geom type="box" pos="0 0 {tz}" size="{tx} {ty} 0.003" rgba="0.62 0.63 0.65 1"/>').format(
            pz=-h / 2, ph=h / 2 - 0.003, fz=-h + 0.004, tz=-0.003, tx=E.STAND_TOP[0], ty=E.STAND_TOP[1])
    fl = ""
    if floor:
        fl = '<geom name="floor" type="plane" pos="0 0 {z}" size="1 1 0.01" rgba="0.97 0.97 0.95 1"/>'.format(
            z=-E.STAND_H if stand else -0.0001)
    cam_xml = []
    for name, (pos, R, fovy, ortho) in cams.items():
        cam_xml.append('<camera name="{n}" pos="{p}" quat="{q}" fovy="{f}" {o}/>'.format(
            n=name, p=" ".join("%.5f" % v for v in pos), q=" ".join("%.6f" % v for v in E.quat_from_mat(R)),
            f=fovy, o='projection="orthographic"' if ortho else ""))
    return """<mujoco><visual><global offwidth="{W}" offheight="{W}"/><quality shadowsize="4096"/>
      <headlight ambient="0.45 0.45 0.45" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/></visual>
      <worldbody><light pos="0.3 -0.4 1.0" dir="-0.3 0.4 -1" diffuse="0.35 0.35 0.35" castshadow="{sh}"/>
      {fl}{st}{b}{c}</worldbody></mujoco>""".format(W=PANEL, fl=fl, st=st, b="".join(bodies), c="".join(cam_xml),
                                                    sh="true" if shadows else "false")


def ortho_cam(view, centre, extent):
    """An orthographic camera looking at `centre`: view 'top' (down, image right = +x), 'front' (along -x, image
    right = -y... seen from +x) or 'side' (along +y, from -y; image right = +x)."""
    centre = np.asarray(centre, float)
    if view == "top":
        x, y = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    elif view == "front":
        x, y = np.array([0, 1.0, 0]), np.array([0, 0, 1.0])     # looking along -x from +x
    else:
        x, y = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])     # looking along +y from -y
    z = np.cross(x, y)
    return centre + 0.5 * z, np.stack([x, y, z], axis=1), extent, True


def persp_cam(target, dist=0.42, az=-35.0, el=28.0, fovy=38.0):
    target = np.asarray(target, float)
    a, e = math.radians(az), math.radians(el)
    pos = target + dist * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    return pos, E.look_at(pos, target), fovy, False


def render(xml, cam):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, PANEL, PANEL)
    r.update_scene(d, cam)
    img = r.render().copy()
    r.enable_segmentation_rendering()
    r.update_scene(d, cam)
    seg = r.render()[..., 0].copy()
    r.close()
    floor = m.geom("floor").id if "floor" in [m.geom(i).name for i in range(m.ngeom)] else -2
    img[(seg < 0)] = BG
    return img, seg


def draw_ruler(img, origin_px, px_per_m, length_mm, vertical=False, start_mm=0):
    """Ruler: a dark bar with ticks every mm (short), 5 mm (mid) and 10 mm (long). origin_px = pixel of the 0 mark
    (row, col). The ruler runs right (or up, when vertical)."""
    r0, c0 = origin_px
    col = np.array([40, 40, 45], np.uint8)
    L = int(round(length_mm * px_per_m / 1000.0))
    if not vertical:
        img[max(0, r0 - 1):r0 + 1, max(0, c0):min(img.shape[1], c0 + L + 1)] = col
    else:
        img[max(0, r0 - L):r0 + 1, max(0, c0 - 1):c0 + 1] = col
    for mm in range(0, int(length_mm) + 1):
        off = int(round(mm * px_per_m / 1000.0))
        h = 14 if mm % 10 == 0 else (9 if mm % 5 == 0 else 5)
        if not vertical:
            cc = c0 + off
            if 0 <= cc < img.shape[1]:
                img[r0:min(img.shape[0], r0 + h), cc] = col
                if mm % 10 == 0 and cc + 1 < img.shape[1]:
                    img[r0:min(img.shape[0], r0 + h), cc + 1] = col
        else:
            rr = r0 - off
            if 0 <= rr < img.shape[0]:
                img[rr, max(0, c0 - h):c0] = col
                if mm % 10 == 0 and rr - 1 >= 0:
                    img[rr - 1, max(0, c0 - h):c0] = col
    return img


def world_to_px(cam, p):
    """Pixel (row, col) of world point p in an orthographic camera."""
    pos, R, extent, _o = cam
    rel = np.asarray(p, float) - pos
    s = PANEL / extent
    col = PANEL / 2 + s * float(rel @ R[:, 0])
    row = PANEL / 2 - s * float(rel @ R[:, 1])
    return int(round(row)), int(round(col))


def render_pages(kit):
    """The six pictures of a kit's manual, as RGB arrays, in order."""
    poses = assembly_pose(kit)
    c = kit["chassis"]
    L = c["length"]
    pages = {}

    def parts_upto(step, fade_before=True):
        out = []
        for typ, d, pos, R, st in poses:
            if st > step:
                continue
            rgb = E.PAINT[typ]
            if fade_before and st < step:
                rgb = faded(rgb, 0.6)
            out.append((typ, d, pos, R, rgb))
        return out

    # page 0: the parts, laid flat in a row, from above
    flat = []
    xcur = 0.0
    gap = 0.018
    row = [("chassis", c, 1), ("axle", kit["axle"], 2), ("wheel", kit["wheel"], 4), ("cabin", kit["cabin"], 1)]
    for typ, d, n in row:
        for k in range(n):
            hx, hy, hz = E.part_footprint(typ, d)
            if typ == "axle":
                hx = d["dia"] / 2
            pos = np.array([xcur + hx, 0.0, hz])
            if typ == "wheel":
                pos[1] = (k % 2 - 0.5) * (d["dia"] + 0.008)
                if k % 2 == 0 and k > 0:
                    pass
            if typ == "axle":
                pos[1] = 0.0
            flat.append((typ, d, pos.copy(), np.eye(3), E.PAINT[typ]))
            if not (typ == "wheel" and k % 2 == 0):
                xcur += 2 * hx + gap
    total = xcur - gap
    centre = np.array([total / 2, 0.0, 0.0])
    ext = total * 1.25
    cam_top = ortho_cam("top", centre, ext)
    xml = scene_xml(flat, False, {"top": cam_top}, floor=False, shadows=False)
    img, _ = render(xml, "top")
    s = PANEL / ext
    r0, c0 = world_to_px(cam_top, [0.0, -0.075, 0.0])
    draw_ruler(img, (r0, c0), s, int(total * 1000) + 5)
    page = np.concatenate([img[PANEL // 4: 3 * PANEL // 4], img[PANEL // 4: 3 * PANEL // 4]], axis=1)
    pages["step_0_parts"] = img

    def two_panel(step, ortho_view, ortho_centre, ortho_ext, ruler_at, ruler_len, vertical_ruler=None, stand=True,
                  persp_target=(0, 0, 0.02)):
        ps = parts_upto(step)
        cam_p = persp_cam(persp_target)
        img1, _ = render(scene_xml(ps, stand, {"p": cam_p}), "p")
        full = [(t, d, p, R, E.PAINT[t]) for (t, d, p, R, _c) in ps]
        cam_o = ortho_cam(ortho_view, ortho_centre, ortho_ext)
        img2, _ = render(scene_xml(full, False, {"o": cam_o}, floor=False, shadows=False), "o")
        sc = PANEL / ortho_ext
        draw_ruler(img2, world_to_px(cam_o, ruler_at), sc, ruler_len)
        if vertical_ruler is not None:
            draw_ruler(img2, world_to_px(cam_o, vertical_ruler[0]), sc, vertical_ruler[1], vertical=True)
        sep = np.full((PANEL, 4, 3), 200, np.uint8)
        return np.concatenate([img1, sep, img2], axis=1)

    ext = L * 1.5
    pages["step_1_chassis"] = two_panel(1, "top", [0, 0, 0], ext, [-L / 2, -c["width"] / 2 - 0.012, 0],
                                        int(L * 1000) + 5)
    pages["step_2_axles"] = two_panel(2, "top", [0, 0, 0], ext, [-L / 2, -kit["axle"]["length"] / 2 - 0.012, 0],
                                      int(L * 1000) + 5)
    zc = c["base_t"] + kit["axle"]["dia"] / 2
    span = kit["axle"]["length"] + 0.02
    pages["step_3_wheels"] = two_panel(3, "front", [0, 0, zc], max(ext, span * 1.4),
                                       [0.05, -span / 2, zc - kit["wheel"]["dia"] / 2 - 0.010], int(span * 1000),
                                       vertical_ruler=([0.05, span / 2 + 0.008, zc - kit["wheel"]["dia"] / 2], 60))
    top = c["base_t"] + c["block_h"] + kit["cabin"]["height"]
    pages["step_4_cabin"] = two_panel(4, "side", [0, 0, top / 2], ext,
                                      [-L / 2, -0.1, zc - kit["wheel"]["dia"] / 2 - 0.010], int(L * 1000) + 5,
                                      vertical_ruler=([L / 2 + 0.01, -0.1, zc - kit["wheel"]["dia"] / 2], 70))
    pages["step_5_done"] = two_panel(5, "top", [0, 0, 0], ext, [-L / 2, -span / 2 - 0.012, 0], int(L * 1000) + 5,
                                     stand=False, persp_target=(0, 0, 0.0))
    return [(name, pages[name]) for name in sorted(pages)]


def png_bytes(img):
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="PNG", compress_level=6)
    return buf.getvalue()


def render_manual(kit):
    """The manual as the controller receives it: [{"name", "png_b64"}], step_0 to step_5."""
    import base64
    return [{"name": name, "png_b64": base64.b64encode(png_bytes(img)).decode("ascii")}
            for name, img in render_pages(kit)]


if __name__ == "__main__":
    kit = E.load_kit(sys.argv[1])
    os.makedirs(sys.argv[2], exist_ok=True)
    for name, img in render_pages(kit):
        with open(os.path.join(sys.argv[2], name + ".png"), "wb") as f:
            f.write(png_bytes(img))
        print(name)
