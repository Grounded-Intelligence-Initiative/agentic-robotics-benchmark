"""North-up rectified onboard views of a furnished MuJoCo scene (CPU / OSMesa)."""

import base64
from io import BytesIO
import os
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import mujoco as mj
from PIL import Image

PAINT = {
    "hall": "b6ae9e",
    "oak": "c99762",
    "sage": "87af90",
    "slate": "849db5",
    "rose": "c095aa",
    "wall": "656577",
    "furniture": "285750",
    "dust": "684127",
    "box": "e4ac29",
    "red": "cb4d5d",
    "blue": "397ecb",
    "dock": "24a99c",
}


def color(k):
    v = PAINT.get(k, k)
    return " ".join(str(int(v[i : i + 2], 16) / 255) for i in (0, 2, 4)) + " 1"


def geom(node, x, y, z, size, paint, kind="box", **kwargs):
    return ET.SubElement(
        node,
        "geom",
        type=kind,
        pos=f"{x} {-y} {z}",
        size=size,
        rgba=color(paint),
        **kwargs,
    )


class Cameras:
    def __init__(self, w, overview=False):
        self.w = w
        scene = ET.Element("mujoco")
        visual = ET.SubElement(scene, "visual")
        ET.SubElement(
            visual,
            "global",
            offwidth="840" if overview else "84",
            offheight="640" if overview else "84",
        )
        ET.SubElement(visual, "quality", offsamples="1")
        ET.SubElement(
            visual, "headlight", ambient="1 1 1", diffuse="0 0 0", specular="0 0 0"
        )
        root = ET.SubElement(scene, "worldbody")
        ET.SubElement(
            root,
            "light",
            name="daylight",
            directional="true",
            dir="-.4 .55 -1",
            diffuse=".63 .61 .55",
            specular="0 0 0",
            active="false",
        )
        for (x, y), room in w.area.items():
            if (x, y) not in w.wall:
                geom(root, x, y, -0.035, ".498 .498 .035", room)
                if room != "hall":
                    geom(root, x + 0.47, y, 0.002, ".005 .495 .002", "a7a298")
        for x, y in w.wall:
            geom(root, x, y, 0.61, ".497 .497 .61", "wall")
            geom(root, x, y, 1.23, ".497 .497 .012", "wall")
            for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                if (x + dx, y + dy) in w.area and (x + dx, y + dy) not in w.wall:
                    size = ".48 .012 .05" if dy else ".012 .48 .05"
                    geom(root, x + 0.49 * dx, y + 0.49 * dy, 0.07, size, "d9d8cd")
        for (x, y), kind in w.furniture.items():
            for dx in (-0.31, 0.31):
                for dy in (-0.29, 0.29):
                    geom(root, x + dx, y + dy, 0.20, ".047 .047 .20", "775f4d")
            geom(root, x, y, 0.44, ".44 .38 .085", "furniture")
            if kind != "table":
                geom(root, x, y + 0.34, 0.66, ".44 .06 .24", "furniture")
                for dx in (-0.39, 0.39):
                    geom(root, x + dx, y, 0.55, ".05 .34 .12", "furniture")
                if kind == "sofa":
                    geom(root, x - 0.23, y + 0.23, 0.67, ".105 .085 .12", "bcb391")
            else:
                geom(root, x + 0.24, y + 0.22, 0.57, ".072 .063", "bf9b76", "cylinder")
                for dx, dy in ((0.025, 0.04), (-0.05, 0.02), (0.05, -0.045)):
                    geom(
                        root,
                        x + 0.24 + dx,
                        y + 0.22 + dy,
                        0.67,
                        ".06 .043 .09",
                        "618357",
                        "ellipsoid",
                    )
        for paint, (x, y) in {**w.mats, "dock": w.dock}.items():
            geom(root, x, y, 0.014, ".43 .42 .014", paint)
            for dx in (-0.36, 0.36):
                geom(root, x + dx, y, 0.033, ".012 .37 .006", "d8d5bd")
        for x, y in w.dust:
            geom(root, x, y, 0.031, ".19 .014", "dust", "cylinder", name=f"d{x}_{y}")
            for dx, dy in ((-0.17, 0.08), (0.19, -0.09), (0.13, 0.18)):
                geom(
                    root,
                    x + dx,
                    y + dy,
                    0.032,
                    ".052 .01",
                    "dust",
                    "cylinder",
                    name=f"f{x}_{y}_{dx}",
                )
        box = ET.SubElement(root, "body", name="carton")
        geom(box, 0, 0, 0.21, ".35 .34 .21", "box")
        geom(box, 0.19, 0, 0.426, ".025 .34 .006", "b79155")
        for ident, radius in (("ground", 1), ("air", 3)):
            body = ET.SubElement(root, "body", name=ident)
            ET.SubElement(
                body,
                "camera",
                name=ident,
                pos="0 0 3.4",
                orthographic="true",
                fovy=str(2 * radius + 1),
            )
            z = 0.12 if ident == "ground" else 1.9
            geom(
                body,
                0,
                0,
                z,
                ".31 .10" if ident == "ground" else ".16 .065",
                "3a4148",
                "cylinder",
                group="3",
            )
            if ident == "ground":
                geom(body, 0, 0, z + 0.1, ".25 .018", "b6bab9", "cylinder", group="3")
                geom(
                    body,
                    0,
                    0.10,
                    z + 0.15,
                    ".072 .035",
                    "425153",
                    "cylinder",
                    group="3",
                )
            if ident == "air":
                for dx in (-0.22, 0.22):
                    for dy in (-0.22, 0.22):
                        geom(
                            body,
                            dx * 0.5,
                            dy * 0.5,
                            z,
                            ".13 .03 .08",
                            "59666a",
                            "ellipsoid",
                            group="3",
                        )
                        geom(
                            body,
                            dx,
                            dy,
                            z,
                            ".136 .018",
                            "8b9bab",
                            "cylinder",
                            group="3",
                        )
        self.model = mj.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
        self.data = mj.MjData(self.model)
        self.renderer = mj.Renderer(self.model, height=84, width=84)
        self.pov = mj.MjvOption()
        self.pov.geomgroup[3] = 0
        self.dust_geoms = {
            p: [
                self.model.geom(n).id
                for n in [f"d{p[0]}_{p[1]}"]
                + [f"f{p[0]}_{p[1]}_{v}" for v in (-0.17, 0.19, 0.13)]
            ]
            for p in w.dust
        }
        self.last_dust = set(w.dust)

    def sync(self):
        w = self.w
        for n, p in (("carton", w.box), ("ground", w.ground), ("air", w.air)):
            self.model.body(n).pos[:] = (p[0], -p[1], 0)
        for p in self.last_dust - w.dust:
            self.model.geom_rgba[self.dust_geoms[p], 3] = 0
        self.last_dust = set(w.dust)
        mj.mj_forward(self.model, self.data)

    def view(self, name):
        ground = name == "ground"
        rad, pos = (1, self.w.ground) if ground else (3, self.w.air)
        pitch = 84 // (2 * rad + 1)
        self.renderer.update_scene(self.data, camera=name, scene_option=self.pov)
        im = self.renderer.render()
        for j in range(2 * rad + 1):
            for i in range(2 * rad + 1):
                p = (pos[0] + i - rad, pos[1] + j - rad)
                if not self.w.visible(pos, p, not ground):
                    im[j * pitch : (j + 1) * pitch, i * pitch : (i + 1) * pitch] = 0
        return im

    def observation(self):
        self.sync()

        def encode(im):
            f = BytesIO()
            Image.fromarray(im).save(f, format="PNG", compress_level=1)
            return base64.b64encode(f.getvalue()).decode()

        return dict(
            instruction=self.w.prompt,
            ground=encode(self.view("ground")),
            air=encode(self.view("air")) if self.w.fuel else None,
            moved=self.w.motion.copy(),
            left=self.w.budget - self.w.t,
            flight=self.w.fuel,
        )

    def overview(self):
        self.sync()
        cam = mj.MjvCamera()
        cam.lookat[:] = (self.w.width / 2 - 0.5, -self.w.height / 2 + 0.5, 0.17)
        cam.distance = self.w.width * 1.28
        cam.azimuth = 88
        cam.elevation = -60
        opt = mj.MjvOption()
        opt.geomgroup[3] = 1
        self.model.vis.headlight.ambient[:] = (0.54, 0.54, 0.54)
        self.model.light("daylight").active[0] = 1
        try:
            with mj.Renderer(self.model, height=550, width=800) as view:
                view.update_scene(self.data, camera=cam, scene_option=opt)
                return Image.fromarray(view.render())
        finally:
            self.model.vis.headlight.ambient[:] = (1, 1, 1)
            self.model.light("daylight").active[0] = 0

    def close(self):
        self.renderer.close()
