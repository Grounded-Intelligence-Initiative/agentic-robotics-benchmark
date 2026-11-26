"""North-up rectified onboard views of a furnished MuJoCo scene (CPU / OSMesa)."""

import base64
from io import BytesIO
import os
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import mujoco as mj
from PIL import Image
from interior import add_materials, decorate, populate, sweepings

PAINT = {
    "hall": "c5bba5",
    "oak": "bc936b",
    "sage": "98ad95",
    "slate": "8b9eac",
    "rose": "c2a2a4",
    "wall": "656577",
    "furniture": "285750",
    "dust": "684127",
    "box": "e4ac29",
    "red": "cb4d5d",
    "blue": "397ecb",
    "dock": "24a99c",
    "runner": "945078",
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
            offwidth="1280" if overview else "84",
            offheight="840" if overview else "84",
        )
        ET.SubElement(visual, "quality", offsamples="2", shadowsize="2048")
        ET.SubElement(visual, "rgba", haze=".19 .23 .23 1")
        ET.SubElement(
            visual, "headlight", ambient=".86 .86 .86", diffuse="0 0 0", specular="0 0 0"
        )
        asset = ET.SubElement(scene, "asset")
        files = add_materials(asset, PAINT)
        files["framed_print.obj"] = (
            "v -.36 -.002 -.425\nv .36 -.002 -.425\nv .36 -.002 .425\nv -.36 -.002 .425\n"
            "v 0 .002 0\nvt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
            "f 1/1 2/2 3/3\nf 1/1 3/3 4/4\nf 2/2 1/1 5/3\nf 3/3 2/2 5/3\nf 4/4 3/3 5/3\nf 1/1 4/4 5/3\n"
        ).encode()
        ET.SubElement(asset,"mesh",name="framed_print",file="framed_print.obj")
        files["sweepings.obj"]=sweepings()
        ET.SubElement(asset,"mesh",name="sweepings",file="sweepings.obj",inertia="shell")
        ET.SubElement(asset,"material",name="glazing",rgba=".75 .87 .91 1",
                      emission=".40",specular=".70",shininess=".70")
        ET.SubElement(asset,"material",name="ceilingpaint",rgba=".96 .95 .90 1",emission=".33")
        root = ET.SubElement(scene, "worldbody")
        ET.SubElement(
            root,
            "light",
            name="daylight",
            directional="true",
            dir="-.44 .42 -1",
            diffuse=".19 .19 .18",
            specular="0 0 0",
            active="true",
        )
        ET.SubElement(root,"light",name="fill",directional="true",dir=".45 -.65 -.6",
                      diffuse=".08 .09 .10",castshadow="false",active="false")
        geom(root,(w.width-1)/2,(w.height-1)/2,-.17,
             f"{w.width/2+.22} {w.height/2+.22} .14","79776c")
        # Continuous material coordinates across each room instead of repeating a
        # texture or drawing a seam on every navigation cell.
        for room in ("hall","oak","sage","slate","rose"):
            xx,yy=zip(*(p for p,r in w.area.items() if room=="hall" or r==room))
            cx,cy=(min(xx)+max(xx))/2,(min(yy)+max(yy))/2
            sx,sy=(max(xx)-min(xx)+1)/2,(max(yy)-min(yy)+1)/2
            if room!="hall":
                sx+=.37
                sy+=.37
            ET.SubElement(root,"geom",type="box",pos=f"{cx} {-cy} {-.06 if room=='hall' else -.04}",
                          size=f"{sx} {sy} .045",material="finish_"+room)
        for x, y in w.wall:
            geom(root,x,y,1.20,".115 .115 1.20","ffffff",material="plaster")
            geom(root,x,y,2.415,".13 .13 .015","wall")
            for dx, dy in ((0, 1), (1, 0)):
                if (x+dx,y+dy) not in w.wall:
                    continue
                cx,cy=x+dx*.5,y+dy*.5
                windows = (x in (0,w.width-1) and dy or y in (0,w.height-1) and dx) and (x+y)%4 in (1,2)
                radius=".497 .12" if dx else ".12 .497"
                if windows:
                    geom(root,cx,cy,.39,radius+" .39","ffffff",material="plaster")
                    geom(root,cx,cy,1.47,(".49 .027" if dx else ".027 .49")+" .64","ffffff",material="glazing")
                    geom(root,cx,cy,1.47,".018 .018 .68","e7e6dc")
                    geom(root,cx,cy,1.47,(".49 .035" if dx else ".035 .49")+" .016","eee9dc")
                    geom(root,cx,cy,.79,(".50 .165" if dx else ".165 .50")+" .022","c7bca2")
                    geom(root,cx,cy,2.26,radius+" .14","ffffff",material="plaster")
                else:
                    geom(root,cx,cy,1.2,radius+" 1.2","ffffff",material="plaster")
                geom(root,cx,cy,.068,(".498 .145" if dx else ".145 .498")+" .063","e7e1d1")
                geom(root,cx,cy,2.416,radius+" .016","wall")
        decorate(w,root)
        geom(root,(w.width-1)/2,(w.height-1)/2,2.53,
             f"{w.width/2} {w.height/2} .065","ffffff",group="5",material="ceilingpaint")
        populate(w,root,asset,files)
        for x,y in w.rugs:
            geom(root,x,y,.012,".47 .40 .009","ffffff",material="textile")
            for offset in (-.38,.38):
                geom(root,x+offset,y,.022,".014 .40 .001","c88ba9")
                geom(root,x,y+offset,.022,".43 .008 .001","d7b9ae")
            for j in range(11):
                for edge in (-.477,.477):
                    geom(root,x+edge,y+(j-5)*.073,.012,".032 .007 .002","c4b3a0")
        for paint, (x, y) in {**w.mats, "dock": w.dock}.items():
            geom(root, x, y, 0.014, ".43 .42 .014", paint)
            for dx in (-0.36, 0.36):
                geom(root, x + dx, y, 0.033, ".012 .37 .006", "d8d5bd")
        for x, y in w.dust:
            ET.SubElement(root,"geom",type="mesh",mesh="sweepings",pos=f"{x} {-y} .011",
                          rgba=color("dust"),name=f"d{x}_{y}",contype="0",conaffinity="0")
        box = ET.SubElement(root, "body", name="carton")
        geom(box, 0, 0, 0.21, ".35 .34 .21", "box")
        geom(box, 0.19, 0, 0.426, ".025 .34 .006", "b79155")
        for ident, radius in (("ground", 1), ("air", 3)):
            body = ET.SubElement(root, "body", name=ident)
            ET.SubElement(
                body,
                "camera",
                name=ident,
                pos="0 0 4.2",
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
        self.model = mj.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"), assets=files)
        self.data = mj.MjData(self.model)
        self.renderer = mj.Renderer(self.model, height=84, width=84)
        self.pov = mj.MjvOption()
        self.pov.geomgroup[3] = 0
        self.dust_geoms = {
            p: [self.model.geom(f"d{p[0]}_{p[1]}").id]
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

    def presentation(self,cam,ceiling=False):
        self.sync()
        opt=mj.MjvOption()
        opt.geomgroup[3]=1
        opt.geomgroup[5]=int(ceiling)
        self.model.vis.headlight.ambient[:]=(.43,.45,.47)
        self.model.light("daylight").diffuse[:]=(.72,.68,.60)
        self.model.light("fill").active[0]=1
        try:
            with mj.Renderer(self.model,height=590,width=1040) as view:
                view.update_scene(self.data,camera=cam,scene_option=opt)
                return Image.fromarray(view.render())
        finally:
            self.model.vis.headlight.ambient[:]=(.86,.86,.86)
            self.model.light("daylight").diffuse[:]=(.19,.19,.18)
            self.model.light("fill").active[0]=0

    def interior(self,room):
        import math
        self.sync()
        body=self.model.body("display_"+room)
        center=self.data.xpos[body.id].copy()
        inward=-self.data.xmat[body.id].reshape(3,3)[:,1]
        lateral=self.data.xmat[body.id].reshape(3,3)[:,0]
        cam=mj.MjvCamera()
        # Stand inside the room and look toward its framed print; never a solver view.
        cells=[p for p,r in self.w.area.items() if r==room and p not in self.w.furniture and not self.w.blocked(p)]
        # Pick a free corner in the same room (not a fixed offset that can lie inside a cabinet).
        def position(p):
            return self.data.xpos[body.id]*0+(p[0],-p[1],1.38)
        def utility(p):
            vec=position(p)-center
            return float(vec@inward) + .32*abs(float(vec@lateral))
        eye=position(max(cells,key=utility))
        focus=center+inward*1.20
        cam.lookat[:]=(focus[0],focus[1],.94)
        dx,dy=focus[0]-eye[0],focus[1]-eye[1]
        cam.azimuth=math.degrees(math.atan2(dy,dx))
        cam.elevation=-math.degrees(math.atan2(eye[2]-.94,math.hypot(dx,dy)))
        cam.distance=math.hypot(dx,dy)
        return self.presentation(cam,ceiling=True)

    def overview(self):
        self.sync()
        cam = mj.MjvCamera()
        cam.lookat[:] = (self.w.width / 2 - 0.5, -self.w.height / 2 + 0.5, 0.17)
        cam.distance = max(self.w.width, self.w.height) * 1.82
        cam.azimuth = 113
        cam.elevation = -55
        return self.presentation(cam)

    def close(self):
        self.renderer.close()
