"""Blender (EEVEE, headless): the task's rasterised camera frames from the site's USD assets.
    blender -b --python bl_render.py -- --scene scene.json --shots shots.json --out <dir> [--samples 16]
scene.json: {"hall": {"usd", "t": [x, y, z]}, "props": [{"usd", "pos": [x, y, z], "yaw": rad}], "lamps": [[x, y, z], ...],
             "lamp_w": watts per lamp, "world": grey level, "extra": [usd files at the origin]}
shots.json: {"w", "h", "fx", "fy", "cx", "cy", "shots": [{"name", "eye": [3], "R": 3x3 (world from optical frame: x right,
             y down, z forward)}]}
One scene, two renderers: these are the meshes, materials and maps the OVRTX look renders."""
import bpy, json, math, os, sys
from mathutils import Matrix, Vector
argv = sys.argv[sys.argv.index("--") + 1:]
def arg(k, d=None):
    return argv[argv.index(k) + 1] if k in argv else d
S = json.load(open(arg("--scene"))); Q = json.load(open(arg("--shots"))); out = arg("--out"); os.makedirs(out, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
def imp(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.usd_import(filepath=path, import_lights=False, import_cameras=False)
    new = [o for o in bpy.data.objects if o not in before]
    roots = [o for o in new if o.parent is None]
    e = bpy.data.objects.new("grp_%d" % len(bpy.data.objects), None); sc.collection.objects.link(e)
    for r in roots:
        r.parent = e
    return e, new
e, _ = imp(S["hall"]["usd"]); e.location = Vector(S["hall"]["t"])
cache = {}
for i, p in enumerate(S.get("props", [])):
    if p["usd"] not in cache:
        cache[p["usd"]] = imp(p["usd"])
        g = cache[p["usd"]][0]
    else:
        src, objs = cache[p["usd"]]
        g = bpy.data.objects.new("inst_%d" % i, None); sc.collection.objects.link(g)
        m = {}
        for o in [src] + objs:
            if o is src:
                continue
            c = o.copy(); sc.collection.objects.link(c); m[o] = c
        for o, c in m.items():
            c.parent = m.get(o.parent, g)
    g.location = Vector(p["pos"]); g.rotation_euler = (0, 0, p.get("yaw", 0.0))
for x in S.get("extra", []):
    imp(x)
for i, l in enumerate(S.get("lamps", [])):
    d = bpy.data.lights.new("lamp_%d" % i, "POINT"); d.energy = S.get("lamp_w", 400.0); d.shadow_soft_size = 0.4; d.color = (1.0, 0.96, 0.9)
    try:
        d.shadow_maximum_resolution = float(S.get("shadow_res", 0.03))
    except Exception as ex:
        print("light settings:", ex)
    o = bpy.data.objects.new("lamp_%d" % i, d); o.location = Vector(l); sc.collection.objects.link(o)
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True
w.node_tree.nodes["Background"].inputs[0].default_value = (S.get("world", 0.05),) * 3 + (1.0,)
sc.render.engine = "BLENDER_EEVEE"
try:
    sc.eevee.shadow_pool_size = "1024"
    sc.eevee.taa_render_samples = int(arg("--samples", "16")); sc.eevee.use_gtao = True; sc.eevee.use_shadows = True; sc.eevee.use_raytracing = False
except Exception as ex:
    print("eevee settings:", ex)
sc.view_settings.view_transform = "Standard"; sc.view_settings.exposure = float(S.get("exposure", 0.0))
sc.render.image_settings.file_format = "PNG"; sc.render.image_settings.color_mode = "RGB"
W, H = Q["w"], Q["h"]; sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = W, H, 100
cd = bpy.data.cameras.new("cam"); co = bpy.data.objects.new("cam", cd); sc.collection.objects.link(co); sc.camera = co
cd.sensor_fit = "HORIZONTAL"; cd.sensor_width = 36.0; cd.lens = 36.0 * Q["fx"] / W
cd.shift_x = -(Q["cx"] - W / 2.0) / W; cd.shift_y = (Q["cy"] - H / 2.0) / W; cd.clip_start = 0.05; cd.clip_end = 200.0
F = Matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))
for s in Q["shots"]:
    R = Matrix(s["R"]) @ F
    M = R.to_4x4(); M.translation = Vector(s["eye"]); co.matrix_world = M
    sc.render.filepath = os.path.join(out, s["name"] + ".png"); os.makedirs(os.path.dirname(sc.render.filepath), exist_ok=True)
    bpy.ops.render.render(write_still=True)
import time as _t
print("[bl_render] %d shots" % len(Q["shots"]))
