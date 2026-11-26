"""Render the same furniture meshes and floor finishes in viewer and robot cameras."""

from functools import lru_cache
from io import BytesIO
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).with_name("furnishings")
SURFACES = Path(__file__).with_name("interior_materials")
MATERIALS = """936944 684936 829599 343e45 69767a bfcaca
3c6864 304f59 e8dfce 879d9e 507d55 eecb82 bcb6a2 8d8c80""".split()
HEIGHT = {
    "bedDouble": 0.65,
    "cabinetBedDrawer": 0.64,
    "lampRoundFloor": 1.35,
    "loungeSofaLong": 0.8,
    "tableCoffee": 0.42,
    "loungeChair": 0.76,
    "bookcaseOpen": 1.53,
    "desk": 0.80,
    "chairDesk": 0.84,
    "plantSmall1": 0.74,
    "kitchenSink": 0.80,
    "kitchenStove": 0.82,
    "kitchenFridge": 1.58,
    "kitchenCabinetDrawer": 0.82,
    "computerScreen": 0.38,
    "books": 0.16,
}


def png(arr):
    out = BytesIO()
    Image.fromarray(arr.astype("uint8")).save(out, "PNG")
    return out.getvalue()


@lru_cache(maxsize=12)
def finish(room, swatch):
    base = np.asarray(list(bytes.fromhex(swatch)))
    name = {
        "oak": "herringbone_parquet",
        "rose": "laminate_floor_03",
        "hall": "terrazzo_tiles",
        "slate": "terrazzo_tiles",
        "sage": "herringbone_parquet",
        "plaster": "painted_plaster_wall",
        "veneer": "oak_veneer_02",
        "runner": "denim_fabric",
    }[room]
    with Image.open(SURFACES / (name + ".jpg")) as im:
        raw = np.asarray(im, dtype=float)
    norm = raw / np.maximum(raw.mean(axis=(0, 1)), 1)
    return png(np.clip(base * (0.32 + 0.68 * norm.clip(0.60, 1.35)), 0, 255))


@lru_cache(maxsize=1)
def furniture_atlas():
    out = Image.new("RGB", (512, 512))
    for i, swatch in enumerate(MATERIALS):
        typ = "veneer" if i in (0, 1) else "runner" if i in (6, 7, 8, 12) else None
        tile = (
            Image.open(BytesIO(finish(typ, swatch))).resize(
                (128, 128), Image.Resampling.LANCZOS
            )
            if typ
            else Image.new("RGB", (128, 128), "#" + swatch)
        )
        out.paste(tile, ((i % 4) * 128, (i // 4) * 128))
    return png(np.asarray(out))


@lru_cache(maxsize=1)
def sweepings():
    # A thin irregular patch and scattered grains, not a pile of rigid cylinders.
    rng = np.random.default_rng(4311)
    verts = []
    faces = []

    def flake(x, y, a, b, n):
        i = len(verts) + 1
        verts.append((x, y, 0.0025))
        for angle in np.arange(n) * 2 * np.pi / n:
            jitter = rng.uniform(0.78, 1.2)
            verts.append(
                (x + np.cos(angle) * a * jitter, y + np.sin(angle) * b * jitter, 0.0003)
            )
        for j in range(n):
            faces.append((i, i + 1 + j, i + 1 + (j + 1) % n))

    flake(0, 0, 0.075, 0.052, 13)
    for _ in range(180):
        x, y = np.clip(rng.normal(size=2), -2.5, 2.5) * (0.111, 0.079)
        s = rng.uniform(0.003, 0.020)
        flake(x, y, s, 0.64 * s, 5)
    return (
        "\n".join(
            ["v %.5f %.5f %.5f" % v for v in verts] + ["f %s %s %s" % f for f in faces]
        )
        + "\n"
    ).encode()


@lru_cache(maxsize=4)
def print_texture(index):
    im = Image.new(
        "RGB", (320, 400), ("#efe5cd", "#e0e7dd", "#ede3da", "#e6d8c5")[index]
    )
    pencil = ImageDraw.Draw(im)
    inks = (
        ("#bf6848", "#366761", "#c99b5e"),
        ("#688b87", "#405669", "#bb9460"),
        ("#a67173", "#354e5a", "#d2a66f"),
        ("#af7149", "#525a48", "#cbb28e"),
    )[index]
    pencil.ellipse((140, 44, 238, 142), fill=inks[2])
    pencil.polygon(
        [
            (20, 292),
            (20, 244),
            (111, 144),
            (186, 217),
            (245, 165),
            (302, 254),
            (302, 292),
        ],
        fill=inks[0],
    )
    pencil.polygon(
        [(20, 313), (20, 266), (130, 220), (225, 258), (302, 201), (302, 313)],
        fill=inks[1],
    )
    for y in (332, 340):
        pencil.line((33, y, 135 if y == 332 else 99, y), fill=inks[1], width=2)
    return png(np.asarray(im))


def add_materials(asset, colors):
    files = {}

    def mat(name, data, repeat="1 1", uniform="false"):
        filename = name + ".png"
        files[filename] = data
        ET.SubElement(asset, "texture", name=name, type="2d", file=filename)
        ET.SubElement(
            asset,
            "material",
            name=name,
            texture=name,
            specular=".08",
            shininess=".08",
            texrepeat=repeat,
            texuniform=uniform,
        )

    for name in ("hall", "oak", "sage", "slate", "rose"):
        mat("finish_" + name, finish(name, colors[name]), ".43 .43", "true")
    mat("plaster", finish("plaster", "dfdbcf"), ".70 .70", "true")
    for i in range(4):
        mat("art" + str(i), print_texture(i))
    mat("textile", finish("runner", colors["runner"]))
    mat("joinery", furniture_atlas())
    return files


def decorate(w, root):
    """Dress a real, room-facing wall, away from doors; all details fit inside its wall cell."""
    for i, room in enumerate(("oak", "sage", "slate", "rose")):
        cells = {p for p, r in w.area.items() if r == room and not w.blocked(p)}
        options = []
        for x, y in sorted(cells):
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (x + dx, y + dy)
                if (
                    q not in w.wall
                    or q[0] in (0, w.width - 1)
                    or q[1] in (0, w.height - 1)
                ):
                    continue
                tx, ty = -dy, dx
                # Choose a suitable uninterrupted interior wall, not a doorway.
                if all((q[0] + tx * j, q[1] + ty * j) in w.wall for j in (-1, 1)):
                    options.append((q, dx, dy))
        if not options:
            continue
        (x, y), dx, dy = options[len(options) // 2]
        # Local X is horizontal along wall, local +Y faces into wall. Signed coordinates
        # in the parent body are MuJoCo world coordinates, not image row coordinates.
        angle = {(0, -1): 0, (0, 1): 180, (1, 0): -90, (-1, 0): 90}[dx, dy]
        body = ET.SubElement(
            root,
            "body",
            name="display_" + room,
            pos=f"{x - dx * 0.18} {-y + dy * 0.18} 0",
            euler=f"0 0 {angle}",
        )

        def piece(pos, size, rgba=None, **extra):
            if rgba:
                extra["rgba"] = rgba
            return ET.SubElement(body, "geom", type="box", pos=pos, size=size, **extra)

        piece("0 -.016 1.62", ".46 .033 .54", ".30 .22 .15 1")
        piece("0 -.056 1.62", ".423 .010 .500", ".91 .89 .82 1")
        # A real vertical textured quad; UVs are invariant to the orientation of its wall.
        ET.SubElement(
            body,
            "geom",
            type="mesh",
            mesh="framed_print",
            material="art" + str(i),
            pos="0 -.070 1.62",
            contype="0",
            conaffinity="0",
        )
        piece("0 -.070 .92", ".60 .09 .026", ".55 .42 .27 1")
        for j in range(3):
            piece(
                f"{-0.36 + j * 0.075} -.09 {1.03 + 0.015 * (j % 2)}",
                ".029 .045 .079",
                (".28 .38 .37 1", ".60 .39 .26 1", ".78 .73 .61 1")[j],
            )
        for u in (-0.68, 0.68):
            piece(f"{u} -.044 1.71", ".042 .04 .14", ".68 .52 .33 1")
            ET.SubElement(
                body,
                "geom",
                type="cylinder",
                pos=f"{u} -.106 1.74",
                size=".060 .105",
                rgba=".97 .84 .61 1",
            )


def populate(w, root, asset, files):
    cache = {}

    def object_at(name, x, y, z, sx, sy, sz, deg):
        key = (name, sx, sy, sz)
        if key not in cache:
            file = name + ".obj"
            files.setdefault(file, (ROOT / file).read_bytes())
            ident = "item" + str(len(cache))
            ET.SubElement(asset, "mesh", name=ident, file=file, scale=f"{sx} {sy} {sz}")
            cache[key] = ident
        ET.SubElement(
            root,
            "geom",
            type="mesh",
            mesh=cache[key],
            material="joinery",
            pos=f"{x} {-y} {z}",
            euler=f"0 0 {deg}",
            contype="0",
            conaffinity="0",
        )

    for x, y, sx, sy, m, a in w.pieces:
        # A rug pad is an ordinary visible object below furniture, not a synthetic label.
        # Keep it low so thin shelves, chair legs and their shadows remain visible.
        ET.SubElement(
            root,
            "geom",
            type="box",
            pos=f"{x} {-y} .009",
            size=f"{0.49 * sx} {0.49 * sy} .009",
            euler=f"0 0 {a}",
            rgba=".185 .249 .247 1",
        )
        object_at(m, x, y, 0.02, sx * 0.93, sy * 0.89, HEIGHT[m], a)
        if m == "desk":
            object_at("computerScreen", x, y, 0.82, 0.52, 0.24, 0.39, a)
        elif m == "tableCoffee":
            object_at("books", x + 0.06, y, 0.44, 0.32, 0.24, 0.13, a)
