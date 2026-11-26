"""Normalize a small, pinned selection of Kenney's CC0 Furniture Pack for MuJoCo."""

from hashlib import sha256
from pathlib import Path
import sys
from zipfile import ZipFile
import numpy as np

MODELS = """bedDouble cabinetBedDrawer loungeSofaLong loungeChair tableCoffee
bookcaseOpen desk chairDesk kitchenSink kitchenStove kitchenFridge kitchenCabinetDrawer
plantSmall1 tableRound chairCushion lampRoundFloor computerScreen books""".split()
SWATCHES = """wood woodDark metal metalDark metalMedium metalLight carpet carpetDarker
carpetWhite glass plant lamp fur _defaultMat""".split()


def convert(archive, destination):
    if sha256(archive.read_bytes()).hexdigest() != "68afa4e6dc8a53942379fb47f1e84ec735d46b77bb1c1ceb968e245693dde067":
        raise ValueError("Expected Kenney Furniture Pack 1.0 from OpenGameArt")
    destination.mkdir(exist_ok=True)
    with ZipFile(archive) as z:
        for name in MODELS:
            vertices, faces = [], []
            for line in z.read(f"Models/{name}.obj").decode().splitlines():
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == "v":
                    x, y, z0 = map(float, parts[1:4])
                    vertices.append((x, -z0, y))
                elif parts[0] == "usemtl":
                    material = SWATCHES.index(parts[1]) + 1
                elif parts[0] == "f":
                    indices = [int(v.split("/")[0]) for v in parts[1:]]
                    faces.append((indices,material-1))
            lo, hi = [list(map(f, zip(*vertices))) for f in (min, max)]
            span = [hi[i] - lo[i] for i in range(3)]
            origin = [(lo[0]+hi[0])/2, (lo[1]+hi[1])/2, lo[2]]
            # Unit bounding box, with the physical dimensions specified by the scene.
            out = ["v " + " ".join(f"{(v[i]-origin[i])/span[i]:.5f}" for i in range(3)) for v in vertices]
            unit=(np.array(vertices)-np.array(lo))/span
            uv={}
            attributed=[]
            for indices,material in faces:
                pts=unit[np.array(indices)-1]
                normal=np.cross(pts[1]-pts[0],pts[2]-pts[0])
                axis=np.argsort(np.abs(normal))[:2]
                axis.sort()
                face=[]
                for vid,p in zip(indices,pts):
                    u,v=p[axis]
                    # Keep bilinear samples inside the material's padded 4x4 atlas slot.
                    pair=(round((material%4+.02+.96*u)/4,6),
                          round((3-material//4+.02+.96*v)/4,6))
                    uv.setdefault(pair,len(uv)+1)
                    face.append(f"{vid}/{uv[pair]}")
                attributed.append("f "+" ".join(face))
            out += [f"vt {u:.6f} {v:.6f}" for u,v in uv]
            (destination / (name+".obj")).write_text("\n".join(out + attributed) + "\n")


if __name__ == "__main__":
    convert(Path(sys.argv[1]), Path(__file__).resolve().parents[1] / "furnishings")
