"""Downsample the verified CC0 diffuse maps for offline use; no downloads at test time."""

from hashlib import md5
from pathlib import Path
import sys
from PIL import Image

MAPS = {
    "herringbone_parquet": "dae9d3dd6d8a2c2e387da22fd8228265",
    "laminate_floor_03": "9ad312ee93c5374713fcc14fa98e87e6",
    "terrazzo_tiles": "12c27a579c2ed49de854b8a86e33647b",
    "painted_plaster_wall": "15ba8c1f63a5ab412b1eabe31728540a",
    "denim_fabric": "3835b5e277dd99784bfe68d985da14ae",
    "oak_veneer_02": "895eda215c5d808bbc42041b498a856c",
}

if __name__ == "__main__":
    dest = Path(__file__).resolve().parents[1] / "interior_materials"
    dest.mkdir(exist_ok=True)
    for key, checksum in MAPS.items():
        path = Path(sys.argv[1]) / ("ale-vacuum-" + key + ".jpg")
        if md5(path.read_bytes()).hexdigest() != checksum:
            raise ValueError(key)
        with Image.open(path) as im:
            im.convert("RGB").resize((512, 512), Image.Resampling.LANCZOS).save(
                dest / (key + ".jpg"), quality=90, optimize=True
            )
