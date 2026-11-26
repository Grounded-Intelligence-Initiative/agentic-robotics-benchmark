"""Generate reviewer images and public RGB examples from real simulator observations."""
import argparse
import base64
from io import BytesIO
import json
from pathlib import Path
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "solution")]
from cameras import Cameras
from controller import Team
from world import Home


def font(size, bold=False):
    name = "DejaVuSans" + ("-Bold" if bold else "") + ".ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default(size=size)


def make(seed, preview, examples):
    w = Home(seed)
    cam, team, frames = Cameras(w, overview=True), Team(), []
    try:
        overview = cam.overview()
        if preview:
            for room in ("oak","sage","slate","rose"):
                try:
                    inside=cam.interior(room)
                except KeyError:
                    continue
                inside.save(preview.with_name(preview.stem+"-"+room+".jpg"),quality=93)
        for _ in range(30):
            obs = cam.observation()
            if w.t in (0, 4, 9, 14, 19, 24, 29):
                frames.append({"type":"act","t":w.t,"obs":obs})
            w.advance(team.choose(w.t,obs))
    finally:
        cam.close()
    if examples:
        examples.parent.mkdir(exist_ok=True,parents=True)
        examples.write_text("".join(json.dumps(r,separators=(",",":"))+"\n" for r in frames))
    if not preview:
        return
    # Place the uncropped house in one panel, and display two simultaneous, unmodified
    # camera observations from an actual trajectory. No labels are overlaid on the inputs.
    out = Image.new("RGB",(1200,710),"#182021")
    draw = ImageDraw.Draw(out)
    draw.text((32,23),"Scout-assisted home vacuum",font=font(33,True),fill="#eff3ef")
    draw.text((34,71),"Furnished household  /  shared 200-tick deadline",font=font(17),fill="#b1c6bd")
    mask = overview.convert("L").point(lambda v: 255 if v>8 else 0)
    bb = mask.getbbox()
    cropped = overview.crop(bb) if bb else overview
    cropped.thumbnail((772,510),Image.Resampling.LANCZOS)
    out.paste(cropped,(26+(772-cropped.width)//2,115+(510-cropped.height)//2))
    draw.text((32,640),"Reviewer view only",font=font(21,True),fill="#e2ece6")
    draw.text((33,672),"Neither robot receives a floor plan.",font=font(16),fill="#adbbb6")
    # Prefer a frame with useful visible ground texture and an airborne scout.
    def detail(frame):
        return sum(len(Image.open(BytesIO(base64.b64decode(frame["obs"][k]))).getcolors(8000) or [])
                   for k in ("ground","air"))
    frame=max(frames[1:],key=detail)
    for y,k,title,subtitle in ((114,"air","Survey drone","7 x 7 local footprint"),
                               (385,"ground","Ground vacuum","3 x 3 local footprint")):
        obs=Image.open(BytesIO(base64.b64decode(frame["obs"][k]))).convert("RGB")
        out.paste(obs.resize((210,210),Image.Resampling.NEAREST),(828,y))
        draw.text((828,y+215),title,font=font(20,True),fill="#e2ece6")
        draw.text((828,y+242),subtitle,font=font(15),fill="#adbbb6")
    draw.text((828,671),f"Same instant  /  tick {frame['t']}",font=font(16),fill="#a4c6b8")
    out.save(preview,optimize=True)


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--seed",type=int,required=True)
    ap.add_argument("--preview",type=Path)
    ap.add_argument("--examples",type=Path)
    args=ap.parse_args()
    make(args.seed,args.preview,args.examples)
