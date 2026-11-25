"""Paired CPU rollouts. The controller sees only the same observations as a submission."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "solution")]
from cameras import Cameras
from controller import Team
from world import Home


def run(seed, mode, budget, flight):
    w = Home(seed, budget, 0 if mode == "solo" else flight)
    signature = hashlib.sha256(
        repr(
            (
                sorted(w.wall),
                sorted(w.furniture.items()),
                w.box,
                sorted(w.mats.items()),
                sorted(w.dust),
                w.prompt,
                w.dock,
            )
        ).encode()
    ).hexdigest()[:16]
    cam = Cameras(w)
    team = Team(mode)
    air_moves = 0
    try:
        while w.t < budget:
            act = team.choose(w.t, cam.observation())
            w.advance(act)
            air_moves += w.motion[1]
            if w.result()["success"]:
                break
        return dict(
            seed=seed,
            scene=signature,
            mode=mode,
            budget=budget,
            flight=flight if mode != "solo" else 0,
            air_moves=air_moves,
            **w.result(),
        )
    finally:
        cam.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, nargs="+", required=True)
    p.add_argument(
        "--modes",
        nargs="+",
        choices=["adaptive", "fixed", "solo"],
        default=["solo", "fixed", "adaptive"],
    )
    p.add_argument("--ticks", type=int, default=150)
    p.add_argument("--flight", type=int, default=36)
    p.add_argument("--preview", type=Path)
    args = p.parse_args()
    if args.preview:
        from PIL import Image, ImageDraw

        w = Home(args.seeds[0])
        cam = Cameras(w, overview=True)
        try:
            im = Image.new("RGB", (800, 760), "#16202b")
            im.paste(cam.overview(), (0, 0))
            for i, name in enumerate(("ground", "air")):
                tile = Image.fromarray(cam.view(name)).resize(
                    (168, 168), Image.Resampling.NEAREST
                )
                im.paste(tile, (100 + i * 405, 566))
                ImageDraw.Draw(im).text(
                    (100 + i * 405, 740), ("Vacuum RGB", "Scout RGB")[i], fill="white"
                )
            im.save(args.preview)
        finally:
            cam.close()
    for seed in args.seeds:
        for mode in args.modes:
            print(json.dumps(run(seed, mode, args.ticks, args.flight)), flush=True)
