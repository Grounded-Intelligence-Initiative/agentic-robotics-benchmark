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
    no_scout = mode in ("solo", "informed-solo")
    w = Home(seed, budget, 0 if no_scout else flight)
    signature = hashlib.sha256(
        repr(
            (
                sorted(w.wall),
                sorted(w.furniture.items()),
                w.box,
                sorted(w.mats.items()),
                sorted(w.dust),
                sorted(w.rugs),
                sorted(w.pieces),
                w.prompt,
                w.dock,
                w.card,
                w.mount,
                w.request,
                w.stage,
                w.target,
            )
        ).encode()
    ).hexdigest()[:16]
    cam = Cameras(w)
    team = Team(mode)
    if mode == "informed-solo":
        # Explicit diagnostic intervention: isolate loss of the shared map from loss of the work order.
        team.order = (set(w.request), w.stage, w.target)
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
            flight=0 if no_scout else flight,
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
        choices=["adaptive", "fixed", "solo", "informed-solo"],
        default=["solo", "informed-solo", "fixed", "adaptive"],
    )
    p.add_argument("--ticks", type=int, default=200)
    p.add_argument("--flight", type=int, default=36)
    p.add_argument("--preview", type=Path)
    args = p.parse_args()
    if args.preview:
        from gallery import make

        make(args.seeds[0], args.preview, None)
    for seed in args.seeds:
        for mode in args.modes:
            print(json.dumps(run(seed, mode, args.ticks, args.flight)), flush=True)
