"""How much room the image-only reference has, on development seeds, under other settings than the stated ones.

    /opt/venv/bin/python3 tests/review/margins.py <out.jsonl> --seeds 3001-3100 [--light 1.4] [--presses 500]
        [--frames 120] [--blocks 2]

Runs solution/reference_policy.py against the robot service's own Runtime in one process (no socket, no engine), in
the task image, with the world's constants overridden as asked: the floodlight's reach (the reference is told the
same value, as the instruction would), the press and frame budgets, and `--blocks k`, with which the reference sees
every frame through k x k pixel blocks (a coarser camera). One row per seed: deliveries, presses, frames, contacts.
Used for the design measurements recorded in README.md of this folder; nothing here is part of an episode.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
sys.path.insert(0, str(TASK/"solution"))
import numpy as np  # noqa: E402
import service  # noqa: E402
import world  # noqa: E402
import reference_policy  # noqa: E402


class Local:
    """The reference's client: the service's Runtime, called directly."""

    def __init__(self, seed):
        self.runtime = service.Runtime(tempfile.mkdtemp(), seed, 0)

    def request(self, method, **params):
        try:
            return self.runtime.call({"method": method, "params": params})
        except (TypeError, KeyError, OverflowError) as exc:
            raise ValueError(str(exc))

    def observe(self):
        return self.request("observe")

    def act(self, action):
        return self.request("act", action=action)

    def advance(self):
        return self.request("advance")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out")
    p.add_argument("--seeds", default="3001-3100")
    p.add_argument("--light", type=float)
    p.add_argument("--presses", type=int)
    p.add_argument("--frames", type=int)
    p.add_argument("--blocks", type=int, default=1)
    args = p.parse_args()
    if args.light is not None:
        world.LIGHT_RADIUS = reference_policy.LIGHT = args.light
    if args.presses is not None:
        world.ACTION_LIMIT = args.presses
    if args.frames is not None:
        world.FRAME_LIMIT = args.frames
    service.REQUEST_LIMIT = world.REQUEST_LIMIT = world.ACTION_LIMIT+world.FRAME_LIMIT+40
    if args.blocks > 1:
        decode, k = reference_policy.decode, args.blocks

        def coarse(obs):
            im = decode(obs).astype(float)
            n = im.shape[0]//k
            small = im.reshape(n, k, n, k, 3).mean(axis=(1, 3))
            return np.repeat(np.repeat(small, k, axis=0), k, axis=1).round().astype(np.uint8)
        reference_policy.decode = coarse
    first, last = (int(v) for v in args.seeds.split("-"))
    with open(args.out, "a") as out:
        for seed in range(first, last+1):
            client = Local(seed)
            try:
                reference_policy.solve(client)
                error = None
            except Exception as exc:        # a crash of the reference is a row, not the end of the sweep
                error = f"{type(exc).__name__}: {exc}"
            w = client.runtime.world
            row = {"seed": seed, "light": world.LIGHT_RADIUS, "presses_budget": world.ACTION_LIMIT,
                   "frames_budget": world.FRAME_LIMIT, "blocks": args.blocks, "error": error, **w.metrics()}
            out.write(json.dumps(row)+"\n")
            out.flush()
            w.close()


if __name__ == "__main__":
    main()
