"""Review: what the Go1's mast camera can see along the way (an information audit, no solver).

    /opt/venv/bin/python3 -B tests/review/observability.py --n 16          # development sites
    /opt/venv/bin/python3 -B tests/review/observability.py --hidden

The oracle walks each site on its true pose. Every 0.4 m of travel this script renders the eight mast detents
itself (outside the mission's exposure budget, nothing is sent to the controller) and counts, by colour, the
bollard caps, the pallet stack and the dock beacon in view. It answers whether a program restricted to the stated
sensors has something to re-localise on wherever its odometry has drifted, sees where the pallet stack stands
before it has to choose an opening, and sees the dock's beacon before it docks.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests/runtime"), str(TASK/"tests/setup_payload"), str(TASK/"solution/oracle"),
                str(TASK/"tests/review")]
import mujoco  # noqa: E402
import oracle  # noqa: E402
from controls import Local  # noqa: E402


def blobs(mask, least=30):
    labels, n = ndimage.label(mask)
    return sum(1 for size in ndimage.sum(mask, labels, range(1, n+1)) if size >= least)


class Watcher(Local):
    def __init__(self, seed):
        super().__init__(seed)
        self.samples, self.mark, self.renderer = [], None, None

    def act(self, action):
        obs = super().act(action)
        dog = self.w.go1
        if dog is not None and (self.mark is None or np.linalg.norm(dog.xy-self.mark) >= .40):
            self.mark = dog.xy.copy()
            if self.renderer is None:
                self.renderer = mujoco.Renderer(dog.model, 256, 256)
            caps = stack = beacon = 0
            for detent in range(8):
                self.renderer.update_scene(dog.data, camera=f"mast{detent}")
                r, g, b = self.renderer.render().astype(int).transpose(2, 0, 1)
                caps += blobs((g > 55) & (g > 1.55*r) & (g > 1.70*b))
                stack += int(((r > 90) & (b > 60) & (r > 2.2*g) & (b > 1.6*g)).sum())
                beacon += blobs((b > 110) & (b > 2.0*r) & (b > 1.8*g))
            self.samples.append({"x": float(dog.xy[0]), "caps": caps, "stack_px": stack, "beacon": beacon})
        return obs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hidden", action="store_true")
    parser.add_argument("--n", type=int, default=16)
    args = parser.parse_args()
    graded = json.loads((TASK/"tests/hidden/sites.json").read_text())["seeds"]
    draws = [int(x) for x in np.random.default_rng(20261120).integers(1, 2**62, size=args.n)]
    seeds = graded if args.hidden else [s for s in draws if s not in set(graded)]
    worst = {"caps_min": 99, "stack_seen_before_middle_rack": 1, "beacon_seen_before_dock": 1}
    for i, seed in enumerate(seeds):
        robot = Watcher(seed)
        oracle.mission(robot, "review")
        middle = robot.w.racks[1].near
        before = [s for s in robot.samples if s["x"] < middle-.45]
        last = [s for s in robot.samples if s["x"] > robot.w.racks[2].far+.3]
        row = {"site": i, "outcome": robot.w.go1.outcome, "samples": len(robot.samples),
               "caps_min": min(s["caps"] for s in robot.samples),
               "caps_median": int(np.median([s["caps"] for s in robot.samples])),
               "stack_seen_before_middle_rack": int(any(s["stack_px"] > 150 for s in before)),
               "beacon_seen_before_dock": int(any(s["beacon"] for s in last))}
        print(json.dumps(row), flush=True)
        worst["caps_min"] = min(worst["caps_min"], row["caps_min"])
        for key in ("stack_seen_before_middle_rack", "beacon_seen_before_dock"):
            worst[key] = min(worst[key], row[key])
        robot.w.close()
    print(json.dumps({"sites": len(seeds), "worst": worst}), flush=True)


if __name__ == "__main__":
    main()
