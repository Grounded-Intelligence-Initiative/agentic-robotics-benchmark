"""Author-only matched frame registration controls via the public Unix client."""
import argparse
import base64
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent/"setup_payload"), str(HERE.parents[1]/"solution/oracle")]
from client import Client
from oracle import deploy
from registration import mast_scan, register
from survey import scan

ARMS = ("registered", "unregistered")
FLIP = 1 << 33


class EvidenceClient(Client):
    def __init__(self, path, evidence):
        super().__init__(path)
        self.evidence = evidence

    def act(self, action):
        obs = super().act(action)
        if self.evidence and obs["phase"] == 0:
            x, y = (round(v, 2) for v in obs["tracking_xy_m"])
            if x in (-.40, .68) and y in (-1.50, 0., 1.50):
                (self.evidence/f"rgb-{x:g}-{y:g}.png").write_bytes(
                    base64.b64decode(obs["images"][0]))
        return obs

    def look(self, detent):
        obs = super().look(detent)
        if self.evidence:
            (self.evidence/f"mast-{detent}.png").write_bytes(base64.b64decode(obs["images"][0]))
        return obs


def trial(seed, arm, directory, save_frames, pose_only=False, fixed=None):
    directory.mkdir()
    (directory/"seed").write_text(str(seed))
    command = [sys.executable, "-B", str(HERE.parent/"runtime/service.py"), "--mode", "graded",
               "--directory", ".", "--socket", "socket", "--seed-file", "seed"]
    start = time.monotonic()
    with (directory/"service.log").open("w") as log:
        proc = subprocess.Popen(command, cwd=directory, stdout=log, stderr=subprocess.STDOUT)
        try:
            while not (directory/"ready").exists():
                if proc.poll() is not None or time.monotonic()-start > 10:
                    raise RuntimeError(f"service failed to start: {directory}")
                time.sleep(.02)
            client = EvidenceClient(str(directory/"socket"), directory if save_frames else None)
            surveyed = scan(client)
            passages, _ = surveyed.passages()
            chart = surveyed.landmarks()
            if pose_only:
                initial = client.advance()
                local, offset = mast_scan(client, initial)
                pose, details = register(local, chart, offset)
                report = {"applied_pose": pose.tolist(), "ground_cap_xy": local.tolist(), **details}
            elif arm == "registered":
                report = deploy(client, surveyed)
            else:
                def frozen(local, chart, offset):
                    return np.array(fixed if fixed is not None else [0., 0., 0.]), {
                        "matched_caps": 0, "ground_caps": len(local), "uav_caps": len(chart)}
                report = deploy(client, surveyed, align=frozen)
            # Author scoring only after the returned estimate is final. No truth goes to deploy().
            state = json.loads((directory/"state.json").read_text())
            assert (state["done"] or pose_only) and not state["internal_errors"]
            truth = state["metrics"]["launch_pose_world"]
            error = report["applied_pose"][2]-truth[2]
            return {
                **report,
                "translation_error_m": math.dist(report["applied_pose"][:2], truth[:2]),
                "yaw_error_deg": abs(math.degrees(math.atan2(math.sin(error), math.cos(error)))),
                "requests": state["requests"],
                "busy_seconds": state["busy_seconds"],
                "wall_seconds": round(time.monotonic()-start, 2),
                "rgb_passages": passages,
                "uav_cap_xy": chart.tolist(),
                **state["metrics"],
            }
        finally:
            proc.terminate()
            proc.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--sites", nargs="+", type=int, default=list(range(8)))
    parser.add_argument("--arms", nargs="+", choices=ARMS, default=list(ARMS))
    parser.add_argument("--pilot", action="store_true", help="two development scenes, both starts; pose only")
    parser.add_argument("--mirrors", nargs="*", type=int, default=[0, 3],
                        help="also run the opposite launch in these otherwise identical scenes")
    parser.add_argument("--replay-from", type=Path,
                        help="freeze a previously successful public RGB estimate on both launches")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir()
    seeds = json.loads((HERE.parent/"hidden/sites.json").read_text())["seeds"]
    fixed = {}
    if args.replay_from:
        if args.pilot:
            parser.error("choose either pilot or fixed-calibration replay")
        for row in map(json.loads, args.replay_from.read_text().splitlines()):
            if row["arm"] == "registered" and not row["mirror"] and row["success"]:
                fixed[row["site"]] = row["applied_pose"]
    with (out/"results.jsonl").open("w") as stream:
        cases = ([(i, seed, reflected) for i, seed in enumerate([1701, 1719]) for reflected in (False, True)]
                 if args.pilot else [(i, seeds[i], False) for i in args.sites]
                 + [(i, seeds[i], True) for i in args.mirrors])
        for site, seed, reflected in cases:
            for arm in (["registered"] if args.pilot else ["replay"] if args.replay_from else args.arms):
                key = f"{site}{'m' if reflected else ''}-{arm}"
                result = trial(seed ^ (FLIP if reflected else 0), arm, out/key,
                               save_frames=(arm == "registered" and site in (0, 1)), pose_only=args.pilot,
                               fixed=fixed[site] if arm == "replay" else None)
                row = json.dumps({"site": site, "mirror": reflected, "arm": arm, **result})
                print(row, flush=True)
                print(row, file=stream, flush=True)


if __name__ == "__main__":
    main()
