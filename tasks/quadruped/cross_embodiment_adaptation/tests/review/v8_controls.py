"""Matched mobility controls on the public Unix socket; truth only after parking/stopping."""
import argparse
import base64
import json
import math
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent/"setup_payload"), str(HERE.parents[1]/"solution/oracle")]
from client import Client
from oracle import deploy
from survey import scan

ARMS = ("adaptive", "ordinary", "crouch")
ORDER_FLIP = 1 << 39


class EvidenceClient(Client):
    def __init__(self, path, evidence):
        super().__init__(path)
        self.evidence = evidence

    def act(self, action):
        obs = super().act(action)
        if self.evidence and obs["phase"] == 0:
            x, y = (round(v, 2) for v in obs["tracking_xy_m"])
            if x in (1.41, 1.93, 4.55, 5.07) and y in (-1.50, 0., 1.50):
                (self.evidence/f"rgb-{x:g}-{y:g}.png").write_bytes(
                    base64.b64decode(obs["images"][0]))
        return obs

    def look(self, detent):
        obs = super().look(detent)
        if self.evidence:
            (self.evidence/f"mast-{detent}.png").write_bytes(base64.b64decode(obs["images"][0]))
        return obs


def trial(seed, arm, directory, save_frames, flat, alternate_survey):
    directory.mkdir()
    (directory/"seed").write_text(str(seed))
    service = [str(Path(__file__).resolve()), "--flat-service"] if flat else [str(HERE.parent/"runtime/service.py")]
    command = [sys.executable, "-B", *service, "--mode", "graded",
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
            surveyed = scan(client, (0., -1.42, 1.42)) if alternate_survey else scan(client)
            passages, _ = surveyed.passages()
            report = deploy(client, surveyed, posture=arm)
            # The estimator, mode scheduler and gait have already finished.
            state = json.loads((directory/"state.json").read_text())
            assert state["done"] and not state["internal_errors"]
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
                "duct_first": bool(seed & ORDER_FLIP) if not flat else None,
                **state["metrics"],
            }
        finally:
            proc.terminate()
            proc.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--sites", nargs="+", type=int, default=list(range(8)))
    parser.add_argument("--arms", nargs="+", choices=(*ARMS, "intermediate"), default=list(ARMS))
    parser.add_argument("--dev", action="store_true", help="development seed 1701, both fixture orders")
    parser.add_argument("--swap", action="store_true", help="swap order with all other geometry held fixed")
    parser.add_argument("--flat", action="store_true", help="same body/site without either fitting")
    parser.add_argument("--alternate-survey", action="store_true", help="reverse and offset the outer flight rows")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir()
    seeds = json.loads((HERE.parent/"hidden/sites.json").read_text())["seeds"]
    with (out/"results.jsonl").open("w") as stream:
        cases = ([(0, 1701, False), (0, 1701, True)] if args.dev
                 else [(i, seeds[i], args.swap) for i in args.sites])
        for site, seed, swap in cases:
            for arm in args.arms:
                key = f"{site}{'s' if swap else ''}-{arm}"
                result = trial(seed ^ (ORDER_FLIP if swap else 0), arm, out/key,
                               save_frames=(arm == "adaptive" and site in (0, 2)), flat=args.flat,
                               alternate_survey=args.alternate_survey)
                row = json.dumps({"site": site, "swapped": swap, "flat": args.flat,
                                  "alternate_survey": args.alternate_survey, "arm": arm, **result})
                print(row, flush=True)
                print(row, file=stream, flush=True)


if __name__ == "__main__":
    if sys.argv[1:2] == ["--flat-service"]:
        from functools import partial
        sys.path.insert(0, str(HERE.parent/"runtime"))
        import service
        del sys.argv[1]
        service.World = partial(service.World, flat_control=True)
        service.main()
    else:
        main()
