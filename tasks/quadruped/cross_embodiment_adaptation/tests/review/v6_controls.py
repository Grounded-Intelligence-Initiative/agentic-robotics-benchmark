"""Author-only RGB/point-robot interventions through the public Unix client."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent/"setup_payload"), str(HERE.parents[1]/"solution/oracle")]
from client import Client
from oracle import deploy
from survey import scan

ARMS = {"rgb": None, "point-first": 0, "point-second": 2}


class EvidenceClient(Client):
    def __init__(self, path, evidence):
        super().__init__(path)
        self.evidence = evidence

    def act(self, action):
        obs = super().act(action)
        if self.evidence and obs["phase"] == 0:
            x, y = (round(v, 2) for v in obs["tracking_xy_m"])
            if x in (1.65, 4.95) and y in (-1.10, 0., 1.10):
                (self.evidence/f"rgb-{x:g}-{y:g}.png").write_bytes(
                    base64.b64decode(obs["images"][0]))
        return obs


def trial(seed, arm, directory, save_frames):
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
            path = surveyed.route(point_rack=ARMS[arm])
            deploy(client, path)
            state = json.loads((directory/"state.json").read_text())
            assert state["done"] and not state["internal_errors"]
            return {
                "requests": state["requests"],
                "busy_seconds": state["busy_seconds"],
                "wall_seconds": round(time.monotonic()-start, 2),
                "rgb_passages": passages,
                "planned_path_m": path,
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
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir()
    seeds = json.loads((HERE.parent/"hidden/sites.json").read_text())["seeds"]
    with (out/"results.jsonl").open("w") as stream:
        for site in args.sites:
            for arm in args.arms:
                result = trial(seeds[site], arm, out/f"{site}-{arm}", arm == "rgb" and site in (0, 1))
                row = json.dumps({"site": site, "arm": arm, **result})
                print(row, flush=True)
                print(row, file=stream, flush=True)


if __name__ == "__main__":
    main()
