"""Visible-seed practice. Raw success only; this is not an ALE verdict."""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from square_nut_env import SquareNutEpisode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--submission", default=os.environ.get("SUBMISSION_DIR", "/home/user/submission"))
    parser.add_argument("--episodes", type=int, default=3)
    args = parser.parse_args()
    submission = Path(args.submission)
    if not (submission / "run.sh").is_file():
        parser.error("submission/run.sh is missing")
    proc = subprocess.Popen(["bash", "run.sh"], cwd=submission, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, bufsize=1)
    lines = queue.Queue()

    def pump():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()

    def read(timeout=10):
        try:
            value = lines.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError("controller timed out")
        if value is None:
            raise RuntimeError("controller exited")
        return value.strip()

    def request(value):
        proc.stdin.write(json.dumps(value) + "\n")
        proc.stdin.flush()
        return json.loads(read())

    results = []
    try:
        if read(40) != "READY":
            raise RuntimeError("controller did not print READY")
        for seed in (11, 22, 33, 44, 55)[:max(1, min(5, args.episodes))]:
            world = SquareNutEpisode(seed)
            try:
                request({"type": "reset"})
                for t in range(world.env.horizon):
                    obs = world.observe()
                    reply = request({"type": "act", "t": t, "obs": obs})
                    _, done = world.step(reply["action"])
                    if done:
                        break
                results.append({"seed": seed, "success": world.success, "steps": world.t})
            finally:
                world.close()
        request({"type": "close"})
    finally:
        if proc.poll() is None:
            proc.kill()
    print(json.dumps({"success_rate": sum(row["success"] for row in results) / len(results),
                      "episodes": results, "note": "visible practice seeds only"}))


if __name__ == "__main__":
    main()
