"""The image-only reference (reference_policy.py) through the robot socket: no token, no ground truth.

Not what `solve.sh` runs (the engine's oracle is oracle_run.py). `tests/review/run_episode.sh <seed> <out> reference`
runs it the way an agent's session runs, and its ledger is graded like any other.
"""
import json

from client import Client
from reference_policy import solve


def main():
    obs = solve(Client())
    print(json.dumps({k: obs[k] for k in ("delivered", "deliveries", "steps_remaining", "frames_remaining",
                                          "requests_remaining", "done")}), flush=True)


if __name__ == "__main__":
    main()
