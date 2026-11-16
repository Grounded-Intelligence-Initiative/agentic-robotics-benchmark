"""The engine's oracle: the privileged planner (oracle_policy.py), driving the robot service through its socket.

It carries the oracle token (solution/oracle_token, which no agent sandbox ever holds): the service answers `unlock`
with ground truth, records the token's sha256 in its ledger, and the grader accepts the privileged requests only
because that hash equals tests/oracle_token.sha256. The grader also recognises the oracle by the engine's own
trajectory (agent name "oracle") and skips the direct-control audit for it, since a script necessarily talks to the
socket.
"""
import json
from pathlib import Path

from client import Client
from oracle_policy import solve


def main():
    token = (Path(__file__).resolve().parent/"oracle_token").read_text().strip()
    obs = solve(Client(), token)
    print(json.dumps({k: obs[k] for k in ("delivered", "deliveries", "steps_remaining", "frames_remaining",
                                          "requests_remaining", "done")}), flush=True)


if __name__ == "__main__":
    main()
