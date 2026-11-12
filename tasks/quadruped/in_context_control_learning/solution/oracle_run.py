"""The engine's oracle: the image-only scripted reference (policy.py), driving the robot service through its socket.

It sees exactly what the agent sees (the image and the status of each reply) and is the task's feasibility proof; it
is not an LLM result. The grader recognises the oracle by the engine's own trajectory (agent name "oracle") and skips
the direct-control audit for it, since a script necessarily talks to the socket.
"""
import json

from client import Client
from policy import solve


def main():
    obs = solve(Client())
    print(json.dumps({k: obs[k] for k in ("delivered", "deliveries", "steps_remaining", "requests_remaining", "done")}),
          flush=True)


if __name__ == "__main__":
    main()
