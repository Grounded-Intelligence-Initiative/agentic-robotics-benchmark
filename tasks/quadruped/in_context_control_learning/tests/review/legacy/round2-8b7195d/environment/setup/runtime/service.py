"""Single-threaded, root-owned world service with an append-only private ledger.

The ledger (state.json) records every request in order, with its parameters and outcome. It is the one record of the
agent phase that the verifier reads: the task declares it as an artifact, the engine copies it out of the agent's
sandbox when the agent stops and restores it, root-owned, in the fresh verifier sandbox, which replays its requests
against the same scene to score the episode.

The oracle's channel: a request {"method": "unlock", "params": {"token": <text>}} switches the world to privileged
replies (ground truth). The service keeps no secret: it records the sha256 of the token it was sent, never the token,
and the grader, which alone holds the expected hash (tests/oracle_token.sha256), zeroes any run that used the channel
with another token. The MCP bridge does not expose it, so an agent that follows the rules cannot send it at all.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socketserver

from world import KIND, REQUEST_LIMIT, VERSION, World


ALLOWED = {"observe": set(), "act": {"action"}, "advance": set(), "unlock": {"token_sha256"}}


def dispatch(world, method, params):
    """One request against the world; the verifier's replay calls the same function."""
    if (not isinstance(method, str) or method not in ALLOWED or not isinstance(params, dict)
            or set(params) != ALLOWED[method]):
        raise ValueError("operation/arguments not allowed; reset is never available")
    return getattr(world, method)(**params)


def recorded(method, params):
    """The parameters as the ledger keeps them: a token is replaced by its sha256."""
    if method == "unlock" and isinstance(params, dict) and set(params) == {"token"} and isinstance(params["token"], str):
        return {"token_sha256": hashlib.sha256(params["token"].encode()).hexdigest()}
    if method == "unlock":
        return {}        # refused by dispatch; nothing of a malformed unlock is kept
    return params


class Runtime:
    def __init__(self, directory, seed, scene_index, render=True):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        os.chmod(self.directory, 0o700)
        self.seed, self.scene_index = int(seed), int(scene_index)
        self.world = World(self.seed, render=render)
        self.count = 0
        self.events = []
        self.flush()

    def flush(self):
        state = {"kind": KIND, "version": VERSION, "seed": self.seed, "scene_index": self.scene_index,
                 "requests": self.count, "events": self.events, "metrics": self.world.metrics()}
        temp = self.directory/"state.tmp"
        temp.write_text(json.dumps(state))
        os.chmod(temp, 0o600)
        temp.replace(self.directory/"state.json")

    def call(self, req):
        self.count += 1
        if self.count > REQUEST_LIMIT:
            self.flush()
            raise ValueError("request budget exhausted")
        if not isinstance(req, dict) or set(req) != {"method", "params"}:
            self.flush()
            raise ValueError("invalid request envelope")
        method = req["method"]
        params = recorded(method, req["params"])
        event = {"method": method, "params": params}
        self.events.append(event)
        try:
            result = dispatch(self.world, method, params)
            event["steps"], event["delivered"] = self.world.steps, self.world.delivered
            result["requests_remaining"] = REQUEST_LIMIT-self.count
            return result
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            event["error"] = str(exc)
            raise
        finally:
            self.flush()


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(3)
        try:
            line = self.rfile.readline(8193)
            if len(line) > 8192:
                raise ValueError("request too large")
            result = self.server.runtime.call(json.loads(line))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            result = {"error": str(exc)}
        self.wfile.write((json.dumps(result)+"\n").encode())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", default="/var/lib/visual_robotics")
    parser.add_argument("--socket", default="/run/visual_robotics/world.sock")
    parser.add_argument("--scene-file", required=True, help="root-only JSON {seed, scene_index} written by setup")
    args = parser.parse_args()
    scene = json.loads(Path(args.scene_file).read_text())
    runtime = Runtime(args.directory, scene["seed"], scene["scene_index"])
    path = Path(args.socket)
    path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError("refuse to overwrite a live or pre-existing socket")
    (Path(args.directory)/"pid").write_text(str(os.getpid()))
    with socketserver.UnixStreamServer(str(path), Handler) as server:
        os.chmod(path, 0o666)
        server.runtime = runtime
        server.serve_forever()


if __name__ == "__main__":
    main()
