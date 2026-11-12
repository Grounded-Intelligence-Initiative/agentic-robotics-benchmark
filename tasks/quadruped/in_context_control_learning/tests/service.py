"""Single-threaded, root-owned world service with an append-only private ledger.

The ledger (state.json) records every request in order, with its parameters and outcome. It is the one record of the
agent phase that the verifier reads: the task declares it as an artifact, the engine copies it out of the agent's
sandbox when the agent stops and restores it, root-owned, in the fresh verifier sandbox, which replays its requests
against the same scene to score the episode.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import socketserver

from world import KIND, REQUEST_LIMIT, VERSION, World


ALLOWED = {"observe": set(), "act": {"action"}, "advance": set()}


def dispatch(world, method, params):
    """One request against the world; the verifier's replay calls the same function."""
    if (not isinstance(method, str) or method not in ALLOWED or not isinstance(params, dict)
            or set(params) != ALLOWED[method]):
        raise ValueError("operation/arguments not allowed; reset is never available")
    return getattr(world, method)(**params)


class Runtime:
    def __init__(self, directory, seed, scene_index):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        os.chmod(self.directory, 0o700)
        self.seed, self.scene_index = int(seed), int(scene_index)
        self.world = World(self.seed)
        self.count = 0
        self.events = []
        (self.directory/"frames").mkdir(exist_ok=True, mode=0o700)
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
        method, params = req["method"], req["params"]
        event = {"method": method, "params": params}
        self.events.append(event)
        try:
            result = self.dispatch(method, params)
            event["frame_id"] = result["frame_id"]
            event["steps"], event["delivered"] = self.world.steps, self.world.delivered
            result["requests_remaining"] = REQUEST_LIMIT-self.count
            return result
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            event["error"] = str(exc)
            raise
        finally:
            self.flush()

    def dispatch(self, method, params):
        result = dispatch(self.world, method, params)
        for i, encoded in enumerate(result.get("images", [])):
            (self.directory/"frames"/f'{result["frame_id"]:04d}-{i}.png').write_bytes(base64.b64decode(encoded))
        return result


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
