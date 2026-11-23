"""Single-threaded, root-owned world service with an append-only private ledger.

events.jsonl grows by one row per service call; state.json is its atomic final receipt.
Both are root-owned artifacts restored together into the separate verifier.
"""
import argparse
import json
import os
from pathlib import Path
import socketserver

from world import KIND, VERSION, World


ALLOWED = {"observe": set(), "act": {"target_rad", "ticks"}, "advance": set(), "recover": set()}


def dispatch(world, method, params):
    """One request against the world; the verifier's replay calls the same function."""
    if (not isinstance(method, str) or method not in ALLOWED or not isinstance(params, dict)
            or set(params) != ALLOWED[method]):
        raise ValueError("operation or arguments not allowed")
    return getattr(world, method)(**params)


class Runtime:
    def __init__(self, directory, seed, scene_index, render=True):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        os.chmod(self.directory, 0o700)
        self.seed, self.scene_index = int(seed), int(scene_index)
        self.world = World(self.seed, render=render, scene_index=self.scene_index)
        self.count = 0
        self.journal = self.directory/"events.jsonl"
        self.journal.touch(mode=0o600, exist_ok=False)
        self.flush()

    def flush(self):
        state = {"kind": KIND, "version": VERSION, "seed": self.seed, "scene_index": self.scene_index,
                 "requests": self.count, "metrics": self.world.metrics(),
                 "state_fingerprint": self.world.state_fingerprint()}
        temp = self.directory/"state.tmp"
        temp.write_text(json.dumps(state))
        os.chmod(temp, 0o600)
        temp.replace(self.directory/"state.json")

    def call(self, req):
        if not isinstance(req, dict) or set(req) != {"method", "params"}:
            raise ValueError("invalid request envelope")
        method, params = req["method"], req["params"]
        self.count += 1
        event = {"method": method, "params": params}
        try:
            result = dispatch(self.world, method, params)
            event["frame_id"] = result["frame_id"]
            event["steps"], event["delivered"] = self.world.steps, self.world.delivered
            return result
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            event["error"] = str(exc)
            raise
        finally:
            event["state_fingerprint"] = self.world.state_fingerprint()
            with self.journal.open("a") as stream:
                stream.write(json.dumps(event, separators=(",", ":"))+"\n")
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
