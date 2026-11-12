"""Single-threaded, root-owned robot service with a private ledger.

Two modes behind the same Unix socket and the same calls:

* ``commissioning`` (the agent's sandbox): development sites drawn at random, ``new_site``
  starts a fresh one. It answers what the robot sees and nothing more: no site truth, no score.
* ``graded`` (the verifier): one mission on one hidden site; ``new_site`` is refused and
  ``truth`` answers only a caller holding the token whose sha256 the grader passed in.

The ledger (``state.json`` in the 0700 directory) is rewritten atomically after every request;
it carries the mission's metrics and the seconds the service spent answering requests, which
the grader subtracts from wall time to measure the program's own time.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import socketserver
import sys
import time
import traceback

from world import World

REQUESTS = 640                    # requests per mission (observe, act, advance, truth)
MAX_EVENTS = 1200


class Runtime:
    def __init__(self, directory, mode, seed=None, token_sha256=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        os.chmod(self.directory, 0o700)
        self.mode = mode
        self.token_sha256 = token_sha256
        self.busy = 0.
        self.sites = 0
        self.world = None
        self.internal_errors = 0
        self.request_started = None
        self._start(seed)

    def _start(self, seed=None):
        if self.world is not None:
            self.world.close()
            self.world = None
        self.seed = secrets.randbits(64) if seed is None else int(seed)
        self.world = World(self.seed)
        self.requests = 0
        self.exhausted = False
        self.events = []
        self.sites += 1
        self.flush()

    def flush(self):
        state = {"mode": self.mode, "sites": self.sites, "requests": self.requests,
                 "busy_seconds": round(self.busy, 4),
                 "done": bool(self.world and (self.world.done or self.exhausted)),
                 "phase": self.world.phase if self.world else 0,
                 "metrics": self.world.metrics() if self.world else {},
                 "internal_errors": self.internal_errors, "request_started": self.request_started,
                 "events": self.events}
        temp = self.directory/"state.tmp"
        temp.write_text(json.dumps(state))
        temp.replace(self.directory/"state.json")

    def call(self, req):
        started = time.monotonic()
        # Wall-clock start of the request in flight, for the grader's program-time measure.
        self.request_started = time.time()
        self.flush()
        try:
            if not isinstance(req, dict) or set(req) != {"method", "params"}:
                raise ValueError("a request is {\"method\": ..., \"params\": {...}}")
            method, params = req["method"], req["params"]
            if not isinstance(method, str) or not isinstance(params, dict):
                raise ValueError("a request is {\"method\": ..., \"params\": {...}}")
            if method == "new_site":
                if self.mode != "commissioning":
                    raise ValueError("new_site is a commissioning call; a graded mission is one site")
                if params:
                    raise ValueError("new_site takes no parameters")
                self._start()
                self.requests = 1
                return self._answer(self.world.observe())
            self.requests += 1
            if self.requests > REQUESTS:
                self.exhausted = True
                raise ValueError(f"this mission's {REQUESTS} requests are spent; it has ended")
            event = {"m": method}
            if method == "act" and isinstance(params.get("action"), list):
                event["a"] = params["action"][:1]
            if len(self.events) < MAX_EVENTS:
                self.events.append(event)
            try:
                return self._answer(self.dispatch(method, params))
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                event["error"] = str(exc)[:120]
                raise
        except (ValueError, TypeError, KeyError, OverflowError):
            raise
        except Exception:
            # The service's own failure (a render, the simulator), never the caller's: recorded so the
            # grader runs the mission again instead of charging it to the program.
            self.internal_errors += 1
            print(traceback.format_exc(), file=sys.stderr, flush=True)
            raise RuntimeError("internal service error")
        finally:
            self.busy += time.monotonic()-started
            self.request_started = None
            self.flush()

    def _answer(self, observation):
        observation["requests_remaining"] = max(0, REQUESTS-self.requests)
        return observation

    def dispatch(self, method, params):
        allowed = {"observe": set(), "act": {"action"}, "advance": set()}
        if self.mode == "graded":
            allowed["truth"] = {"token"}
        if method not in allowed or set(params) != allowed[method]:
            raise ValueError("calls are observe(), act(action), advance()"
                             + (", new_site()" if self.mode == "commissioning" else ""))
        if method == "truth":
            token = params["token"]
            if (not isinstance(token, str) or not self.token_sha256
                    or hashlib.sha256(token.encode()).hexdigest() != self.token_sha256):
                raise ValueError("forbidden")
            return self.world.truth()
        return getattr(self.world, method)(**params)


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(3)
        try:
            line = self.rfile.readline(8193)
            if len(line) > 8192:
                raise ValueError("request too large")
            result = self.server.runtime.call(json.loads(line))
        except (ValueError, TypeError, KeyError, OverflowError, RuntimeError) as exc:
            result = {"error": str(exc)}
        except OSError:
            return
        try:
            self.wfile.write((json.dumps(result)+"\n").encode())
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("commissioning", "graded"), required=True)
    parser.add_argument("--directory", default="/var/lib/visual_robotics")
    parser.add_argument("--socket", default="/run/visual_robotics/world.sock")
    parser.add_argument("--seed-file", help="graded: a root-only file holding the site's seed")
    parser.add_argument("--token-sha256", help="graded: sha256 of the oracle's token")
    args = parser.parse_args()
    seed = None
    if args.mode == "graded":
        if not args.seed_file:
            parser.error("--seed-file is required in graded mode")
        seed = int(Path(args.seed_file).read_text().strip())
    runtime = Runtime(args.directory, args.mode, seed, args.token_sha256)
    path = Path(args.socket)
    path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError("refuse to overwrite a live or pre-existing socket")
    (Path(args.directory)/"pid").write_text(str(os.getpid()))
    with socketserver.UnixStreamServer(str(path), Handler) as server:
        os.chmod(path, 0o666)
        server.runtime = runtime
        (Path(args.directory)/"ready").write_text("1")
        server.serve_forever()


if __name__ == "__main__":
    main()
