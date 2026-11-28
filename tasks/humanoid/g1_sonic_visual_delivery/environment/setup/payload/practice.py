"""Inspect mobile G1 cameras or run one continuous walk-to-placement episode."""

import argparse
import base64
from contextlib import contextmanager, suppress
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from delivery_io import LiveEpisode, validate_action
from sonic_sim.motion_cost import MeasuredSession
from sonic_sim import DeliverySession


class WireError(Exception):
    """A solver missed its deadline or returned an unusable reply."""


class Wire:
    def __init__(self, process):
        self.input = process.stdin.fileno()
        self.output = process.stdout.fileno()
        os.set_blocking(self.input, False)
        os.set_blocking(self.output, False)
        self.buffer = bytearray()

    @staticmethod
    def wait(read, write, deadline):
        remaining = deadline - time.monotonic()
        if remaining > 0:
            readable, writable, _ = select.select(read, write, [], remaining)
            if readable or writable:
                return
        raise WireError("solver exceeded its deadline")

    def line(self, deadline):
        while b"\n" not in self.buffer:
            self.wait([self.output], [], deadline)
            data = os.read(self.output, 65536)
            if not data:
                raise WireError("solver closed stdout")
            self.buffer.extend(data)
            if b"\n" not in self.buffer and len(self.buffer) > 4 * 65536:
                raise WireError("solver reply exceeds 65,536 characters")
        line, _, self.buffer = self.buffer.partition(b"\n")
        try:
            decoded = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise WireError("invalid UTF-8 on solver stdout") from exc
        if len(decoded) + 1 > 65536:
            raise WireError("solver reply exceeds 65,536 characters")
        return line.strip()

    def ready(self):
        deadline = time.monotonic() + 60
        for _ in range(200):
            if self.line(deadline) == b"READY":
                return
        raise WireError("solver never printed READY")

    def request(self, message):
        deadline = time.monotonic() + 15
        payload = json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n"
        remaining = memoryview(payload.encode("utf-8"))
        try:
            while remaining:
                self.wait([], [self.input], deadline)
                remaining = remaining[os.write(self.input, remaining):]
        except BrokenPipeError as exc:
            raise WireError("solver closed stdin") from exc
        for _ in range(1000):
            line = self.line(deadline)
            if line:
                try:
                    reply = json.loads(line)
                except (ValueError, RecursionError) as exc:
                    raise WireError("invalid solver JSON on stdout") from exc
                if isinstance(reply, dict):
                    return reply
                raise WireError("solver reply must be a JSON object")
        raise WireError("solver produced too many blank lines")


@contextmanager
def solver(directory):
    directory = directory.resolve()
    if not (directory / "run.sh").is_file():
        raise ValueError(f"missing submission entry point {directory / 'run.sh'}")
    with subprocess.Popen(
        ["bash", "run.sh"], cwd=directory, stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, bufsize=0, start_new_session=True,
    ) as process:
        try:
            wire = Wire(process)
            wire.ready()
            ack = wire.request({"type": "reset"})
            if set(ack) != {"ok"} or ack["ok"] is not True:
                raise WireError("reset requires a true acknowledgment")
            yield wire
            wire.request({"type": "close"})
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=1)
        finally:
            # A launcher can exit before its helpers, including on a broken reply.
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)


def inspect(episode, out):
    frame = episode.observe()["planner_vision"]
    out.mkdir(parents=True, exist_ok=True)
    for camera, image in frame["images"].items():
        (out/f"{camera}.png").write_bytes(base64.b64decode(image, validate=True))
        (out/f"{camera}-depth-mm.png").write_bytes(base64.b64decode(frame["depth_mm"][camera], validate=True))
    (out/"intrinsics.json").write_text(json.dumps(frame["intrinsics"], indent=2)+"\n")
    print(f"Saved RGB, millimetre depth and lens intrinsics to {out}")


def ask(wire, obs, horizon):
    tick = obs["tick"]
    reply = wire.request({"type": "act", "t": tick, "obs": obs})
    if set(reply) != {"action"}:
        raise WireError(f"tick {tick} requires a single action")
    try:
        return validate_action(reply["action"], tick, horizon)
    except ValueError as exc:
        raise WireError(f"invalid action at tick {tick}: {exc}") from exc


def run(episode, args):
    session = episode.session
    meter = MeasuredSession(session)
    live = LiveEpisode(meter)
    counts = dict(alternate_queries=0, alternate_first_rows_differed=0, identical_queries=0,
                  identical_first_rows_differed=0, unlit_query_groups=0)
    with solver(args.submission) as wire:
        while session.tick < args.steps and not session.state["done"]:
            obs = live.observe()
            rows = ask(wire, obs, session.horizon)
            if session.tick % 100 == 0:
                other = live.opposite_cue()
                if other is not None:
                    alternatives = ask(wire, other, session.horizon)
                    repeated = ask(wire, obs, session.horizon)
                    counts["unlit_query_groups"] += int(not session.state["metrics"]["docked"])
                    counts["alternate_queries"] += 1
                    counts["alternate_first_rows_differed"] += rows[0] != alternatives[0]
                    counts["identical_queries"] += 1
                    counts["identical_first_rows_differed"] += rows[0] != repeated[0]
            live.advance({"rows": rows[:args.steps-session.tick]}, obs["observation_id"])
    state = session.state
    print(json.dumps(dict(task="visual_delivery", seed=args.seed,
                          start_mode=session.episode_geometry["start_mode"],
                          shape=session.episode_geometry["shape"], target_side=session.episode_geometry["target_side"],
                          steps=session.tick, reason=state["reason"] or "step_limit", metrics=state["metrics"],
                          repeats=counts, motion=meter.action_metrics()), allow_nan=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="command", required=True)
    inspector = modes.add_parser("inspect", help="export the initial three RGB-D views")
    runner = modes.add_parser("run", help="run a public seed through the native scene")
    for sub in (inspector, runner):
        sub.add_argument("--seed", type=int, required=True)
        sub.add_argument("--assets-root", type=Path, default=Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets")))
        sub.add_argument("--start", choices=("near", "far"))
        sub.add_argument("--shape", choices=("box", "cylinder"))
        sub.add_argument("--target", choices=("left", "right"))
    inspector.add_argument("--out", type=Path, required=True)
    runner.add_argument("--steps", type=int, default=1100)
    runner.add_argument("--submission", type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or not 1 <= getattr(args, "steps", 1) <= 1100:
        parser.error("seed must be unsigned 64-bit and steps must be 1 through 1100")
    with DeliverySession("visual_delivery", args.seed, args.assets_root, start_mode=args.start,
                         shape=args.shape, target_side=args.target) as session:
        if args.command == "inspect":
            inspect(LiveEpisode(session), args.out)
        else:
            run(LiveEpisode(session), args)


if __name__ == "__main__":
    # Unwind the process-group and simulator contexts when a practice job is stopped.
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))
    signal.signal(signal.SIGHUP, lambda signum, frame: sys.exit(128 + signum))
    try:
        main()
    except (OSError, ValueError, WireError) as exc:
        sys.exit(f"practice: {exc}")
