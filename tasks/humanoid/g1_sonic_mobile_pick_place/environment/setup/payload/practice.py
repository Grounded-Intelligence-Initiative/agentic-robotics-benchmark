"""Inspect mobile G1 cameras or run one continuous walk-to-placement episode."""

import argparse
import base64
from contextlib import ExitStack, contextmanager, suppress
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from mobile_io import HORIZON, LiveEpisode, stamp_observation, validate_action
from sonic_sim import MobileSession


# Public practice times span the approach and the work at the table.
REPEAT_TICKS = (0, 130, 400, 650)


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
    for camera, data in frame["images"].items():
        (out / f"{camera}.png").write_bytes(base64.b64decode(data, validate=True))
    (out / "calibration.json").write_text(
        json.dumps(frame["calibration"], indent=2) + "\n", encoding="utf-8"
    )
    print(f"Saved tick {frame['tick']} camera views and calibration to {out}")


def ask(wire, obs, horizon):
    tick = obs["tick"]
    reply = wire.request({"type": "act", "t": tick, "obs": obs})
    if set(reply) != {"action"}:
        raise WireError(f"tick {tick} requires a single action")
    try:
        return validate_action(reply["action"], tick, horizon)
    except ValueError as exc:
        raise WireError(f"invalid action at tick {tick}: {exc}") from exc


def repeated_query(wire, live_obs, shadow, counts):
    tick = live_obs["tick"]
    horizon = shadow.session.horizon
    # Both alternatives branch from the same committed controller memory.
    other = stamp_observation({
        **shadow.observe(),
        "last_hand": live_obs["last_hand"].copy(),
        "last_observation_id": live_obs["last_observation_id"],
    })
    if REPEAT_TICKS.index(tick) % 2 == 0:
        live_rows, other_rows = ask(wire, live_obs, horizon), ask(wire, other, horizon)
    else:
        other_rows, live_rows = ask(wire, other, horizon), ask(wire, live_obs, horizon)
    counts["alternate_queries"] += 1
    counts["alternate_first_codes_differed"] += live_rows[0]["codes"] != other_rows[0]["codes"]
    if tick == REPEAT_TICKS[0]:
        repeat = ask(wire, live_obs, horizon)
        counts["identical_queries"] += 1
        counts["identical_first_codes_differed"] += live_rows[0]["codes"] != repeat[0]["codes"]
    return live_rows


def run(episode, args):
    session = episode.session
    counts = dict(alternate_queries=0, alternate_first_codes_differed=0,
                  identical_queries=0, identical_first_codes_differed=0)
    shadow_seed = args.seed ^ 0xA53C
    last_repeat = max(tick for tick in REPEAT_TICKS if tick < args.steps)
    with ExitStack() as shadow_resources, solver(args.submission) as wire:
        shadow = LiveEpisode(shadow_resources.enter_context(
            MobileSession(session.task, shadow_seed, args.assets_root)
        ))
        while session.tick < args.steps and not session.state["done"]:
            obs = episode.observe()
            tick = obs["tick"]
            if shadow is not None and tick in REPEAT_TICKS:
                rows = repeated_query(wire, obs, shadow, counts)
            else:
                rows = ask(wire, obs, session.horizon)
            # Validate against the real horizon before trimming to a debugging cutoff.
            applied = episode.advance({"rows": rows[:args.steps - tick]}, obs["observation_id"])
            if shadow is not None:
                if tick < last_repeat:
                    shadow.advance({"rows": rows[:applied]}, obs["observation_id"])
                if tick >= last_repeat or shadow.session.state["done"]:
                    shadow_resources.close()
                    shadow = None
    state = session.state
    print(json.dumps({
        "task": args.task, "seed": args.seed, "steps": state["tick"],
        "reason": state["reason"] or "step_limit", "metrics": state["metrics"],
        "repeats": counts,
    }, allow_nan=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect_parser = commands.add_parser("inspect", help="save the three calibrated initial camera views")
    run_parser = commands.add_parser("run", help="run a submission on a public practice seed")
    for command in (inspect_parser, run_parser):
        command.add_argument("--task", choices=("box_to_pad", "cylinder_to_pad"), required=True)
        command.add_argument("--seed", type=int, required=True)
        command.add_argument("--assets-root", type=Path, default=Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets")))
    inspect_parser.add_argument("--out", type=Path, required=True)
    run_parser.add_argument("--submission", type=Path, required=True)
    run_parser.add_argument("--steps", type=int, default=HORIZON,
                            help=f"stop after this many physical ticks (default {HORIZON})")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64:
        parser.error("--seed must be an unsigned 64-bit integer")
    if args.command == "run" and not 1 <= args.steps <= HORIZON:
        parser.error(f"--steps must be between 1 and {HORIZON}")
    with MobileSession(f"{args.task}_challenge", args.seed, args.assets_root) as session:
        episode = LiveEpisode(session)
        if args.command == "inspect":
            inspect(episode, args.out)
        else:
            run(episode, args)


if __name__ == "__main__":
    # Unwind the process-group and simulator contexts when a practice job is stopped.
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))
    signal.signal(signal.SIGHUP, lambda signum, frame: sys.exit(128 + signum))
    try:
        main()
    except (OSError, ValueError, WireError) as exc:
        sys.exit(f"practice: {exc}")
