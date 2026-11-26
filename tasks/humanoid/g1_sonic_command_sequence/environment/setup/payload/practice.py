"""Run a submission on the public continuous forward, turn and departure course."""
import argparse
from collections import deque
from contextlib import ExitStack
import gzip
import json
import math
import os
from pathlib import Path
import signal
import sys

from practice_wire import WireError, solver
from sequence_io import HORIZON, Course, SequenceEpisode, rebase_observation, validate_action
from sonic_loco import LocomotionSession



def ask(wire, obs):
    tick = obs["tick"]
    reply = wire.request({"type": "act", "t": tick, "obs": obs})
    if set(reply) != {"action"}:
        raise WireError(f"tick {tick} requires a single action")
    try:
        return validate_action(reply["action"], tick, obs["horizon"])
    except ValueError as exc:
        raise WireError(f"invalid action at tick {tick}: {exc}") from exc


def run(args):
    angle = args.angle_deg if args.turn == "left" else -args.angle_deg
    course = Course(args.first_m, angle, args.last_m)
    opposite = Course(args.first_m, -angle, args.last_m)
    counts = {name: {"trials": 0, "first_codes_changed": 0}
              for name in ("state", "turn", "identical")}
    reveals = []
    history = deque(maxlen=40)
    checked = set()
    with ExitStack() as resources:
        session = resources.enter_context(LocomotionSession(args.assets_root, args.seed, horizon=HORIZON))
        episode = SequenceEpisode(session, course)
        wire = resources.enter_context(solver(args.submission))
        trace = None
        if args.trace:
            opener = gzip.open if args.trace.suffix == ".gz" else open
            trace = resources.enter_context(opener(args.trace, "wt", encoding="utf-8"))
        while not episode.done and session.tick < args.steps:
            obs = episode.observe()
            tick, stage = obs["tick"], obs["stage"]
            new_stage = not reveals or reveals[-1]["stage"] != stage
            if new_stage:
                reveals.append({"tick": tick, "stage": stage, "command": obs["command"]})
                history.clear()
            alternatives = []
            if stage == "pivot" and new_stage:
                alternatives.append(("turn", episode.observe(opposite)))
            # Compare against a previously observed state from this same command.
            past = next((item for item in history if 10 <= tick - item["tick"] <= 25), None)
            if past is not None and stage not in checked and obs["stage_tick"] >= 35:
                alternatives.append(("state", rebase_observation(
                    past, tick, obs["stage_tick"], obs["last_observation_id"])))
                checked.add(stage)
            if new_stage:
                alternatives.append(("identical", obs))
            # Vary presentation order; only the observation selected below drives physics.
            if (len(reveals) + tick) % 2:
                replies = [(kind, ask(wire, packet)) for kind, packet in alternatives]
                rows = ask(wire, obs)
            else:
                rows = ask(wire, obs)
                replies = [(kind, ask(wire, packet)) for kind, packet in alternatives]
            for kind, other in replies:
                counts[kind]["trials"] += 1
                counts[kind]["first_codes_changed"] += int(other[0]["codes"] != rows[0]["codes"])
            history.append(obs)
            applied = episode.advance({"rows": rows}, obs["observation_id"], max_ticks=args.steps - tick)
            if trace:
                trace.write(json.dumps({"observation": obs, "proposed_rows": rows, "applied_rows": applied},
                                       separators=(",", ":"), allow_nan=False) + "\n")
        result = episode.summary()
        result.update(seed=args.seed, reason=episode.reason or "step_limit", reveals=reveals, repeats=counts)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--turn", required=True, choices=("left", "right"), help="host-side public pivot direction")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--first-m", type=float, default=5.0, help="first forward distance in metres (4.5 to 5.5)")
    parser.add_argument("--angle-deg", type=float, default=90.0, help="positive turn magnitude (75 to 105 degrees)")
    parser.add_argument("--last-m", type=float, default=1.5, help="final forward distance in metres (1 to 2)")
    parser.add_argument("--steps", type=int, default=HORIZON, help="maximum physical rows to apply, up to 2400")
    parser.add_argument("--submission", type=Path, default=Path("/home/user/submission"))
    parser.add_argument("--assets-root", type=Path, default=Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets")))
    parser.add_argument("--trace", type=Path, help="save actual public observations, proposed rows and applied counts as JSONL or .gz")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or not 1 <= args.steps <= HORIZON:
        parser.error("--seed must be an unsigned 64-bit integer and --steps must be in [1, 2400]")
    for value, low, high, label in ((args.first_m, 4.5, 5.5, "--first-m"),
                                     (args.angle_deg, 75., 105., "--angle-deg"),
                                     (args.last_m, 1., 2., "--last-m")):
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f"{label} must be in [{low:g}, {high:g}]")
    print(json.dumps(run(args), allow_nan=False))


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))
    signal.signal(signal.SIGHUP, lambda signum, frame: sys.exit(128 + signum))
    try:
        main()
    except (OSError, ValueError, WireError) as exc:
        sys.exit(f"practice: {exc}")
