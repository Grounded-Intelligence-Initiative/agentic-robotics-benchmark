"""Run a locomotion submission on a public route and report its measured outcome."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import random
import signal
import sys

from loco_io import CHUNK, TASKS, LiveEpisode, Route, practice_route, rebase_observation, validate_action
from sonic_loco import LocomotionSession
from practice_wire import WireError, solver


def ask(wire, obs):
    tick = obs["tick"]
    reply = wire.request({"type": "act", "t": tick, "obs": obs})
    try:
        return validate_action(reply.get("action"), tick, obs["horizon"])
    except ValueError as exc:
        raise WireError(f"invalid action at tick {tick}: {exc}") from exc


def run(episode, args):
    session = episode.session
    rng = random.Random(args.seed ^ 0x84D31)
    eligible = range(10, args.steps, CHUNK)
    probe_ticks = set(rng.sample(eligible, min(3, len(eligible))))
    history = deque(maxlen=31)
    counts = dict(goal_trials=0, goal_first_codes_differed=0, state_trials=0,
                  state_first_codes_differed=0, identical_trials=0, identical_first_codes_differed=0)
    with solver(args.submission) as wire:
        while not episode.score.done and session.tick < args.steps:
            obs = episode.observe()
            tick = obs["tick"]
            other = None
            if tick == 0:
                name = "right_turn_stop" if episode.route.name == "straight_stop" else "straight_stop"
                other = episode.observe(practice_route(name))
                kind = "goal"
            elif tick in probe_ticks:
                earlier = [item for item in history if 8 <= tick - item["tick"] <= 30]
                past = rng.choice(earlier)
                other = rebase_observation(past, tick, obs["last_observation_id"])
                kind = "state"
            if other is None:
                rows = ask(wire, obs)
            else:
                if rng.randrange(2):
                    other_rows, rows = ask(wire, other), ask(wire, obs)
                else:
                    rows, other_rows = ask(wire, obs), ask(wire, other)
                counts[f"{kind}_trials"] += 1
                counts[f"{kind}_first_codes_differed"] += rows[0]["codes"] != other_rows[0]["codes"]
                if tick == 0:
                    identical = ask(wire, obs)
                    counts["identical_trials"] += 1
                    counts["identical_first_codes_differed"] += rows[0]["codes"] != identical[0]["codes"]
            history.append(obs)
            episode.advance({"rows": rows}, obs["observation_id"], max_ticks=args.steps - tick)
    native = session.state["metrics"]
    return {
        **episode.score.summary(),
        "seed": args.seed, "reason": episode.score.reason or "step_limit",
        "base_acceleration_xy_rms_m_s2": native["base_acceleration_xy_rms_m_s2"],
        "target_step_rms_rad": native["target_step_rms_rad"], "repeats": counts,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=TASKS, required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--assets-root", type=Path, default=Path("/opt/sonic/assets"))
    parser.add_argument("--submission", type=Path, default=Path("/home/user/submission"))
    parser.add_argument("--steps", type=int, help="stop after this many physical ticks")
    parser.add_argument("--distance-m", type=float, help="forward distance for straight_stop (default 5)")
    parser.add_argument("--turn-deg", type=float, help="signed yaw in degrees for right_turn_stop (default -90)")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64:
        parser.error("--seed must be an unsigned 64-bit integer")
    route = practice_route(args.task)
    if args.distance_m is not None:
        if args.task != "straight_stop" or not math.isfinite(args.distance_m) or args.distance_m <= 0:
            parser.error("--distance-m requires a positive finite distance on straight_stop")
        route = Route(args.task, args.distance_m, 0.)
    if args.turn_deg is not None:
        if args.task != "right_turn_stop" or not math.isfinite(args.turn_deg) or args.turn_deg >= 0:
            parser.error("--turn-deg requires a finite negative angle on right_turn_stop")
        route = Route(args.task, 0., math.radians(args.turn_deg))
    with LocomotionSession(args.assets_root, args.seed) as session:
        if args.steps is None:
            args.steps = session.horizon
        if not 1 <= args.steps <= session.horizon:
            parser.error(f"--steps must be between 1 and {session.horizon}")
        result = run(LiveEpisode(session, route), args)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))
    signal.signal(signal.SIGHUP, lambda signum, frame: sys.exit(128 + signum))
    try:
        main()
    except (OSError, ValueError, WireError) as exc:
        sys.exit(f"practice: {exc}")
