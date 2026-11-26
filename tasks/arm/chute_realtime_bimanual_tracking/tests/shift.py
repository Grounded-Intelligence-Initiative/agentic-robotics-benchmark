"""Shift mode: one long episode of many events whose regularities are hidden but fixed for the shift.

A shift draws its own constants once (how fast the track is, how the table pushes are aimed, the feeder's rhythm, how
the airborne target flies and evades). Every event of the shift is drawn from them, so a controller that estimates
them from the first events can do better on the later ones; the grader reports early and late success separately.
Pure numpy, no simulator imports. Coordinates: robot start frame, +x forward, +y left, z up, metres, 30 Hz.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

HZ = 30
KINDS_A = ("track", "belt", "drop")
KINDS_B = ("passive", "evasive")
# 20 YCB items of NVIDIA's public Isaac assets (Isaac/Props/YCB/Axis_Aligned); the pitcher base is left out because it
# is wider (0.137 m) than the R1Pro gripper opens (0.127 m). A shift uses each item once.
CATALOG = ("002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "006_mustard_bottle",
           "007_tuna_fish_can", "008_pudding_box", "009_gelatin_box", "010_potted_meat_can", "011_banana",
           "021_bleach_cleanser", "024_bowl", "025_mug", "035_power_drill", "036_wood_block",
           "037_scissors", "040_large_marker", "051_large_clamp", "052_extra_large_clamp", "061_foam_brick")
# Fixed stations of task A: the robot has to drive between them (they are 1.2 m apart, out of one arm's reach).
# track: a part rolls down an incline; belt: a part rides a running conveyor and falls off its open end; drop: a part
# is released from an overhead feeder and falls straight down.
STATIONS = {"track": [1.1, 1.2, 0.0], "belt": [1.0, 0.0, 0.0], "drop": [0.7, -1.2, 0.0]}
LIMIT_S = {"track": 25, "belt": 15, "drop": 8, "passive": 40, "evasive": 40}


@dataclass
class Event:
    index: int
    kind: str
    release_step: int                 # steps after the event starts
    limit_steps: int
    target: dict                      # {"shape", "mass", "position", "velocity"} or flight parameters
    floor_fails: bool


@dataclass
class ShiftPlan:
    task: str                         # "A" or "B"
    seed: int
    constants: dict
    events: list = field(default_factory=list)


def make_shift(task: str, seed: int, n_events: int, station: str | None = None, salt: int = 0) -> ShiftPlan:
    """station: for task A, run the whole shift at one feed point ("track", "belt" or "drop"); None mixes all three.
    salt: the grader's secret (tests/hidden/shifts.json). With it, a seed gives a shift that nobody holding this file and
    the seed can reproduce: a controller cannot recognise a graded shift from its first parts and read off the rest."""
    g = np.random.default_rng([{"A": 11, "B": 12}[task], seed] + ([int(salt)] if salt else []))
    u = lambda lo, hi: float(g.uniform(lo, hi))
    if task == "A":
        c = {
            "track_slope_deg": u(4.0, 7.0),              # a chute that looks like one: parts visibly pick up speed
            "track_grip": u(0.025, 0.046),               # friction = tan(slope) - grip: sliding parts gain 0.25 to 0.45 m/s2
            "track_push": u(0.05, 0.15),                 # release speed, m/s: gravity does the rest (0.5 to 0.65 m/s at the hand, up to 0.9 at the end)
            "belt_speed": u(0.10, 0.25),                 # the conveyor runs at one speed for the whole shift
            "belt_lane": u(-0.12, 0.12), "belt_lane_sd": 0.03,          # where across the belt the line sets parts down
            "belt_gap_s": u(1.0, 3.0),                   # set-down delay after the event starts
            "feeder_delay_s": u(1.5, 4.0), "feeder_delay_sd": 0.08,      # the feeder's rhythm: the only cue for a drop
            "feeder_height": u(1.6, 1.9),
            "feeder_offset": [u(-0.08, 0.08), u(-0.08, 0.08)],
            "mass_scale": u(0.7, 1.3),                  # this shift's items are lighter or heavier than nominal
            "order_bias": g.dirichlet([2.0, 2.0, 2.0]).tolist(),         # how often each station fires this shift
        }
        c["station"] = station or "mixed"
        c["track_friction"] = float(np.tan(np.radians(c["track_slope_deg"])) - c["track_grip"])
        assert n_events <= len(CATALOG), "a shift never repeats an item"
        items = [CATALOG[k] for k in g.permutation(len(CATALOG))[:n_events]]   # every part of a shift is a different item
        plan = ShiftPlan(task, seed, c)
        for i in range(n_events):
            kind = KINDS_A[int(g.choice(3, p=c["order_bias"]))]
            if station is not None:
                kind = station
            sx, sy, _ = STATIONS[kind]
            if kind == "track":
                tgt = {"shape": "roller", "along": u(0.35, 0.5), "speed": c["track_push"] + float(g.normal(0, 0.02))}
                rel = int(u(1.0, 3.0) * HZ)
            elif kind == "belt":
                # The part rides the belt toward its open end (toward the robot) and falls off it unless it is taken.
                lane = c["belt_lane"] + float(g.normal(0, c["belt_lane_sd"]))
                tgt = {"shape": ["can", "box", "bottle"][int(g.integers(3))], "position": [sx + 0.95, sy + lane, 0.0],
                       "yaw": u(-3.14, 3.14), "belt_speed": c["belt_speed"]}
                rel = int(max(0.3, c["belt_gap_s"] + float(g.normal(0, 0.1))) * HZ)
            else:
                d = max(0.5, c["feeder_delay_s"] + float(g.normal(0, c["feeder_delay_sd"])))
                tgt = {"shape": ["ball", "block", "bottle"][int(g.integers(3))],
                       "position": [sx + c["feeder_offset"][0], sy + c["feeder_offset"][1], c["feeder_height"]], "velocity": [0, 0, 0]}
                rel = int(d * HZ)
            tgt["mass_scale"] = u(0.9, 1.1)
            tgt["item"] = items[i]
            plan.events.append(Event(i, kind, rel, LIMIT_S[kind] * HZ, tgt, kind != "track"))
        return plan
    c = {
        "speed": u(0.25, 0.8), "turn_rate": u(0.6, 2.0),
        "evasive_share": u(0.3, 0.7),
        "flee_radius": u(0.15, 0.35), "flee_gain": u(2.0, 5.0),
        "flee_side": float(g.choice([-1.0, 1.0])),      # evasive targets of this shift dodge to one side
        "altitude": u(0.8, 1.3), "altitude_pull": u(0.0, 1.5),   # they hold a preferred height
        "box": [[0.4, -0.6, 0.6], [1.1, 0.6, 1.5]],
    }
    plan = ShiftPlan(task, seed, c)
    for i in range(n_events):
        kind = "evasive" if g.random() < c["evasive_share"] else "passive"
        tgt = {"shape": "ball", "mass": 0.02, "position": [u(0.5, 0.9), u(-0.4, 0.4), c["altitude"] + u(-0.15, 0.15)],
               "flight_seed": int(g.integers(1 << 30))}
        plan.events.append(Event(i, kind, 0, LIMIT_S[kind] * HZ, tgt, False))
    return plan


class ShiftScore:
    """Per-event outcomes of one shift, with the early/late split that shows adaptation."""

    def __init__(self, plan: ShiftPlan):
        self.plan, self.outcomes = plan, []

    def record(self, event: Event, caught: bool, reason: str, steps: int):
        self.outcomes.append({"index": event.index, "kind": event.kind, "caught": bool(caught), "reason": reason, "steps": int(steps)})

    def summary(self) -> dict:
        n = len(self.plan.events)
        done = {o["index"]: o for o in self.outcomes}
        got = [bool(done.get(i, {}).get("caught", False)) for i in range(n)]   # an event never reached counts as missed
        third = max(1, n // 3)
        by_kind = {}
        for e in self.plan.events:
            k = by_kind.setdefault(e.kind, [0, 0]); k[1] += 1; k[0] += int(got[e.index])
        return {"events": n, "caught": int(sum(got)), "rate": sum(got) / n,
                "rate_first_third": sum(got[:third]) / third, "rate_last_third": sum(got[-third:]) / third,
                "by_kind": {k: {"caught": v[0], "events": v[1]} for k, v in by_kind.items()}}
