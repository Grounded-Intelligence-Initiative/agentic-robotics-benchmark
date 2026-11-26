"""Reproduce two equal-prefix public courses and a time-only control on both branches."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

TESTS = Path(__file__).resolve().parents[1]
TASK = TESTS.parent
sys.path[:0] = [str(TESTS), str(TASK / "environment"), str(TASK / "solution")]
from gait import GaitPolicy
from robotics_grader import seed_rng
from sequence_io import Course, SequenceEpisode
from sonic_loco import LocomotionSession
from sonic_sim.encoder import ReferenceEncoder
from verify import run_episode, summarize


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class Actor:
    def __init__(self, controller):
        self.policy = controller

    def reset(self):
        pass

    def act(self, tick, obs):
        c = self.policy.codes(obs["stage_tick"], obs["command"], obs["observation"])
        return {"rows": [{"codes": c.tolist()}]}


class Blind:
    """Replay the right-turn tape on its original clock; hold its final row after tape end."""
    def __init__(self, tape):
        self.tape = tape

    def reset(self):
        pass

    def act(self, tick, obs):
        return {"rows": [{"codes": self.tape[min(tick, len(self.tape) - 1)]}]}


def actual(course, actor, assets):
    prefix = hashlib.sha256()
    tape = []
    branch = None
    with LocomotionSession(assets, 7, horizon=2400) as sim:
        ep = SequenceEpisode(sim, course)
        while not ep.done:
            obs = ep.observe()
            action = actor.act(obs["tick"], obs)
            c = action["rows"][0]["codes"]
            if obs["stage"] == "approach":
                # Receipt includes the complete actual public packet before each applied command.
                prefix.update(bytes.fromhex(obs["observation_id"]))
                prefix.update(bytes(c))
            elif obs["stage"] == "pivot" and obs["stage_tick"] == 0:
                branch = {"tick": obs["tick"], "codes": c, "initial_state_sha256": digest({
                    "observation": obs["observation"], "decoder_history": obs["decoder_history"]})}
            ep.advance(action, obs["observation_id"])
            tape.append(c)
        return {"prefix_sha256": prefix.hexdigest(), "reveal": branch, "score": ep.summary()}, tape


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=Path("/opt/sonic/assets"))
    args = parser.parse_args()
    config = json.loads((TESTS / "grader_config.json").read_text())
    actor = Actor(GaitPolicy(ReferenceEncoder(args.assets / "policy/model_encoder.onnx")))
    courses = [Course(5., -90., 1.5), Course(5., 90., 1.5)]
    right, tape = actual(courses[0], actor, args.assets)
    left, _ = actual(courses[1], actor, args.assets)
    assert right["score"]["success"] and left["score"]["success"]
    assert right["prefix_sha256"] == left["prefix_sha256"]
    r, l = right["reveal"], left["reveal"]
    assert r["tick"] == l["tick"] and r["initial_state_sha256"] == l["initial_state_sha256"]
    different = sum(a != b for a, b in zip(r["codes"], l["codes"]))
    assert different > 0
    live = [run_episode(actor, c, 7, seed_rng(778 + i), config, args.assets) for i, c in enumerate(courses)]
    blind = [run_episode(Blind(tape), c, 7, seed_rng(778 + i), config, args.assets) for i, c in enumerate(courses)]
    assert live[0]["applied_rows"] == right["score"]["applied_rows"]
    assert live[1]["applied_rows"] == left["score"]["applied_rows"]
    results = {"frozen_public_seed": 7, "courses": [[5, -90, 1.5], [5, 90, 1.5]],
               "prefix_packets_and_codes_sha256": right["prefix_sha256"], "turn_reveal_tick": r["tick"],
               "actual_reveal_state_history_sha256": r["initial_state_sha256"],
               "first_turn_changed_coordinates": different,
               "nominal_right_tape_sha256": hashlib.sha256(np.asarray(tape, np.uint8).tobytes()).hexdigest()}
    fields = ("success", "reason", "stages_completed", "simulated_s", "applied_rows", "scalar_codes",
              "joint_accel_rms_rad_s2", "state_pairs", "state_changes", "state_duplicates", "state_noise",
              "branch_pairs", "branch_changes", "branch_duplicates", "branch_noise")
    for name, trials in (("reference", live), ("blind_right_tape", blind)):
        summary = summarize(trials, config)
        results[name] = {r["direction"]: {k: r[k] for k in fields} for r in trials}
        results[name]["match_lock_ok"] = summary["match_lock_ok"]
    assert results["reference"]["match_lock_ok"] and not results["blind_right_tape"]["match_lock_ok"]
    assert blind[0]["success"] and not blind[1]["success"]
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
