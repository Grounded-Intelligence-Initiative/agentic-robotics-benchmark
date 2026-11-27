"""V9 author controls: paired camera probes and a crossed open-loop placement."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution"), str(TASK/"environment/setup/payload")]

import numpy as np
from arm_dynamics import pose_row
from arm_evaluation import trace
from belt_drive import CassetteProbe, HOME
from model_rollout import predict
from reconstruction import create_scene
from world import World, drive_cassette


def probe(seed, cassette):
    w = World("arm", seed, cassette=cassette)
    try:
        obs = w.observe()
        initial = [hashlib.sha256(s.encode()).hexdigest() for s in obs["images"]]
        commissioning = CassetteProbe()
        while (action := commissioning.action(obs)) is not None:
            assert action[2:] == [0., 0.] and w.steps < 48
            obs = w.act(action)
        assert commissioning.cassette == cassette and not w.damage and not w.grasped
        np.testing.assert_allclose(w.position(), HOME, atol=.005)
        return dict(cassette=cassette, observed=commissioning.cassette, dx_m=commissioning.shifts,
                    steps=w.steps, damage=w.damage, reset_rgb=initial)
    finally:
        w.close()


def run_tape(seed, cassette, actions):
    w = World("arm", seed, render=False, shipping=False, stop_parked=True, cassette=cassette)
    try:
        rows = []
        for action in actions:
            w.act(action)
            rows.append(pose_row(w.data))
            if w.done:
                break
        return w.success, w.damage, np.asarray(rows)
    finally:
        w.close()


def main():
    seed = 35101
    rows = [probe(seed, c) for c in ((1, .8, 1.2), (-1, .8, 1.2), (1, 1.2, .8))]
    print(json.dumps({"safe_probes": rows}), flush=True)
    assert all(r["reset_rgb"] == rows[0]["reset_rgb"] for r in rows)
    original = drive_cassette(seed)
    sign, a, b = original
    opposite = (-sign, a, b)
    tape, expected = trace(seed, 0xB35C, 1)
    reversed_tape = [[-u, -v, z, p] for u, v, z, p in tape]
    outcomes = []
    for mount, actions, matched in ((original, tape, True), (opposite, tape, False),
                                    (opposite, reversed_tape, True), (original, reversed_tape, False)):
        ok, damage, poses = run_tape(seed, mount, actions)
        assert bool(ok) == matched
        if matched:
            np.testing.assert_allclose(poses, expected, atol=1e-10, rtol=0)
            assert not damage
        outcomes.append(dict(mount=mount, reversed=actions is reversed_tape, placed=bool(ok),
                             damage=damage, shell_xyz=poses[-1, 3:6].tolist()))
    # Test reconstructed MJCF and the public loader, not just two calls on the same model.
    w = World("arm", seed, render=False, shipping=False, stop_parked=True)
    try:
        with tempfile.TemporaryDirectory() as tmp:
            scene = Path(tmp)/"scene.xml"
            errors = {}
            for label, c in (("matched", original), ("opposed", opposite),
                             ("wrong_A", (sign, 2.-a, b)), ("wrong_B", (sign, a, 2.-b))):
                scene.write_text(create_scene(w.block, w.tray, w.obstacle, w.arm_gains,
                                              w.arm_mass, w.arm_com, w.arm_proof, cassette=c))
                predicted = np.asarray(predict(str(scene), tape))
                errors[label] = float(np.sqrt(np.mean(((predicted-expected)*[1., 1., 1., 1., 1., 1., .69])**2)))
            print(json.dumps({"placement_counterfactual": outcomes, "rollout_rmse": errors}), flush=True)
            assert errors["matched"] < 1e-6 and min(errors[k] for k in errors if k != "matched") > .05
    finally:
        w.close()


if __name__ == "__main__":
    main()
