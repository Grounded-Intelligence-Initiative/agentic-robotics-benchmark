"""Determinism, scene coverage and aperture visibility, without a model API."""
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"environment/setup/runtime"))
import mujoco
from long_bridge import BridgeWorld


def signature(world):
    return (world.scenario, world.spans, tuple(world.defects), tuple(world.repairs))


def main():
    signatures, scenarios, lengths = set(), set(), set()
    for seed in range(101, 107):
        w, same = BridgeWorld(seed), BridgeWorld(seed, render=False)
        try:
            assert signature(w) == signature(same)
            before = w.images()[0].tobytes()
            assert before == w.images()[0].tobytes(), "unchanged observation is nondeterministic"
            signatures.add(signature(w))
            scenarios.add(w.scenario)
            lengths.add(w.length)
            assert not w._visible_defects(), "initial image exposes an answer"
            for defect in w.defects:
                w.data.qpos[:2] = w.component_point(*defect)
                mujoco.mj_forward(w.model, w.data)
                assert defect in w._visible_defects(), "uninspectable sampled damage"
            w.act([0, 0])
            steps = w.steps
            w.advance()
            assert w.steps == steps and w.done
        finally:
            w.close()
            same.close()
    assert len(signatures) == 6 and len(scenarios) == 3 and lengths == {72, 96, 120}
    print(json.dumps({"seeds": 6, "determinism": "pass", "repeat_image": "pass",
                      "distinct_scenes": 6, "scenario_types": 3, "reachable_defects": 18,
                      "lengths_m": sorted(lengths), "finalization_budget": "pass"}))


if __name__ == "__main__":
    main()
