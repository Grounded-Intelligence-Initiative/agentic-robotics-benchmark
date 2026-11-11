"""Reviewer-only determinism, persistence and random-scene checks; no model API."""
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
import numpy as np
from world import World


def signature(w):
    return json.dumps({
        "mapping": w.mapping, "gain": w.gain, "walls": w.walls,
        "block": w.block.tolist(), "tray": w.tray.tolist(), "gains": w.arm_gains.tolist(),
        "target": w.target.tolist(), "gap": w.bridge_half_gap,
        "width": w.bridge_half_width, "buildings": w.buildings,
        "starts": [p.tolist() for p in w.nav_starts], "radius": w.large_radius,
    }, sort_keys=True)


def main():
    kind = json.loads((TASK/"tests/grader_config.json").read_text())["kind"]
    signatures = []
    for seed in (101, 102, 103, 104, 105, 106):
        w, same = World(kind, seed), World(kind, seed, render=False)
        try:
            assert signature(w) == signature(same), "same seed changed the task"
            signatures.append(signature(w))
            if kind in ("icl", "cross"):
                walls, mapping = list(w.walls), list(w.mapping)
                w.steps = 12
                w.completed.add(0)
                w.advance()
                assert w.walls == walls, "map changed across the phase transition"
                assert w.steps == 12, "phase switch restored action budget"
                if kind == "icl":
                    assert w.mapping == mapping, "ICL mapping changed mid-conversation"
                else:
                    assert .26 <= w.radius <= .32
            if kind == "rescue":
                assert all(abs(w.target[0]-x) > sx+.95 or abs(w.target[1]-y) > sy+.95
                           for x, y, sx, sy, _ in w.buildings)
            assert all(np.isfinite(w.data.qpos)), "invalid initial configuration"
        finally:
            w.close()
            same.close()
    assert len(set(signatures)) == len(signatures), "different seeds gave identical scenes"
    print(json.dumps({"kind": kind, "seeds": 6, "determinism": "pass", "variation": "pass",
                      "phase_persistence": "pass" if kind in ("icl", "cross") else "not_applicable"}))


if __name__ == "__main__":
    main()
