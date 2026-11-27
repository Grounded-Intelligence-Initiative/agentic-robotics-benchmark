"""Regenerate the agent's example instruments from the hidden plant family.

Run from the task folder:  python3 tests/gen/make_payload.py

Writes environment/setup/payload/instruments/<name>/ (request.json, calibration/motors.csv,
calibration/endoscope_pose.json) for three example instruments, in exactly the format of the
hidden ones; environment/setup/render/specs.json, from which setup renders their calibration
videos (the joint angles at every frame and the endoscope's true pose: root-only in
/opt/ale/setup, never in the agent home); byte-identical copies of tests/endoscope.py,
tests/render_frames.py and tests/instrument_model/ beside it; and the instrument model the
datasheet describes, environment/setup/payload/instrument_model/ (the same meshes).
Deterministic (fixed seeds).
"""

import json
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)

import case as C  # noqa: E402

PAYLOAD = os.path.join(TASK, "environment", "setup", "payload", "instruments")
RENDER = os.path.join(TASK, "environment", "setup", "render")
EXAMPLES = (("instrument_a", 4101), ("instrument_b", 4102), ("instrument_c", 4103))


def main():
    specs = {}
    os.makedirs(RENDER, exist_ok=True)
    for name, seed in EXAMPLES:
        rng = np.random.default_rng(seed)
        cs = C.make_case(rng, n_episodes=4, episode_seconds=20.0)
        folder = os.path.join(PAYLOAD, name)
        if os.path.isdir(folder):
            shutil.rmtree(folder)
        C.write_case(folder, cs)
        spec = dict(cs["rec"]["spec"])
        spec["stamps"] = [round(v, 7) for v in spec["stamps"]]
        spec["angles"] = [[round(v, 9) for v in row] for row in spec["angles"]]
        specs[name] = spec
        print(name, len(cs["rec"]["motor_rows"]), "motor rows,", len(spec["stamps"]), "frames")
    with open(os.path.join(RENDER, "specs.json"), "w") as f:
        json.dump(specs, f, separators=(",", ":"))
    for fn in ("endoscope.py", "render_frames.py"):
        shutil.copyfile(os.path.join(TESTS, fn), os.path.join(RENDER, fn))
    model = os.path.join(TESTS, "instrument_model")
    for dst in (os.path.join(RENDER, "instrument_model"),
                os.path.join(os.path.dirname(PAYLOAD), "instrument_model")):
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(model, dst)


if __name__ == "__main__":
    main()
