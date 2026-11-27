"""Render a calibration recording's stereo endoscope video.

  python3 render_frames.py <specs.json> <root>      (setup: every instrument in specs.json)

render_spec(spec, out) writes <out>/frames.csv (frame index and camera time) and the left and
right images as JPEG, from a spec made by case.calibration_recording: the joint angles at each
frame, the endoscope's true pose, the tissue texture, the noise seed and the smoke intervals.
The grader calls render_spec for each hidden instrument; setup runs main() with a byte-identical
copy in environment/setup/render/. Deterministic for a given spec.
"""

import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import endoscope as E  # noqa: E402


def render_spec(spec, out):
    stamps, angles = np.array(spec["stamps"]), np.array(spec["angles"])
    endo = E.Stereo(np.array(spec["R"]), np.array(spec["pos"]), int(spec["tissue"]),
                    occluder=spec.get("occluder"))
    rng = np.random.default_rng(int(spec["noise_seed"]))
    for side in ("left", "right"):
        os.makedirs(os.path.join(out, side), exist_ok=True)
    with open(os.path.join(out, "frames.csv"), "w") as f:
        f.write("frame,camera_t_s\n")
        for k, (t, q) in enumerate(zip(stamps, angles)):
            f.write("{:d},{:.5f}\n".format(k, t))
            haze = 0.0
            for t0, t1, h in spec.get("smoke", []):
                if t0 <= t <= t1:
                    # the haze builds up and clears over half a second
                    haze = max(haze, h * min(1.0, (t - t0) / 0.5, (t1 - t) / 0.5))
            for side, img in zip(("left", "right"), endo.render(q[0], q[1], q[2], rng, haze)):
                Image.fromarray((img * 255.0 + 0.5).astype(np.uint8)).save(
                    os.path.join(out, side, "{:05d}.jpg".format(k)), quality=92, subsampling=0)
    endo.close()


def _job(args):
    spec, out = args
    render_spec(spec, out)
    return out


def main():
    from multiprocessing import Pool
    src, root = sys.argv[1], sys.argv[2]
    with open(src) as f:
        specs = json.load(f)
    jobs = [(spec, os.path.join(root, name, "calibration", "endoscope"))
            for name, spec in sorted(specs.items())]
    with Pool(2) as pool:
        for out in pool.imap_unordered(_job, jobs):
            print("[render]", out, flush=True)


if __name__ == "__main__":
    main()
