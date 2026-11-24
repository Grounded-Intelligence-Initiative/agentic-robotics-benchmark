"""Maintainers' tool: render a procedure natively (Isaac Sim on this machine's GPU) with the reference controller
holding the camera, and keep frames to look at.

    <isaac python> tests/review/render_preview.py --out DIR [--seed N | --hidden K | --dev K] [--every 20] [--canary]
                   [--controller reference|idle|diver] [--steps N]

Writes DIR/<tag>_<step>.jpg (the frame a controller would get), with --raw also the renderer's untouched frame, and
prints the render timing and the episode's summary. Not part of grading.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))
sys.path.insert(0, os.path.join(TASK, "environment", "setup", "payload", "given"))
sys.path.insert(0, os.path.join(TASK, "solution", "reference"))

from endosim import procedure  # noqa: E402
from endosim.render_client import Renderer  # noqa: E402
from endosim.session import Session  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--seeds", help="several seeds, comma-separated, one after the other in one renderer")
    ap.add_argument("--kind")
    ap.add_argument("--scope")
    ap.add_argument("--hidden", type=int)
    ap.add_argument("--dev", type=int)
    ap.add_argument("--every", type=int, default=20)
    ap.add_argument("--steps", type=int, default=800)
    ap.add_argument("--stop", type=int)
    ap.add_argument("--canary", action="store_true")
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--controller", default="reference")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    r = Renderer(os.path.join(TASK, "tests"), log_path=os.path.join(a.out, "renderer.log"))
    print("renderer up in %.1f s: %s" % (time.time() - t0, r.started), flush=True)
    if a.canary:
        t0 = time.time()
        res = r.canary()
        print("canary (%.1f s):" % (time.time() - t0), json.dumps(res), flush=True)
    import base64
    import controller as ref
    for seed in ([int(x) for x in a.seeds.split(",")] if a.seeds else [a.seed]):
        factors, steps = {}, a.steps
        if a.kind:
            factors["kind"] = a.kind
        if a.scope:
            factors["scope"] = a.scope
        if a.hidden is not None or a.dev is not None:
            path = os.path.join(TASK, "tests", "hidden", "procedures.json") if a.hidden is not None else \
                os.path.join(TASK, "environment", "commission_service", "dev_procedures.json")
            doc = json.load(open(path))
            item = doc["procedures"][a.hidden if a.hidden is not None else a.dev]
            seed, factors, steps = item["seed"], item.get("factors") or {}, doc.get("steps", steps)
        proc = procedure.generate(seed, factors, steps=steps, pool="dev" if a.dev is not None else "all")
        print("procedure:", proc.facts(), flush=True)
        ctl = ref.Controller()
        t0 = time.time()
        sess = Session(proc, r, privileged=True)
        print("scene loaded in %.1f s" % (time.time() - t0), flush=True)
        tag = "s%d" % seed
        f0, ms0, x0 = r.frames, r.render_ms, r.extra_renders
        t0 = time.time()
        stop = a.stop or steps
        while not sess.done and sess.t < stop:
            if a.raw and sess.t % a.every == 0:
                sess._serial += 1
                jpeg, png = r.render(sess.episode.render_state(sess._serial), raw=True)
                open(os.path.join(a.out, "%s_%04d_raw.png" % (tag, sess.t)), "wb").write(png)
            obs = sess.observation()
            if sess.t % a.every == 0:
                open(os.path.join(a.out, "%s_%04d.jpg" % (tag, sess.t)), "wb").write(base64.b64decode(obs["image"]))
            if a.controller == "reference":
                action = ctl.act(sess.t, obs)
            elif a.controller == "diver":       # into the tissue, then back into the cannula to clean, then in again
                q, t = obs["joints"], sess.t
                action = [0.0, 0.0, 0.05, 0.0] if t < 60 or t >= 170 else [0.0, 0.0, -0.05 if q[2] > 0.02 else 0.0, 0.0]
            else:
                action = [0.0, 0.0, 0.0, 0.0]
            sess.step(action)
        n = max(1, r.frames - f0)
        print("frames %d in %.1f s wall (%.1f ms each in the renderer, %.3f extra renders per frame, restarts %d)" % (
            n, time.time() - t0, (r.render_ms - ms0) / n, (r.extra_renders - x0) / n, r.restarts), flush=True)
        print("summary:", sess.episode.summary(), flush=True)
    print("info:", r.info(), flush=True)
    r.close()


if __name__ == "__main__":
    main()
