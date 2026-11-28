"""Maintainers' tool: run a state-reading controller through procedures without rendering (seconds, no GPU).

    python3 tests/review/fast_eval.py [--seeds 1,2,3 | --hidden | --dev] [--controller reference|idle] [--probe-rate 0.07]

The reference controller reads `obs["privileged"]`, never the picture, so its score does not depend on the renderer:
this runs the grader's own Session and Episode with a null renderer, the probes included, in-process. It is how the
reference was tuned and how the fixed sets were checked; the anchor itself is measured through tests/verify.py's loop
with the real renderer (tests/review/run_native.py, and `ale validate`).
"""

import argparse
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TASK, "tests"))
sys.path.insert(0, os.path.join(TASK, "environment", "setup", "payload", "given"))
sys.path.insert(0, os.path.join(TASK, "solution", "reference"))

import numpy as np  # noqa: E402

from endosim import model, procedure  # noqa: E402
from endosim.session import NullRenderer, Session  # noqa: E402
from robotics_grader import replies_differ  # noqa: E402


class Idle(object):
    def reset(self):
        pass

    def act(self, t, obs):
        return [0.0, 0.0, 0.0, 0.0]


def run(proc, controller, probe_rate=0.0, rng=None, frame_delay=None, verbose=False):
    rng = rng or random.Random(0)
    sess = Session(proc, NullRenderer(), frame_delay=frame_delay, privileged=True)
    controller.reset()
    probes = changed = 0
    while not sess.done:
        obs = sess.observation()
        fake = sess.probe_copy(obs, rng) if rng.random() < probe_rate else None
        if fake is not None:
            if rng.random() < 0.5:
                other = controller.act(sess.t, fake)
                action = controller.act(sess.t, obs)
            else:
                action = controller.act(sess.t, obs)
                other = controller.act(sess.t, fake)
            probes += 1
            changed += int(replies_differ(other, action, canonical=lambda a: [float(x) for x in model.clip_command(a)]))
        else:
            action = controller.act(sess.t, obs)
        sess.step(action)
        if sess.episode.taken_over:
            sess.finish_idle()                # as the grader: no more requests after a takeover
    out = sess.episode.summary()
    out.update(probes=probes, probes_changed=changed)
    if verbose:
        tr = sess.episode.trace
        runs, k = [], 0
        while k < len(tr):
            j = k
            while j < len(tr) and tr[j] == tr[k]:
                j += 1
            if tr[k]:
                runs.append((k + 1, j, "+".join(tr[k])))
            k = j
        out["bad_runs"] = runs
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="1,2,3,4,5,6,7,8")
    ap.add_argument("--hidden", action="store_true")
    ap.add_argument("--dev", action="store_true")
    ap.add_argument("--controller", default="reference")
    ap.add_argument("--probe-rate", type=float, default=0.07)
    ap.add_argument("--steps", type=int, default=800)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    if a.hidden or a.dev:
        path = os.path.join(TASK, "tests", "hidden", "procedures.json") if a.hidden else \
            os.path.join(TASK, "environment", "commission_service", "dev_procedures.json")
        doc = json.load(open(path))
        items = [(p["seed"], p.get("factors"), doc.get("steps", a.steps)) for p in doc["procedures"]]
    else:
        items = [(int(s), None, a.steps) for s in a.seeds.split(",")]
    pool = "dev" if a.dev else "all"
    if a.controller == "reference":
        import controller as ref
        ctl = ref.Controller()
    else:
        ctl = Idle()
    vals, probes, changed = [], 0, 0
    for seed, factors, steps in items:
        proc = procedure.generate(seed, factors, steps=steps, pool=pool)
        res = run(proc, ctl, probe_rate=a.probe_rate, rng=random.Random(seed), verbose=a.verbose)
        vals.append(res["useful_view_fraction"])
        probes, changed = probes + res["probes"], changed + res["probes_changed"]
        f = proc.facts()
        print("{:>20} {:<12} scope {:>2} x{} a{} s{} | {:.3f} | fail lens {:.2f} zone {:.2f} zoom {:.2f} hor {:.2f} mot {:.2f} | contacts {} first {}".format(
            seed, f["kind"], f["scope"], f["exchanges"], int(f["assistant"]), int(f["smoke"]), res["useful_view_fraction"],
            res["failed_lens"], res["failed_zone"], res["failed_zoom"], res["failed_horizon"], res["failed_motion"],
            res["lens_contacts"], res["first_usable_step"]) + " | probes {} changed {}".format(res["probes"], res["probes_changed"])
            + " | takeover {} | critical {} least seen {}".format(res["takeover"], res["critical_actions"], res["critical_min_seen"]))
        if a.verbose:
            print("    ", res["bad_runs"])
            print("    ", [g for g in proc.gestures if g[0] <= steps])
    print("mean useful_view_fraction {:.4f}  (min {:.3f}, max {:.3f}, n {})".format(float(np.mean(vals)), min(vals), max(vals), len(vals)))
    print("probe pairs {}, of which the action differed in {} ({:.1%})".format(probes, changed, changed / max(1, probes)))


if __name__ == "__main__":
    main()
