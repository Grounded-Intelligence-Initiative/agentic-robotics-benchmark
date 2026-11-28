"""Author counterfactuals: no steering, and identical turn trains with exchanged leg plugs."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
from reference import Controller, HOME, pose
from service import dispatch
from world import World


def heading(w):
    r = w.data.xmat[w.trunk].reshape(3,3)
    return math.atan2(r[1,0], r[0,0])


def straight(seed, index):
    w = World(seed, render=True, scene_index=index)
    try:
        operator = Controller(lambda method, **params: dispatch(w, method, params))
        operator.commission()
        w.advance()
        phase, closest_second = 0., math.inf
        for _ in range(1000):  # diagnostic failure if the spatial endpoint is never reached
            x, y = w.data.xpos[w.trunk,:2]
            gx, gy, yaw = w.stations[w.current-1]
            dist = math.hypot(gx-x, gy-y)
            if w.current == 2:
                closest_second = min(closest_second, dist)
            # Give the no-steering control privileged timing of its stop. It never uses truth to steer.
            if dist < .102 and abs(math.remainder(yaw-heading(w), 2*math.pi)) < math.radians(17):
                if operator.act(HOME, 400)["goal_reached"]:
                    if w.delivered == 6:
                        break
                    w.advance()
                    phase = 0.
            if x > max(p[0] for p in w.stations[:2])+.85:
                break
            phase += 14*.002*1.9
            if operator.act(pose(phase, .10, 0.), 14)["fallen"]:
                raise RuntimeError("straight control fell before passing the second bay")
        else:
            raise RuntimeError("straight control censored before spatial endpoint")
        return {"seed": seed, "calibration": w.calibration, "approaches": w.approaches, "docks": w.delivered,
                "poses": w.steps, "ticks": w.ticks, "falls": w.falls, "final_x_m": float(w.data.qpos[0]),
                "closest_second_m": closest_second if math.isfinite(closest_second) else None,
                "milestone_fraction": w.metrics()["milestone_fraction"]}
    finally:
        w.close()


def turns(seed, index, sagittal_only=False):
    w = World(seed, render=False, scene_index=index)
    results = {}
    try:
        for sign, side in ((1, "left"), (-1, "right")):
            for swapped in (False, True):
                w.recover()
                operator = Controller(lambda method, **p: dispatch(w, method, p))
                operator.legs_by_port = tuple(leg ^ int(swapped) for leg in w.legs_by_port)
                a0, xy0 = heading(w), w.data.qpos[:2].copy()
                for k in range(56):
                    phase = (k+1)*14*.002*1.9
                    if sagittal_only:
                        # Isolate fore/aft differential action; the reference also abducts the feet.
                        right, left = pose(phase, sign*.065, 0.), pose(phase, -sign*.065, 0.)
                        cmd = right[:3]+left[3:6]+right[6:9]+left[9:]
                    else:
                        cmd = pose(phase, 0., sign*.065)
                    if operator.act(cmd, 14)["fallen"]:
                        raise RuntimeError("turn sensitivity probe fell")
                angle = math.degrees(math.remainder(heading(w)-a0, 2*math.pi))
                results[side+("_exchanged" if swapped else "_calibrated")] = {
                    "yaw_deg": angle, "drift_m": float(np.linalg.norm(w.data.qpos[:2]-xy0))}
        return {"seed": seed, "physical_legs_by_port": w.legs_by_port, **results}
    finally:
        w.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, help="one development seed; default: all registered seeds")
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    seeds = hidden if args.seed is None else [args.seed]
    rows = []
    for seed in seeds:
        index = hidden.index(seed) if seed in hidden else -1
        row = {"seed": seed, "straight": straight(seed, index), "turns": turns(seed, index),
               "sagittal": turns(seed, index, True)}
        a, b, c = row["straight"], row["turns"], row["sagittal"]
        row["pass"] = (a["calibration"] == 2 and a["docks"] == a["approaches"] == 1 and
                       all(sign*b[side+"_calibrated"]["yaw_deg"] > 18
                           and sign*(b[side+"_calibrated"]["yaw_deg"]-b[side+"_exchanged"]["yaw_deg"]) > 18
                           and sign*c[side+"_calibrated"]["yaw_deg"] > 0 > sign*c[side+"_exchanged"]["yaw_deg"]
                           for sign,side in ((1, "left"), (-1, "right"))))
        (args.output/f"{seed}.json").write_text(json.dumps(row, indent=2)+"\n")
        rows.append(row)
        print(json.dumps(row), flush=True)
    passed = all(r["pass"] for r in rows)
    (args.output/"summary.json").write_text(json.dumps({"passed": passed, "runs": rows}, indent=2)+"\n")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
