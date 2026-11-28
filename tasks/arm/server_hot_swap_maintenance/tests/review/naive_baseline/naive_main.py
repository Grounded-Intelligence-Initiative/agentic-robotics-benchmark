"""A naive baseline (maintainers' review only): the reference's script (oracle.py, copied in beside this file),
told which parts failed and every part's state, but planning with the server at its NOMINAL pose (no calibration):
every pose it reads from the truth that is tied to the server (the rack, the slots, the fans' bays, the caddies and
fans in the server) is moved by the inverse of the server's hidden pose error, as an uncalibrated controller would
see it. Parts on the tables are left where they are. An upper bound on what skipping calibration can score.
Needs the oracle token (run it with the reference's .oracle_token)."""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import oracle_main  # noqa: E402
from oracle import Oracle  # noqa: E402

RACK_NOM = np.array([0.31, 0.0, 0.18])


def nominalise(T):
    rp, rR = np.array(T["rack_pos"]), np.array(T["rack_R"])
    f = lambda p: (RACK_NOM + rR.T @ (np.asarray(p) - rp)).tolist()
    fR = lambda R: (rR.T @ np.asarray(R)).tolist()
    near = lambda p: abs(p[1]) < 0.30 and p[0] > 0.26
    T = dict(T)
    T["rack_pos"], T["rack_R"] = RACK_NOM.tolist(), np.eye(3).tolist()
    T["slot_frames"] = [f(p) for p in T["slot_frames"]]
    T["fan_frames"] = [f(p) for p in T["fan_frames"]]
    for key in ("caddies", "fans"):
        d = {}
        for n, c in T[key].items():
            c = dict(c)
            if near(c["pos"]):
                c["pos"], c["R"] = f(c["pos"]), fR(c["R"])
                if "lever_pos" in c:
                    c["lever_pos"] = f(c["lever_pos"])
            d[n] = c
        T[key] = d
    return T


class NaiveOracle(Oracle):
    def act(self, obs):
        obs = dict(obs, truth=nominalise(obs["truth"]))
        return Oracle.act(self, obs)


oracle_main.Oracle = NaiveOracle

if __name__ == "__main__":
    oracle_main.main()
