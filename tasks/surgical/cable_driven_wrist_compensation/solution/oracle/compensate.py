"""Feed-forward motor commands for requested wrist trajectories.

  python3 compensate.py <request.json> <response.json>     (params.json beside this file)

Pitch first (its inverse needs only the pitch model), then each jaw with the planned pitch
as the coupling input and width scale, then each motor's servo lag; episode after episode,
each starting from the play states the one before left.
"""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wristmodel as WM  # noqa: E402


def compensate(params, req):
    """The episodes run back to back from the state the calibration recording left (nothing
    re-settles the wrist): each joint's play states are carried from one episode's inverse
    to the next."""
    models = [WM.JointModel.from_dict(params[k]) for k in ("pitch", "jaw1", "jaw2")]
    taus = params["tau"]
    start = params.get("state", {})
    states = [(np.array(start[k]["y"]), start[k]["v"]) if k in start else None
              for k in ("pitch", "jaw1", "jaw2")]
    out = []
    for ep in req["episodes"]:
        pyg = np.asarray(ep["desired"], dtype=float)
        pitch = pyg[:, 0]
        targets = (pitch, pyg[:, 1] + 0.5 * pyg[:, 2], pyg[:, 1] - 0.5 * pyg[:, 2])
        q = np.empty((len(pyg), 3))
        for i in range(3):
            q[:, i], states[i] = models[i].invert(targets[i], pitch, states[i])
        cmd = np.column_stack([WM.lag_inverse(q[:, i], taus[i]) for i in range(3)])
        out.append({"id": ep["id"], "motor_cmd": np.round(cmd, 7).tolist()})
    return {"episodes": out}


def main():
    req_path, resp_path = sys.argv[1], sys.argv[2]
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "params.json")) as f:
        params = json.load(f)
    with open(req_path) as f:
        req = json.load(f)
    with open(resp_path, "w") as f:
        json.dump(compensate(params, req), f)


if __name__ == "__main__":
    main()
