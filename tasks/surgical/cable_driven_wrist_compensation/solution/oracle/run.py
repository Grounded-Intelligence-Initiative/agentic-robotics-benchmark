"""The reference program: calibrate a freshly mounted instrument from its calibration
recording, then compensate the requested trajectories.

  python3 run.py <instrument_dir> <response.json>

<instrument_dir>/calibration/ is the recording (motor log, stereo endoscope video, reported
endoscope pose); <instrument_dir>/request.json the trajectories. If the identification fails,
the stock firmware's commands are sent instead, so every episode is answered.
"""

import json
import os
import sys
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compensate as CO  # noqa: E402
import fit  # noqa: E402


def stock(req):
    out = []
    for ep in req["episodes"]:
        pyg = np.asarray(ep["desired"], dtype=float)
        tp, j1, j2 = pyg[:, 0], pyg[:, 1] + 0.5 * pyg[:, 2], pyg[:, 1] - 0.5 * pyg[:, 2]
        cmd = np.column_stack([tp / 1.0186, (j1 + 0.8154 * tp) / 1.2177,
                               (j2 + 0.8154 * tp) / 1.2177])
        out.append({"id": ep["id"], "motor_cmd": np.round(cmd, 7).tolist()})
    return {"episodes": out}


def main():
    folder, resp_path = sys.argv[1], sys.argv[2]
    with open(os.path.join(folder, "request.json")) as f:
        req = json.load(f)
    with open(resp_path, "w") as f:      # an answer at once, in case the rest runs out of time
        json.dump(stock(req), f)
    try:
        resp = CO.compensate(fit.identify(folder), req)
    except Exception:
        traceback.print_exc()
        return
    tmp = resp_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(resp, f)
    os.replace(tmp, resp_path)


if __name__ == "__main__":
    main()
