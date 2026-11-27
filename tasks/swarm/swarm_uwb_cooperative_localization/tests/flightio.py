"""Writing a flight folder in the format the agent's program reads. Shared by the grader
(hidden flights) and tests/gen/make_flights.py (calibration and practice flights)."""

import json
import os

import numpy as np

import world as W


def site_dict():
    return {
        "frame": "warehouse: x along the long wall, y across, z up from the floor; metres",
        "hall": {"x": [W.HALL[0], W.HALL[1]], "y": [W.HALL[2], W.HALL[3]],
                 "z": [W.HALL[4], W.HALL[5]]},
        "racks": [{"x": [b[0], b[1]], "y": [b[2], b[3]], "z": [b[4], b[5]]} for b in W.RACKS],
        "beacons": [{"id": W.N_DRONES + i, "position": [round(float(v), 3) for v in p]}
                    for i, p in enumerate(W.ANCHORS)],
        "drones": [{"id": i, "pad": [round(float(v), 3) for v in W.PADS[i]]}
                   for i in range(W.N_DRONES)],
    }


def write_flight(folder, sensors, duration_s, truth=None, truth_rate=10.0):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "site.json"), "w") as f:
        json.dump(site_dict(), f, indent=1)
    with open(os.path.join(folder, "flight.json"), "w") as f:
        json.dump({"duration_s": duration_s, "drones": list(range(W.N_DRONES)),
                   "output_rate_hz": 10.0}, f, indent=1)
    with open(os.path.join(folder, "uwb.csv"), "w") as f:
        f.write("t_s,initiator,responder,range_m\n")
        for t, a, b, r in sensors["uwb"]:
            f.write("{:.3f},{:d},{:d},{:.4f}\n".format(t, int(a), int(b), r))
    for i in range(W.N_DRONES):
        fl, gy = sensors["flow"][i], sensors["gyro"][i]
        with open(os.path.join(folder, "drone_{}.csv".format(i)), "w") as f:
            f.write("t_s,flow_vx_mps,flow_vy_mps,tof_m,gyro_z_rps\n")
            for row, g in zip(fl, gy):
                vals = ["{:.3f}".format(row[0])]
                vals += ["" if not np.isfinite(v) else "{:.4f}".format(v) for v in row[1:4]]
                vals.append("{:.5f}".format(g[1]))
                f.write(",".join(vals) + "\n")
    if truth is not None:
        step = int(round(1.0 / truth_rate / truth["dt"]))
        with open(os.path.join(folder, "truth.csv"), "w") as f:
            f.write("t_s,drone,x_m,y_m,z_m,yaw_rad\n")
            for k in range(0, len(truth["t"]), step):
                for i in range(W.N_DRONES):
                    p = truth["P"][i, k]
                    yaw = (truth["Y"][i, k] + np.pi) % (2 * np.pi) - np.pi
                    f.write("{:.2f},{:d},{:.4f},{:.4f},{:.4f},{:.4f}\n".format(
                        truth["t"][k], i, p[0], p[1], p[2], yaw))


def truth_samples(truth, rate=10.0):
    """Truth positions at the output instants: (times (M,), positions (D, M, 3))."""
    step = int(round(1.0 / rate / truth["dt"]))
    idx = np.arange(0, len(truth["t"]), step)
    return truth["t"][idx], truth["P"][:, idx, :]
