"""A minimal program for the belt station (scores 0): it reads the station's facts and the work order, takes one front
frame, moves the right arm a little in cartesian mode, reads the wrist force/torque and finishes. Copy it to
/home/user/submission/belt.py and build from there (cell_api.md, cell_manual.md, catalog.md)."""
import numpy as np

from beltcell.client import decode_frame


def solve(cell):
    info = cell.describe()
    wo = cell.get_work_order()
    print("engine", wo["engine_id"], "belt", wo["belt_part_number"], "window", info["drive"]["tensioner"]["window_marks_deg"])
    rgb, depth = decode_frame(cell.get_camera("front"))
    print("front frame", rgb.shape, "valid depth", float(np.isfinite(depth).mean()))
    st = cell.get_state()
    p = st["right"]["tcp_pos"]
    q = st["right"]["tcp_quat"]
    cell.set_control("right", "cartesian", 1.0)
    cell.command(right={"tcp_pos": [p[0] + 0.03, p[1], p[2]], "tcp_quat": q})
    cell.tare("right")
    r = cell.step(100)
    st = cell.get_state()
    print("sim_time", r["sim_time"], "right tcp", st["right"]["tcp_pos"], "F/T", st["right"]["ft"])
    cell.finish()
