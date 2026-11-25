"""A minimal program (it scores 0: it moves the hand above the tower, takes one picture and stops).

Copy it to ~/submission/play.py and grow it. Run it on a practice tower with
    /opt/venv/bin/python3 ~/commission/run_tower.py --site 1
"""

import sys

from towercell.client import decode_frame, hand_quat


def solve(cell):
    info = cell.describe()
    print("episode %.0f s, N_cap %d, frames %d" % (info["duration_s"], info["n_cap"], info["frame_budget"]),
          file=sys.stderr)
    st = cell.step(target={"pos": [0.50, 0.0, 0.50], "quat": hand_quat(0.0)}, speed=0.2, n=200)
    while not st["motion"]["goal_reached"]:
        st = cell.step(n=50)
    rgb, depth = decode_frame(cell.get_camera("left", depth=True))
    print("t %.2f tcp %s wrench %s frame %s" % (st["t"], st["tcp"]["pos"], st["wrench"]["force_N"], rgb.shape),
          file=sys.stderr)
    cell.finish()
