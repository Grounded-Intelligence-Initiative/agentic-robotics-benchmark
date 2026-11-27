"""Right-arm wooden-block reference for the public practice layout."""
import math

import numpy as np

WOOD = "right_wood_play_piece"


def aim_wood_at_receiver(rows, arm_base, target, limits):
    """Preserve the live-state grasp and aim the tested wood motion at the bin."""
    bearing = math.atan2(target[1] - arm_base[1], target[0] - arm_base[0])
    for row in rows[7:]:
        row[1] = bearing
        if row[0] <= 3.90:
            row[2] -= .19
            row[3] -= .25
            row[4] -= .30
        else:
            row[2] += .32
            row[-1] = .04695
        row[1:7] = np.clip(row[1:7], limits[:,0], limits[:,1]).tolist()
