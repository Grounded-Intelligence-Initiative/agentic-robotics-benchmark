"""Boundary regressions for a loaded lift and quiet, fully released placement."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "environment"))
from sonic_sim.physics import ContactSample, PAD_HALF_WIDTH, TaskScore, delivery_config


def main():
    for shape in ("box", "cylinder"):
        config = delivery_config(shape, 19)
        weight = 9.81 * config.mass
        goal = np.array([0., 0., 0.78])
        score = TaskScore(goal[2], config)
        bearing = ContactSample(hand_force_z=weight, opposed=True, hand_contact=True)
        seated = ContactSample(support_force_z=weight, pad_force_z=weight)
        def advance(n, contacts, dx=0., dz=0., speed=0.):
            for _ in range(n):
                score.update(dt=.002, object_pos=goal + (dx, 0., dz), goal_pos=goal,
                             object_velocity=np.array([speed, 0., 0., 0., 0., 0.]),
                             object_rotation=np.eye(3), contacts=contacts, upright=True)
        advance(200, seated)
        assert not score.lifted and not score.placed
        advance(20, ContactSample(hand_force_z=weight), dz=.05)
        assert not score.lifted
        advance(19, bearing, dz=.05)
        assert not score.lifted
        advance(1, bearing, dz=.05)
        assert score.lifted
        advance(130, ContactSample(support_force_z=weight, pad_force_z=weight, hand_contact=True))
        advance(130, seated, dx=PAD_HALF_WIDTH)
        advance(130, seated, speed=.081)
        assert not score.placed
        advance(124, seated)
        assert not score.placed
        advance(1, seated)
        assert score.placed
    print("both object shapes passed prelift, opposition, hold, containment, hand-release and velocity boundaries")


if __name__ == "__main__":
    main()
