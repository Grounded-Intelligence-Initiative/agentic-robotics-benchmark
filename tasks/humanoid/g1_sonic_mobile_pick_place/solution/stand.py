"""Frozen upright standing code used by the planner."""

import numpy as np

# One frozen V1 encoder evaluation offline: default upright joint pose and zero velocity.
# fmt: off
STAND_CODES = np.array([
    15,16,15,16,14,13,20,16,18,15,17,13,13,13,15,16,15,16,16,14,19,15,17,14,16,17,18,11,17,18,17,14,
    20,18,16,14,15,14,17,16,14,12,15,15,13,15,16,15,16,17,18,17,17,18,18,16,16,20,16,12,16,17,17,17,
], dtype=int)
# fmt: on
