"""Generate public native FSQ actions from a measured tabletop and G1 pose."""

from pathlib import Path
from camera_input import target_visible, station_gap
from optical_servo import OpticalServo
import numpy as np
import hashlib
from stand import STAND_CODES


def checked_asset(name, expected):
    path = Path(__file__).with_name(name)
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError("Unexpected asset digest for " + name)
    return path


class Approach:
    def __init__(self):
        self.gait = np.load(
            checked_asset(
                "approach_codes.npy",
                "7e3d779fad10ac6f235e52ddba4f7b8ef08f434bf391ef0ecd6a8b8e61be3151",
            ),
            allow_pickle=False,
        )
        self.reset()

    def reset(self):
        self.walk_start = None
        self.classified = False
        self.manipulating = False

    def snapshot(self):
        return self.walk_start, self.classified, self.manipulating

    def restore(self, state):
        self.walk_start, self.classified, self.manipulating = state

    def actions(self, tick, rgb, views):
        if self.manipulating:
            return None
        if not self.classified:
            gap = station_gap(views)
            if gap is not None:
                self.walk_start = tick + 30 if gap > 1.0 else None
                self.classified = True
        lit = target_visible(rgb)
        if lit and (self.walk_start is None or tick >= self.walk_start + 220):
            self.manipulating = True
            return None
        rows = []
        for current in range(tick, tick + 5 - tick % 5):
            index = current - self.walk_start if self.walk_start is not None else -1
            code = self.gait[index] if 0 <= index < len(self.gait) else STAND_CODES
            rows.append({"codes": code.tolist(), "hand": [0.0, 0.0]})
        return rows


class DeliveryPolicy:
    """Run the approach and visual hand controller in the same decoder history."""

    def __init__(self, assets_root):
        from visual_body import VisualBody

        self.body = VisualBody(assets_root)
        self.table = checked_asset(
            "cartesian_fsq.npz",
            "70752eff1b4c6286cb305de08cf4a61a06893aa3ee0cf1c4605b6544e4c84915",
        )
        self.approach = Approach()
        self.reset()

    def reset(self):
        self.approach.reset()
        self.body.reset()
        self.servo = OpticalServo(self.table)
        self.frame_tick, self.rows = None, None

    def snapshot(self):
        # The spatial table and kinematic mesh stay read-only across proposals.
        return (
            self.approach.snapshot(),
            self.body.snapshot(),
            self.servo.snapshot(),
            self.frame_tick,
            self.rows,
        )

    def restore(self, state):
        mode, calibration, arm, self.frame_tick, self.rows = state
        self.approach.restore(mode)
        self.body.restore(calibration)
        self.servo.restore(arm)

    def act(self, tick, proprio, rgb, views, last_hand):
        del last_hand
        if not rgb:
            if self.frame_tick is None or not 0 < tick - self.frame_tick < len(
                self.rows
            ):
                raise ValueError("An initial camera packet is required")
            return self.rows[tick - self.frame_tick :]
        rows = self.approach.actions(tick, rgb, views)
        if rows is None:
            pose = self.body.estimate(proprio, views, tick)
            rows = self.servo.actions(pose, views, self.body)
        self.frame_tick, self.rows = tick, rows
        return rows
