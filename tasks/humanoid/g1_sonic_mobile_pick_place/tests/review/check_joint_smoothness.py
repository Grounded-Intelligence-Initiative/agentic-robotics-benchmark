"""Check pooled 0.10 s native velocity differences, including short prefixes and resets."""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "environment")]
from mobile_score import AuditedSession
from verify import summarize


class VelocityTrace:
    def __init__(self, velocities):
        self.velocities = velocities
        self.tick = 0
        self.episode_geometry = {"start_pelvis_xyz": [0, 0, .75], "start_ankle_xyz": np.zeros((2, 3))}
        self.last_substeps = [{"time_s": 0.}]
        self.last_target = np.zeros(29)

    @property
    def state(self):
        return {"observation": {"dq": self.velocities[self.tick]}}

    def step(self, *args):
        self.tick += 1
        self.last_substeps = []
        return self.state


def check():
    rows, intervals = [], []
    rng = np.random.default_rng(71)
    # Each trajectory starts at a different velocity. The second episode aborts after four rows.
    for length, end in ((21, "success"), (5, "aborted"), (6, "aborted"), (1, "aborted")):
        trace = rng.normal(size=(length, 29))
        monitor = AuditedSession(VelocityTrace(trace))
        for _ in trace[1:]:
            monitor.step(np.zeros(64), 0., 0.)
        row = monitor.report()
        expected = (trace[5:] - trace[:-5]) / .1
        assert row["joint_accel_samples"] == expected.size
        np.testing.assert_allclose(row["joint_accel_square_sum"], np.sum(expected**2), rtol=1e-14)
        intervals.extend(expected.ravel())
        rows.append(dict(row, shape="box", outcome=end, probes=0, duplicates=0, sensitive=0,
                         noisy=0, lost=0, native_placed=0, native_success=0, complete=0))
    config = dict(shapes=["box"], minimum_probe_fraction=.12, minimum_probe_opportunities_per_episode=1)
    result = summarize(rows, config)
    assert result["episodes_aborted"] == 3 and result["joint_acceleration_samples"] == 493
    np.testing.assert_allclose(result["joint_accel_rms_rad_s2"], np.sqrt(np.mean(np.square(intervals))))
    empty = summarize([rows[-1]], config)
    assert empty["joint_acceleration_samples"] == 0 and empty["joint_accel_rms_rad_s2"] == 0
    print("Four episodes, three aborted prefixes, 493 scalar acceleration samples; independent pooled RMS matched.")


if __name__ == "__main__":
    check()
