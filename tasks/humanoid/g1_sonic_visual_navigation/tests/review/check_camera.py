"""Check that a changed physical gate is observable and an unrevealed future pivot is private."""
import argparse
import base64
from contextlib import ExitStack
from io import BytesIO
import json
import os

import numpy as np
from PIL import Image

from navigation_io import Course, HORIZON, NavigationEpisode
from sonic_loco import LocomotionSession
from sonic_loco.obstacles import make_layout


def pixels(packet, field):
    return np.asarray(Image.open(BytesIO(base64.b64decode(packet["camera"][field]))))


def check(assets):
    fields = {"q", "dq", "base_gyro", "gravity_body", "yaw_rate_rad_s", "velocity_body",
              "left_foot_contact", "right_foot_contact"}
    course = Course(6., 90., 1.5)
    alternate_future = Course(6., -83., 1.9)
    layout_key = 7
    live_side = make_layout(layout_key, course.first_m).gates[0].side
    opposite = next(i for i in range(64) if make_layout(i, course.first_m).gates[0].side == -live_side)
    action = {"rows": [{"codes": [16]*64}]}
    differences = {}
    with ExitStack() as stack:
        def start(route, obstacle_key):
            env = stack.enter_context(LocomotionSession(assets, 7, horizon=HORIZON))
            return NavigationEpisode(env, route, obstacle_seed=obstacle_key)
        main = start(course, layout_key)
        twin = start(alternate_future, layout_key)
        changed = start(course, opposite)
        for tick in range(11):
            live, other, changed_packet = main.observe(), twin.observe(), changed.observe()
            assert live == other and live["tick"] == tick
            assert set(live["observation"]) == fields
            assert live["observation"] == changed_packet["observation"]
            if tick == 0:
                for modality in ("rgb_png", "depth_mm_png"):
                    differences[modality] = int(np.count_nonzero(pixels(live, modality) != pixels(changed_packet, modality)))
                    assert differences[modality] > 0, modality
            for ep, packet in ((main, live), (twin, other), (changed, changed_packet)):
                ep.advance(action, packet["observation_id"])
            for key in ("qpos", "qvel", "ctrl"):
                assert np.array_equal(getattr(main.session.data, key), getattr(twin.session.data, key))
                assert np.array_equal(getattr(main.session.data, key), getattr(changed.session.data, key))
    return dict(robot_seed=7, layout_seed=layout_key, alternative_layout_seed=opposite,
                identical_future_prefix_rows=11, native_motion_rows_checked=11, changed_reset_values=differences)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets-root", default=os.getenv("SONIC_ASSETS", "/opt/sonic/assets"))
    print(json.dumps(check(parser.parse_args().assets_root), indent=2))
