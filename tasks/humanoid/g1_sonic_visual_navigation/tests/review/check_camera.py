"""Show the moving shutter in true RGB-D while keeping an unrevealed future maneuver private."""

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
    fields = {
        "q",
        "dq",
        "base_gyro",
        "gravity_body",
        "yaw_rate_rad_s",
        "velocity_body",
        "left_foot_contact",
        "right_foot_contact",
    }
    live_course = Course(6.25, 90.0, 1.5)
    hidden_pivot = Course(6.25, -84.0, 1.9)
    action = {"rows": [{"codes": [16] * 64}]}
    with ExitStack() as stack:

        def start(course, phases):
            env = stack.enter_context(LocomotionSession(assets, 7, horizon=HORIZON))
            layout = make_layout(7, course.first_m, service="both", tail="right", phases=phases)
            return NavigationEpisode(env, course, obstacle_seed=7, layout=layout)

        live = start(live_course, (2.0, 9.0))
        twin = start(hidden_pivot, (2.0, 9.0))
        swapped = start(live_course, (9.0, 2.0))
        changed = {}
        for tick in range(11):
            current, future, alt = live.observe(), twin.observe(), swapped.observe()
            assert current == future and current["tick"] == tick and set(current["observation"]) == fields
            assert (
                current["observation"] == alt["observation"]
                and current["decoder_history"] == alt["decoder_history"]
            )
            if tick == 0:
                for modality in ("rgb_png", "depth_mm_png"):
                    changed[modality] = int(
                        np.count_nonzero(pixels(current, modality) != pixels(alt, modality))
                    )
                    assert changed[modality] > 0, modality
            for ep, packet in ((live, current), (twin, future), (swapped, alt)):
                ep.advance(action, packet["observation_id"])
            for key in ("qpos", "qvel", "ctrl"):
                assert np.array_equal(getattr(live.session.data, key), getattr(twin.session.data, key)), key
        assert not live.summary()["obstacle_contact"] and not swapped.summary()["obstacle_contact"]
    return dict(
        robot_seed=7,
        layout_seed=7,
        scene_phases_right_left_s=[2.0, 9.0],
        alternate_phases_s=[9.0, 2.0],
        identical_future_prefix_rows=11,
        unchanged_robot_sensors_on_phase_swap=11,
        changed_reset_values=changed,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets-root", default=os.getenv("SONIC_ASSETS", "/opt/sonic/assets"))
    print(json.dumps(check(parser.parse_args().assets_root), indent=2))
