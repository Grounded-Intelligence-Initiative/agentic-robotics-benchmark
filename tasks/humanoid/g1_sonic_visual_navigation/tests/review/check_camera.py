"""Check live/alternate sensor isolation against a twin native public-prefix replay."""
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


def pixels(packet, field):
    return np.asarray(Image.open(BytesIO(base64.b64decode(packet['camera'][field]))))


def check(assets):
    fields = {'q', 'dq', 'base_gyro', 'gravity_body', 'yaw_rate_rad_s', 'velocity_body',
              'left_foot_contact', 'right_foot_contact'}
    forward = Course(5., 90., 1.5)
    hidden_future = Course(5., -83., 1.9)
    further = Course(5.8, -83., 1.9)
    action = {'rows': [{'codes': [16]*64}]}
    with ExitStack() as stack:
        def start(course):
            env = stack.enter_context(LocomotionSession(assets, 7, horizon=HORIZON))
            return NavigationEpisode(env, course)
        main, twin = start(forward), start(hidden_future)
        for tick in range(11):
            actual, other = main.observe(), twin.observe()
            assert actual == other and actual['tick'] == tick
            assert set(actual['observation']) == fields
            moved = main.observe(alternative=further)
            relit = main.relight(actual, .78)
            for packet in (moved, relit):
                assert all(actual[k] == packet[k] for k in actual if k not in ('camera','observation_id'))
                assert np.any(pixels(actual, 'rgb_png') != pixels(packet, 'rgb_png'))
            assert np.any(pixels(actual, 'depth_mm_png') != pixels(moved, 'depth_mm_png'))
            assert actual['camera']['depth_mm_png'] == relit['camera']['depth_mm_png']
            assert main.observe() == actual
            for episode, packet in ((main, actual), (twin, other)):
                episode.advance(action, packet['observation_id'])
            for key in ('qpos','qvel','ctrl'):
                assert np.array_equal(getattr(main.session.data, key), getattr(twin.session.data, key))
    return {'seed': 7, 'native_twin_rows': 11, 'identical_public_live_packets': 11,
            'counterfactual_and_relighting_checks': 11, 'native_state_checks': 11}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets-root', default=os.getenv('SONIC_ASSETS', '/opt/sonic/assets'))
    print(json.dumps(check(parser.parse_args().assets_root), indent=2))
