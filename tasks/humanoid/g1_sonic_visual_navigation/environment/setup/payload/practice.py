"""Run a public visual course, or inspect its first egocentric RGB-D frame."""
import argparse
import base64
from contextlib import ExitStack
import gzip
import json
import os
from pathlib import Path
import signal
import sys

from navigation_io import Course, HORIZON, NavigationEpisode, validate_action
from practice_wire import WireError, solver
from sonic_loco import LocomotionSession
from sonic_loco.obstacles import make_layout
from sonic_loco.racks import VARIANTS


def run(args, course):
    with ExitStack() as stack:
        session = stack.enter_context(LocomotionSession(args.assets_root, args.seed, horizon=HORIZON))
        if (args.right_phase_s is None) != (args.left_phase_s is None):
            raise ValueError("supply both phase offsets together")
        phases = None if args.right_phase_s is None else (args.right_phase_s,args.left_phase_s)
        layout = make_layout(args.layout_seed,course.first_m,tail=None if args.tail=="auto" else args.tail,
                             service=None if args.service=="auto" else args.service,
                             phase_seed=args.phase_seed,phases=phases,rack_seed=args.rack_seed,
                             left_rack=args.left_rack,right_rack=args.right_rack)
        episode = NavigationEpisode(session, course, layout=layout)
        if args.inspect:
            args.inspect.mkdir(parents=True, exist_ok=True)
            obs = episode.observe()
            for field, name in [('rgb_png', 'rgb.png'), ('depth_mm_png', 'depth-mm.png')]:
                (args.inspect/name).write_bytes(base64.b64decode(obs['camera'][field]))
            calibration = dict(width=128, height=96, vertical_fov_deg=75., center_px=[63.5, 47.5],
                               camera_xyz_in_pelvis_m=[.28, 0., .33], optical_forward='+x', image_right='-y',
                               image_up='+z', depth='optical-axis millimetres; 0 invalid')
            (args.inspect/'calibration.json').write_text(json.dumps(calibration, indent=2)+'\n')
            return dict(files=['rgb.png', 'depth-mm.png', 'calibration.json'], tick=session.tick)
        wire = stack.enter_context(solver(args.submission))
        trace = None
        if args.trace:
            opener = gzip.open if args.trace.suffix == '.gz' else open
            trace = stack.enter_context(opener(args.trace, 'wt', encoding='utf-8'))
        while not episode.done and session.tick < args.steps:
            obs = episode.observe()
            reply = wire.request({'type': 'act', 't': session.tick, 'obs': obs})
            if set(reply) != {'action'}:
                raise WireError('act requires a single action')
            rows = validate_action(reply['action'], session.tick, session.horizon)
            applied = episode.advance(reply['action'], obs['observation_id'], max_ticks=args.steps - session.tick)
            if trace:
                trace.write(json.dumps(dict(observation=obs, proposed_rows=rows, applied_rows=applied),
                                       allow_nan=False, separators=(',', ':'))+'\n')
        result = episode.summary()
        result.update(seed=args.seed, layout_seed=args.layout_seed, reason=episode.reason or 'step_limit')
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turn', choices=('left', 'right'), default='left')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--layout-seed', type=int, default=7, help='independent furniture layout seed')
    parser.add_argument('--tail', choices=('auto','none','left','right'), default='auto',
                        help='choose the optional downstream fixed opening')
    parser.add_argument('--service', choices=('auto','both','left','right'), default='auto',
                        help='choose which first-wall leaves have full travel')
    parser.add_argument('--phase-seed', type=int, help='independently sample both shutter phases')
    parser.add_argument('--right-phase-s', type=float, help='right phase at reset in [0,12) seconds')
    parser.add_argument('--left-phase-s', type=float, help='left phase at reset in [0,12) seconds')
    parser.add_argument('--rack-seed',type=int,help='independent seed for both permanent outgoing racks')
    parser.add_argument('--left-rack',choices=VARIANTS,help='outgoing left branch: roof tier and lateral opening')
    parser.add_argument('--right-rack',choices=VARIANTS,help='outgoing right branch: roof tier and lateral opening')
    parser.add_argument('--first-m', type=float, default=6.25)
    parser.add_argument('--angle-deg', type=float, default=90.)
    parser.add_argument('--last-m', type=float, default=4.)
    parser.add_argument('--steps', type=int, default=HORIZON)
    parser.add_argument('--submission', type=Path, default=Path('/home/user/submission'))
    parser.add_argument('--assets-root', type=Path, default=Path(os.environ.get('SONIC_ASSETS', '/opt/sonic/assets')))
    parser.add_argument('--trace', type=Path)
    parser.add_argument('--inspect', type=Path, metavar='DIR')
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or not 1 <= args.steps <= HORIZON or not 75 <= args.angle_deg <= 105:
        parser.error('invalid robot seed, step limit, or unsigned angle magnitude')
    course = Course(args.first_m, args.angle_deg * (1 if args.turn == 'left' else -1), args.last_m)
    print(json.dumps(run(args, course), allow_nan=False))


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))
    signal.signal(signal.SIGHUP, lambda signum, frame: sys.exit(128 + signum))
    try:
        main()
    except (OSError, ValueError, WireError) as exc:
        sys.exit(f'practice: {exc}')
