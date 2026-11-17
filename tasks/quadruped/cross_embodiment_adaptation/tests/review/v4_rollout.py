"""Author-only native rollout diagnostics; run substantial suites remotely.

This direct-World harness is NOT an official verifier/anchor measurement. It uses
privileged state and no rendering by default. No seed selection or acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/'tests/runtime'))
sys.path.insert(0, str(TASK/'solution/oracle'))
import world as W
import oracle as O


def mission(seed, mode, render=False, frames=None):
    w = W.World(seed, render=render)
    trace, error = [], None
    started = time.monotonic()
    try:
        if frames:
            from PIL import Image
            for camera in W.CAMERAS:
                Image.fromarray(w.image(camera)).save(frames/f'{seed}-scout-{camera}.png')
        O.run(w, get_truth=w.truth, mode=mode, trace=trace)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    if frames and w.phase:
        from PIL import Image
        for camera in W.CAMERAS:
            Image.fromarray(w.image(camera)).save(frames/f'{seed}-carrier-{camera}.png')
    result = dict(seed=seed, mode=mode, seconds=time.monotonic()-started, error=error,
                  **w.metrics(), trace=trace)
    w.close()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', default='1001', help='hidden, comma-separated integers, or inclusive LO-HI')
    parser.add_argument('--modes', default='full', help='full,blind,scout_route,xy,circumscribed')
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.seeds == 'hidden':
        seeds = json.loads((TASK/'tests/hidden/sites.json').read_text())['seeds']
    elif '-' in args.seeds:
        lo, hi = map(int, args.seeds.split('-'))
        seeds = range(lo, hi+1)
    else:
        seeds = list(map(int, args.seeds.split(',')))
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for seed in seeds:
        for mode in args.modes.split(','):
            row = mission(seed, mode, args.render, args.out if args.render else None)
            (args.out/f'{seed}-{mode}.json').write_text(json.dumps(row, indent=2)+'\n')
            rows.append({k: v for k, v in row.items() if k != 'trace'})
            print(json.dumps(rows[-1]), flush=True)
    sources = list((TASK/'tests/runtime').glob('*.py'))+[TASK/'solution/oracle/oracle.py',
               TASK/'tests/hidden/sites.json', TASK/'tests/hidden/candidates.json', TASK/'tests/grader_config.json']
    hashes = {str(p.relative_to(TASK)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    summary = dict(version=4, kind='privileged native direct-World diagnostics, not official ALE verification',
                   source_sha256=hashes, mujoco_version=W.mujoco.__version__, rows=rows)
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return int(any(not r['success'] for r in rows if r['mode'] == 'full'))


if __name__ == '__main__':
    sys.exit(main())
