"""Author render smoke: four actual public camera frames, no rollouts."""
import argparse
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/'tests/runtime'))
from PIL import Image
from world import World, CAMERAS

parser = argparse.ArgumentParser()
parser.add_argument('--seed', type=int, default=1001)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
w = World(args.seed)
try:
    for phase in range(2):
        if phase:
            w.steps[0] = 12
            w.advance()
        for camera in CAMERAS:
            im = w.image(camera)
            assert im.std() > 10, 'blank frame'
            Image.fromarray(im).save(args.out/f'{args.seed}-{phase}-{camera}.png')
finally:
    w.close()
