"""probe: 200 MB on stderr."""
import sys


def solve(cell):
    cell.step(1)
    blk = "x" * 1000000
    for _ in range(200):
        sys.stderr.write(blk)
