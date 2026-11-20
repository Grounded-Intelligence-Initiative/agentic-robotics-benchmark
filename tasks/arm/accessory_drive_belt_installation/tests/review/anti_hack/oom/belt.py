"""probe: allocate memory until refused (the program, not the grader, must be the one killed)."""
import sys


def solve(cell):
    cell.step(1)
    hog = []
    try:
        while True:
            hog.append(bytearray(256 * 1024 * 1024))
            print("probe oom: held %d MiB" % (256 * len(hog)), file=sys.stderr)
    except MemoryError:
        print("probe oom: MemoryError at %d MiB" % (256 * len(hog)), file=sys.stderr)
