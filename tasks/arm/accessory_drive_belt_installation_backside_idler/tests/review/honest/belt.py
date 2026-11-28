"""Fit the accessory belt (drive B) with the two arms: entry point for the grader's launcher."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hseq  # noqa: E402


def solve(cell):
    given = os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))
    s = hseq.Seq(cell, given)
    err = s.run()
    print("[belt] end:", err, file=sys.stderr)
    for row in s.log:
        print("[belt]", str(row)[:400], file=sys.stderr)
