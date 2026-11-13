"""probe: the oracle's own install, cut short at the moment the lever is loaded: when its tensioner return would
start (the left hand holds the lever at the install stop, the last span hangs outside T), the program returns without letting the
tensioner back, so the switch-off lets go of the lever. Expected: damage (tensioner_overspeed) on every engine that
reaches that point, belt_acceptance 0. run_all.sh assembles this folder with solution/honest/ (file uses_oracle)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hseq  # noqa: E402


class _Cut(BaseException):
    """not an Exception: the oracle's own failure handling (safe_end lets the lever back) must not catch it"""


def _cut(self):
    hseq.log("probe: return while the lever is held at %s" % (self.r.grasp.get(self.T),))
    raise _Cut()


hseq.Seq.st_return_T = _cut


def solve(cell):
    given = os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))
    s = hseq.Seq(cell, given)
    try:
        s.run()
    except _Cut:
        print("[probe] returning with the lever held", file=sys.stderr)
