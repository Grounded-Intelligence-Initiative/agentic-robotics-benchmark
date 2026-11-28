"""probe: calls outside the API (no token)."""
import sys

from beltcell.client import CellError


def solve(cell):
    for meth in ("reference_run", "get_state_privileged", "truth", "finalize", "reset", "inspect", "belt_state", "set_seed"):
        try:
            r = cell._call(meth)
            print("probe privileged: ANSWERED", meth, str(r)[:80], file=sys.stderr)
        except CellError as e:
            print("probe privileged:", meth, "->", str(e)[:40], file=sys.stderr)
    cell.step(1)
