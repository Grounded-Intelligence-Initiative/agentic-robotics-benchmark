"""probe: a deeply nested JSON line and a malformed line on the raw wire."""
import sys


def solve(cell):
    cell.step(1)
    cell._send('{"id": 99, "method": "step", "params": ' + "[" * 200000 + "]" * 200000 + "}\n")
    try:
        print("probe deep_nest: reply", cell._recv()[:80], file=sys.stderr)
    except Exception as e:  # noqa: BLE001
        print("probe deep_nest:", type(e).__name__, file=sys.stderr)
