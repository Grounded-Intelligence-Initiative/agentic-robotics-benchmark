"""probe: 200000 requests written without reading any reply (the request queue must stay bounded)."""
import json
import sys


def solve(cell):
    cell.step(1)
    line = json.dumps({"id": 1, "method": "get_state", "params": {}}) + "\n"
    try:
        for _ in range(200000):
            cell._send(line)
    except Exception as e:  # noqa: BLE001
        print("probe flood:", type(e).__name__, file=sys.stderr)
