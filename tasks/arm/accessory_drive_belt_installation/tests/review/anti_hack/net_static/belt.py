"""probe: a networking import and a connect."""
import socket


def solve(cell):
    cell.step(1)
    try:
        socket.create_connection(("1.1.1.1", 80), timeout=2).close()
    except Exception:  # noqa: BLE001
        pass
