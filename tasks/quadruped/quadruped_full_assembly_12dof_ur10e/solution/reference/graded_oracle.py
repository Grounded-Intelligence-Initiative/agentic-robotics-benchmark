"""Oracle driver of the graded episode (dual mode): plays it with the privileged reference program through the
agent-side client, as an agent would drive it (connect_graded, start, the cell's calls, finish), except that the start
carries the oracle token."""

import argparse
import importlib.util
import os
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--token", required=True, help="the oracle token file")
    ap.add_argument("--program", required=True, help="the reference program (solve(cell))")
    args = ap.parse_args(argv)
    given = os.environ.get("ARB_GIVEN_DIR") or os.path.join(os.path.expanduser("~"), "given")
    sys.path.insert(0, os.path.join(given, "lib"))
    from arm.assembly_info.client import CellError, connect_graded
    with open(args.token) as fh:
        token = fh.read().strip()
    spec = importlib.util.spec_from_file_location("assemble", args.program)
    program = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(program)
    cell = connect_graded()
    try:
        print("[oracle] before the start: %s" % (cell.status(),), flush=True)
        print("[oracle] start: %s" % (cell._call("start", oracle_token=token),), flush=True)
        try:
            program.solve(cell)
        finally:
            try:
                print("[oracle] finish: %s" % (cell.finish(),), flush=True)
            except CellError as e:
                print("[oracle] finish: %s" % (e,), flush=True)
            print("[oracle] at the end: %s" % (cell.status(),), flush=True)
    finally:
        cell.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
