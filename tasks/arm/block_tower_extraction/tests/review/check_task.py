"""Maintainers' consistency checks (nothing runs this at verify). Exit 0 when every check passes.

  1. the hidden set: every package of hidden/seeds.json exists and matches its sha256; selection.json agrees;
  2. hidden and practice seeds are disjoint (environment/commission_service/sites.json);
  3. the oracle token: sha256(solution/reference/oracle_token) == hidden/oracle_token.sha256;
  4. anchor.json: metric block_score, direction higher, a positive value, full_at in (0, 1];
  5. the byte copies: payload == tests/setup_payload; the given client == tests/towercell/client.py; the commissioning
     service's towercell/ and cellkit/wire.py == tests/; given_hashes.json covers the payload's files;
  6. the numerics fingerprint of this machine == tests/numerics.json (runs numerics_check.py; needs BT_PANDA_DIR);
  7. with --oracle: the reference program, in-process on the four hidden towers, reproduces anchor.json's value
     (about 20 minutes, the towers in parallel).
Usage: python3 tests/review/check_task.py [--oracle]
"""
import filecmp
import hashlib
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def same_tree(a, b):
    c = filecmp.dircmp(a, b, ignore=["__pycache__"])
    if c.left_only or c.right_only or c.diff_files or c.funny_files:
        return False
    return all(same_tree(os.path.join(a, d), os.path.join(b, d)) for d in c.common_dirs)


def checks(run_oracle):
    out = []
    hd = os.path.join(TESTS, "hidden")
    pins = json.load(open(os.path.join(hd, "seeds.json")))["towers"]
    sel = json.load(open(os.path.join(hd, "selection.json")))
    ok = all(os.path.isfile(os.path.join(hd, e["file"])) and sha(os.path.join(hd, e["file"])) == e["sha256"] for e in pins)
    ok = ok and sorted(v["sha256"] for v in sel["chosen"].values()) == sorted(e["sha256"] for e in pins)
    out.append(("hidden set pinned", ok, "%d towers" % len(pins)))
    hidden = [json.load(open(os.path.join(hd, e["file"])))["seed"] for e in pins]
    sites = json.load(open(os.path.join(TASK, "environment", "commission_service", "sites.json")))
    practice = [s["seed"] for s in sites["sites"]]
    out.append(("hidden / practice seeds disjoint", not set(hidden) & set(practice),
                "%d hidden, %d practice" % (len(hidden), len(practice))))
    tok = open(os.path.join(TASK, "solution", "reference", "oracle_token"), "rb").read().strip()
    want = open(os.path.join(hd, "oracle_token.sha256")).read().strip()
    out.append(("oracle token sha256", hashlib.sha256(tok).hexdigest() == want, ""))
    a = json.load(open(os.path.join(TESTS, "anchor.json")))
    out.append(("anchor.json", a.get("metric") == "block_score" and a.get("direction") == "higher" and
                isinstance(a.get("value"), (int, float)) and a["value"] > 0 and 0 < a.get("full_at", 0) <= 1,
                "value %s full_at %s" % (a.get("value"), a.get("full_at"))))
    pay = os.path.join(TASK, "environment", "setup", "payload")
    svc = os.path.join(TASK, "environment", "commission_service")
    ok = same_tree(pay, os.path.join(TESTS, "setup_payload"))
    ok = ok and filecmp.cmp(os.path.join(TESTS, "towercell", "client.py"),
                            os.path.join(pay, "given", "lib", "towercell", "client.py"), shallow=False)
    ok = ok and same_tree(os.path.join(TESTS, "towercell"), os.path.join(svc, "towercell"))
    ok = ok and filecmp.cmp(os.path.join(TESTS, "cellkit", "wire.py"), os.path.join(svc, "cellkit", "wire.py"),
                            shallow=False)
    gh = json.load(open(os.path.join(TESTS, "given_hashes.json")))["files"]
    for root, dirs, fns in os.walk(pay):
        dirs[:] = [d for d in dirs if d != "__pycache__"]
        for fn in fns:
            p = os.path.join(root, fn)
            ok = ok and gh.get(os.path.relpath(p, pay)) == sha(p)
    out.append(("byte copies and given hashes", ok, ""))
    want_fp = json.load(open(os.path.join(TESTS, "numerics.json")))["fingerprint"]
    try:
        r = subprocess.run([sys.executable, "-I", "-B", os.path.join(TESTS, "numerics_check.py")], cwd=TESTS,
                           capture_output=True, text=True, timeout=600, env=dict(os.environ))
        got = json.loads(r.stdout.strip().splitlines()[-1])["fingerprint"]
    except Exception as e:  # noqa: BLE001
        got = "error: %s" % e
    out.append(("numerics fingerprint", got == want_fp, "got %s" % got[:16]))
    if run_oracle:
        out.append(oracle_reproduces(pins, a))
    return out


def oracle_reproduces(pins, anchor):
    """The reference program in-process (no wire, the same cell code and calls) on every hidden tower."""
    import multiprocessing as mp
    with mp.get_context("fork").Pool(len(pins)) as pool:
        scores = pool.map(_one, [os.path.join(TESTS, "hidden", e["file"]) for e in pins])
    mean = sum(scores) / len(scores)
    return ("oracle reproduces the anchor", abs(mean - anchor["value"]) < 1e-9,
            "per tower %s, mean %.6f, anchor %s" % ([round(s, 6) for s in scores], mean, anchor["value"]))


def _one(path):
    import importlib
    from towercell.client import Cell, CellError
    from towercell.server import CellServer
    from towercell.tower import draw_tower
    pk = json.load(open(path))
    srv = CellServer(draw_tower(int(pk["seed"]), pk.get("overrides")), duration_s=float(pk.get("duration_s", 2400.0)),
                     privileged=True)

    class Proxy(Cell):
        def __init__(self):
            self._id = 0

        def _call(self, method, **params):
            if srv.done and method not in ("get_state", "finish"):
                raise CellError("shift_over", "over")
            r = srv.handle(method, params)
            if isinstance(r, dict) and set(r) <= {"error", "detail"} and "error" in r:
                raise CellError(r["error"], r.get("detail"))
            return r

    sys.path.insert(0, os.path.join(TASK, "solution", "reference"))
    mod = importlib.import_module("play")
    try:
        mod.solve(Proxy())
    except CellError:
        pass
    srv._end(srv.done_reason or "finished")
    return float(srv.summary()["tower_score"])


def main():
    allok = True
    for name, ok, detail in checks("--oracle" in sys.argv):
        allok &= bool(ok)
        print("%s %s %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
