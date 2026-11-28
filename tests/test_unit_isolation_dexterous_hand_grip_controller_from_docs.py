"""dexterous_hand_grip_controller_from_docs: the two units' controllers never see, charge or kill each other, and nothing
the first unit's controller wrote reaches the second's.

Both units run under ONE runner account (`nobody`), ONE AFTER THE OTHER: grader.acceptance.run_workers starts a unit's
worker (its plant and, through grader.runtime.wire.Runner, its controller) only after the previous unit's worker has
ended and the clean-up after it (grader.acceptance.UnitIsolation.after) is done. As root (the real verifier) a Runner's
view of "its" processes is every process of the runner account (CPU charge, the stray count of the pass condition "no
process left", the kills); with one controller alive at a time that view is its own. The clean-up kills every process
of the account, removes the unit's run folder and everything the account owns under the world-writable places of the
image (grader.acceptance.RUNNER_WRITABLE_DIRS, at least /tmp, /var/tmp and /dev/shm) and its System V IPC objects.

The checks (stdlib only; root-only behaviour is read from the code, the clean-up's file part is run on folders of this
test's own in a child interpreter): run_workers waits for each worker and cleans up after it inside the loop that
starts it; verify_main gives run_workers the clean-up; the list of writable places holds the floor; the clean-up
removes a planted file and folder from each place and the run folder.
"""
import ast
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)
GRADER = TASK / "tests" / "grip" / "grader"


def _fn(path, name, cls=None):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    nodes = tree.body
    if cls:
        nodes = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls).body
    return next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)


class UnitIsolation(unittest.TestCase):
    def test_the_units_run_one_after_the_other(self):
        fn = _fn(GRADER / "acceptance.py", "run_workers")
        loops = [n for n in fn.body if isinstance(n, ast.For)]
        self.assertEqual(len(loops), 1, "run_workers has one loop over the units")
        body = ast.unparse(loops[0])
        self.assertIn("subprocess.Popen", body)
        start, wait = body.index("subprocess.Popen"), body.index(".wait(")
        self.assertLess(start, wait, "the worker started in the loop is waited for in the same pass")
        self.assertIn("isolation.after", body[wait:], "the clean-up after the unit runs before the next unit starts")
        self.assertIn("isolation.before", body[:start], "before a unit, no process of the runner account is alive")

    def test_verify_gives_the_clean_up_to_the_units(self):
        main = _fn(GRADER / "verify_main.py", "_main_locked")
        calls = [n for n in ast.walk(main) if isinstance(n, ast.Call) and ast.unparse(n.func).endswith("run_workers")]
        self.assertEqual(len(calls), 1)
        kw = {k.arg: ast.unparse(k.value) for k in calls[0].keywords}
        self.assertIn("UnitIsolation", kw.get("isolation", ""), kw)

    def test_the_writable_places_hold_the_floor(self):
        tree = ast.parse((GRADER / "acceptance.py").read_text(encoding="utf-8"))
        val = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == "RUNNER_WRITABLE_DIRS" for t in n.targets))
        self.assertLessEqual({"/tmp", "/var/tmp", "/dev/shm"}, set(val))

    def test_the_clean_up_removes_what_the_first_unit_left(self):
        with tempfile.TemporaryDirectory() as td:
            places = [os.path.join(td, "w%d" % i) for i in (1, 2, 3)]
            run = os.path.join(td, "run")
            for d in places + [os.path.join(run, "home")]:
                os.makedirs(d)
            for d in places:
                Path(d, "left_by_unit1").write_text("x")
                os.makedirs(os.path.join(d, "dir_by_unit1", "inner"))
            code = ("import importlib.util, json, os, sys\n"
                    "spec = importlib.util.spec_from_file_location('grip_acceptance_probe', sys.argv[1])\n"
                    "A = importlib.util.module_from_spec(spec); spec.loader.exec_module(A)\n"
                    "iso = A.UnitIsolation('nobody', writable_dirs=json.loads(sys.argv[2]), sweep_uid=os.getuid())\n"
                    "print(json.dumps(iso.after(1, {'controller_root': sys.argv[3]})))\n")
            p = subprocess.run([sys.executable, "-I", "-B", "-c", code, str(GRADER / "acceptance.py"),
                                json.dumps(places), run], capture_output=True, text=True, timeout=120)
            self.assertEqual(p.returncode, 0, p.stderr[-2000:])
            out = json.loads(p.stdout.strip().splitlines()[-1])
            self.assertEqual([os.listdir(d) for d in places], [[], [], []])
            self.assertFalse(os.path.exists(run))
            self.assertEqual((out["files_removed"], out["run_folder_removed"]), (6, True), out)


if __name__ == "__main__":
    unittest.main()
