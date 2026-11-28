"""dexterous_hand_grip_controller_from_docs: what a controller's native code writes to file descriptor 1 never reaches
the wire.

The wire between the controller and the grader is the controller process's original stdout (one JSON reply per line)
and stdin (the requests); tests/grip/grader/runtime/wire.py ends the controller for the rest of the unit on the first
reply line that is not a JSON object (a protocol error). The given interface promises "Anything a controller prints
goes to a log, not to the bus". Python's print is rebound to stderr by the client; native code (a C library's printf,
os.write(1, ...), MuJoCo's default warning handler) writes to fd 1 itself, so the client must take the wire as a
duplicate of fd 1 and point fd 1 at stderr before the import (os.dup2(2, 1)), as the hand assembly task's launcher does.

The checks (stdlib only, nothing downloaded): the shipped client (tests/grip/grader/runtime/client.py) is started the
way wire.Runner starts it (python -B -E -s -P -c BOOT <lib> <home>, its own session, stdin / stdout pipes, stderr to a
file) on a controller that writes to fd 1 with os.write at import, in setup and in step and also prints; the wire side
must receive exactly the JSON lines (the filter's line, which the client writes before the import, then ready, setup,
step, end) and the log the native lines. The given self-test's
child side (environment/setup/payload/given/controller/check_controller.py child_main) must redirect fd 1 as well, so
that the agent's self-test and the acceptance treat such a controller alike.
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)
CLIENT = TASK / "tests" / "grip" / "grader" / "runtime" / "client.py"
WIRE = TASK / "tests" / "grip" / "grader" / "runtime" / "wire.py"
SELFTEST = TASK / "environment" / "setup" / "payload" / "given" / "controller" / "check_controller.py"

CONTROLLER = r'''
import os
os.write(1, b"NATIVE-AT-IMPORT\n")
print("python print at import")


class Controller:
    def __init__(self, workdir):
        os.write(1, b"NATIVE-IN-INIT\n")

    def setup(self, record, cell):
        os.write(1, b"NATIVE-IN-SETUP\n")
        return {"period_ms": 10, "reads": [[132, 4]]}

    def step(self, obs):
        os.write(1, b"NATIVE-IN-STEP\n")
        return {"write": {}, "tactile_zero": [], "grasped": False, "decline": False}
'''


def _boot():
    tree = ast.parse(WIRE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "BOOT" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("wire.py defines no BOOT")


class NativeWritesStayOffTheWire(unittest.TestCase):
    def test_native_writes_to_fd1_stay_off_the_wire(self):
        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        lib, home, program = tmp / "lib", tmp / "home", tmp / "program"
        for d in (lib, home, program):
            d.mkdir()
        shutil.copyfile(CLIENT, lib / "mc_client.py")
        (program / "controller.py").write_text(CONTROLLER, encoding="utf-8")
        err_path = tmp / "stderr.log"
        with open(err_path, "wb") as err:
            p = subprocess.Popen([sys.executable, "-B", "-E", "-s", "-P", "-c", _boot(), str(lib), str(home)], cwd=program,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, start_new_session=True,
                                 env={"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "HOME": str(home),
                                      "LANG": "C.UTF-8"})
            reqs = [{"op": "setup", "record": {}, "cell": {}}, {"op": "step", "obs": {}}, {"op": "end"}]
            out, _ = p.communicate("".join(json.dumps(r) + "\n" for r in reqs).encode(), timeout=60)
        lines = [ln for ln in out.decode("utf-8", "replace").splitlines() if ln]
        bad = [ln for ln in lines if not ln.startswith("{")]
        self.assertEqual(bad, [], "native writes reached the wire: %r" % bad)
        self.assertEqual(len(lines), 5, lines)        # the filter's line, ready, setup, step, end
        self.assertIn('"filter"', lines[0])
        log = err_path.read_text(errors="replace")
        for tag in ("NATIVE-AT-IMPORT", "NATIVE-IN-INIT", "NATIVE-IN-SETUP", "NATIVE-IN-STEP", "python print"):
            self.assertIn(tag, log)

    def test_the_client_redirects_fd1_before_the_import(self):
        src = CLIENT.read_text(encoding="utf-8")
        self.assertRegex(src, r"os\.dup2\(\s*2\s*,\s*1\s*\)|os\.dup2\(\s*sys\.stderr\.fileno\(\)\s*,\s*1\s*\)",
                         "client.py rebinds sys.stdout only; fd 1 still is the wire")

    def test_the_given_self_test_redirects_fd1_too(self):
        src = SELFTEST.read_text(encoding="utf-8")
        tree = ast.parse(src)
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "child_main")
        self.assertRegex(ast.get_source_segment(src, fn), r"os\.dup2\(",
                         "the given self-test's child side rebinds sys.stdout only")


if __name__ == "__main__":
    unittest.main()
