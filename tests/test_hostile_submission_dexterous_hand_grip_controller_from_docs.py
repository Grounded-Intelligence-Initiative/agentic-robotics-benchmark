"""dexterous_hand_grip_controller_from_docs: a submission cannot end the verify without a verdict.

Every path of the root grader that reads what the agent wrote must fail that item closed, never raise past the
verdict and never skip silently:
  * the static import audit (grader/runtime/netaudit.py scan_imports): a .py the parser cannot handle (100,000 chained
    unary signs: RecursionError or MemoryError in Python 3.12's parser, not SyntaxError) is a hit, not a crash and not
    a skip; so is a file whose declared encoding Python would use to import it but the audit does not decode;
  * the grader's own process parses no source: verify_main.static_checks runs the parse of controller.py and the import
    audit in a child process under its own limits (netaudit.audit_in_child), whose result or failure is a verdict;
  * the oracle token is lstat-checked and size-capped before it is read (token_state);
  * a wire reply line the parser cannot read (nested 100,000 deep, 200 KB, under the 1 MiB line cap) is the
    controller's protocol error, not an exception of the unit worker (wire.Runner.call catches RecursionError);
  * the controller's processes are the memory killer's first choice (oom_score_adj 1000 by choom inside runuser), not
    0 as the grader's own processes.
Stdlib only; the audit runs in a child interpreter with the task's tests/grip first on its path (it imports the task's
own contract package, which this test process must not), on a temporary folder.
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
GRIP = TASK / "tests" / "grip"
GRADER = GRIP / "grader"
# the audit in a child interpreter: tests/grip first on its path, then one call; its result on the last line as JSON
PROBE = ("import json, sys; sys.path.insert(0, sys.argv[1]); from grader.runtime import netaudit as n; "
         "print(json.dumps(getattr(n, sys.argv[2])(*json.loads(sys.argv[3]))))")


def _audit_call(name, *args):
    p = subprocess.run([sys.executable, "-I", "-B", "-c", PROBE, str(GRIP), name, json.dumps(list(args))],
                       capture_output=True, text=True, timeout=600)
    if p.returncode != 0:
        raise AssertionError("the audit's child failed: %s" % p.stderr[-2000:])
    return json.loads(p.stdout.strip().splitlines()[-1])


def _caught(path, handler):
    """The exception names an except clause catches: a literal tuple, one name, or a module-level tuple by its name."""
    t = handler.type
    if isinstance(t, ast.Tuple):
        return {ast.unparse(e) for e in t.elts}
    if isinstance(t, ast.Name):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for n in tree.body:
            if isinstance(n, ast.Assign) and any(isinstance(x, ast.Name) and x.id == t.id for x in n.targets) \
                    and isinstance(n.value, ast.Tuple):
                return {ast.unparse(e) for e in n.value.elts}
        return {t.id}
    return {ast.unparse(t)} if t is not None else {"BaseException"}


def _src(path, name, cls=None):
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text)
    nodes = tree.body
    if cls:
        nodes = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls).body
    fn = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)
    return fn, ast.get_source_segment(text, fn)


class ImportAudit(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))

    def test_a_deep_expression_is_a_hit_not_a_crash(self):
        (self.tmp / "helper.py").write_text("x = " + "-" * 100000 + "1\n", encoding="utf-8")
        hits = _audit_call("scan_imports", str(self.tmp))       # a RecursionError / MemoryError fails the child
        self.assertTrue(hits, "an unparsable helper was skipped by the audit")

    def test_a_syntax_error_is_a_hit_not_a_skip(self):
        (self.tmp / "helper.py").write_text("def (:\n", encoding="utf-8")
        self.assertTrue(_audit_call("scan_imports", str(self.tmp)), "a file that does not parse was skipped")

    def test_a_utf7_declaration_cannot_hide_an_import(self):
        body = "import socket\n".encode("utf-7")
        (self.tmp / "helper.py").write_bytes(b"# -*- coding: utf-7 -*-\n" + body.replace(b"\n", b"+AAo-"))
        self.assertTrue(_audit_call("scan_imports", str(self.tmp)), "a utf-7 file passed the audit unread")


class GraderPaths(unittest.TestCase):
    def test_the_grader_process_parses_no_source(self):
        fn, seg = _src(GRADER / "verify_main.py", "static_checks")
        calls = {ast.unparse(n.func) for n in ast.walk(fn) if isinstance(n, ast.Call)}
        self.assertFalse(calls & {"ast.parse", "compile", "netaudit.scan_imports", "scan_imports"}, calls)
        self.assertIn("netaudit.audit_in_child", calls, "static_checks does not run the checks in a child")
        tree = ast.parse((GRADER / "verify_main.py").read_text(encoding="utf-8"))
        every = {ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
        self.assertFalse(every & {"ast.parse", "compile"}, "verify_main parses source in the grader's process")

    def test_the_child_audit_reports_an_unparsable_file(self):
        with tempfile.TemporaryDirectory() as td:
            Path(td, "controller.py").write_text("class Controller:\n    pass\n", encoding="utf-8")
            Path(td, "helper.py").write_text("x = " + "-" * 100000 + "1\n", encoding="utf-8")
            res, why = _audit_call("audit_in_child", td, 4 << 20, 60, 120)
        self.assertIsNone(why)
        self.assertTrue(res["controller_file"])
        self.assertEqual(res["n_hits"], 1, res)

    def test_the_token_is_size_capped_before_it_is_read(self):
        _fn, seg = _src(GRADER / "verify_main.py", "token_state")
        self.assertRegex(seg, r"st_size|getsize", "token_state reads the token file whole, with no size cap")

    def test_a_wire_line_the_parser_cannot_read_is_a_protocol_error(self):
        path = GRADER / "runtime" / "wire.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        runner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Runner")
        handlers = [h for t in ast.walk(runner) if isinstance(t, ast.Try)
                    and any("json.loads" in ast.unparse(s) for s in t.body) for h in t.handlers]
        self.assertGreaterEqual(len(handlers), 2, "the runner parses the start lines and the replies")
        for h in handlers:
            self.assertLessEqual({"ValueError", "RecursionError", "MemoryError"}, _caught(path, h),
                                 "a line json cannot parse must be the controller's protocol error")
        loads = [n for n in ast.walk(runner) if isinstance(n, ast.Call) and ast.unparse(n.func) == "json.loads"]
        self.assertEqual(len(loads), len(handlers), "every json.loads of the runner sits in such a try")

    def test_the_controller_is_the_memory_killers_first_choice(self):
        src = (GRADER / "runtime" / "wire.py").read_text(encoding="utf-8")
        _fn, seg = _src(GRADER / "runtime" / "wire.py", "start", cls="Runner")
        self.assertNotIn("echo 0 > /proc/self/oom_score_adj", src)
        self.assertIn("oom_first(", seg, "Runner.start does not start the controller through oom_first")
        fn, oseg = _src(GRADER / "runtime" / "wire.py", "oom_first")
        self.assertIn("CHOOM", oseg)
        self.assertRegex(src, r'OOM_ADJ = 1000')
        self.assertRegex(src, r'shutil\.which\("choom"\)')


if __name__ == "__main__":
    unittest.main()
