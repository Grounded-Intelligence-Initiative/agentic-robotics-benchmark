"""Stdlib-only oracle-capability regressions; no simulator, subprocesses or root required.

Run with python3 -B tests/review/check_security.py. Author-review oracle runs must explicitly
call run_mission(..., oracle_authorized=True); ordinary submissions must use the default.
"""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

STAGE = Path(__file__).resolve().parents[1]
TASK = STAGE.parent
sys.path.insert(0, str(STAGE))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


grader = load_module("security_grader", STAGE/"grader.py")
# Exercise the actual service authorization without importing MuJoCo or its world.
world_module = types.ModuleType("world")
world_module.World = None
with mock.patch.dict(sys.modules, {"world": world_module}):
    service = load_module("security_service", STAGE/"runtime"/"service.py")


class TrajectoryTests(unittest.TestCase):
    def authorized(self, payload):
        with mock.patch.dict(os.environ, {"ALE_TRAJECTORY_PATH": "/engine/trajectory.json"}, clear=True), \
                mock.patch.object(Path, "read_text", return_value=json.dumps(payload)):
            return grader.oracle_authorized_from_trajectory()

    def test_engine_oracle_atif_authorizes(self):
        self.assertTrue(self.authorized({"schema_version": "ATIF-v1.8", "agent": {"name": "oracle"}}))

    def test_participant_and_malformed_identity_do_not_authorize(self):
        for payload in (None, [], {}, {"agent": {"name": "oracle"}},
                        {"schema_version": "native", "agent": {"name": "oracle"}},
                        {"schema_version": "ATIF-v1.8", "agent": "oracle"},
                        {"schema_version": "ATIF-v1.8", "agent": {"name": "claude-code", "model_name": "oracle"}},
                        {"schema_version": "ATIF-v1.8", "agent": {"name": "codex"},
                         "steps": [{"message": "agent.name=oracle"}]}):
            with self.subTest(payload=payload):
                self.assertFalse(self.authorized(payload))

    def test_missing_or_unreadable_trajectory_denies(self):
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(Path, "read_text") as read:
            self.assertFalse(grader.oracle_authorized_from_trajectory())
            read.assert_not_called()
        for error in (OSError("missing"), ValueError("invalid JSON")):
            with mock.patch.dict(os.environ, {"ALE_TRAJECTORY_PATH": "/engine/trajectory.json"}, clear=True), \
                    mock.patch.object(Path, "read_text", side_effect=error):
                self.assertFalse(grader.oracle_authorized_from_trajectory())


class CapabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Deliberately use the historically committed credential, without printing it.
        cls.token = (TASK/"solution"/"oracle"/"token.txt").read_text().strip()
        cls.token_hash = hashlib.sha256(cls.token.encode()).hexdigest()

    def service_argv(self, **authorization):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            proc = mock.Mock()
            proc.poll.return_value = 1  # stop immediately after inspecting the service launch
            with mock.patch.multiple(grader, WORK=root/"work", SOCKET=root/"run"/"world.sock"), \
                    mock.patch.object(grader.os, "chown"), \
                    mock.patch.object(grader.subprocess, "Popen", return_value=proc) as popen:
                result = grader.run_mission(0, 1, os.getuid(), os.getgid(), {}, self.token_hash,
                                            attempt=0, **authorization)
                self.assertTrue(result["infra"])
                popen.assert_called_once()
                return popen.call_args.args[0]

    def dispatch_truth(self, argv, token):
        runtime = service.Runtime.__new__(service.Runtime)
        runtime.mode = "graded"
        runtime.token_sha256 = (argv[argv.index("--token-sha256")+1]
                                if "--token-sha256" in argv else None)
        runtime.world = mock.Mock()
        runtime.world.truth.return_value = {"privileged": True}
        return runtime, lambda: runtime.dispatch("truth", {"token": token})

    def test_historical_token_cannot_unlock_default_mission(self):
        argv = self.service_argv()
        self.assertNotIn("--token-sha256", argv)
        self.assertNotIn(self.token_hash, argv)
        runtime, invoke = self.dispatch_truth(argv, self.token)
        with self.assertRaisesRegex(ValueError, "forbidden"):
            invoke()
        runtime.world.truth.assert_not_called()
        runtime.dispatch("observe", {})
        runtime.world.observe.assert_called_once_with()

    def test_explicit_authorization_still_requires_correct_token(self):
        argv = self.service_argv(oracle_authorized=True)
        runtime, invoke = self.dispatch_truth(argv, self.token)
        self.assertEqual(invoke(), {"privileged": True})
        runtime.world.truth.assert_called_once_with()
        runtime, invoke = self.dispatch_truth(argv, "incorrect-token")
        with self.assertRaisesRegex(ValueError, "forbidden"):
            invoke()
        runtime.world.truth.assert_not_called()

    def test_truthy_non_boolean_does_not_authorize(self):
        for value in (False, None, "true", 1):
            with self.subTest(value=value):
                self.assertNotIn("--token-sha256", self.service_argv(oracle_authorized=value))

    def test_main_passes_engine_authorization_to_both_attempts(self):
        for name in ("oracle", "claude-code", "codex"):
            with self.subTest(agent=name), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                hidden, stage = root/"hidden", root/"stage"
                hidden.mkdir()
                stage.mkdir()
                (hidden/"sites.json").write_text(json.dumps({"seeds": [1]}))
                if name == "oracle":
                    (hidden/"oracle_token.sha256").write_text(self.token_hash)
                trajectory = root/"trajectory.json"
                trajectory.write_text(json.dumps({"schema_version": "ATIF-v1.8", "agent": {"name": name}}))
                record = {"site": 0, "infra": False, "success": 1., "switched": 1., "scout_actions": 12,
                          "carrier_actions": 20, "contact_stop": 0., "program_seconds": 1.,
                          "wall_seconds": 1., "reason": "mission ended", "cleanup_ok": True}
                with contextlib.ExitStack() as stack:
                    stack.enter_context(mock.patch.dict(os.environ, {"ALE_TRAJECTORY_PATH": str(trajectory)},
                                                        clear=True))
                    stack.enter_context(mock.patch.multiple(grader, HIDDEN=hidden, WORK=root/"work", RUN=root/"run"))
                    for attr, value in (("stage_config", {}), ("read_anchor", (1., {"metric": "success", "full_at": .98})),
                                        ("agent_user", "user"), ("secure_exec_path", None),
                                        ("stop_agent_processes", True), ("copy_submission", None),
                                        ("stage_dir", str(stage)), ("stage_public_client", None),
                                        ("selftest", (True, "ok"))):
                        stack.enter_context(mock.patch.object(grader, attr, return_value=value))
                    stack.enter_context(mock.patch.object(grader.pwd, "getpwnam",
                                                          return_value=types.SimpleNamespace(
                                                              pw_uid=1000, pw_gid=1000, pw_dir=str(root/"home"))))
                    mission = stack.enter_context(mock.patch.object(grader, "run_mission", side_effect=[
                        {"infra": True, "reason": "injected service failure"}, record]))
                    verdict = stack.enter_context(mock.patch.object(grader, "write_verdict"))
                    stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
                    grader.main()
                    self.assertEqual(mission.call_count, 2)
                    for attempt, invocation in enumerate(mission.call_args_list):
                        self.assertIs(invocation.kwargs["oracle_authorized"], name == "oracle")
                        self.assertEqual(invocation.kwargs["attempt"], attempt)
                        self.assertEqual(invocation.args[5], self.token_hash if name == "oracle" else None)
                    self.assertEqual(verdict.call_args.args[0]["metrics"]["scripted_oracle"], float(name == "oracle"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
