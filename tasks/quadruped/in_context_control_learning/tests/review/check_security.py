"""Fast stdlib-only audit/replay regressions. Run with python3 -B tests/review/check_security.py."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
from audit import audit_trajectory  # noqa: E402


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Import the real dispatcher and grader without importing MuJoCo or constructing a world.
world_module = types.ModuleType("world")
world_module.KIND, world_module.VERSION = "icl", 5
world_module.World = None
with mock.patch.dict(sys.modules, {"world": world_module}):
    service = load_module("security_service", STAGE/"service.py")
    with mock.patch.dict(sys.modules, {"service": service}):
        grader = load_module("security_grader", STAGE/"grader.py")


def call(name, arguments=None, mcp=None):
    result = {"function_name": name, "arguments": {} if arguments is None else arguments}
    if mcp is not None:
        result["extra"] = {"ale": {"mcp": mcp}}
    return result


def trajectory(*calls):
    return {"schema_version": "ATIF-v1.8", "agent": {"name": "claude-code"},
            "steps": [{"source": "agent", "tool_calls": list(calls)}]}


class AuditTests(unittest.TestCase):
    def setUp(self):
        self.params = {"target_rad": [0, .9, -1.8]*4, "ticks": 10}
        self.events = [{"method": "observe", "params": {}},
                       {"method": "act", "params": self.params}]
        self.robot = [call("mcp__robot__robot_observe"), call("mcp__robot__robot_act", self.params)]

    def test_legitimate_read_only_and_discovery_tools(self):
        names = ("Read", "Glob", "Grep", "LS", "TodoWrite", "TodoRead", "TaskCreate", "TaskUpdate",
                 "TaskList", "TaskGet", "TaskOutput", "TaskStop", "ToolSearch", "ListMcpResourcesTool",
                 "ReadMcpResourceTool", "EnterPlanMode", "ExitPlanMode", "AskUserQuestion")
        codex = ("update_plan", "view_image", "request_user_input", "tool_search_tool", "list_mcp_resources",
                 "list_mcp_resource_templates", "read_mcp_resource")
        for name in names + codex + tuple("functions." + n for n in codex):
            with self.subTest(tool=name):
                passed, reason, stats = audit_trajectory(trajectory(call(name), *self.robot), self.events)
                self.assertTrue(passed, reason)
                self.assertEqual(stats["harmless_calls"], 1)

    def test_unknown_and_executable_tools_fail_closed(self):
        for name in ("mcp__node_repl__js", "mcp__python__run", "mcp__calculator__evaluate", "custom_tool",
                     "js", "functions.exec", "exec_command", "Bash", "Write", "Edit", "Agent", "Task",
                     "apply_patch", "web.run", "ScheduleWakeup", ""):
            with self.subTest(tool=name):
                passed, reason, _ = audit_trajectory(trajectory(*self.robot, call(name)), self.events)
                self.assertFalse(passed)
                self.assertTrue(reason)

    def test_other_mcp_servers_cannot_borrow_safe_names(self):
        for tool in ("Read", "ToolSearch", "list_mcp_resources", "robot_act"):
            for c in (call("mcp__other__" + tool),
                      call(tool, mcp={"server": "other", "tool": tool})):
                with self.subTest(call=c):
                    self.assertFalse(audit_trajectory(trajectory(c, *self.robot), self.events)[0])

    def test_engine_mcp_metadata_and_string_arguments(self):
        calls = [call("sanitized_observe", mcp={"server": "robot", "tool": "robot_observe"}),
                 call("sanitized_act", json.dumps(self.params),
                      mcp={"server": "robot", "tool": "robot_act"})]
        self.assertTrue(audit_trajectory(trajectory(*calls), self.events)[0])

    def test_ledger_is_an_ordered_subsequence(self):
        rejected = call("mcp__robot__robot_act", {"action": "invalid"})
        self.assertTrue(audit_trajectory(trajectory(rejected, *self.robot), self.events)[0])
        self.assertFalse(audit_trajectory(trajectory(*reversed(self.robot)), self.events)[0])
        self.assertFalse(audit_trajectory(trajectory(*self.robot), self.events + [self.events[1]])[0])

    def test_ticks_joint_vectors_and_recovery_are_bound(self):
        for key, changed in (("ticks", 11), ("target_rad", [False, .9, -1.8]*4),
                             ("target_rad", [0, .91, -1.8]*4)):
            calls = [self.robot[0], call("mcp__robot__robot_act", {**self.params, key: changed})]
            self.assertFalse(audit_trajectory(trajectory(*calls), self.events)[0])
        self.assertTrue(audit_trajectory(trajectory(call("mcp__robot__robot_recover")),
                                         [{"method": "recover", "params": {}}])[0])


class FakeWorld:
    def __init__(self):
        self.steps = self.delivered = self.collisions = 0
        self.memory = 0.
        self.closed = False

    def act(self, target_rad, ticks):
        self.steps += 1
        self.memory = float(target_rad[0])

    def observe(self):
        return {}

    def advance(self):
        raise ValueError("not reached")

    def state_fingerprint(self):
        return {"qpos": [0., 0.], "qvel": [0., 0.], "actuator": [self.memory]}

    def metrics(self):
        return {"steps": self.steps, "delivered": self.delivered, "collisions": self.collisions}

    def close(self):
        self.closed = True


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.world = FakeWorld()
        self.state = {"seed": 1, "scene_index": 0, "events": [
            {"method": "act", "params": {"target_rad": [1], "ticks": 1}, "steps": 1, "delivered": 0},
            {"method": "observe", "params": {}, "steps": 1, "delivered": 0},
            {"method": "advance", "params": {}, "error": "not reached"}],
            "state_fingerprint": {"qpos": [0., 0.], "qvel": [0., 0.], "actuator": [1.]}}
        for event in self.state["events"]:
            event["state_fingerprint"] = copy.deepcopy(self.state["state_fingerprint"])

    def replay(self, state):
        with mock.patch.object(grader, "World", return_value=self.world) as factory:
            try:
                return grader.replay(state)
            finally:
                factory.assert_called_once_with(1, render=False, scene_index=0)
                self.assertTrue(self.world.closed)

    def assert_task_error(self, state, message):
        log = io.StringIO()
        with contextlib.redirect_stderr(log), self.assertRaises(SystemExit) as exc:
            self.replay(state)
        self.assertEqual(exc.exception.code, 1)
        self.assertIn("environment failure", log.getvalue())
        self.assertIn(message, log.getvalue())

    def test_matching_fingerprint_with_observation_and_error(self):
        self.assertEqual(self.replay(self.state), {"steps": 1, "delivered": 0, "collisions": 0})

    def test_matching_event_fingerprints_including_error(self):
        for event in self.state["events"]:
            event["state_fingerprint"] = copy.deepcopy(self.state["state_fingerprint"])
        self.assertEqual(self.replay(self.state)["steps"], 1)

    def test_early_drift_fails_even_when_final_state_would_match(self):
        initial = copy.deepcopy(self.state["state_fingerprint"])
        initial["actuator"] = [.5]
        self.state["events"][0]["state_fingerprint"] = initial
        self.state["events"].append({"method": "act", "params": {"target_rad": [0], "ticks": 1},
                                     "steps": 2, "delivered": 0})
        self.state["state_fingerprint"]["actuator"] = [0.]
        self.assert_task_error(self.state, "request 1 (state fingerprint)")
        self.assertEqual(self.world.steps, 1)

    def test_error_event_fingerprint_is_checked(self):
        self.state["events"][2]["state_fingerprint"] = "diverged"
        self.assert_task_error(self.state, "request 3 (state fingerprint)")

    def test_null_event_fingerprint_fails(self):
        self.state["events"][0]["state_fingerprint"] = None
        self.assert_task_error(self.state, "request 1 (state fingerprint)")

    def test_v5_event_fingerprint_cannot_be_omitted(self):
        del self.state["events"][0]["state_fingerprint"]
        self.assert_task_error(self.state, "request 1 (state fingerprint)")

    def test_actuator_drift_fails_even_when_all_counters_match(self):
        state = copy.deepcopy(self.state)
        state["state_fingerprint"]["actuator"] = [0.]
        self.assert_task_error(state, "fingerprint differs")

    def test_missing_fingerprint_is_not_silently_accepted(self):
        del self.state["state_fingerprint"]
        self.assert_task_error(self.state, "fingerprint is missing")

    def test_null_fingerprint_is_not_silently_accepted(self):
        self.state["state_fingerprint"] = None
        self.assert_task_error(self.state, "fingerprint is missing")

    def test_ledger_fingerprint_requires_world_support(self):
        self.world.state_fingerprint = None
        self.assert_task_error(self.state, "state fingerprint")

    def test_legacy_world_without_fingerprints(self):
        self.world.state_fingerprint = None
        del self.state["state_fingerprint"]
        for event in self.state["events"]:
            del event["state_fingerprint"]
        with mock.patch.object(grader, "VERSION", 2):
            self.assertEqual(self.replay(self.state)["steps"], 1)

    def test_new_version_cannot_opt_out_of_fingerprints(self):
        self.world.state_fingerprint = None
        del self.state["state_fingerprint"]
        self.assert_task_error(self.state, "state fingerprint")

    def test_existing_counter_check_still_fails_as_task_error(self):
        self.state["events"][0]["steps"] = 2
        self.assert_task_error(self.state, "poses, deliveries")


if __name__ == "__main__":
    unittest.main(verbosity=2)
