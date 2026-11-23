"""V5 native joint, camera, scoring, recovery and ledger regressions."""
import base64
import contextlib
import io
import json
import math
import os
from pathlib import Path
import socketserver
import sys
import tempfile
import threading
import unittest
from unittest import mock

import mujoco
import numpy as np
from PIL import Image

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"environment/setup/payload"), str(TASK/"solution")]
from grader import replay
import grader
import mcp_stdio
from client import Client
from lane import JOINTS
from mcp_stdio import METHODS, TOOLS
from reference import pose
from service import ALLOWED, Handler, Runtime, dispatch
from world import World

HOME = [0., .9, -1.8]*4


class DirectBridgeTests(unittest.TestCase):
    def test_native_image_and_direct_partial_credit(self):
        seeds = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runtime = Runtime(root/"ledger", seeds[0], 0)
            self.addCleanup(runtime.world.close)
            ops = [("observe", {}), ("act", {"target_rad": [0., 1.09, -2.18]*4, "ticks": 450}),
                   ("recover", {}), ("advance", {})]
            wire = [{"jsonrpc": "2.0", "id": i, "method": "tools/call",
                     "params": {"name": "robot_"+op, "arguments": p}} for i, (op, p) in enumerate(ops)]
            path = str(root/"world.sock")
            with socketserver.UnixStreamServer(path, Handler) as server:
                server.runtime = runtime
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                try:
                    output = io.StringIO()
                    with mock.patch.object(mcp_stdio, "Client", lambda: Client(path)), \
                            mock.patch.object(sys, "stdin", io.StringIO("\n".join(map(json.dumps, wire)))), \
                            contextlib.redirect_stdout(output):
                        mcp_stdio.main()
                finally:
                    server.shutdown()
                    thread.join()
            replies = [json.loads(row)["result"]["content"] for row in output.getvalue().splitlines()]
            self.assertEqual([len(c) for c in replies], [2, 1, 1, 1])
            self.assertEqual(replies[0][1]["mimeType"], "image/png")
            self.assertEqual(json.loads(replies[1][0]["text"])["calibration_checks"], 1)
            trajectory = {"agent": {"name": "fixture"}, "schema_version": "ATIF-v1.8", "steps": [
                {"source": "agent", "tool_calls": [{"function_name": "mcp__robot__robot_"+op,
                                                     "arguments": p} for op, p in ops]}]}
            traj = root/"trajectory.json"
            traj.write_text(json.dumps(trajectory))
            with mock.patch.object(grader, "stage_dir", return_value=TASK/"tests"), \
                    mock.patch.object(grader, "LEDGER", root/"ledger/state.json"), \
                    mock.patch.dict(os.environ, {"ALE_TRAJECTORY_PATH": str(traj)}), \
                    mock.patch.object(grader, "read_anchor", return_value=(1., {"metric": "milestone_fraction",
                                                                                "full_at": 1.})), \
                    mock.patch.object(grader, "write_verdict") as write:
                grader.main()
                verdict = write.call_args.args[0]
                self.assertAlmostEqual(verdict["rewards"]["reward"], 1/14)
                self.assertEqual(verdict["metrics"]["ledger_events_matched"], len(ops))
                self.assertEqual(verdict["metrics"]["direct_control_audit_passed"], 1.)
                trajectory["steps"][0]["tool_calls"].pop(2)
                traj.write_text(json.dumps(trajectory))
                grader.main()
                self.assertEqual(write.call_args.args[0]["rewards"]["reward"], 0.)


class NativeTests(unittest.TestCase):
    def setUp(self):
        self.w = World(101, render=False)
        self.addCleanup(self.w.close)

    def test_native_free_base_and_one_servo_per_joint(self):
        w, m = self.w, self.w.model
        self.assertEqual((m.nq, m.nv, m.nu, m.neq, m.ncam), (19, 18, 12, 0, 1))
        self.assertEqual(m.jnt_type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(m.cam_bodyid[0], w.trunk)
        self.assertEqual(m.actuator_trnid[:, 0].tolist(), [m.joint(n+"_joint").id for n in JOINTS])
        targets = [0., .89, -1.78, .01, .91, -1.81, -.01, .92, -1.83, .02, .93, -1.84]
        w.act(targets, 2)
        expected = np.empty((4,3))
        expected[list(w.legs_by_port)] = np.array(targets).reshape(4,3)
        np.testing.assert_array_equal(w.data.ctrl, expected.ravel())
        self.assertFalse(np.any(w.data.xfrc_applied) or np.any(w.data.qfrc_applied))
        w.advance()
        w.act(targets, 2)
        np.testing.assert_array_equal(w.data.ctrl, expected.ravel())

    def test_api_is_explicit_without_motion_macros(self):
        self.assertEqual(set(METHODS.values()), set(ALLOWED))
        schema = next(t["inputSchema"] for t in TOOLS if t["name"] == "robot_act")
        self.assertEqual(set(schema["required"]), {"target_rad", "ticks"})
        before = self.w.state_fingerprint()
        for method, params in (("truth", {}), ("advance", {"seed": 9}), ("act", {"action": [0]}),
                               ("act", {"target_rad": HOME, "ticks": True}),
                               ("act", {"target_rad": HOME, "ticks": 501}), ("observe", {"camera": "top"}),
                               *(("act", {"target_rad": t, "ticks": 1}) for t in
                                 (HOME[:-1], [False]+HOME[1:], [float("nan")]+HOME[1:], [0]*12))):
            with self.subTest(method=method, params=params), self.assertRaises(ValueError):
                dispatch(self.w, method, params)
        self.assertEqual(before, self.w.state_fingerprint())

    def test_gait_moves_free_base_by_alternating_ground_contacts(self):
        w = self.w
        # This is the level-floor actuator test; the split-deck transfer has its own paired trials.
        w.data.qpos[:2] = [2.4,2.5]
        mujoco.mj_forward(w.model,w.data)
        floor = w.model.geom("landing").id
        feet = [w.model.geom(leg).id for leg in ("FR", "FL", "RR", "RL")]
        seen = [set() for _ in feet]
        start = w.data.qpos[0]
        for k in range(88):  # four cycles of the same direct flat-floor targets
            target = np.array(pose((k+1)*12*.002*1.9, .10, 0.)).reshape(4, 3)
            target = target[list(w.legs_by_port)]
            w.act(target.ravel().tolist(), 12)
            ground = {b if a == floor else a for a, b in w.data.contact.geom if floor in (a, b)}
            for states, foot in zip(seen, feet):
                states.add(foot in ground)
            self.assertFalse(w.fallen or np.any(w.data.xfrc_applied) or np.any(w.data.qfrc_applied))
        self.assertGreater(w.data.qpos[0] - start, .10)
        self.assertEqual(seen, [{False, True}]*4)

    def test_rgb_has_no_effect_on_physics_or_numeric_telemetry(self):
        w = self.w
        before = w._physics_digest(w.data)
        w.render_enabled = True
        a, b = w.observe(), w.observe()
        self.assertEqual(set(a), {"images", "phase", "calibration_checks", "dock", "approach_reached",
                                  "goal_reached", "fallen", "done", "frame_id"})
        self.assertEqual(a["images"], b["images"])
        im = Image.open(io.BytesIO(base64.b64decode(a["images"][0])))
        self.assertEqual((im.mode, im.size), ("RGB", (384, 288)))
        self.assertGreater(np.asarray(im).std(), 12)
        self.assertEqual(w._physics_digest(w.data), before)
        self.assertEqual(w.metrics()["milestone_fraction"], 0)
        w.render_enabled = False
        for _ in range(125):
            w.observe()
        self.assertNotIn("images", w.act(HOME, 1))
        self.assertEqual(w.frame_id, 127)

    def test_neutral_does_not_score_and_recovery_refunds_nothing(self):
        w = self.w
        home = w.data.qpos.copy()
        w.act(HOME, 500)
        self.assertEqual(w.metrics()["milestone_fraction"], 0)
        w.act([0., 1.09, -2.18]*4, 450)
        w.observe()
        self.assertEqual(w.calibration, 1)
        for method in (w.recover, w.advance, w.recover):
            counters = w.steps, w.ticks, w.frame_id, w.data.time, w.calibration
            method()
            np.testing.assert_array_equal(w.data.qpos, home)
            self.assertEqual(counters, (w.steps, w.ticks, w.frame_id, w.data.time, w.calibration))
            self.assertEqual(w.metrics()["milestone_fraction"], 1/14)
        for _ in range(3):
            w.act(HOME, 500)
        self.assertEqual((w.approaches, w.delivered), (0, 0))
        self.assertAlmostEqual(w.metrics()["simulated_seconds"], w.ticks*.002)
        self.assertEqual(w.steps, w.metrics()["calibration_pose_calls"]+w.metrics()["route_pose_calls"])

    def test_fall_and_rerack_on_unfinished_leg(self):
        w = self.w
        w.advance()
        start = w.data.qpos.copy()
        w.data.qpos[3:7] = [0., 1., 0., 0.]
        mujoco.mj_forward(w.model, w.data)
        w.act(HOME, 500)
        self.assertTrue(w.fallen)
        frozen = w.state_fingerprint()
        with self.assertRaises(ValueError):
            w.act(HOME, 1)
        self.assertEqual(frozen, w.state_fingerprint())
        clock = w.data.time
        w.recover()
        np.testing.assert_array_equal(start, w.data.qpos)
        self.assertEqual((w.data.time, w.falls, w.current, w.recoveries), (clock, 1, 1, 1))
        self.assertEqual(w.metrics()["milestone_fraction"], 0)
        self.assertFalse(w.act(HOME, 12)["fallen"])

    def test_approach_then_quiet_aligned_dwell_and_checkpoint(self):
        w = self.w
        w.advance()  # skipping commissioning never grants those points
        x, y, yaw = w.stations[0]
        # Place a test fixture state and feed the tick predicate; the cohort uses actual integrated motion.
        def place(offset, heading):
            w.data.qpos[:2] = [x+offset, y]
            w.data.qpos[3:7] = [math.cos(heading/2), 0, 0, math.sin(heading/2)]
            mujoco.mj_forward(w.model, w.data)
        def ticks(n):
            for _ in range(n):
                w._tick()
        w.data.qvel[:] = 0
        place(.20, yaw)
        ticks(39)
        self.assertEqual(w.approaches, 0)
        ticks(1)
        self.assertEqual(w.metrics()["milestone_fraction"], 1/14)
        w.recover()
        self.assertEqual(w.approaches, 1)
        place(.05, yaw+.5)
        ticks(200)
        self.assertEqual(w.delivered, 0)
        # Settle a genuine two-runner stance before testing individual dwell ticks.
        side = np.sign(w.data.geom_xpos[w.runners[1],1])
        joints = pose(0.,0.,0.,height=.25,lift=0.,track=(-.052,.052))
        place(.025, yaw)
        w.data.qpos[1:3] = [side*.061,.255]
        w.data.qpos[7:] = w.data.ctrl[:] = joints
        mujoco.mj_forward(w.model,w.data)
        mujoco.mj_step(w.model,w.data,nstep=350)
        mujoco.mj_forward(w.model,w.data)
        self.assertGreaterEqual(min(w._runner_loads()),8.)
        ticks(174)
        w.data.qvel[0] = .13
        ticks(1)
        w.data.qvel[:] = 0
        ticks(174)
        self.assertEqual(w.delivered, 0)
        ticks(1)
        self.assertEqual(w.metrics()["milestone_fraction"], 2/14)
        before = w.data.qpos.copy()
        with self.assertRaises(ValueError):
            w.recover()
        w.advance()
        np.testing.assert_array_equal(before, w.data.qpos)
        w.act(HOME, 10)
        w.recover()
        np.testing.assert_array_equal(before, w.data.qpos)
        with self.assertRaises(ValueError):
            w.advance()
        self.assertEqual((w.approaches, w.delivered, w.current), (1, 1, 2))

    def test_service_and_separate_replay_agree_including_rejected_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            service = Runtime(tmp, 101, 0)
            self.addCleanup(service.world.close)
            for method, params in (("observe", {}), ("act", {"target_rad": [0.,1.09,-2.18]*4, "ticks": 450}),
                                   ("recover", {}), ("advance", {}), ("advance", {}),
                                   ("act", {"target_rad": HOME, "ticks": 25}), ("observe", {})):
                if method == "advance" and service.world.current:
                    with self.assertRaises(ValueError):
                        service.call({"method": method, "params": params})
                else:
                    service.call({"method": method, "params": params})
            state = json.loads((Path(tmp)/"state.json").read_text())
            state["events"] = [json.loads(s) for s in (Path(tmp)/"events.jsonl").read_text().splitlines()]
            self.assertEqual((len(state["events"]), state["requests"]), (7, 7))
            self.assertEqual(replay(state), state["metrics"])
            self.assertEqual(state["metrics"]["milestone_fraction"], 1/14)


if __name__ == "__main__":
    unittest.main(verbosity=2)
