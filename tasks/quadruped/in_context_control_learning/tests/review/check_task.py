"""Reviewer self-checks of the delivery-bay task, run inside the task image from the task folder:

    /opt/venv/bin/python3 tests/review/check_task.py

The hidden set's two copies agree, the world's runtime copy and the grader's copy agree, the same seed gives the same
scene, the same requests give the same episode bit for bit with or without rendering-free replays, the loading stop
turns and slows the robot as the instruction says, and the direct-control audit passes and fails where it should.
"""
import filecmp
import json
import math
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
import numpy as np  # noqa: E402
from audit import audit_trajectory  # noqa: E402
from world import LOAD_AFTER, Scene, World  # noqa: E402


def check(label, condition):
    print(("ok   " if condition else "FAIL ")+label, flush=True)
    if not condition:
        raise SystemExit(1)


def run(seed, presses, rng_seed):
    world, rng = World(seed, render=False), np.random.default_rng(rng_seed)
    for _ in range(presses):
        if world.done:
            break
        if world.delivered > world.current:
            world.advance()
        world.act([int(rng.integers(4))/3])
    return world


def main():
    check("hidden set: tests/hidden_scenes.json == environment/setup/scenes.json",
          filecmp.cmp(TASK/"tests/hidden_scenes.json", TASK/"environment/setup/scenes.json", shallow=False))
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    candidates = json.loads((TASK/"tests/review/hidden_set/candidates.json").read_text())["seeds"]
    check("hidden set: 24 distinct seeds, all from the pre-registered candidates",
          len(hidden) == 24 == len(set(hidden)) and set(hidden) <= set(candidates))
    for name in ("world.py", "geometry.py", "scene_assets.py", "pbr_scene.py", "pbr_renderer.py", "service.py"):
        check(f"runtime copy: environment/setup/runtime/{name} == tests/{name}",
              filecmp.cmp(TASK/"environment/setup/runtime"/name, TASK/"tests"/name, shallow=False))
    check("scene: same seed, same scene", all(Scene(s).features() == Scene(s).features() for s in hidden))
    check("scene: different seeds, different scenes", len({json.dumps(Scene(s).features()) for s in hidden}) == 24)
    a, b = run(hidden[0], 300, 7), run(hidden[0], 300, 7)
    check("episode: same requests, same episode (qpos bitwise)",
          a.metrics() == b.metrics() and np.array_equal(a.data.qpos, b.data.qpos))
    for seed in hidden:
        s = Scene(seed)
        turn = abs((math.degrees(s.loaded_heading-s.heading)+180) % 360-180)
        boxes = s.wall_boxes()
        check(f"scene {seed}: loaded turn {turn:.0f} deg in [60, 180], load {s.load:.2f} in [0.6, 0.75], "
              f"{len(boxes)-4} partition segments", 60 <= turn <= 180 and .6 <= s.load <= .75 and len(boxes) == 10)
    w = World(hidden[0], render=False)
    before = w.velocity_command(0)
    w.delivered, w.current = LOAD_AFTER, LOAD_AFTER-1
    w.advance()
    after = w.velocity_command(0)
    angle = abs((math.degrees(math.atan2(after[1], after[0])-math.atan2(before[1], before[0]))+180) % 360-180)
    check(f"loading stop: button 0 turns by {angle:.0f} deg and slows to {np.linalg.norm(after)/np.linalg.norm(before):.2f}",
          60 <= angle <= 180.001 and abs(np.linalg.norm(after)/np.linalg.norm(before)-w.scene.load) < 1e-9)

    def call(name, args=None, server="robot"):
        extra = {"ale": {"mcp": {"server": server, "tool": name}}} if server else {}
        return {"function_name": f"mcp__{server}__{name}" if server else name, "arguments": args or {},
                "extra": extra}

    def traj(*calls):
        return {"schema_version": "ATIF-v1.8", "agent": {"name": "claude-code", "model_name": "m"},
                "steps": [{"source": "agent", "tool_calls": [c]} for c in calls]}

    events = [{"method": "observe", "params": {}}, {"method": "act", "params": {"action": [1/3]}},
              {"method": "act", "params": {"action": [1]}}]
    robot = [call("robot_observe"), call("robot_act", {"action": [1/3]}), call("robot_act", {"action": [1]})]
    check("audit: robot calls equal to the ledger pass", audit_trajectory(traj(*robot), events)[0])
    check("audit: an extra ledger request (a script on the socket) fails",
          not audit_trajectory(traj(*robot), events+[{"method": "act", "params": {"action": [0]}}])[0])
    check("audit: a robot call that never reached the service (rejected by the harness) passes",
          audit_trajectory(traj(robot[0], call("robot_act", {"action": 2}), *robot[1:]), events)[0])
    check("audit: 1 and 1.0 are the same press",
          audit_trajectory(traj(robot[0], robot[1], call("robot_act", {"action": [1.0]})), events)[0])
    check("audit: to-do and read-only tools pass",
          audit_trajectory(traj(call("TaskCreate", {"subject": "x"}, None), call("Read", {"file_path": "/x"}, None),
                                *robot), events)[0])
    for name in ("Bash", "Write", "Edit", "Agent", "Task", "NotebookEdit", "shell", "apply_patch", "exec_command"):
        check(f"audit: {name} fails", not audit_trajectory(traj(*robot, call(name, {}, None)), events)[0])
    check("audit: requests out of order fail", not audit_trajectory(traj(robot[0], robot[2], robot[1]), events)[0])
    print("all checks passed")


if __name__ == "__main__":
    main()
