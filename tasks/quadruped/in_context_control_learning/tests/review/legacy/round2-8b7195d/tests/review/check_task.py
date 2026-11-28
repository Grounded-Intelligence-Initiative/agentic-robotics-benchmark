"""Reviewer self-checks of the delivery-bay task, run inside the task image from the task folder:

    /opt/venv/bin/python3 tests/review/check_task.py

The hidden set's two copies agree, the world's runtime copy and the grader's copy agree, the same seed gives the same
scene, the same requests give the same episode bit for bit with and without rendering, every scene's gaits and
layout are inside the ranges the instruction states, the camera follows the instruction's pinhole model and the
floodlight its radius, the budgets end what they should, the oracle's channel is recorded as a hash and accepted only
with the oracle's token, and the direct-control audit passes and fails where it should.
"""
import filecmp
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from audit import audit_trajectory  # noqa: E402
import service  # noqa: E402
import world as world_module  # noqa: E402
from world import (ACTION_LIMIT, DARK, FENCE, FRAME_LIMIT, LIGHT_RADIUS, LOAD_AFTER, REQUEST_LIMIT,  # noqa: E402
                   WALL_HALF, Scene, World)


def check(label, condition):
    print(("ok   " if condition else "FAIL ")+label, flush=True)
    if not condition:
        raise SystemExit(1)


def run(seed, presses, rng_seed, render=False, looks=0):
    world, rng = World(seed, render=render), np.random.default_rng(rng_seed)
    for n in range(presses):
        if world.done:
            break
        if world.delivered > world.current:
            world.advance()
        world.act([int(rng.integers(4))/3])
        if looks and n % (presses//looks) == 0:
            world.observe()
    return world


def frame(world, x, y, degrees):
    """The camera frame at a pose (a teleport, for the camera checks only)."""
    world.data.qpos[:3] = [x, y, math.radians(degrees)]
    world.data.qvel[:] = 0
    mujoco.mj_forward(world.model, world.data)
    return world.image().astype(int)


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
    c = run(hidden[0], 300, 7, render=True, looks=6)
    same = {k: v for k, v in a.metrics().items() if k != "frames"} == \
        {k: v for k, v in c.metrics().items() if k != "frames"}
    check("episode: the same with the camera rendering (qpos bitwise; frames are counted, not scored)",
          np.array_equal(a.data.qpos, c.data.qpos) and c.frames == 6 and same)
    c.close()

    for seed in hidden:
        s = Scene(seed)
        turn = abs((math.degrees(s.loaded_heading-s.heading)+180) % 360-180)
        parts = [world_module._room(s.start[0], s.walls_x)]+s.goal_rooms
        ok = (60 <= turn <= 180 and .6 <= s.load <= .75 and len(s.wall_boxes()) == 10
              and .2 <= s.speeds["forward"]*.175 <= .27 and .55 <= s.speeds["backward"]/s.speeds["forward"] <= .75
              and all(12 <= math.degrees(s.speeds[k])*.175 <= 22 for k in ("left", "right"))
              and -1.5 <= s.walls_x[0] <= -.9 and .9 <= s.walls_x[1] <= 1.5
              and all(.6 <= o[0][1] <= .75 and 1. <= o[1][1] <= 1.25 and all(abs(c)+w/2 <= 3.4 for c, w in o)
                      for o in s.openings)
              and all(a != b for a, b in zip(parts, parts[1:]))
              and all(min(FENCE-abs(g[0]), FENCE-abs(g[1])) >= .44
                      and min(abs(g[0]-x) for x in s.walls_x) >= WALL_HALF+.44 for g in [s.start]+s.goals))
        check(f"scene {seed}: gaits, loading stop, partitions, openings and points inside the instruction's ranges", ok)

    w = World(hidden[0], render=False)
    forward_button = w.scene.mapping.index("forward")
    left_button = w.scene.mapping.index("left")
    for _ in range(3):
        w.act([forward_button/3])
    p0 = w.position()
    w.act([forward_button/3])
    step = float(np.linalg.norm(w.position()-p0))
    check(f"gait: a steady forward press is {step:.4f} m = 0.175 x the commanded speed",
          abs(step-.175*w.scene.speeds["forward"]) < 2e-3)
    for _ in range(2):
        w.act([left_button/3])
    y0 = w.yaw()
    reply = w.act([left_button/3])
    check(f"gait: a steady left press turns {math.degrees(w.yaw()-y0):.2f} deg = 0.175 x the commanded rate, "
          "and the compass reads the heading in whole degrees",
          abs((w.yaw()-y0)-.175*w.scene.speeds["left"]) < 2e-3
          and reply["heading_deg"] == int(round(math.degrees(w.yaw()))) % 360)
    before = w.command(forward_button)[0]
    w.delivered, w.current = LOAD_AFTER, LOAD_AFTER-1
    w.advance()
    after = w.command(forward_button)[0]
    check(f"loading stop: the robot is set down at the loaded heading and forward slows to {after/before:.2f}",
          abs(after/before-w.scene.load) < 1e-9 and abs(w.yaw()-w.scene.loaded_heading) < 1e-9)

    # The camera: a pinhole at 0.25 m, 90 degrees, the horizon in the middle row; the floodlight reaches 1.0 m.
    w = World(hidden[0])
    s = w.scene
    y = next(y for y in np.arange(-3., 3., .05)
             if all(abs(y-c) > width/2+.8 for c, width in s.openings[1]))       # a stretch of plain wall
    face = s.walls_x[1]-WALL_HALF
    w.model.geom_pos[w.goal_geom][:2] = [s.walls_x[0]-1., y-2.]                 # the pad out of these views
    for d in (.5, .8):
        im = frame(w, face-d, y, 0.)
        column = im[:, 100]
        lit = column.max(axis=1) > 20
        panel = lit & (column[:, 2]-column[:, 0] > -4)
        foot = int(np.nonzero(panel)[0].max())+1          # lower edge of the lowest panel pixel
        top = int(np.nonzero(lit)[0].min())               # upper edge of the highest lit pixel (the cap)
        check(f"camera: a wall {d} m ahead has its foot at row {foot} (model {128+32/d:.1f}) and its top at row "
              f"{top} (model {128-128*.474/(d-.015):.1f})",
              abs(foot-(128+32/d)) <= 1.5 and abs(top-(128-128*.474/(d-.015))) <= 1.5)
    im = frame(w, (s.walls_x[0]+s.walls_x[1])/2, y, 90.)
    lit = im.max(axis=2) > 20
    edge = int(np.nonzero(lit[:, 128])[0].min())
    check(f"light: on open floor the lit floor ends at row {edge} (model {128+32/LIGHT_RADIUS:.1f}), all else is "
          f"{DARK}", abs(edge-(128+32/LIGHT_RADIUS)) <= 1.5 and bool((im[~lit] == DARK).all()) and not lit[:120].any())
    gx, gy = (s.walls_x[0]+s.walls_x[1])/2, y+.7
    w.model.geom_pos[w.goal_geom][:2] = [gx, gy]
    im = frame(w, gx, y, 90.)
    green = im[:, :, 1]-np.maximum(im[:, :, 0], im[:, :, 2]) > 30
    rows, cols = np.nonzero(green)
    check(f"pad: 0.7 m ahead it is green, centred on column {cols.mean():.0f}, its far edge at row {rows.min()} "
          f"(model {128+128*.21/.9:.1f})", abs(cols.mean()-127.5) < 2 and abs(rows.min()-(128+128*.21/.9)) <= 1.5)
    w.close()

    # Budgets.
    w = World(hidden[0], render=False)
    for _ in range(FRAME_LIMIT):
        w.observe()
    try:
        w.observe()
        refused = False
    except ValueError:
        refused = True
    check(f"budget: frame {FRAME_LIMIT+1} is refused, and the robot can still be driven",
          refused and w.act([0])["frames_remaining"] == 0)
    w = run(hidden[1], ACTION_LIMIT+5, 3)
    check(f"budget: the episode is done after {ACTION_LIMIT} presses", w.done and w.steps == ACTION_LIMIT)
    with tempfile.TemporaryDirectory() as folder:
        runtime = service.Runtime(folder, hidden[0], 0, render=False)
        for _ in range(REQUEST_LIMIT):
            try:
                runtime.call({"method": "act", "params": {"action": [5]}})      # invalid presses still count
            except ValueError:
                pass
        try:
            runtime.call({"method": "observe", "params": {}})
            refused = False
        except ValueError as exc:
            refused = "budget" in str(exc)
        check(f"budget: request {REQUEST_LIMIT+1} is refused", refused and runtime.world.steps == 0)

    # The oracle's channel.
    with tempfile.TemporaryDirectory() as folder:
        runtime = service.Runtime(folder, hidden[0], 0, render=False)
        plain = runtime.call({"method": "act", "params": {"action": [0]}})
        reply = runtime.call({"method": "unlock", "params": {"token": "not the token"}})
        state = json.loads((Path(folder)/"state.json").read_text())
        check("oracle channel: no ground truth before unlock; the ledger keeps the token's sha256, never the token",
              "truth" not in plain and "truth" in reply and "scene" in reply
              and state["events"][-1]["params"] == {"token_sha256": hashlib.sha256(b"not the token").hexdigest()}
              and "not the token" not in json.dumps(state))
    expected = (TASK/"tests/oracle_token.sha256").read_text().split()[0]
    token = TASK/"solution/oracle_token"
    if token.exists():
        check("oracle channel: tests/oracle_token.sha256 is the hash of solution/oracle_token",
              hashlib.sha256(token.read_text().strip().encode()).hexdigest() == expected)
    check("oracle channel: the hash is in tests/ only",
          all(expected not in path.read_text(errors="ignore")
              for folder in ("environment", "solution") for path in (TASK/folder).rglob("*") if path.is_file()))
    sys.path.insert(0, str(TASK/"environment/setup/payload"))
    import mcp_stdio
    check("bridge: the agent's tools are observe, act and advance, nothing else",
          {t["name"] for t in mcp_stdio.TOOLS} == set(mcp_stdio.METHODS) == {"robot_observe", "robot_act",
                                                                              "robot_advance"}
          and set(mcp_stdio.METHODS.values()) == {"observe", "act", "advance"})

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
    check("audit: an unlock in the ledger is not a robot tool call and fails",
          not audit_trajectory(traj(*robot), events+[{"method": "unlock", "params": {"token_sha256": expected}}])[0])
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
