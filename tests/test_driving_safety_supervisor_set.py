"""driving_safety_supervisor: the fixed hidden set, the grader's checks of it, and what the agent is given.

The payload's copies of the bridge, the protocol, the client, the guard and the probe sprites are the grader's; the
hidden set is 24 episodes, one per graded cell in grading order, with distinct seeds in [2**32, 2**62), distinct lock
keys and a layout digest each, every one the first kept candidate of its cell in the committed selection record,
kept by the pre-registered rule, selected with the files that are committed now; nothing agent-visible holds a value
of the set or names the reference's token; the practice grader refuses the graded towns; a malformed set is a task
error whose message names no value of it, and so is a layout the simulator builds differently; the mock's layout
digest follows the spec alone; footprints.json covers every prop a staging spawns, and the car; a red light counts
once for a crossing; the expert's replies and memory do not depend on the order of a probe pair.
"""
import contextlib
import copy
import hashlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/driving/driving_safety_supervisor"
TESTS = TASK / "tests"
PAYLOAD = TASK / "environment/setup/payload"


# the task's module names, which other tasks' tests may import from their own folders: imported here with those
# set aside, and taken out of sys.modules again afterwards
_NAMES = ("dev_batch", "drive_bridge", "drive_protocol", "drive_variants", "episode_score", "robotics_grader",
          "run_episode", "select_set", "sim_client", "solver", "verify")


def _load():
    saved_path, saved_flag = list(sys.path), sys.dont_write_bytecode
    saved = {n: sys.modules.pop(n) for n in list(sys.modules) if n.split(".")[0] in _NAMES}
    sys.dont_write_bytecode = True  # (no __pycache__ in the task folder)
    try:
        for d in ("solution", "tests/review", "tests"):
            sys.path.insert(0, str(TASK / d))
        import drive_bridge  # noqa: E402
        import drive_variants  # noqa: E402
        import select_set  # noqa: E402
        import solver  # noqa: E402
        import verify  # noqa: E402
        return drive_bridge, drive_variants, select_set, solver, verify
    finally:
        sys.path[:] = saved_path
        sys.dont_write_bytecode = saved_flag
        for n in [n for n in sys.modules if n.split(".")[0] in _NAMES]:
            del sys.modules[n]
        sys.modules.update(saved)


drive_bridge, drive_variants, select_set, solver, verify = _load()
SET = json.loads((TESTS / "hidden/episodes.json").read_text())
SELECTION = json.loads((TESTS / "hidden/selection.json").read_text())
CONFIG = json.loads((TESTS / "grader_config.json").read_text())


class CopiesTests(unittest.TestCase):
    def test_payload_copies_are_the_graders(self):
        for name in ("drive_bridge.py", "drive_protocol.py", "sim_client.py", "netguard.py"):
            self.assertEqual((PAYLOAD / "drive" / name).read_bytes(), (TESTS / name).read_bytes(), name)
        ours = sorted(p.name for p in (PAYLOAD / "drive/probe_sprites").iterdir())
        theirs = sorted(p.name for p in (TESTS / "probe_sprites").iterdir())
        self.assertEqual(ours, theirs)
        for name in ours:
            self.assertEqual((PAYLOAD / "drive/probe_sprites" / name).read_bytes(),
                             (TESTS / "probe_sprites" / name).read_bytes(), name)


class SetTests(unittest.TestCase):
    def test_format_cells_order_seeds_keys(self):
        self.assertEqual(SET["format"], verify.SET_FORMAT)
        cells = drive_variants.graded_cells(CONFIG["towns"])
        episodes = SET["episodes"]
        self.assertEqual([(e["town"], e["variant"]) for e in episodes], cells)
        self.assertEqual(len(episodes), 24)
        self.assertEqual(sum(1 for c in cells if c == ("Town05", "shoulder_pass")), 3)
        kind_of = {v: k for k, vs in drive_variants.VARIANTS.items() for v in vs}
        for e in episodes:
            self.assertEqual(e["kind"], kind_of[e["variant"]])
            self.assertIs(type(e["seed"]), int)
            self.assertTrue(1 << 32 <= e["seed"] < 1 << 62)
            self.assertIn(e["distractors"], (0, 1, 2))
            for field in ("lock_key", "layout_sha256"):
                self.assertRegex(e[field], r"\A[0-9a-f]{64}\Z")
        for field in ("seed", "lock_key", "id"):
            self.assertEqual(len({e[field] for e in episodes}), 24, field)
        self.assertEqual(len(verify.load_set(CONFIG)), 24)

    def test_set_is_the_selections_first_kept_per_cell(self):
        self.assertEqual(SELECTION["format"], select_set.SEL_FORMAT)
        by_id = {c["id"]: c for c in SELECTION["candidates"]}
        for k, e in enumerate(SET["episodes"]):
            c = by_id[e["id"]]
            self.assertEqual(c["cell"], k)
            self.assertEqual(c["selection"], "kept")
            for field in ("town", "variant", "kind", "seed", "distractors", "lock_key"):
                self.assertEqual(c[field], e[field], field)
            self.assertEqual(c["run"]["layout"], e["layout_sha256"])
            self.assertEqual(select_set.keep_clauses(c), [])
            earlier = [d for d in SELECTION["candidates"] if d["cell"] == k and d["draw"] < c["draw"]]
            for d in earlier:          # every earlier candidate of the cell ran and failed the rule
                self.assertEqual(d["status"], "ran")
                self.assertTrue(select_set.keep_clauses(d))
                self.assertEqual(d["selection"], "dropped")
        self.assertEqual(SELECTION["set"]["episodes_sha256"],
                         hashlib.sha256((TESTS / "hidden/episodes.json").read_bytes()).hexdigest())

    def test_selected_with_the_committed_files(self):
        now = select_set.hashes(select_set.FROZEN)
        changed = [p for p in select_set.FROZEN if SELECTION["frozen"][p] != now[p]]
        self.assertEqual(changed, [], "a file the set was selected with has changed: select the set again")


class AgentVisibleTests(unittest.TestCase):
    def test_no_value_of_the_set_and_no_token(self):
        needles = set()
        for e in SET["episodes"]:
            needles.update((str(e["seed"]), e["lock_key"], e["layout_sha256"]))
        needles.add((TASK / "solution/reference/oracle_token").read_text().strip())
        needles.add((TESTS / "oracle_token.sha256").read_text().split()[0])
        needles.update((".oracle_token", "oracle_token"))
        files = [TASK / "instruction.md"] + [p for p in (TASK / "environment").rglob("*") if p.is_file()]
        for path in files:
            data = path.read_bytes()
            for needle in needles:
                self.assertNotIn(needle.encode(), data, "{} holds a value it must not".format(path))

    def test_practice_grader_refuses_graded_towns(self):
        with tempfile.TemporaryDirectory() as tmp:
            sub = os.path.join(tmp, "submission")
            os.makedirs(sub)
            Path(sub, "run.sh").write_text("#!/bin/bash\necho READY\n")
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
            proc = subprocess.run([sys.executable, "-B", str(PAYLOAD / "practice_grader/grade.py"), "--towns", "Town04",
                                   "--backend", "mock", "--submission", sub], capture_output=True, text=True,
                                  timeout=60, env=env)
        self.assertEqual(proc.returncode, 2, proc.stderr[-400:])
        self.assertIn("invalid choice", proc.stderr)


class GraderChecksTests(unittest.TestCase):
    def _fails(self, fn):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
            fn()
        return err.getvalue()

    def _values(self, spec):
        out = set()
        for e in spec["episodes"] if isinstance(spec, dict) and isinstance(spec.get("episodes"), list) else ():
            if isinstance(e, dict):
                out.update(str(e.get(k)) for k in ("seed", "lock_key", "layout_sha256", "id") if e.get(k) is not None)
        return {v for v in out if len(v) > 3}

    def test_malformed_sets_are_task_errors_naming_no_value(self):
        good = copy.deepcopy(SET)
        cases = []

        def case(fn):
            spec = copy.deepcopy(good)
            fn(spec)
            cases.append(spec)
        case(lambda s: s.update(format="driving_safety_supervisor fixed set 0"))
        case(lambda s: s["episodes"].pop())
        case(lambda s: s["episodes"].__setitem__(slice(0, 2), [s["episodes"][1], s["episodes"][0]]))
        case(lambda s: s["episodes"][3].update(kind="occluded_pedestrian"))
        case(lambda s: s["episodes"][4].update(seed=12345))
        case(lambda s: s["episodes"][5].update(seed=True))
        case(lambda s: s["episodes"][6].update(seed=s["episodes"][2]["seed"]))
        case(lambda s: s["episodes"][7].update(distractors=3))
        case(lambda s: s["episodes"][8].update(lock_key=s["episodes"][8]["lock_key"].upper()))
        case(lambda s: s["episodes"][9].update(lock_key=s["episodes"][1]["lock_key"]))
        case(lambda s: s["episodes"][10].update(layout_sha256="0" * 63))
        case(lambda s: s["episodes"][11].pop("id"))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "episodes.json")
            for spec in cases:
                with open(path, "w") as f:
                    json.dump(spec, f)
                message = self._fails(lambda: verify.load_set(CONFIG, path))
                self.assertIn("TASK ERROR", message)
                for value in self._values(good):
                    self.assertNotRegex(message, r"(?<![0-9a-f]){}(?![0-9a-f])".format(re.escape(value)))
            with open(path, "w") as f:
                f.write("{not json")
            self.assertIn("TASK ERROR", self._fails(lambda: verify.load_set(CONFIG, path)))

    def test_layout_check(self):
        class FakeSim(object):
            def __init__(self, layout):
                self.layout = layout

            def reset(self, spec):
                return {"truth": {"layout": self.layout}}

            def step(self, mode, control):
                return {}
        want = SET["episodes"][0]["layout_sha256"]
        verify.LayoutChecked(FakeSim(want), want, 0).reset({})
        message = self._fails(lambda: verify.LayoutChecked(FakeSim("f" * 64), want, 0).reset({}))
        self.assertIn("another layout", message)
        self.assertNotIn(want, message)
        self._fails(lambda: verify.LayoutChecked(FakeSim(None), want, 0).reset({}))

    def test_mock_layout_digest(self):
        def layout(seed, kind="construction_zone"):
            return drive_bridge.MockBackend(None).reset({"town": "Town01", "kind": kind, "seed": seed})["truth"]["layout"]
        self.assertEqual(layout(5), layout(5))
        self.assertNotEqual(layout(5), layout(6))
        self.assertNotEqual(layout(5), layout(5, "occluded_pedestrian"))


class FootprintTests(unittest.TestCase):
    def test_every_staged_prop_and_the_car(self):
        table = json.loads((TASK / "solution/footprints.json").read_text())
        measured = set(table["footprints"])
        unmeasured = set(re.findall(r"static\.prop\.[a-z0-9_]+", table["source"].split("No entry:")[1]))
        spawned = set()
        for name in ("drive_variants.py", "drive_bridge.py"):
            spawned |= set(re.findall(r"static\.prop\.[a-z0-9_]+", (TESTS / name).read_text()))
        self.assertTrue(spawned)
        self.assertEqual(spawned - measured - unmeasured, set())
        self.assertIn("vehicle.lincoln.mkz_2020", measured)


class RedLightCountTests(unittest.TestCase):
    """CarlaBackend._red_light_check on stand-ins for CARLA's objects."""

    class State(object):
        Red, Yellow, Green = "red", "yellow", "green"

    def _backend(self):
        test = self

        class Light(object):
            def __init__(self, ident):
                self.id, self.state = ident, test.State.Red

            def get_state(self):
                return self.state

        class Ego(object):
            light = None

            def is_at_traffic_light(self):
                return self.light is not None

            def get_traffic_light(self):
                return self.light

        class Waypoint(object):
            def __init__(self, junction):
                self.is_junction = junction

        class Map(object):
            junction = False

            def get_waypoint(self, loc):
                return Waypoint(self.junction)

        b = type("Backend", (), {})()
        b.carla = type("carla", (), {"TrafficLightState": test.State})
        b.ego, b.map = Ego(), Map()
        b.nominal = type("Nominal", (), {"traffic_light_manager": lambda self: False})()
        b.tracker = type("Tracker", (), {"time": 0.0, "red_lights": 0})()
        b.red_candidate, b.red_counted = None, None
        return b, Light

    def _tick(self, b, at_light, in_junction, speed):
        b.ego.light = at_light
        b.map.junction = in_junction
        b.tracker.time += 0.05
        return drive_bridge.CarlaBackend._red_light_check(b, None, speed)

    def test_once_per_crossing(self):
        b, Light = self._backend()
        first, second = Light(7), Light(8)
        self._tick(b, first, False, 7.0)               # at the line, red
        for _ in range(12):                            # moving through the junction, still in its trigger
            self._tick(b, first, True, 5.0)
        self.assertEqual(b.tracker.red_lights, 1)
        for _ in range(4):                             # through, beyond the light
            self._tick(b, None, True, 5.0)
        self._tick(b, None, False, 5.0)                # out of the junction: the crossing is over
        self.assertIsNone(b.red_counted)
        self._tick(b, second, False, 7.0)
        self._tick(b, second, True, 7.0)               # the next junction's red light counts again
        self.assertEqual(b.tracker.red_lights, 2)

    def test_green_and_standing_do_not_count(self):
        b, Light = self._backend()
        light = Light(9)
        self.assertTrue(self._tick(b, light, False, 0.0))   # held at red: excused
        light.state = self.State.Green
        for _ in range(10):
            self._tick(b, light, True, 6.0)
        self.assertEqual(b.tracker.red_lights, 0)


class ProbeOrderTests(unittest.TestCase):
    """E0: with a probe pair every 4 steps, the expert's replies and carried memory are the same whichever request
    of the pair comes first (a synthetic road: a box in the lane, an oncoming car, a red light at 70 m)."""

    def _episode(self, order, steps=120, lane=3.5):
        e = solver.Expert()
        e.reset()
        X, Y, YAW, V, wheel = 0.0, 0.0, 0.0, 0.0, 0.0
        out = []
        for t in range(steps):
            c, s = math.cos(YAW), math.sin(YAW)

            def loc(x, y):
                dx, dy = x - X, y - Y
                return round(dx * c + dy * s, 2), round(-dx * s + dy * c, 2)
            route, i0 = [], int(X // 2)
            for i in range(max(0, i0 - 5), i0 + 45):
                lx, ly = loc(2.0 * i, 0.0)
                llx, lly = loc(2.0 * i, -lane)
                left = {"x": llx, "y": lly, "width": lane, "same_direction": False}
                route.append({"x": lx, "y": ly, "width": lane, "junction": 72.0 <= 2.0 * i <= 90.0, "left": left,
                              "lefts": [left], "rights": []})
            bx, by = loc(45.0, 0.2)
            ox, oy = loc(160.0 - 0.9 * t, -lane)
            sx, sy = loc(70.0, 0.0)
            priv = {"actors": [{"id": 100, "type": "prop", "blueprint": "static.prop.box01", "x": bx, "y": by,
                                "z": 0.3, "yaw": round(math.degrees(-YAW), 1), "vx": 0.0, "vy": 0.0, "length": 0.65,
                                "width": 0.65, "height": 0.7},
                               {"id": 900, "type": "vehicle", "blueprint": "vehicle.audi.a2", "x": ox, "y": oy,
                                "z": 0.0, "yaw": round(math.degrees(math.pi - YAW), 1), "vx": round(-9.0 * c, 2),
                                "vy": round(9.0 * s, 2), "length": 3.7, "width": 1.8, "height": 1.5,
                                "reverse": False}],
                    "route": route, "lights": [{"id": 5, "x": sx, "y": sy, "state": "red" if t < 90 else "green"}],
                    "red_light": False, "speed_limit": 8.33}
            obs = {"t": t, "speed": round(V, 3), "nominal": [0.75, 0.0, 0.0], "privileged": priv}
            if t % 4 == 1:
                fake = copy.deepcopy(obs)
                fake["privileged"]["actors"].append({"id": -1, "type": "walker", "x": 9.6, "y": 0.3, "yaw": 0.0,
                                                     "vx": 0.0, "vy": 0.0, "length": 0.6, "width": 0.6})
                if order == "probe-first":
                    probe_reply, a = e.act(t, fake), e.act(t, obs)
                else:
                    a, probe_reply = e.act(t, obs), e.act(t, fake)
                out.append(["probe", [float(x) for x in probe_reply]])
            else:
                a = e.act(t, obs)
            a = [float(x) for x in a]
            out.append([a, json.dumps(e.mem, sort_keys=True, default=str)])
            thr, st, br = (0.75, 0.0, 0.0) if a[0] == 0 else (a[1], a[2], a[3])
            for _ in range(2):
                wheel += (st - wheel) * (1 - math.exp(-0.05 / 0.09))
                V = max(0.0, V + (3.0 * thr - 8.0 * br - 0.3) * 0.05)
                YAW += V * math.tan(0.815 * 1.22 * wheel) / 2.9 * 0.05
                X += V * math.cos(YAW) * 0.05
                Y += V * math.sin(YAW) * 0.05
        return out

    def test_both_orders_alike(self):
        self.assertEqual(self._episode("probe-first"), self._episode("real-first"))


if __name__ == "__main__":
    unittest.main()
