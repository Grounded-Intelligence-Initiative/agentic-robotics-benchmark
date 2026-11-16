"""keyed_nut_peg_assembly: the commissioning service, the data sheet and the takt.

The agent's sandbox holds no simulator it can read. It holds the cell's data sheet (environment/setup/payload/:
datasheet.md, cell_data.py, the commissioning client and runner, the wire helper) and a commissioning service
(environment/commission_service/) that runs the grader's own simulator (a byte copy of tests/keyed_env.py) on
development kits, for a fixed number of episodes, under an account of its own. The service must run the graded
episode's limits, on kits that are neither graded kits nor candidates of the graded set's selection, and never answer
ground truth, a score or a completion verdict. The data sheet's numbers are the simulator's. The takt is 40 s per nut.

Standard library only (the host-side suite has no numpy or mujoco): the task's Python files are read as text or syntax
trees, never imported.
"""

import ast
import json
import re
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/keyed_nut_peg_assembly"
SERVICE = TASK / "environment/commission_service"
PAYLOAD = TASK / "environment/setup/payload"
KITS = json.loads((SERVICE / "kits.json").read_text())
SELECTION = json.loads((TASK / "tests/hidden/selection.json").read_text())
EPISODES = json.loads((TASK / "tests/ref/episodes.json").read_text())["episodes"]
GRADER = json.loads((TASK / "tests/grader_config.json").read_text())
LATEST = SELECTION["amendments"][-1] if SELECTION.get("amendments") else SELECTION
INSTRUCTION = (TASK / "instruction.md").read_text()
DATASHEET = (PAYLOAD / "datasheet.md").read_text()

#: The constants the data sheet's module shares with the simulator: the same name, the same expression.
SHARED = ("NUTS", "CYCLE_STEPS", "MAX_STEPS", "MAX_DPOS", "MAX_DROT", "TABLE_Z", "TABLE_HALF", "ROBOT_BASE",
          "WORKSPACE_LO", "WORKSPACE_HI", "NSEC", "LAYER", "NLAYER", "R_HOLE", "R_GROOVE", "R_NUT", "NUT_H",
          "HANDLE_LEN", "HANDLE_W", "HANDLE_Z", "SHAFT_R", "PEG_H", "COLLAR", "COLLAR_H", "PIN_Z", "PIN_R", "PIN_W",
          "PIN_HH", "TURN_LAYERS", "RISERS", "PEG_REGION", "PEG_MIN_SPACING", "NUT_REGION", "NUT_PEG_MIN_DIST",
          "NUT_MIN_SPACING", "HANDLE_CLEAR", "NUT_YAW_CENTER", "NUT_YAW_SPREAD", "ENGAGE_XY", "GROOVE_TOP",
          "FORCE_ANGLE_TOL", "FORCE_Z_TOL", "FORCE_STEPS", "GRIP_KP", "RGB_NOISE", "WRENCH_NOISE", "WRENCH_BIAS",
          "INSTAB_ROBOT_QVEL", "INSTAB_NUT_VEL", "HOME_Q", "NULLSPACE_Q", "CAMERAS", "WRIST", "CAMERA_NAMES",
          "OBS_KEYS", "NUT_COLOURS")
#: The helpers it shares with the simulator: the same name, the same body.
SHARED_FUNCTIONS = ("look_at", "camera_intrinsics", "fixed_camera_matrices", "wrap_angle", "groove_cells")


def assignments(path):
    """{name: syntax tree of the value} of a module's top-level simple assignments."""
    out = {}
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            out[node.targets[0].id] = ast.dump(node.value)
    return out


def functions(path):
    """{name: syntax tree of the body, docstring dropped} of a module's top-level functions."""
    out = {}
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.FunctionDef):
            body = node.body[1:] if ast.get_docstring(node) is not None else node.body
            out[node.name] = (ast.dump(node.args), [ast.dump(n) for n in body])
    return out


class CommissioningService(unittest.TestCase):
    def test_runs_the_graders_cell(self):
        self.assertEqual((SERVICE / "keyed_env.py").read_bytes(), (TASK / "tests/keyed_env.py").read_bytes(),
                         "commission_service/keyed_env.py differs from tests/keyed_env.py")
        self.assertEqual(sorted(p.name for p in SERVICE.iterdir() if p.name != "__pycache__"),
                         ["commissiond.py", "keyed_env.py", "kits.json"])

    def test_development_kits_are_not_graded_kits(self):
        kits = KITS["kits"]
        self.assertEqual(len(kits), 16)
        self.assertEqual(len({k["kit_seed"] for k in kits}), 16)
        dev_seeds = {k["kit_seed"] for k in kits} | {k["layout_seed"] for k in kits}
        candidate_seeds = {c["kit_seed"] for c in SELECTION["candidates"]} | {c["layout_seed"] for c in SELECTION["candidates"]}
        graded_seeds = {e["seed"] for e in EPISODES}
        self.assertFalse(dev_seeds & candidate_seeds, "a development seed is a candidate's")
        self.assertFalse(dev_seeds & graded_seeds, "a development seed is a graded episode's")
        graded = [sorted(json.dumps(n["groove"], sort_keys=True) for n in e["kit"]["nuts"]) for e in EPISODES]
        for k in kits:
            nuts = k["kit"]["nuts"]
            self.assertEqual(len(nuts), 3)
            self.assertEqual(sorted(k["kit"]["order"]), [1, 2, 3])
            self.assertEqual([n["id"] for n in nuts], ["%s-%d" % (k["name"], i) for i in range(3)])
            self.assertNotIn(sorted(json.dumps(n["groove"], sort_keys=True) for n in nuts), graded,
                             "%s has a graded kit's grooves" % k["name"])

    def test_development_kits_are_the_pre_registered_ones(self):
        dev = LATEST["development_kits"]
        cands = {c["id"]: c for c in dev["candidates"]}
        for c in dev["candidates"]:
            self.assertEqual((c["kit_seed"], c["layout_seed"]), (7001 + c["i"], 11001 + c["i"]))
        names = [k["name"] for k in KITS["kits"]]
        for k in KITS["kits"]:
            self.assertEqual((k["kit_seed"], k["layout_seed"]), (cands[k["name"]]["kit_seed"], cands[k["name"]]["layout_seed"]))
        if "selection_run" in dev:                   # once the reference has run them: the first 16 it completed
            runs = dev["selection_run"]["runs"]
            self.assertEqual([r["id"] for r in runs], [c["id"] for c in dev["candidates"]])
            kept = [r["id"] for r in runs if r["kept"]]
            self.assertEqual(names, kept[:16])
            self.assertEqual(dev["kits"], names)
            for r in runs:
                if r["kept"]:
                    self.assertEqual((r["kit_complete"], r["nuts_assembled"], r["drop_reasons"]), (1, 3, []))
                    self.assertLess(r["steps"], 1200)
        else:                                        # pre-registered, not run yet: the first 16 candidates
            self.assertEqual(names, [c["id"] for c in dev["candidates"]][:16])

    def test_window_and_limits(self):
        self.assertEqual((KITS["episodes"], KITS["open"]), (32, 2))
        self.assertEqual(KITS["controller_budget_s"], GRADER["controller_budget_s"])
        self.assertEqual(KITS["image_charge_s"], GRADER["image_charge_s"])
        self.assertEqual(KITS["probe_rate"], GRADER["probe_rate"])
        self.assertGreaterEqual(KITS["silence_s"], GRADER["step_timeout_s"])
        for text in (INSTRUCTION, DATASHEET):
            flat = " ".join(text.split())
            self.assertIn("32 episodes", flat)
            self.assertIn("16 development kits", flat)
            self.assertIn("at most 2 at a time", flat)
            self.assertIn("%d s without a request" % KITS["silence_s"], flat)
        runner = (PAYLOAD / "commission/run_kit.py").read_text()
        self.assertIn("CONTROLLER_BUDGET = %.1f" % GRADER["controller_budget_s"], runner)
        charges = GRADER["image_charge_s"]
        self.assertIn('IMAGE_CHARGE = {"left": %s, "right": %s, "wrist": %s}' % (
            charges["left"], charges["right"], charges["wrist"]), runner)
        self.assertIn("STEP_TIMEOUT = %.1f" % GRADER["step_timeout_s"], runner)

    def test_answers_no_truth_score_or_verdict(self):
        src = (SERVICE / "commissiond.py").read_text()
        tree = ast.parse(src)
        used = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        # what the service touches of the cell: its episode calls, its step count and limit, its fault, the work
        # order's colours (to name a ruined nut), the cameras; never its state or the grader's credit
        for forbidden in ("nut_state", "pin_in_wall", "on_peg", "peg_poses", "layout", "specs", "cells", "broken",
                          "force_count", "target_peg", "data", "model", "wrench_bias", "credit", "release_count",
                          "episode_score", "nuts_assembled", "progress", "assembled", "stage_now", "ground_truth",
                          "depth_now", "_finger_touch"):
            self.assertNotIn(forbidden, used, "the commissioning service reads %s" % forbidden)
        for word in ("keyed_credit", "GradedNutEnv", "kit_complete", "_gt"):
            self.assertNotIn(word, src)
        self.assertIn('E.NutEnv(entry["kit"]["nuts"], early_end=True)', src)
        public = None
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "PUBLIC" for t in node.targets):
                public = ast.literal_eval(node.value)
        self.assertEqual(public, ("reset", "step", "close"))

    def test_image_closes_the_service_to_the_agent(self):
        docker = (TASK / "environment/Dockerfile").read_text()
        self.assertIn("COPY commission_service /opt/keyedcell-commission", docker)
        self.assertIn("chmod -R u+rwX,go-rwx /opt/keyedcell-commission", docker)
        for probe in ("! runuser -u user -- ls /opt/keyedcell-commission",
                      "! runuser -u user -- cat /opt/keyedcell-commission/keyed_env.py",
                      "! runuser -u user -- cat /opt/keyedcell-commission/kits.json"):
            self.assertIn(probe, docker)
        instructions = [line.split()[0] for line in docker.splitlines()
                        if re.match(r"^(FROM|RUN|COPY|ADD|ENV|WORKDIR|USER|ENTRYPOINT|CMD)\b", line)]
        self.assertEqual(instructions[-1], "WORKDIR")
        self.assertIn("\nWORKDIR /\n", docker)
        setup = (TASK / "environment/setup/setup.sh").read_text()
        self.assertIn("runuser -u keyedcell --", setup)
        self.assertNotIn("keyed_env", setup)


class AgentSide(unittest.TestCase):
    def test_agent_has_no_simulator(self):
        files = sorted(p.relative_to(PAYLOAD).as_posix() for p in PAYLOAD.rglob("*")
                       if p.is_file() and "__pycache__" not in p.parts)
        self.assertEqual(files, ["cell_data.py", "commission/client.py", "commission/run_kit.py", "datasheet.md",
                                 "helpers/wire_skeleton.py"])
        for rel in files:
            text = (PAYLOAD / rel).read_text()
            for word in ("import mujoco", "MjSpec", "MjModel", "mj_step", "NutEnv", "SCENE_XML", "nut_body_xml",
                         "_osc_torque", "sample_kit", "sample_nut_spec", "sample_layout", "solref", "condim",
                         "keyed_env"):
                self.assertNotIn(word, text, "%s holds %s" % (rel, word))

    def test_data_sheet_numbers_are_the_cells(self):
        cell, sheet = assignments(TASK / "tests/keyed_env.py"), assignments(PAYLOAD / "cell_data.py")
        for name in SHARED:
            self.assertIn(name, cell)
            self.assertEqual(sheet.get(name), cell[name], "cell_data.%s is not the simulator's" % name)
        cell_f, sheet_f = functions(TASK / "tests/keyed_env.py"), functions(PAYLOAD / "cell_data.py")
        for name in SHARED_FUNCTIONS:
            self.assertEqual(sheet_f.get(name), cell_f[name], "cell_data.%s is not the simulator's" % name)

    def test_takt(self):
        cell = (TASK / "tests/keyed_env.py").read_text()
        self.assertIn("\nNUTS = 3 ", cell)
        self.assertIn("\nCYCLE_STEPS = 400 ", cell)
        self.assertIn("\nMAX_STEPS = NUTS * CYCLE_STEPS ", cell)
        for text in (INSTRUCTION, DATASHEET):
            flat = " ".join(text.split())
            self.assertIn("1200 steps", flat)
            self.assertIn("40 s per nut", flat)
            self.assertNotIn("2400", flat)
        self.assertIn("ends at 1200 steps", " ".join(INSTRUCTION.split()))

    def test_instruction_names_only_what_the_agent_has(self):
        for name in ("datasheet.md", "cell_data.py", "commission/client.py", "commission/run_kit.py",
                     "helpers/wire_skeleton.py"):
            self.assertIn(name, INSTRUCTION)
            self.assertTrue((PAYLOAD / name).is_file(), name)
        for gone in ("keyed_env.py", "practice_kit", "practice_bench", "dev sim", "sample_kit"):
            self.assertNotIn(gone, INSTRUCTION)


if __name__ == "__main__":
    unittest.main()
