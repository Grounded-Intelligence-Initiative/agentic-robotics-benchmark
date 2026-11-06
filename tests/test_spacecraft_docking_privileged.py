"""fault_adaptive_spacecraft_docking: the privileged reference channel, the fixed hidden episode set, the network
guard and what the agent is given.

The grader hands the simulator's current truth only to a submission whose .oracle_token (a regular file, never a
link) hashes to tests/hidden/oracle_token.sha256; nothing agent-visible names the channel; building the block draws
nothing and changes nothing; the block carries every quantity env.step reads that is in effect now and nothing drawn
for the future; a run without the token sends exactly the requests it sent before the channel existed; the hidden
episodes are a fixed set of 48 distinct 64-bit seeds, each pinned by its world digest; an episode's probe stream
depends only on the set's key and its seed; the reference reproduces its recorded result on the set; a verify that
cannot install the network filter is a task error before any controller starts; and the agent gets the plant model
as a document, with no copy of the simulator and nothing that grades a controller.
"""
import ast
import copy
import hashlib
import io
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

TASK = Path(__file__).resolve().parents[1] / "tasks/space/fault_adaptive_spacecraft_docking"
MB_NONET = Path(__file__).resolve().parents[1] / "tasks/arm/arm_hidden_mechanism_box/tests/nonet_exec.py"


def _load():
    saved_path, saved_flag = list(sys.path), sys.dont_write_bytecode
    sys.dont_write_bytecode = True  # (no __pycache__ in the task folder)
    try:
        for d in ("solution/privileged", "tests/review", "tests"):
            sys.path.insert(0, str(TASK / d))
        import oracle  # noqa: E402
        import oracle_batch  # noqa: E402
        import verify  # noqa: E402
        return verify, oracle, oracle_batch
    finally:
        sys.path[:] = saved_path
        sys.dont_write_bytecode = saved_flag


verify, oracle, oracle_batch = _load()
SET = json.loads((TASK / "tests/hidden/episodes.json").read_text())
SELECTION = json.loads((TASK / "tests/hidden/selection.json").read_text())


def scripted(t, seed):
    """A deterministic command that is not a controller: mostly small duties, a few large ones."""
    return [((t * 7 + i * 13 + seed % 97) % 23) / 22.0 * (0.3 if (t + i) % 5 else 1.0) for i in range(6)]


def old_perturbed_observation(obs, rng):
    """verify.py's probe copy before the channel existed (kept here to pin the untokened requests)."""
    changed = list(obs)
    changed[0] += rng.uniform(-0.28, 0.28)
    changed[1] += rng.uniform(-0.28, 0.28)
    yaw = math.atan2(changed[3], changed[2]) + rng.uniform(-0.18, 0.18)
    changed[2], changed[3] = math.cos(yaw), math.sin(yaw)
    return changed


class TokenTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.sub = os.path.join(self.tmp.name, "submission")
        os.makedirs(self.sub)
        self.token = b"0123456789abcdef" * 4
        self.digest = os.path.join(self.tmp.name, "oracle_token.sha256")
        with open(self.digest, "w") as f:
            f.write(hashlib.sha256(self.token).hexdigest() + "\n")
        self.path = os.path.join(self.sub, ".oracle_token")

    def tearDown(self):
        self.tmp.cleanup()

    def state(self):
        return verify.token_state(self.sub, digest_path=self.digest)

    def test_no_token_is_neither(self):
        self.assertEqual(self.state(), (False, False))

    def test_matching_token_is_privileged(self):
        with open(self.path, "wb") as f:
            f.write(self.token + b"\n")
        self.assertEqual(self.state(), (True, False))

    def test_wrong_token_is_a_mismatch(self):
        with open(self.path, "wb") as f:
            f.write(b"f" * 64)
        self.assertEqual(self.state(), (False, True))

    def test_a_link_is_never_followed(self):
        real = os.path.join(self.tmp.name, "real_token")
        with open(real, "wb") as f:
            f.write(self.token)
        os.symlink(real, self.path)
        self.assertEqual(self.state(), (False, True))

    def test_a_directory_a_fifo_or_a_huge_file_is_a_mismatch(self):
        os.makedirs(self.path)
        self.assertEqual(self.state(), (False, True))
        os.rmdir(self.path)
        os.mkfifo(self.path)  # (opened without blocking: no writer needed, no hang)
        self.assertEqual(self.state(), (False, True))
        os.remove(self.path)
        with open(self.path, "wb") as f:
            f.write(self.token + b" " * (verify.TOKEN_MAX_BYTES + 10))
        self.assertEqual(self.state(), (False, True))

    def test_the_reference_token_matches_the_digest(self):
        token = (TASK / "solution/privileged/oracle_token").read_bytes().strip()
        self.assertEqual(len(token), 64)  # 256 bits, hex
        want = (TASK / "tests/hidden/oracle_token.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), want)
        self.assertEqual(verify.TOKEN_DIGEST, str(TASK / "tests/hidden/oracle_token.sha256"))

    def test_nothing_agent_visible_names_the_channel(self):
        token = (TASK / "solution/privileged/oracle_token").read_text().strip()
        digest = (TASK / "tests/hidden/oracle_token.sha256").read_text().split()[0]
        needles = [token, digest, ".oracle_token", "oracle_token", "privileged_mode", '"privileged"',
                   verify.PRIV_FORMAT, verify.SET_FORMAT, SET["probe_key"]] + [str(e["seed"]) for e in SET["episodes"]]
        visible = [TASK / "instruction.md"] + [p for p in (TASK / "environment").rglob("*") if p.is_file()]
        for p in visible:
            text = p.read_text(errors="replace")
            for needle in needles:
                self.assertNotIn(needle, text, "{} mentions {}".format(p.relative_to(TASK), needle[:16]))


class EpisodeSetTests(unittest.TestCase):
    def test_fixed_set_is_48_distinct_64_bit_seeds(self):
        seeds, key = verify.load_episodes(str(TASK / "tests/hidden/episodes.json"))
        self.assertEqual(len(seeds), 48)  # instruction.md: "48 hidden episodes"
        self.assertEqual(len(set(seeds)), len(seeds))
        self.assertTrue(all(2 ** 32 <= s < 2 ** 64 for s in seeds))
        self.assertTrue(0 < key < 2 ** 256)
        self.assertEqual(SET["format"], verify.SET_FORMAT)
        for e in SET["episodes"]:
            self.assertEqual(verify.world_digest(e["seed"]), e["world_sha256"])
        self.assertTrue("48 hidden episodes" in (TASK / "instruction.md").read_text(),
                        "instruction.md does not say '48 hidden episodes'")

    def test_the_set_decides_the_count(self):
        self.assertNotIn("episodes", json.loads((TASK / "tests/grader_config.json").read_text()))

    def test_a_changed_world_is_a_task_error(self):
        spec = copy.deepcopy(SET)
        spec["episodes"][3]["world_sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "episodes.json")
            with open(path, "w") as f:
                json.dump(spec, f)
            with self.assertRaises(SystemExit):
                verify.load_episodes(path)

    def test_the_selection_record_matches_the_set(self):
        kept = [c for c in sorted(SELECTION["candidates"], key=lambda c: c["draw"]) if c["status"] == "kept"]
        self.assertEqual([c["seed"] for c in kept], [e["seed"] for e in SET["episodes"]])
        self.assertEqual(SELECTION["probe_key"], SET["probe_key"])
        self.assertTrue(all(c["success"] and not c["aborted"] and c["inprocess_equal"] for c in kept))

    def test_probe_stream_depends_on_key_and_seed_only(self):
        seeds, key = verify.load_episodes(str(TASK / "tests/hidden/episodes.json"))
        a = verify.episode_stream(key, seeds[0])
        b = verify.seed_rng(entropy=(key << 64) | seeds[0])
        self.assertEqual([a.random() for _ in range(50)], [b.random() for _ in range(50)])
        c = verify.episode_stream(key, seeds[1])
        d = verify.episode_stream(key, seeds[0])
        self.assertNotEqual([c.random() for _ in range(5)], [d.random() for _ in range(5)])


def _env_vars(env):
    d = {k: copy.deepcopy(v) for k, v in vars(env).items() if k != "rng"}
    d["_rng_counter"] = env.rng._counter
    return d


class BlockTests(unittest.TestCase):
    SEEDS = [(1 << 33) + 7 * i for i in range(20)]

    def test_building_the_block_draws_nothing_and_changes_nothing(self):  # (I1)
        for seed in self.SEEDS[:5]:
            env = verify.FaultAdaptiveDocking()
            env.reset(seed)
            for t in range(env.max_steps):
                before = _env_vars(env)
                block = verify.privileged_block(env)
                self.assertEqual(_env_vars(env), before)
                self.assertEqual(block["format"], verify.PRIV_FORMAT)
                self.assertEqual(set(block["state"]),
                                 set(verify.PRIV_STATE + verify.PRIV_ROUTING + verify.PRIV_CONSTANTS))
                self.assertEqual(json.loads(json.dumps(block)), block)  # (floats round-trip exactly)
                if env.step(scripted(t, seed))[2]:
                    break

    def test_the_block_covers_what_step_reads_and_nothing_else_is_unclassified(self):
        sent = set(verify.PRIV_STATE + verify.PRIV_ROUTING + verify.PRIV_CONSTANTS)
        never = set(verify.NEVER_SENT_FUTURE + verify.NEVER_SENT_OBSERVATION + verify.NEVER_SENT_OUTCOME)
        self.assertFalse(sent & never)
        self.assertEqual(oracle.STATE, verify.PRIV_STATE)
        self.assertEqual(oracle.ROUTING, verify.PRIV_ROUTING)
        self.assertEqual(oracle.PRIV_FORMAT, verify.PRIV_FORMAT)
        tree = ast.parse((TASK / "tests/env.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "FaultAdaptiveDocking")
        methods = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
        class_attrs = {t.id for n in cls.body if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Name)}
        read = set()
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in ("step", "_plume_column", "goal_state", "port_radius",
                                                               "episode_quality"):
                for node in ast.walk(fn):
                    if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                            and node.value.id == "self" and isinstance(node.ctx, ast.Load)):
                        read.add(node.attr)
        read -= methods | class_attrs
        self.assertEqual(read - sent - never, set(), "read by the plant but neither sent nor withheld")
        self.assertEqual(sent - read, set(), "sent but never read by the plant")
        env = verify.FaultAdaptiveDocking()
        env.reset(self.SEEDS[0])
        for t in range(200):
            env.step(scripted(t, self.SEEDS[0]))
        self.assertEqual(set(vars(env)) - sent - never, set(), "an attribute of env.py that the block does not classify")

    def test_a_replica_of_the_block_predicts_the_next_block_exactly_after_the_rekey(self):
        counts = {}
        for seed in self.SEEDS:
            env = verify.FaultAdaptiveDocking()
            env.reset(seed)
            prev = None
            for t in range(env.max_steps):
                block = json.loads(json.dumps(verify.privileged_block(env)))["state"]
                if prev is not None:
                    pst, pu = prev
                    r = oracle.replica(pst)
                    r.step(pu)
                    ok = oracle.same_state(r, block)
                    if pst["rekey_active"]:
                        phase = "after_rekey"
                    elif pst["routing_active"]:
                        phase = "between" if not block["rekey_active"] else "rekey_transition"
                    else:
                        phase = "before_events" if not block["routing_active"] else "event_transition"
                    counts[(phase, ok)] = counts.get((phase, ok), 0) + 1
                u = scripted(t, seed)
                prev = (block, u)
                if env.step(u)[2]:
                    break
        for phase in ("before_events", "between", "after_rekey"):
            self.assertEqual(counts.get((phase, False), 0), 0, phase)
            self.assertGreater(counts.get((phase, True), 0), 0, phase)
        self.assertEqual(counts.get(("rekey_transition", True), 0), 0)
        self.assertEqual(counts.get(("event_transition", True), 0), 0)


class FakeSolver(object):
    """A scripted program on the wire: it records every request exactly as the kit serialises it and answers with an
    action computed from the observation (so the episode is non-trivial and every probe pair differs)."""

    def __init__(self):
        self.requests = []

    def reset(self):
        self.requests.append(json.dumps({"type": "reset"}))

    def act(self, t, obs):
        self.requests.append(json.dumps({"type": "act", "t": int(t), "obs": obs}))
        vec = obs["vector"] if isinstance(obs, dict) else obs
        return [abs(math.sin(3.0 * vec[0] + 2.0 * vec[1] + vec[2] + 0.7 * i)) * 0.4 for i in range(6)]


def old_episode(seed, solver, rng):
    """verify.py's episode loop before the fixed set and the channel (minus the seed draw), for (I3)."""
    lock = verify.ProbeLock(rng, rate=0.06, pass_fraction=0.35, tolerance=1e-4)
    env = verify.FaultAdaptiveDocking()
    obs = env.reset(seed)
    solver.reset()
    quality = 0.0
    for t in range(env.max_steps):
        if lock.due():
            action = lock.probe(solver, t, obs, perturbed=old_perturbed_observation(obs, rng))
        else:
            action = solver.act(t, obs)
        obs, quality, done, info = env.step(action)
        if done:
            break
    return quality, lock


class ProbeCopyTests(unittest.TestCase):
    def test_the_copy_draws_three_numbers_and_shifts_the_vector_as_before(self):  # (I2)
        env = verify.FaultAdaptiveDocking()
        obs = env.reset(SET["episodes"][0]["seed"])
        plain = verify.perturbed_observation(obs, verify.seed_rng(entropy=5))
        rng = verify.seed_rng(entropy=5)
        self.assertEqual(plain, old_perturbed_observation(obs, rng))
        self.assertEqual(rng._counter, 3)
        wire = {"vector": obs, "privileged": verify.privileged_block(env)}
        rng = verify.seed_rng(entropy=5)
        tok = verify.perturbed_observation(wire, rng)
        self.assertEqual(rng._counter, 3)
        self.assertEqual(tok["vector"], plain)
        ref = verify.seed_rng(entropy=5)
        dx, dy, dyaw = ref.uniform(-0.28, 0.28), ref.uniform(-0.28, 0.28), ref.uniform(-0.18, 0.18)
        st, st0 = tok["privileged"]["state"], wire["privileged"]["state"]
        self.assertEqual(st["x"], st0["x"] + dx)
        self.assertEqual(st["y"], st0["y"] + dy)
        self.assertEqual(st["theta"], (st0["theta"] + dyaw + math.pi) % (2.0 * math.pi) - math.pi)
        self.assertEqual({k: v for k, v in st.items() if k not in ("x", "y", "theta")},
                         {k: v for k, v in st0.items() if k not in ("x", "y", "theta")})
        self.assertEqual(tok["privileged"]["format"], verify.PRIV_FORMAT)
        self.assertEqual(wire["privileged"]["state"]["x"], st0["x"])  # (the real block is untouched)

    def test_both_modes_consume_the_lock_stream_identically(self):  # (I2)
        seed, key = SET["episodes"][1]["seed"], int(SET["probe_key"], 16)
        counters = []
        for privileged in (False, True):
            lock = verify.ProbeLock(None, rate=0.06, pass_fraction=0.35, tolerance=1e-4)
            lock.rng = verify.episode_stream(key, seed)
            _, _, steps = verify.run_episode(seed, FakeSolver(), lock, privileged)
            counters.append((lock.rng._counter, lock.total, steps))
        self.assertEqual(counters[0], counters[1])
        self.assertEqual(counters[0][0], counters[0][2] + 4 * counters[0][1])  # 1 per step + 3 + 1 per probe

    def test_an_untokened_run_sends_the_requests_it_sent_before(self):  # (I3)
        key = int(SET["probe_key"], 16)
        for e in SET["episodes"][:3]:
            old, new = FakeSolver(), FakeSolver()
            q_old, lock_old = old_episode(e["seed"], old, verify.episode_stream(key, e["seed"]))
            lock = verify.ProbeLock(None, rate=0.06, pass_fraction=0.35, tolerance=1e-4)
            lock.rng = verify.episode_stream(key, e["seed"])
            q_new, _, _ = verify.run_episode(e["seed"], new, lock, False)
            self.assertEqual(old.requests, new.requests)
            self.assertEqual(q_old, q_new)
            self.assertEqual((lock_old.total, lock_old.passed), (lock.total, lock.passed))


class CopyTests(unittest.TestCase):
    def test_the_reference_s_simulator_copy_is_the_grader_s(self):
        self.assertEqual((TASK / "tests/env.py").read_bytes(),
                         (TASK / "solution/privileged/spacecraft_docking_env.py").read_bytes())

    def test_the_launcher_is_the_mechanism_box_one(self):
        self.assertEqual((TASK / "tests/nonet_exec.py").read_bytes(), MB_NONET.read_bytes())

    def test_the_planning_quota_covers_the_family(self):
        worst = len(oracle.FAMILY) * (oracle.STEPS - 118) + oracle.PLAN_OFFSET  # (t_p >= 108 + 2 + PLAN_OFFSET)
        self.assertGreaterEqual(oracle.PLAN_OFFSET * oracle.QUOTA, worst)


class AgentVisibleTests(unittest.TestCase):
    """The agent is docked zero-shot (the owner, 2026-11-05): it gets the plant model as a document, not the grader's
    simulator and not a grader of its own on visible episodes."""

    PAYLOAD = TASK / "environment/setup/payload"
    MODEL = PAYLOAD / "given/model.md"

    def visible(self):
        return [TASK / "instruction.md"] + sorted(p for p in (TASK / "environment").rglob("*") if p.is_file())

    def test_the_payload_is_the_model_document_and_the_instruction_points_to_it(self):
        files = sorted(p for p in self.PAYLOAD.rglob("*") if p.is_file())
        self.assertIn(self.MODEL, files)
        for p in files:  # documents only: nothing in the agent's home runs the plant or grades a controller
            self.assertEqual(p.suffix, ".md", "{} is not a document".format(p.relative_to(TASK)))
        self.assertIn("/home/user/given/model.md", (TASK / "instruction.md").read_text())

    def test_no_agent_visible_file_carries_the_simulator_or_a_practice_grader(self):
        code = sorted({line.strip() for line in (TASK / "tests/env.py").read_text().splitlines()
                       if "self." in line and len(line.strip()) >= 20})
        self.assertGreater(len(code), 150)
        names = ("FaultAdaptiveDocking", "spacecraft_docking_env", "practice", "grade.py", "env.py", "_RNG",
                 "blake2b")
        for p in self.visible():
            text = p.read_text(errors="replace")
            where = p.relative_to(TASK)
            for name in names:
                self.assertNotIn(name.lower(), text.lower(), "{} names {}".format(where, name))
            for line in code:
                self.assertNotIn(line, text, "{} carries the simulator's line {!r}".format(where, line))

    def test_the_model_states_the_plant_s_constants(self):
        text = self.MODEL.read_text()
        for constant in ("0.235", "0.205", "0.132", "0.148", "0.998", "0.996", "0.00034", "0.035", "0.012", "0.008",
                         "0.007", "0.006", "0.004", "0.28", "0.18", "1.35", "0.78", "0.12", "0.65", "0.82", "1.14",
                         "6.2", "0.085", "0.060", "0.020", "0.015", "30 + ((K1 + D) mod 13)", "0.24 + 0.028 n",
                         "(0.92, 0.98, 1.04, 1.10, 1.16, 1.20)", "U{58..103}", "U{108..125}", "U{148..165}",
                         "U(1.18, 1.30)", "U{48..81}", "U{7..13}", "U{112..157}", "U{10..18}", "U{1..3}"):
            self.assertIn(constant, text)


class NetworkGuardTests(unittest.TestCase):
    """The set is public: where the seccomp filter cannot be installed, nothing is graded with the network open."""

    def run_main(self, mode, solver):
        with tempfile.TemporaryDirectory() as tmp:
            sub = os.path.join(tmp, "submission")
            os.makedirs(sub)
            with open(os.path.join(sub, "run.sh"), "w") as f:
                f.write("echo READY\n")
            with mock.patch.object(verify, "egress_mode", return_value=mode), \
                    mock.patch.object(verify, "GuardedSolver", side_effect=solver) as started, \
                    mock.patch.object(verify, "write_verdict") as written, \
                    mock.patch.object(verify, "own_submission"), \
                    mock.patch.object(verify, "SUBMISSION", sub), \
                    mock.patch.dict(os.environ, {"ALE_STAGE_DIR": str(TASK / "tests")}), \
                    mock.patch("sys.stderr", new_callable=io.StringIO) as err:
                code = None
                try:
                    verify.main()
                except SystemExit as exc:
                    code = exc.code
        return code, started, written, err.getvalue()

    def test_without_the_filter_the_verify_is_a_task_error_and_no_controller_starts(self):
        code, started, written, err = self.run_main("unavailable", AssertionError("a controller started"))
        self.assertEqual(code, 1)
        started.assert_not_called()
        written.assert_not_called()
        self.assertIn("TASK ERROR: the grading environment cannot block the controller's network", err)

    def test_with_the_filter_the_verdict_records_it(self):
        code, started, written, _ = self.run_main("seccomp", verify.SolverMissing("no submission"))
        self.assertIsNone(code)
        started.assert_called_once()
        metrics = written.call_args[0][0]["metrics"]
        self.assertEqual(metrics["egress_guard_seccomp"], 1.0)
        self.assertNotIn("egress_detected", metrics)


class DeterminismCanaryTests(unittest.TestCase):
    def test_set_episode_0_reproduces_its_recorded_quality(self):
        kept = [c for c in sorted(SELECTION["candidates"], key=lambda c: c["draw"]) if c["status"] == "kept"]
        first = kept[0]
        self.assertEqual(first["seed"], SET["episodes"][0]["seed"])
        got = oracle_batch.run_one(first["seed"], SET["probe_key"])
        self.assertTrue(got["success"])
        self.assertEqual((got["steps"], got["probes"], got["probes_passed"]),
                         (first["steps"], first["probes"], first["probes_passed"]))
        if sys.version_info >= (3, 12):
            self.assertEqual(got["quality"], first["quality"])
        else:  # (before 3.12 the builtin sum is not compensated: fuel can differ in its last bit)
            self.assertAlmostEqual(got["quality"], first["quality"], delta=1e-12)


if __name__ == "__main__":
    unittest.main()
