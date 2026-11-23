"""Private, post-agent arm inputs. Nothing here is installed in the live service."""
import hashlib
import secrets

import numpy as np

from arm_evaluation import reconstruction_cases, transfer_plan
from robotics_grader import seed_rng
from world import LIMITS

SCHEMA = "arm-crossed-belt-replay-v1"


def probe_seeds(plan, master):
    """One fresh held-back key; streams are bound to scene, not consumption/order."""
    key = bytes.fromhex(master)
    if len(key) != 32:
        raise ValueError("expected a 256-bit private probe key")
    return [hashlib.blake2b(int(seed).to_bytes(8, "little"), key=key,
                            person=b"arm-v7-probe", digest_size=32).hexdigest()
            for seed, _ in plan]


def array_hash(x):
    return hashlib.sha256(np.asarray(x, dtype="<f8").tobytes()).hexdigest()


def prepare_arm_inputs(scene_seed, recorded=None):
    if recorded is None:
        stream = seed_rng()
        stimulus = stream.seed(64)
        plan = [[s, list(f)] for s, f in transfer_plan(stream, scene_seed)]
        master = secrets.token_bytes(32).hex()
    else:
        stimulus, plan, master = (recorded[k] for k in ("stimulus_seed", "transfer_plan", "probe_master_hex"))
    cases = reconstruction_cases(scene_seed, stimulus)
    receipt = dict(schema=SCHEMA, exploration_seed=scene_seed, stimulus_seed=stimulus,
                   arm_action_cap=LIMITS["arm"], transfer_plan=plan, probe_master_hex=master,
                   probe_episode_hex=probe_seeds(plan, master),
                   reconstruction=[dict(actions=actions, actions_f64le_sha256=array_hash(actions),
                                        targets_f64le_sha256=array_hash(expected))
                                   for actions, expected in cases])
    if recorded is not None and recorded != receipt:
        raise RuntimeError("recorded arm inputs do not reproduce on this verifier/image")
    return receipt, cases
