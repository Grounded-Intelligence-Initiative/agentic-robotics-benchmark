"""alelab plugin: the real hidden boxes (same robot environment as the agent's dev sim, box_env.py,
with the hidden box models built from the root-only instance file by mech.py).

Instance file (environment/setup/ref/boxes.json): {box id: box config}. Lab instances = the
scored boxes. Every trial starts from a fresh 64-bit seed drawn from OS entropy (box placement
jitter and sensor noise). Episodes never end early (the reply does not reveal when a box is fully
open); replies carry the observation only (images, proprioception, wrench), never box state.

The graded episode (graded_reset; the episode file environment/setup/online/episode.json names the box and its
fixed seed) runs exactly as the offline grader runs a hidden episode: early_end=True, its score is
env.episode_score() (0 after a simulation fault).
"""

import secrets

import box_env as E
import mech

_U8 = "u8[*,*,3]"
OBS_SCHEMA = {
    "t": "int",
    "box": "str",
    "q": "f64[7]",
    "dq": "f64[7]",
    "ee_pos": "f64[3]",
    "ee_quat": "f64[4]",
    "gripper": "float",
    "wrench": "f64[6]",
    "T_world_wrist": "f64[4,4]",
    "left": _U8,
    "right": _U8,
    "wrist": _U8,
}
MESSAGES = {
    "action": "action must be a list of 8 finite numbers (6 motion values, grip, camera mask 0-7)",
}


def _schema(ids):
    s = dict(OBS_SCHEMA)
    s["box"] = "enum:" + "|".join(ids)
    return {
        "info": {"control_dt": "float", "max_steps": "int"},
        "reset": s,
        "step": s,
    }


class _Episode(object):
    def __init__(self, env):
        self.env = env

    def step(self, action):
        obs, done, _info = self.env.step(action)
        return obs, bool(done)


class _GradedEpisode(object):
    """The graded episode, run as the offline grader (tests/verify.py on main) runs a hidden episode:
    BoxEnv(..., early_end=True), so it ends as soon as all of the box's stages are done, after 2500 steps, or at a
    simulation fault (which scores 0)."""

    def __init__(self, env, box_id):
        self.env, self.box = env, box_id

    def step(self, action):
        obs, done, _info = self.env.step(action)
        return obs, bool(done)

    def result(self):
        """The grader's score of one episode were it to end now: env.episode_score(), 0 after a fault. Counted
        stages stay counted and nothing is simulated after the end, so an early end (a finish, the window, the
        session) scores what is counted at that moment. The sub-metrics are the grader's for one episode."""
        env = self.env
        fault = env.fault is not None
        score = 0.0 if fault else float(env.episode_score())
        metrics = {"score_" + self.box: score, "sim_faults": 1.0 if fault else 0.0}
        for (st, _qa), ok in zip(env.stages, env.achieved):
            metrics["stage_%s_%s" % (self.box, st["id"])] = 1.0 if (ok and not fault) else 0.0
        return {"score": score, "metrics": metrics, "detail": {"t": int(env.t), "fault": env.fault}}

    def finish(self):
        return self.result()

    def close(self):
        self.env.close()


class BoxLab(object):
    def __init__(self, instances, api):
        self.api = api
        self.specs = {}
        for bid, cfg in sorted(instances.items()):
            if bid.startswith("_"):
                continue
            box = mech.Box(cfg)
            self.specs[bid] = {"xml": box.xml(), "meta": box.meta()}
        self.SCHEMA = _schema(sorted(self.specs))
        self.MESSAGES = MESSAGES

    def instance_ids(self):
        return sorted(self.specs)

    def info(self):
        return {"control_dt": E.CONTROL_DT, "max_steps": E.MAX_STEPS}

    def validate_action(self, action):
        # depends only on the request and public constants (never on the instance or hidden state)
        try:
            E.parse_action(action)
        except E.ActionError:
            raise self.api.BadRequest("action")
        return [float(v) for v in action]

    def reset(self, instance):
        env = E.BoxEnv(self.specs[instance], box_id=instance, early_end=False)
        obs = env.reset(secrets.randbits(64))
        return _Episode(env), obs

    def graded_reset(self, instance, seed):
        """The graded episode: the hidden box `instance` at the fixed 64-bit `seed` (placement and sensor noise)."""
        env = E.BoxEnv(self.specs[instance], box_id=instance, early_end=True)
        obs = env.reset(seed)
        return _GradedEpisode(env, instance), obs


def make_plugin(instances, api):
    return BoxLab(instances, api)
