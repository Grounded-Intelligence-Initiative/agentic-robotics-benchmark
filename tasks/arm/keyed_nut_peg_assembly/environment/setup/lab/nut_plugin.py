"""alelab plugin: the real hidden nuts (same robot environment as the agent's dev sim, keyed_env.py,
with the hidden nut specs from the root-only instance file).

Instance file (environment/setup/ref/nuts.json): {nut id: nut spec}. Lab instances = the scored
nuts. Every trial starts from a fresh 64-bit seed drawn from OS entropy (peg and nut layout, sensor
noise) and a target peg drawn uniformly from the three band counts. Episodes never end early (the
reply does not reveal when the nut is seated); replies carry the observation only (images,
proprioception, wrench), never object state.
"""

import secrets

import keyed_env as E

_U8 = "u8[*,*,3]"
OBS_SCHEMA = {
    "t": "int",
    "nut": "str",
    "peg_bands": "int",
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
    s["nut"] = "enum:" + "|".join(ids)
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


class NutLab(object):
    def __init__(self, instances, api):
        self.api = api
        self.specs = {}
        for nid, spec in sorted(instances.items()):
            if nid.startswith("_"):
                continue
            self.specs[nid] = E.check_nut_spec(spec)
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
        env = E.NutEnv(self.specs[instance], early_end=False)
        obs = env.reset(secrets.randbits(64), 1 + secrets.randbelow(3))
        return _Episode(env), obs


def make_plugin(instances, api):
    return NutLab(instances, api)
