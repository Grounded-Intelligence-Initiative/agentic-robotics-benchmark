"""alelab plugin: the practice cell of mixed_box_palletizing.

The lab holds the palletizing cell (pallet_cell.py, the file the grader runs) with the practice
cartons and the three practice orders (practice.json). The agent never reads either file: it
meets the cell only through this service, as observations.

A trial is one order: `reset` with the order's id starts it with a fresh seed (printing, arrival
poses, lighting, sensor noise), `step` advances one control step. Replies carry the observation
the grading wire carries, plus `result`: -1 while the order runs and, in the reply to the step
that ends it, the order's shipped volume fraction after the tilt test.
"""

import secrets

import pallet_cell as pc

_IMAGE = {"rgb": "u8[*,*,3]", "depth": "u8[*,*,2]"}
OBS_SCHEMA = {
    "t": "int",
    "q": "f64[4]",
    "dq": "f64[4]",
    "tcp": "f64[4]",
    "wrench": "f64[6]",
    "vacuum": "int",
    "sealed": "int",
    "in_reach": "int",
    "infeed_present": "int",
    "delivered": "int",
    "total": "int",
    "steps_left": "int",
    "images": {name: dict(_IMAGE) for name in pc.CAMERAS},
    "result": "float",
}
MESSAGES = {
    "action": "action must be a list of 7 finite numbers [vx, vy, vz, wyaw, vacuum, cams (integer 0-7), done]",
}


def parse_action(action):
    """pallet_cell.parse_action: the one rule for the cell, the lab and the grader."""
    return pc.parse_action(action)


class _Episode(object):
    def __init__(self, cell):
        self.cell = cell

    def step(self, action):
        obs = self.cell.step(action, raw=True)
        obs["result"] = -1.0
        if self.cell.done:
            obs["result"] = float(self.cell.finish()["shipped_volume_fraction"])
        return obs, bool(self.cell.done)


class PalletLab(object):
    def __init__(self, instances, api):
        self.api = api
        self.catalog = dict(instances["catalog"])
        self.orders = {key: list(value) for key, value in instances["orders"].items()}
        self.SCHEMA = {"info": {"control_dt": "float", "steps_per_carton": "int", "steps_fixed": "int"},
                       "reset": OBS_SCHEMA, "step": OBS_SCHEMA}
        self.MESSAGES = MESSAGES

    def instance_ids(self):
        return sorted(self.orders)

    def info(self):
        return {"control_dt": pc.CONTROL_DT, "steps_per_carton": pc.STEPS_PER_CARTON, "steps_fixed": pc.STEPS_FIXED}

    def validate_action(self, action):
        # depends only on the request (never on the order or on the cell's state)
        try:
            return parse_action(action)
        except ValueError:
            raise self.api.BadRequest("action")

    def reset(self, instance):
        cell = pc.PalletCell(self.catalog)
        obs = cell.reset(self.orders[instance], secrets.randbits(62), raw=True)
        obs["result"] = -1.0
        return _Episode(cell), obs


def make_plugin(instances, api):
    return PalletLab(instances, api)
