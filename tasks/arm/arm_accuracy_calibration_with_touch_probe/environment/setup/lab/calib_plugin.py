"""alelab plugin: the calibration cells of arm_accuracy_calibration_with_touch_probe.

The lab holds every unit (calib_cell.py and units.json, root-only): the practice unit and the
units that are graded. The agent meets a unit only through this service.

A trial is one session on one unit: `reset` with the unit's id starts a measuring session on it (arm at
home, cold, probe_a mounted, a fresh budget of time, probes and images), `reset` with the unit's id and
SERVICE_SUFFIX ("u1-serviced") a re-qualification session on a graded unit after its service
(calib_cell.serviced: one joint's drive replaced; the same start, the budget cell_data.BUDGET_SERVICE, which has
no images: the wrist camera is off, and an image command is refused by the cell as no_images, nothing charged);
`step` carries one command. The practice unit is not serviced: it has no re-qualification instance (no
"p1-serviced" in instance_ids, so the lab answers a request naming one bad_instance, before any trial is charged).
A unit has a fixed number of sessions of each kind (SESSIONS: one measuring session for a graded unit,
cell_data.PRACTICE_SESSIONS (one) for the practice unit; one re-qualification session for a graded unit), counted when a
session's first command arrives and kept in the lab's own state folder, so that neither a new connection nor a
restarted lab gives one back; a session beyond that number answers every command with refused =
no_session_left. The service comes between the two kinds: once a unit's re-qualification session has been
charged, the unit is serviced for good, and every command of a measuring session on it (one still open on
another connection, or a new one) is refused as `serviced`, nothing charged.

There is no acceptance check before grading (cell_data.PRACTICE_CHECKS = 0): acceptance is the customer's
inspection (the grader) after delivery, and the lab has no acceptance rig. The plugin has no `execute` method and
no SCHEMA entry for it, so the server does not offer the op: an `execute` request (with exactly the keys op,
instance, plan) is answered `unsupported` before anything is charged. Nothing about the predictions or the job
programs is reported before grading. The lab's budget (lab.json: 13) is the six graded units' measuring and
re-qualification sessions and the practice unit's one measuring session.

Commands (the `action` of a step), all JSON objects:
    {"do": "move", "q": [6 numbers], "speed": number}     unguarded joint move
    {"do": "probe", "q": [6 numbers]}                      guarded joint move at probing speed
    {"do": "tool", "name": "probe_a" | "probe_b"}          change the tool (at the home pose)
    {"do": "wait", "seconds": number}
    {"do": "image"}                                         one grey image from the wrist camera

Every reply carries `image`: the picture after an image command, a 1 x 1 array otherwise.
"""
import json
import os

import numpy as np

import arm_model as am
import calib_cell as cc

OBS = {"q": "f64[6]", "temp": "f64[6]", "tool": "enum:" + "|".join(cc.MOUNTABLE), "time_s": "float",
       "time_left_s": "float", "probes_used": "int", "probes_left": "int", "images_used": "int",
       "images_left": "int", "damaged": "bool", "image": "u8[*,*]",
       "triggered": "bool", "collided": "bool", "refused": "enum:no|damaged|time_up|no_probes|no_images|not_home|no_session_left|serviced"}
NO_IMAGE = np.zeros((1, 1), np.uint8)
MESSAGES = {
    "action": "not a command: {do: move, q, speed} | {do: probe, q} | {do: tool, name} | {do: wait, seconds} | "
              "{do: image}; q six numbers inside the joint limits",
}
FIELDS = {"move": {"do", "q", "speed"}, "probe": {"do", "q"}, "tool": {"do", "name"}, "wait": {"do", "seconds"},
          "image": {"do"}}
REFUSALS = (("damaged", "damaged"), ("time is used up", "time_up"), ("no probes", "no_probes"), ("no images", "no_images"),
            ("home pose", "not_home"))


def _number(value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise ValueError
    return value


def parse_action(action):
    if not isinstance(action, dict) or action.get("do") not in FIELDS or set(action) != FIELDS[action["do"]]:
        raise ValueError
    do = action["do"]
    if do in ("move", "probe"):
        q = action["q"]
        if not isinstance(q, list) or len(q) != 6:
            raise ValueError
        q = [_number(v, float(am.Q_MIN[j]), float(am.Q_MAX[j])) for j, v in enumerate(q)]
        if do == "probe":
            return ("probe", q)
        return ("move", q, _number(action["speed"], 1e-3, cc.MOVE_SPEED_MAX))
    if do == "tool":
        if action["name"] not in cc.MOUNTABLE:
            raise ValueError
        return ("tool", action["name"])
    if do == "image":
        return ("image",)
    return ("wait", _number(action["seconds"], 1e-3, 600.0))


assert cc.PRACTICE_CHECKS == 0, "the lab implements no acceptance check"
SESSIONS = {"practice": cc.PRACTICE_SESSIONS, "graded": 1, "serviced": 1}
SERVICE_SUFFIX = "-serviced"        # a unit's re-qualification session: reset("u1" + SERVICE_SUFFIX)


class _Ledger(object):
    """Sessions used per unit, on disk in the lab's state folder (written by the lab account)."""

    def __init__(self, path):
        self.path = path

    def read(self):
        try:
            with open(self.path) as handle:
                data = json.load(handle)
            return {str(k): int(v) for k, v in data.items()}
        except FileNotFoundError:
            return {}

    def charge(self, uid):
        """The number of this session (1, 2, ...) for the unit; persisted before anything is measured."""
        data = self.read()
        data[uid] = data.get(uid, 0) + 1
        tmp = self.path + ".tmp"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        try:
            os.write(fd, json.dumps(data).encode())
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, self.path)
        return data[uid]


class _Session(object):
    def __init__(self, lab, iid):
        self.lab, self.iid, self.spent = lab, iid, False
        self.service = iid.endswith(SERVICE_SUFFIX)
        self.uid = iid[:-len(SERVICE_SUFFIX)] if self.service else iid
        self.refusal = "no_session_left"
        self.cell = self._untouched()

    def _untouched(self):
        if self.service:
            return cc.Cell(cc.serviced(self.lab.unit(self.uid)), session=0, budget=cc.BUDGET_SERVICE)
        return cc.Cell(self.lab.unit(self.uid), session=0)

    def _arm(self):
        """The first command of a session charges the unit; False when the unit has none of this kind left (or,
        for a measuring session, when the unit has been serviced)."""
        if not self.service and self.lab.is_serviced(self.uid):
            self.cell, self.refusal = None, "serviced"
            return False
        if not self.spent:
            self.spent = True
            if self.service:
                number = self.lab.ledger.charge(self.iid)
                self.lab.serviced.add(self.uid)
                self.cell = (cc.Cell(cc.serviced(self.lab.unit(self.uid)), session=("service", number),
                                     budget=cc.BUDGET_SERVICE) if number <= SESSIONS["serviced"] else None)
            else:
                number = self.lab.ledger.charge(self.uid)
                self.cell = cc.Cell(self.lab.unit(self.uid), session=number) if number <= self.lab.allowed(self.uid) else None
        return self.cell is not None

    def observation(self, refused="no", **extra):
        cell = self.cell if self.cell is not None else self._untouched()
        obs = cell.observation(triggered=False, collided=False, image=NO_IMAGE)
        obs.update(extra)
        obs["refused"] = refused
        return obs

    def step(self, command):
        if not self._arm():
            return self.observation(refused=self.refusal), False
        cell = self.cell
        try:
            if command[0] == "move":
                obs = cell.move(np.array(command[1]), command[2])
            elif command[0] == "probe":
                obs = cell.probe(np.array(command[1]))
            elif command[0] == "tool":
                obs = cell.change_tool(command[1])
            elif command[0] == "image":
                obs = cell.image()
            else:
                obs = cell.wait(command[1])
        except cc.CellError as exc:
            text = str(exc)
            code = next((code for needle, code in REFUSALS if needle in text), None)
            if code is None:
                raise
            return self.observation(refused=code), False
        obs.setdefault("triggered", False)
        obs.setdefault("collided", False)
        obs.setdefault("image", NO_IMAGE)
        obs["refused"] = "no"
        return obs, False


class CalibLab(object):
    def __init__(self, instances, api):
        self.api = api
        self.units = dict(instances["practice"])
        self.practice = set(self.units)
        self.units.update(instances["graded"])
        self.ledger = _Ledger(str(api.options.get("ledger", "/var/lib/ale-lab/sessions.json")))
        # the units serviced so far (their re-qualification session charged): kept in the ledger, read back here
        self.serviced = {uid for uid in self.units if self.ledger.read().get(uid + SERVICE_SUFFIX, 0) > 0}
        self.SCHEMA = {"info": {"time_s": "float", "probes": "int", "images": "int"}, "reset": OBS, "step": OBS}
        self.MESSAGES = MESSAGES
        self.VALIDATION_PROBES = {
            "action": [{"do": "move", "q": [0.0] * 6, "speed": 1.0}, {"do": "probe", "q": [0.0] * 6},
                       {"do": "probe", "q": [9.0] * 6}, {"do": "tool", "name": "probe_b"},
                       {"do": "tool", "name": "process_c"}, {"do": "wait", "seconds": 5}, {"do": "wait"}, {"do": "image"}, {"do": "image", "q": 1}],
        }

    def instance_ids(self):
        # a re-qualification instance for every graded unit only: the practice unit is not serviced
        return sorted(list(self.units) + [uid + SERVICE_SUFFIX for uid in self.units if uid not in self.practice])

    def is_serviced(self, uid):
        return uid in self.serviced

    def info(self):
        return {"time_s": float(cc.BUDGET["time_s"]), "probes": int(cc.BUDGET["probes"]),
                "images": int(cc.BUDGET["images"])}

    def validate_action(self, action):
        try:
            return parse_action(action)
        except (ValueError, TypeError, KeyError):
            raise self.api.BadRequest("action")

    def reset(self, instance):
        # Nothing is counted here: the unit is charged when the session's first command arrives. The noise of
        # a session depends on the unit and on the session's number, so a session can be reproduced.
        session = _Session(self, instance)
        return session, session.observation()

    def unit(self, uid):
        return cc.draw_unit(self.units[uid])

    def allowed(self, uid):
        return SESSIONS["practice" if uid in self.practice else "graded"]


def make_plugin(instances, api):
    return CalibLab(instances, api)
