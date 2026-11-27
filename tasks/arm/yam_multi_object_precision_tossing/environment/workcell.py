"""One persistent two-arm simulation and its raw per-side outcome counts."""

from dataclasses import dataclass
import math

import mujoco as mj
import numpy as np

from arena import PRECISION_LAYOUT, TABLE_HEIGHT, build_arena, preview_data
from station import DT, SLEW, dimensions, envelope, scene


MAX_ACTIONS = 20
DURATION = 6.0
RECORD_EVERY = 100
KP = np.array([100, 150, 130, 58, 40, 35])
KD = np.array([7, 10, 8, 3, 2, 2])


class NumericalPhysicsError(RuntimeError):
    """The physical rollout is invalid and cannot be scored."""


def check_physics(data, previous_time=None):
    for kind in (mj.mjtWarning.mjWARN_BADQPOS, mj.mjtWarning.mjWARN_BADQVEL,
                 mj.mjtWarning.mjWARN_BADQACC):
        warning = data.warning[kind]
        if warning.number:
            raise NumericalPhysicsError(
                f"MuJoCo {kind.name} at index {warning.lastinfo}; time {data.time:.9g}")
    if not math.isfinite(data.time) or (previous_time is not None and data.time <= previous_time):
        raise NumericalPhysicsError("MuJoCo simulation time failed to advance")
    if not all(np.isfinite(values).all() for values in (data.qpos, data.qvel, data.qacc)):
        raise NumericalPhysicsError("MuJoCo state contains a non-finite value")


@dataclass
class Item:
    id: str
    arm: str
    mode: str
    variant: str
    label: str
    target: str
    body: int
    geoms: frozenset
    attempts: int = 0
    landed: bool = False


class ArenaRound:
    def __init__(self, seed, record=False):
        self.model, _ = build_arena(seed)
        self.data = preview_data(self.model)
        self._sensor = None
        self.arms = {}
        for side in ("left", "right"):
            joints = [self.model.joint(f"{side}_joint{i}") for i in range(1, 9)]
            positions = np.array([joint.qposadr[0] for joint in joints])
            self.arms[side] = {
                "qpos": positions,
                "dof": np.array([joint.dofadr[0] for joint in joints[:6]]),
                "finger_dof": np.array([joint.dofadr[0] for joint in joints[6:]]),
                "drive": np.array([self.model.actuator(f"{joint.name}_drive").id for joint in joints[:7]]),
                "range": np.array([joint.range for joint in joints[:6]]),
                "pads": {self.model.geom(f"{side}_tip_{finger}_pad").id for finger in ("left", "right")},
                "ref": self.data.qpos[positions[:6]].copy(),
                "jaw": float(self.data.qpos[positions[6]]),
            }
        self.items = {}
        for obj in scene(seed)["objects"]:
            color = obj["color"]
            ident = f"left_item{obj['id']}"
            self.add_item(ident, "left", "sorting", color,
                          f"{color} {obj['shape']}", f"left_{color}_bottom")
        for mode, size, *_ in PRECISION_LAYOUT:
            name = "wood" if mode == "wood" else f"{mode}_{size}"
            target = {
                "wood": "bin_floor", "ring": "post", "slot": "slot_floor"
            }[mode]
            self.add_item(f"right_{name}_play_piece", "right", mode, size,
                          name.replace("_", " "), f"right_{name}_{target}")
        self.actions = []
        self.frames = [] if record else None
        self.capture(-1)

    def add_item(self, ident, arm, mode, variant, label, target):
        body = self.model.body(ident).id
        geoms = frozenset(int(i) for i in np.flatnonzero(self.model.geom_bodyid == body)
                          if self.model.geom_contype[i])
        self.items[ident] = Item(ident, arm, mode, variant, label, target, body, geoms)

    def capture(self, action):
        if self.frames is not None:
            self.frames.append((float(self.data.time), action, self.data.qpos.astype(np.float32)))

    def observation(self):
        """Expose synchronized camera pixels and measured robot proprioception."""
        from rgbd_sensor import RGBDSensor

        mj.mj_forward(self.model, self.data)
        check_physics(self.data)
        if self._sensor is None:
            self._sensor = RGBDSensor(self.model)
        return {
            "time": float(self.data.time),
            "arms": {
                name: {
                    "joints": self.data.qpos[arm["qpos"][:6]].tolist(),
                    "joint_velocity": self.data.qvel[arm["dof"]].tolist(),
                    "finger_positions": self.data.qpos[arm["qpos"][6:]].tolist(),
                    "finger_velocity": self.data.qvel[arm["finger_dof"]].tolist(),
                    "reference": arm["ref"].tolist(),
                    "jaw_reference": arm["jaw"],
                }
                for name, arm in self.arms.items()
            },
            "cameras": self._sensor.observe(self.data),
        }

    def close(self):
        if self._sensor is not None:
            self._sensor.close()
            self._sensor = None

    def validate_action(self, action):
        if not isinstance(action, dict) or not isinstance(action.get("item"), str):
            raise ValueError("expected a named arena item")
        item = self.items.get(action["item"])
        if item is None or action.get("arm") != item.arm:
            raise ValueError("item must be handled by the arm on its table")
        arm = self.arms[item.arm]
        try:
            rows = np.asarray(action["waypoints"], dtype=float)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError("invalid waypoints") from exc
        if (rows.ndim != 2 or rows.shape[1] != 8 or not 2 <= len(rows) <= 60
                or not np.isfinite(rows).all() or rows[0, 0] != 0
                or np.any(np.diff(rows[:, 0]) <= 0) or rows[-1, 0] > DURATION
                or np.any(rows[:, 1:7] < arm["range"][:, 0] - 1e-5)
                or np.any(rows[:, 1:7] > arm["range"][:, 1] + 1e-5)
                or np.any((rows[:, 7] < 0) | (rows[:, 7] > .04695))):
            raise ValueError("invalid waypoints")
        return item, arm, rows

    def drive(self, arm, target, jaw):
        data = self.data
        arm["ref"] += np.clip(target - arm["ref"], -SLEW * DT, SLEW * DT)
        data.ctrl[arm["drive"][:6]] = (
            KP * (arm["ref"] - data.qpos[arm["qpos"][:6]])
            - KD * data.qvel[arm["dof"]] + data.qfrc_bias[arm["dof"]]
        )
        data.ctrl[arm["drive"][6]] = jaw

    def contacts(self, item):
        pairs = self.data.contact.geom
        return {int(b) for a, b in pairs if a in item.geoms and b not in item.geoms} | {
            int(a) for a, b in pairs if b in item.geoms and a not in item.geoms
        }

    def valid_receiver(self, item, neighbors):
        if not neighbors:
            return False
        prefix = item.target.rsplit("_", 1)[0] + "_"
        for index in neighbors:
            name = self.model.geom(index).name or ""
            if name.startswith(prefix) and not name.endswith(("_stand", "_base", "_pedestal")):
                continue
            if item.mode == "ring" and name == item.target.replace("_post", "_post_base"):
                continue
            if item.mode == "sorting" and any(
                index in other.geoms and other.variant == item.variant and other.landed
                for other in self.items.values() if other is not item and other.mode == "sorting"
            ):
                continue
            return False
        return True

    def bounds(self, item, target, rotation):
        lower = np.full(3, np.inf)
        upper = np.full(3, -np.inf)
        for index in item.geoms:
            geom = self.model.geom(index)
            position = rotation.T @ (self.data.geom_xpos[index] - target)
            axes = rotation.T @ self.data.geom_xmat[index].reshape(3, 3)
            kind = mj.mjtGeom(int(geom.type[0])).name.removeprefix("mjGEOM_").lower()
            if kind == "capsule":
                extent = np.abs(axes[:, 2]) * geom.size[1] + geom.size[0]
            else:
                extent = envelope(kind, geom.size, axes)
            lower = np.minimum(lower, position - extent)
            upper = np.maximum(upper, position + extent)
        return lower, upper

    def geometry_ok(self, item):
        from station_precision import size_geometry

        data = self.data
        target = data.geom(item.target)
        center = data.xpos[item.body]
        orientation = data.xmat[item.body].reshape(3, 3)
        speed = data.cvel[item.body]
        if np.linalg.norm(speed[3:]) > .09 or np.linalg.norm(speed[:3]) > 1.2:
            return False

        if item.mode == "ring":
            dims = size_geometry("ring", item.variant)
            axis = orientation[:, 2]
            if abs(axis[2]) < .8:
                return False
            half_height = self.model.geom(item.target).size[1]
            offset = abs(center[2] - target.xpos[2])
            return (offset < half_height + .016
                    and np.linalg.norm(center[:2] - target.xpos[:2])
                    + dims["post_radius"] / abs(axis[2]) <= dims["hole_radius"] + .002)


        rotation = target.xmat.reshape(3, 3)
        lower, upper = self.bounds(item, target.xpos, rotation)
        span = self.model.geom(item.target).size[:2].copy()
        if item.mode == "sorting":
            span -= .004
            high = .10
        elif item.mode == "wood":
            span -= .006
            high = .095
        else:
            dims = size_geometry("slot", item.variant)
            span[:] = dims["slot_half_width"], dims["slot_half_length"]
            high = self.model.geom(item.target).size[2] + dims["slot_depth"]
        return (np.all(lower[:2] >= -span - .001) and np.all(upper[:2] <= span + .001)
                and lower[2] >= -.003 and upper[2] < high + .002)

    def scored(self, item):
        return bool(item.landed and self.geometry_ok(item) and self.valid_receiver(item, self.contacts(item)))

    def step(self, action):
        item, arm, rows = self.validate_action(action)
        data = self.data
        start = float(data.time)
        item.attempts += 1
        times = np.arange(round(DURATION / DT)) * DT
        angles = np.column_stack([np.interp(times, rows[:, 0], joint) for joint in rows[:, 1:7].T])
        jaws = rows[np.searchsorted(rows[:, 0], times, side="right") - 1, 7]
        held = air = 0
        in_flight = False
        last_held = -1.0
        picked = crossed = attempted_throw = False
        release_z = lateral_rotation = 0.
        previous = data.xpos[item.body].copy()
        origins = [float(data.qpos[arm["qpos"][0]])] * 81
        net = None
        if item.mode == "sorting" and item.variant in ("blue", "violet"):
            net = self.model.geom(f"left_net_{item.variant}")
        for tick, (angle, jaw) in enumerate(zip(angles, jaws)):
            for side, state in self.arms.items():
                if side == item.arm:
                    self.drive(state, angle, jaw)
                else:
                    self.drive(state, state["ref"], state["jaw"])
            previous_time = float(data.time)
            mj.mj_step(self.model, data)
            check_physics(data, previous_time)
            neighbors = self.contacts(item)
            position = data.xpos[item.body].copy()
            heading = float(data.qpos[arm["qpos"][0]])
            swing = heading - origins[tick % 81]
            origins[tick % 81] = heading
            if arm["pads"] <= neighbors and position[2] > TABLE_HEIGHT + .04:
                held += 1
                if held * DT >= .035:
                    last_held = data.time
                    picked = True
                    item.landed = False
                    in_flight = False
            else:
                held = 0

            if not neighbors and 0 <= data.time - last_held < .055 and not in_flight:
                in_flight = True
                air = 0
                attempted_throw = True
                crossed = False
                lateral_rotation = swing
                release_z = float(position[2])
                last_held = -1.
            if in_flight and not neighbors:
                air += 1
                if net is not None:
                    plane = data.geom(net.id)
                    axes = plane.xmat.reshape(3,3)
                    a = axes.T @ (previous - plane.xpos)
                    b = axes.T @ (position - plane.xpos)
                    if a[0] < 0 <= b[0]:
                        lower, upper = self.bounds(item, plane.xpos, axes)
                        crossed = bool(lower[2] > net.size[2]
                                       and lower[1] >= -net.size[1] and upper[1] <= net.size[1]
                                       and position[2] >= release_z + .12)
            if in_flight and neighbors:
                flight_min = {"sorting": .06, "wood": .20, "ring": .25, "slot": .25}
                lane = True
                if net is not None:
                    lane = crossed and abs(lateral_rotation) < .13
                elif item.mode == "sorting":
                    lane = lateral_rotation > .18 if item.variant == "orange" else lateral_rotation < -.18
                item.landed = air * DT >= flight_min[item.mode] and lane and self.valid_receiver(item, neighbors)
                in_flight = False
            previous = position
            if tick % RECORD_EVERY == 0:
                self.capture(len(self.actions))
        arm["jaw"] = float(jaws[-1])
        self.capture(len(self.actions))
        self.actions.append({
            "arm": item.arm, "item": item.id, "label": item.label, "picked": picked,
            "released": attempted_throw, "scored": self.scored(item),
            "start": round(start, 4), "end": round(float(data.time), 4),
        })

    def result(self):
        items = [{"id": o.id, "arm": o.arm, "label": o.label,
                  "attempts": o.attempts, "scored": self.scored(o)} for o in self.items.values()]
        return {
            "left": sum(o["scored"] for o in items if o["arm"] == "left"),
            "right": sum(o["scored"] for o in items if o["arm"] == "right"),
            "left_total": sum(o["arm"] == "left" for o in items),
            "right_total": sum(o["arm"] == "right" for o in items),
            "actions": len(self.actions), "resets": 1,
            "time": round(float(self.data.time), 4), "items": items, "decisions": self.actions,
        }


def play_round(seed, act, record=False):
    arena = ArenaRound(seed, record=record)
    try:
        for turn in range(MAX_ACTIONS):
            action = act(turn, arena.observation())
            if action == {"finish": True}:
                break
            arena.step(action)
        return arena
    finally:
        arena.close()
