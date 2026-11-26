"""P1 from public RGB, G1 proprioception and the current public decoder history."""
from copy import deepcopy
import json
import sys

import mujoco
import numpy as np

from sonic_sim.decoder import Decoder

from .inverse import InverseCodeSearch
from .landing import LandingPlan
from .robot import inertial_robot
from .vision import measure_scene


class Planner(InverseCodeSearch):
    def __init__(self, decoder, robot):
        self.decoder = decoder
        self.robot = robot
        self.scratch = mujoco.MjData(robot.model)
        self.prev_base = None
        self.track = {}
        self.ground_height = None
        self.last_offset = None
        self.last_detection_tick = None
        self.good_frames = 0
        self.previous_closure = 0.
        super().__init__(LandingPlan(robot), lambda: self.scratch.qfrc_bias[robot.vadr[22:]])

    def _probe(self, obs, codes):
        packed = np.empty(994, dtype=np.float32)
        packed[64:] = self.history
        predictions = []
        for code in codes:
            packed[:64] = (np.asarray(code, np.float32) - 16) / 16
            predictions.append(self.decoder.joint_targets(self.decoder.decode_observation(packed)))
        return predictions

    def _standing(self, obs):
        self._predict(obs, [self.code])

    def decide(self, tick, task, proprio, history, images, cameras):
        self.tick, self.task_label, self.history = tick, task, history
        self.image_tick = tick if images is not None else None
        self.previous_object = self.track.get("object_pos")
        if task == "cylinder_to_pad":
            self.refresh = 32
        if images is not None:
            result = measure_scene(images, cameras, task)
            for key in ("object_pos", "goal_pos"):
                if result[key] is not None:
                    self.track[key] = np.array(result[key])
        obs = {key: np.asarray(value, dtype=float) for key, value in proprio.items()}
        data = self.scratch
        data.qpos[:3] = obs["base_pos"]
        data.qpos[3:7] = obs["base_quat"]
        data.qpos[self.robot.qadr] = obs["q"]
        data.qvel[3:6] = obs["base_gyro"]
        data.qvel[self.robot.vadr] = obs["dq"]
        if self.prev_base is not None:
            data.qvel[:3] = (obs["base_pos"] - self.prev_base) * 50
        self.prev_base = obs["base_pos"]
        mujoco.mj_forward(self.robot.model, data)
        if len(self.track) < 2:
            return self.command(obs, None), (0., 0.)
        return self.act({**obs, **self.track})

    def change_density(self, new_density, evidence):
        event = dict(event="search_refresh", tick=self.tick, previous_refresh=self.refresh,
                     refresh=new_density, phase=self.goal.phase, last_code=self.code.tolist(), **evidence)
        print(json.dumps(event, separators=(",", ":"), allow_nan=False), file=sys.stderr, flush=True)
        self.refresh = new_density

    def act(self, obs):
        # The pinned observer replaces the tracked array only after an actual
        # fresh silhouette fit. An occluded frame retains the previous array.
        fresh = self.image_tick == self.tick and self.track["object_pos"] is not self.previous_object
        if self.task_label == "box_to_pad" and fresh:
            obj = obs["object_pos"]
            hand = obs["right_hand_pos"]
            if self.ground_height is None:
                self.ground_height = float(obj[2])
            offset = obj-hand
            separation = float(np.linalg.norm(offset))
            rise = float(obj[2]-self.ground_height)
            consecutive = self.last_detection_tick == self.tick-5
            drift = float(np.linalg.norm(offset-self.last_offset)) if consecutive else None
            distance_to_pad = float(np.linalg.norm(obs["goal_pos"][:2]-obj[:2]))
            carrying = (self.previous_closure >= .95 and rise >= .050 and separation <= .060
                        and drift is not None and drift <= .015)
            self.good_frames = self.good_frames + 1 if consecutive and carrying else int(carrying)
            evidence = dict(rise_m=rise, object_hand_distance_m=separation, offset_delta_m=drift,
                            object_pad_xy_m=distance_to_pad, consecutive_frames=self.good_frames,
                            previous_right_closure=self.previous_closure)
            if self.refresh == 4 and self.good_frames >= 2:
                self.change_density(32, evidence)
            self.last_offset = offset.copy()
            self.last_detection_tick = self.tick
        action, hand = super().act(obs)
        self.previous_closure = hand[1]
        return action, hand


class P1:
    # Native workspaces contain only public inertial links. All quantities that
    # can affect the next control tick are independently restored for each query.
    _runtime = {"decoder", "robot", "scratch", "arm_bias", "goal"}
    _rigid = {"sim", "model", "fk"}

    def __init__(self, assets):
        self.decoder = Decoder(assets / "policy" / "model_decoder.onnx")
        self.robot = inertial_robot(assets / "menagerie_g1" / "g1_with_hands.xml")
        self.reset()

    def reset(self):
        self.planner = Planner(self.decoder, self.robot)

    def snapshot(self):
        p = self.planner
        state = {k: v for k, v in vars(p).items() if k not in self._runtime}
        goal = {k: v for k, v in vars(p.goal).items() if k not in self._rigid}
        return deepcopy((state, goal, p.scratch.qvel.copy()))

    def restore(self, snapshot):
        p = self.planner
        state, goal, velocity = deepcopy(snapshot)
        for obj, keep, values in ((p, self._runtime, state), (p.goal, self._rigid, goal)):
            for k in set(vars(obj)) - keep - values.keys():
                delattr(obj, k)
            vars(obj).update(values)
        p.scratch.qvel[:] = velocity

    def act(self, public):
        images = None
        if public.fresh:
            images = {name: np.asarray(im) for name, im in public.images.items()}
        action, hands = self.planner.decide(
            public.tick, public.task, public.proprio, public.history, images, public.cameras
        )
        return [{"codes": action["codes"], "hand": list(hands)}]
