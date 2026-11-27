"""Egocentric RGB-D observations and private continuous walking course for G1 SONIC."""
import base64
from collections import deque
from io import BytesIO
from PIL import Image
from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real

import numpy as np

from sonic_loco.score import (
    PhysicalScore, STOP_SPEED_M_S, STOP_YAW_RATE_RAD_S, STRAIGHT_DISK_M,
    TURN_DISK_M, YAW_TOLERANCE_RAD,
)

PROTOCOL = "g1-sonic-visual-navigation/1"
STAGES = ("approach", "pivot", "departure")
STAGE_LIMITS = (1300, 450, 650)
HORIZON = sum(STAGE_LIMITS)
EXTRA_APPROACH_HOLD = 75
CHUNK = 5
CODE_DIM = 64
PALETTE = {"approach": (.10, .28, 1.), "left": (1., .12, .08),
           "right": (1., .08, .85), "heading": (1., .88, .06), "departure": (.14, 1., .20)}
BEACON_HEIGHT = 1.03
STANDOFF = 1.50


def _png(array):
    buf = BytesIO()
    Image.fromarray(array).save(buf, format="PNG", compress_level=1)
    return base64.b64encode(buf.getvalue()).decode("ascii")


@dataclass(frozen=True)
class Course:
    """Verifier-owned parameters, signed counterclockwise angle in degrees."""
    first_m: float
    turn_deg: float
    last_m: float

    def __post_init__(self):
        values = (self.first_m, self.turn_deg, self.last_m)
        if not all(isinstance(x, Real) and not isinstance(x, bool) and math.isfinite(x) for x in values):
            raise ValueError("course parameters must be finite numbers")
        if not (3.0 <= self.first_m <= 6.0 and 75 <= abs(self.turn_deg) <= 105 and .8 <= self.last_m <= 2.5):
            raise ValueError("course parameters are outside the supported route limits")

    def current_goal(self, stage):
        if stage == "approach":
            return float(self.first_m), 0.
        if stage == "pivot":
            return 0., math.radians(self.turn_deg)
        if stage == "departure":
            return float(self.last_m), 0.
        raise ValueError("unknown route stage")


def practice_course(turn):
    if turn not in ("left", "right"):
        raise ValueError("practice turn must be left or right")
    return Course(5., 90. if turn == "left" else -90., 1.5)


def stamp_observation(obs):
    """Receipt covers only public information available for the current query."""
    fields = {k: v for k, v in obs.items() if k not in ("observation_id", "last_observation_id")}
    encoded = json.dumps(fields, sort_keys=True, separators=(",", ":"), allow_nan=False)
    obs["observation_id"] = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    return obs


def validate_action(action, tick, horizon):
    if not isinstance(action, dict) or set(action) != {"rows"}:
        raise ValueError("action requires a rows field")
    rows = action["rows"]
    limit = min(CHUNK - tick % CHUNK, horizon - tick)
    if not isinstance(rows, list) or not 1 <= len(rows) <= limit:
        raise ValueError(f"tick {tick} accepts 1 to {limit} rows")
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != {"codes"}:
            raise ValueError(f"row {i} requires a codes field")
        codes = row["codes"]
        if (not isinstance(codes, list) or len(codes) != CODE_DIM
                or any(type(c) is not int or not 0 <= c <= 31 for c in codes)):
            raise ValueError(f"row {i} requires 64 integer codes in [0,31]")
    return rows


class NavigationEpisode:
    """One native world, three sequential goals, and measured stops at each reveal."""

    def __init__(self, session, course):
        if session.tick != 0 or not isinstance(course, Course):
            raise ValueError("a new sequence requires a reset native session and a Course")
        self.session = session
        self._course = course
        self._stage_index = 0
        self._completed = []
        self.reason = None
        self.last_observation_id = None
        self._camera_tick = None
        self._camera_frame = None
        self._previous_codes = None
        self._code_changes = 0
        self._accel_sum = 0.
        self._accel_samples = 0
        self._velocities = deque([np.array(session.state["observation"]["dq"], dtype=np.float64)], maxlen=5)
        self._start_stage()

    @property
    def stage(self):
        return STAGES[self._stage_index]

    @property
    def stage_tick(self):
        return self.session.tick - self._start_tick

    @property
    def done(self):
        return self.reason is not None

    def _start_stage(self):
        state = self.session.state
        self._start_tick = self.session.tick
        self._start_xy = np.asarray(state["metrics"]["base_xy"])
        self._start_yaw = state["metrics"]["heading_rad"]
        self._axis = np.array([np.cos(self._start_yaw), np.sin(self._start_yaw)])
        self._distance, self._turn = self._course.current_goal(self.stage)
        self._target_xy = self._start_xy + self._distance * self._axis
        self._target_yaw = self._start_yaw + self._turn
        self._stop_tick = None
        self._cap = min(STAGE_LIMITS[self._stage_index], self.session.horizon - self._start_tick)
        self._score = PhysicalScore("right_turn_stop" if self.stage == "pivot" else "straight_stop",
                                    self._distance, -abs(self._turn), state["metrics"],
                                    state["observation"]["dq"], self._cap)

        self.session.beacons = self.landmarks()

    def landmarks(self, alternative=None, brightness=1.):
        """Current optical scene at the measured pose, for live or counterfactual rendering."""
        course = self._course if alternative is None else alternative
        if not isinstance(course, Course) or not .6 <= brightness <= 1.:
            raise ValueError("invalid visual scene")
        distance, turn = course.current_goal(self.stage)
        def light(xy, color):
            return ((*xy, BEACON_HEIGHT), tuple(brightness*x for x in PALETTE[color]))
        if self.stage == "pivot":
            yaw = self._start_yaw + turn
            return (light(self._start_xy + STANDOFF*self._axis, "left" if turn > 0 else "right"),
                    light(self._start_xy + 6.*np.array([math.cos(yaw), math.sin(yaw)]), "heading"))
        return (light(self._start_xy + (distance+STANDOFF)*self._axis, self.stage),)

    def observe(self, alternative=None, brightness=1.):
        if self.done:
            raise ValueError("navigation episode has ended")
        session = self.session
        pro = session.state["observation"]
        if alternative is None and brightness == 1.:
            if self._camera_tick != session.tick:
                rgb, depth = session.egocentric(self.landmarks())
                self._camera_frame = {"rgb_png": _png(rgb), "depth_mm_png": _png(depth)}
                self._camera_tick = session.tick
            camera = self._camera_frame.copy()
        else:
            rgb, depth = session.egocentric(self.landmarks(alternative, brightness), probe=True)
            camera = {"rgb_png": _png(rgb), "depth_mm_png": _png(depth)}
        public = {k: np.asarray(pro[k]).tolist() for k in (
            "q", "dq", "base_gyro", "gravity_body", "yaw_rate_rad_s",
            "left_foot_contact", "right_foot_contact")}
        rotation = session.data.xmat[session._pelvis].reshape(3,3)
        public["velocity_body"] = (rotation.T @ pro["base_velocity_world"]).tolist()
        return stamp_observation({
            "protocol": PROTOCOL, "tick": session.tick, "horizon": session.horizon,
            "observation": public, "decoder_history": session.decoder_history().tolist(),
            "camera": camera,
            "last_observation_id": self.last_observation_id,
        })

    def relight(self, live, brightness):
        """Render a different lamp exposure using exactly the live depth acquisition."""
        if live["tick"] != self.session.tick or live["last_observation_id"] != self.last_observation_id:
            raise ValueError("brightness intervention requires the current public frame")
        rgb, _ = self.session.egocentric(self.landmarks(brightness=brightness), depth=False, probe=True)
        camera = dict(live["camera"], rgb_png=_png(rgb))
        return stamp_observation(dict(live, camera=camera))

    def _record_cost(self, codes, dq):
        if self._previous_codes is not None:
            self._code_changes += int(np.count_nonzero(codes != self._previous_codes))
        self._previous_codes = codes.copy()
        velocity = np.asarray(dq, dtype=np.float64)
        if len(self._velocities) == 5:
            a = (velocity - self._velocities[0]) / .1
            self._accel_sum += float(a @ a)
            self._accel_samples += len(a)
        self._velocities.append(velocity.copy())

    def _grade(self):
        grade = self._score.summary()
        if self.stage == "pivot":
            grade["route"] = "turn_stop"
            grade["max_wrong_direction_turn_rad"] = grade.pop("max_positive_turn_rad")
            if self._turn > 0:
                for k in ("target_turn_rad", "signed_turn_rad", "final_heading_error_rad", "final_yaw_rate_rad_s"):
                    grade[k] *= -1
        return grade

    def _stage_record(self, reason=None):
        return {"stage": self.stage, "revealed_at_tick": self._start_tick,
                "scorer_terminal_tick": self._stop_tick, "exit_tick": self.session.tick,
                "command_distance_m": self._distance, "command_turn_deg": math.degrees(self._turn),
                "reason": reason, "grade": self._grade()}

    def _qualifies_as_stopped(self, sample):
        disk = TURN_DISK_M if self.stage == "pivot" else STRAIGHT_DISK_M
        return (sample["upright"] and not sample["fallen"] and math.dist(sample["base_xy"], self._target_xy) <= disk
                and abs(sample["heading_rad"] - self._target_yaw) <= YAW_TOLERANCE_RAD
                and math.hypot(*sample["velocity_xy"]) <= STOP_SPEED_M_S
                and abs(sample["yaw_rate_rad_s"]) <= STOP_YAW_RATE_RAD_S)

    def _advance_row(self, codes):
        state = self.session.step((codes.astype(np.float32) - 16) / 16)
        samples = self.session.last_substeps
        dq = state["observation"]["dq"]
        self._record_cost(codes, dq)
        if not self._score.done:
            graded = samples
            if self._turn > 0:
                # Reflect only signed angular quantities for the official right-pivot
                # scorer. The unmodified native data drives dynamics, observations and costs.
                graded = tuple({**s, "heading_rad": self._start_yaw - (s["heading_rad"] - self._start_yaw),
                                "yaw_rate_rad_s": -s["yaw_rate_rad_s"]} for s in samples)
            self._score.update_tick(codes, dq, graded)
            if self._score.done:
                self._stop_tick = self.session.tick
        if state["metrics"]["fallen"]:
            return "fall"
        if self._score.done:
            if self._score.reason != "success":
                return self._score.reason
            if any(not self._qualifies_as_stopped(s) for s in samples if s["time_s"] > self._score.time):
                return "lost_standing_hold"
            extra = EXTRA_APPROACH_HOLD if self.stage == "approach" else 0
            if self.session.tick - self._stop_tick >= extra and (
                    self.stage == "departure" or self.session.tick % CHUNK == 0):
                return "success"
        if self.stage_tick >= self._cap:
            return "time_limit"
        return None

    def advance(self, action, observation_id, *, max_ticks=None):
        """Validate the whole chunk, then commit rows until a terminal row or public practice cutoff."""
        if self.done:
            raise ValueError("sequence has ended")
        start = self.session.tick
        rows = validate_action(action, start, self.session.horizon)
        if max_ticks is not None:
            if type(max_ticks) is not int or max_ticks < 1:
                raise ValueError("max_ticks must be a positive integer")
            rows = rows[:max_ticks]
        for row in rows:
            reason = self._advance_row(np.asarray(row["codes"], dtype=np.uint8))
            if reason is None:
                continue
            self._completed.append(self._stage_record(reason))
            if reason != "success" or self.stage == "departure":
                self.reason = reason
            elif self.session.tick == self.session.horizon:
                self.reason = "time_limit"
            else:
                self._stage_index += 1
                self._start_stage()
            if self.done:
                break
        self.last_observation_id = observation_id
        return self.session.tick - start

    def summary(self):
        rows = self.session.tick
        eligible = CODE_DIM * max(0, rows - 1)
        stages = [*self._completed] if self.done else [*self._completed, self._stage_record()]
        return {"success": self.reason == "success", "reason": self.reason, "stages": stages,
                "stages_completed": sum(s["reason"] == "success" for s in self._completed),
                "simulated_s": self.session.state["metrics"]["time_s"], "applied_rows": rows,
                "issued_control_s": rows / 50, "scalar_codes": CODE_DIM * rows,
                "code_changes": self._code_changes, "eligible_code_pairs": eligible,
                "code_change_fraction": self._code_changes / eligible if eligible else 0.,
                "joint_accel_square_sum": self._accel_sum, "joint_accel_samples": self._accel_samples,
                "joint_accel_rms_rad_s2": math.sqrt(self._accel_sum / self._accel_samples) if self._accel_samples else 0.}
