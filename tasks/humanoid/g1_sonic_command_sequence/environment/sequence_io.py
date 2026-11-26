"""Public command wire and host-side continuous physical course for G1 SONIC."""
from collections import deque
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

PROTOCOL = "g1-sonic-command-sequence/1"
STAGES = ("approach", "pivot", "departure")
STAGE_LIMITS = (1300, 450, 650)
HORIZON = sum(STAGE_LIMITS)
EXTRA_APPROACH_HOLD = 75
CHUNK = 5
CODE_DIM = 64
PROPRIO = (
    "q", "dq", "base_pos", "base_quat", "base_gyro", "gravity_body",
    "torso_pos", "torso_quat", "base_velocity_world", "heading_rad", "yaw_rate_rad_s",
    "left_foot_pos", "right_foot_pos", "left_foot_contact", "right_foot_contact",
)


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
        if not (4.5 <= self.first_m <= 5.5 and 75 <= abs(self.turn_deg) <= 105 and 1 <= self.last_m <= 2):
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


def rebase_observation(sample, tick, stage_tick, last_observation_id):
    """Present a measured public packet at the same decision time and with the shared prior receipt."""
    return stamp_observation({**sample, "tick": tick, "stage_tick": stage_tick,
                              "last_observation_id": last_observation_id})


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


class SequenceEpisode:
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

    def observe(self, alternative=None):
        if self.done:
            raise ValueError("sequence has ended")
        if alternative is not None and not isinstance(alternative, Course):
            raise ValueError("alternative must be a Course")
        # Even a counterfactual can reveal only the currently active command.
        distance, turn = (self._course if alternative is None else alternative).current_goal(self.stage)
        goal = self.session.relative_goal(self._start_xy + distance * self._axis)
        pro = self.session.state["observation"]
        command = {"route": "turn_stop" if self.stage == "pivot" else "straight_stop",
                   "target_forward_m": goal["forward_m"], "target_left_m": goal["left_m"],
                   "heading_error_rad": self._start_yaw + turn - pro["heading_rad"]}
        return stamp_observation({
            "protocol": PROTOCOL, "tick": self.session.tick, "horizon": self.session.horizon,
            "stage": self.stage, "stage_tick": self.stage_tick, "command": command,
            "observation": {k: np.asarray(pro[k]).tolist() for k in PROPRIO},
            "decoder_history": self.session.decoder_history().tolist(),
            "last_observation_id": self.last_observation_id,
        })

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
            if self.session.tick - self._stop_tick >= extra:
                return "success"
        if self.stage_tick >= self._cap:
            return "time_limit"
        return None

    def advance(self, action, observation_id, *, max_ticks=None):
        """Validate the whole chunk, then commit up to a reveal, terminal row or debug cutoff."""
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
