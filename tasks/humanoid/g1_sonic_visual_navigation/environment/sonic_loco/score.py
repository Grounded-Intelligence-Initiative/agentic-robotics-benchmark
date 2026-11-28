"""Common physical stop scorer for G1 straight walking and signed in-place right turns."""
from collections import deque
import math

import numpy as np


STRAIGHT_DISK_M = .25
STRAIGHT_CORRIDOR_M = .45
TURN_DISK_M = .30
TURN_EXCURSION_M = .45
WRONG_TURN_LIMIT_RAD = math.radians(15)
YAW_TOLERANCE_RAD = math.radians(10)
STOP_SPEED_M_S = .15
STOP_YAW_RATE_RAD_S = .25
STOP_DWELL_S = .75
FOOT_UNLOAD_N = 5.
FOOT_RELOAD_N = 15.
FOOT_DWELL_S = .020


class FootTransitions:
    """Count changes between sustained loaded and unloaded states after the first stable state."""
    def __init__(self):
        self.loaded = None
        self.candidate = None
        self.dwell = 0.
        self.unloads = 0
        self.reloads = 0

    def update(self, load, dt):
        proposed = True if load >= FOOT_RELOAD_N else False if load <= FOOT_UNLOAD_N else None
        if proposed is None or proposed == self.loaded:
            self.candidate, self.dwell = None, 0.
            return
        if self.candidate != proposed:
            self.candidate, self.dwell = proposed, 0.
        self.dwell += dt
        if self.dwell + 1e-12 >= FOOT_DWELL_S:
            if self.loaded is not None:
                self.reloads += int(proposed)
                self.unloads += int(not proposed)
            self.loaded = proposed
            self.candidate, self.dwell = None, 0.


class PhysicalScore:
    def __init__(self, route_name, distance_m, heading_delta_rad, initial_metrics, initial_dq, horizon, *, corridor_m=STRAIGHT_CORRIDOR_M,
                 station_reference=None, min_head_m=None):
        if route_name not in ("straight_stop", "right_turn_stop"):
            raise ValueError("unknown locomotion route")
        self.route = route_name
        self.corridor_m = float(corridor_m)
        self.distance = float(distance_m)
        self.turn = float(heading_delta_rad)
        self.horizon = int(horizon)
        if (not math.isfinite(self.distance) or not math.isfinite(self.turn) or self.horizon < 1
                or (self.route == "straight_stop" and not (self.distance > 0 and self.turn == 0))
                or (self.route == "right_turn_stop" and not (self.distance == 0 and self.turn < 0))):
            raise ValueError("invalid straight or signed right-turn goal")
        self.x0, self.y0 = initial_metrics["base_xy"]
        self.yaw0 = initial_metrics["heading_rad"]
        self.previous_xy = (self.x0,self.y0)
        if station_reference is not None:
            (self.x0,self.y0),self.yaw0 = station_reference
        self.cos0, self.sin0 = math.cos(self.yaw0), math.sin(self.yaw0)
        self.reason = None
        self.time = float(initial_metrics["time_s"])
        self.t0 = self.time
        self.ticks = 0
        self.hold = self.best_hold = 0.
        self.min_head_m = min_head_m
        self.hold_head_min = None
        self.peak_lateral = self.peak_pivot = self.peak_wrong_yaw = 0.
        self.speed = self.yaw_rate = self.peak_speed = self.peak_yaw_rate = 0.
        self.forward = self.lateral = self.yaw = 0.
        self.path_length = self.angle_travel = 0.
        self.position_error, self.heading_error = self.distance, self.turn
        self.controls = None
        self.changed = self.accel_samples = 0
        self.accel_sum_squares = 0.
        self.velocity_history = deque([self._velocity(initial_dq)], maxlen=5)
        self.feet = [FootTransitions(), FootTransitions()]

    @staticmethod
    def _velocity(dq):
        values = np.asarray(dq, dtype=np.float64)
        if values.shape != (29,) or not np.isfinite(values).all():
            raise ValueError("expected 29 finite measured joint velocities")
        return values.copy()

    @property
    def done(self):
        return self.reason is not None

    def _measure(self, m):
        dt = float(m["time_s"]) - self.time
        if not math.isclose(dt, .002, rel_tol=0, abs_tol=1e-8):
            raise ValueError("scoring requires ordered 2 ms native samples")
        self.time = float(m["time_s"])
        x, y = m["base_xy"]
        self.path_length += math.hypot(x - self.previous_xy[0], y - self.previous_xy[1])
        self.previous_xy = (x, y)
        dx, dy = x - self.x0, y - self.y0
        self.forward = dx * self.cos0 + dy * self.sin0
        self.lateral = -dx * self.sin0 + dy * self.cos0
        yaw = float(m["heading_rad"]) - self.yaw0
        self.angle_travel += abs(yaw - self.yaw)
        self.yaw = yaw
        self.heading_error = self.turn - self.yaw
        self.position_error = math.hypot(self.distance - self.forward, self.lateral)
        self.peak_lateral = max(self.peak_lateral, abs(self.lateral))
        self.peak_pivot = max(self.peak_pivot, math.hypot(dx, dy))
        self.peak_wrong_yaw = max(self.peak_wrong_yaw, self.yaw)
        self.speed = math.hypot(*m["velocity_xy"])
        self.yaw_rate = float(m["yaw_rate_rad_s"])
        self.peak_speed = max(self.peak_speed, self.speed)
        self.peak_yaw_rate = max(self.peak_yaw_rate, abs(self.yaw_rate))
        for side, foot in zip(("left","right"), self.feet):
            foot.update(m[side + "_foot_load_n"], dt)
        if m["fallen"]:
            self.reason = "fall"
            return
        if self.route == "straight_stop":
            violated = self.peak_lateral > self.corridor_m
            disk = STRAIGHT_DISK_M
        else:
            violated = self.peak_pivot > TURN_EXCURSION_M or self.peak_wrong_yaw > WRONG_TURN_LIMIT_RAD
            disk = TURN_DISK_M
        if violated:
            self.reason = "path_violation"
            return
        at_rest_at_goal = (m["upright"] and self.position_error <= disk
                           and abs(self.heading_error) <= YAW_TOLERANCE_RAD
                           and self.speed <= STOP_SPEED_M_S and abs(self.yaw_rate) <= STOP_YAW_RATE_RAD_S)
        if self.min_head_m is not None:
            at_rest_at_goal = at_rest_at_goal and m["head_top_m"]>=self.min_head_m
            if at_rest_at_goal:
                self.hold_head_min = m["head_top_m"] if self.hold_head_min is None else min(self.hold_head_min,m["head_top_m"])
            else:
                self.hold_head_min = None
        self.hold = self.hold + dt if at_rest_at_goal else 0.
        self.best_hold = max(self.best_hold, self.hold)
        if self.hold + 1e-10 >= STOP_DWELL_S:
            self.reason = "success"

    def update_tick(self, codes, dq, substeps):
        if self.done:
            raise ValueError("locomotion episode is already terminal")
        values = np.asarray(codes)
        if (values.shape != (64,) or values.dtype.kind not in "iu"
                or np.any(values < 0) or np.any(values > 31) or len(substeps) != 10):
            raise ValueError("expected one native 64-code row and its ten physical substeps")
        velocity = self._velocity(dq)
        if self.controls is not None:
            self.changed += int(np.count_nonzero(values != self.controls))
        self.controls = values.astype(np.uint8, copy=True)
        self.ticks += 1
        if len(self.velocity_history) == 5:
            accel = (velocity - self.velocity_history[0]) / .1
            self.accel_sum_squares += float(np.dot(accel, accel))
            self.accel_samples += 29
        self.velocity_history.append(velocity)
        for sample in substeps:
            self._measure(sample)
            if self.done:
                break
        if self.ticks >= self.horizon and not self.done:
            self.reason = "time_limit"

    def summary(self):
        eligible = 64 * max(0,self.ticks - 1)
        return dict(route=self.route, reason=self.reason, success=self.reason == "success",
                    pelvis_path_length_m=self.path_length, absolute_yaw_travel_rad=self.angle_travel,
                    route_efficiency=(min(max(self.forward, 0.), self.distance) / self.path_length
                                      if self.route == "straight_stop" and self.path_length else
                                      min(max(-self.yaw, 0.), -self.turn) / self.angle_travel
                                      if self.route == "right_turn_stop" and self.angle_travel else 0.),
                    simulated_s=self.time - self.t0, applied_rows=self.ticks, issued_control_s=self.ticks / 50,
                    scalar_codes=64 * self.ticks, code_changes=self.changed, eligible_code_pairs=eligible,
                    code_change_fraction=self.changed / eligible if eligible else 0.,
                    joint_accel_square_sum=self.accel_sum_squares, joint_accel_samples=self.accel_samples,
                    joint_accel_rms_rad_s2=math.sqrt(self.accel_sum_squares/self.accel_samples) if self.accel_samples else 0.,
                    target_distance_m=self.distance,target_turn_rad=self.turn, forward_m=self.forward,
                    lateral_m=self.lateral, signed_turn_rad=self.yaw, final_xy_error_m=self.position_error,
                    final_heading_error_rad=self.heading_error, max_abs_lateral_m=self.peak_lateral,
                    max_pivot_drift_m=self.peak_pivot, max_positive_turn_rad=self.peak_wrong_yaw,
                    final_speed_m_s=self.speed,final_yaw_rate_rad_s=self.yaw_rate,
                    peak_speed_m_s=self.peak_speed,peak_abs_yaw_rate_rad_s=self.peak_yaw_rate,
                    max_stop_dwell_s=self.best_hold,
                    min_head_height_during_final_dwell_m=self.hold_head_min if self.reason=="success" else None,
                    left_unloads=self.feet[0].unloads,left_reloads=self.feet[0].reloads,
                    right_unloads=self.feet[1].unloads,right_reloads=self.feet[1].reloads)
