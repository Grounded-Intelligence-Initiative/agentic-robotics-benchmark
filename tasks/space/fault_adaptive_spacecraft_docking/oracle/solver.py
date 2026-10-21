import copy
import itertools
import json
import math
import sys

import numpy as np


DT = 0.12
PERMS = np.array(list(itertools.permutations(range(6))), dtype=np.int8)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine], [sine, cosine]])


class Controller:
    def __init__(self):
        self.time = -1
        self.imu = []
        self.poses = {}
        self.port = {}
        self.actions = []
        self.states = []
        self.covs = []
        self.matrix = np.eye(3)
        self.calibrated = False
        self.columns = np.zeros((3, 6))
        self.bias = np.zeros(3)
        self.event = 0
        self.event_time = -100
        self.probe_count = 0
        self.samples = []
        self.mount = np.eye(3)
        self.scale = np.ones(6)
        self.exponent = np.ones(6)
        self.curves = [[] for _ in range(6)]
        self.memory = np.zeros(6)
        self.valves = np.zeros(6)
        self.mapping = np.arange(6)
        self.direction = 1
        self.severe = 0
        self.moderate = 1
        self.armed = True
        self.hyp = None
        self.track = None
        self.locked = False
        self.last_prediction = np.zeros(3)
        self.last_columns = np.zeros((3, 6))
        self.last_z = np.zeros(6)
        self.last_map = np.arange(6)
        self.last_body = np.zeros(3)
        self.phase_rate = 0.0
        self.phase = 0.0
        self.phase_stamp = 0
        self.port_radius = 1.14
        self.previous_port_radius = None
        self.port_radius_rate = 0.0
        self.port_encoder_sign = None
        self.port_sensor_history = {}
        self.plume_nominal_sign = np.array([1, -1, 1, -1, 1, -1], dtype=float)
        self.plume_base_sign = self.plume_nominal_sign.copy()
        self.plume_parity = 1.0
        self.plume_parity_score = {-1.0: 0.0, 1.0: 0.0}
        self.swirl_magnitude = np.array([.92, .98, 1.04, 1.10, 1.16, 1.20])
        self.swirl = self.plume_base_sign * self.swirl_magnitude
        self.strength = 0.82
        self.plume_threshold = .24 + .028 * np.arange(6)
        self.plume_energy = np.zeros(6)
        self.plume_onset = 1.47
        self.plume_samples = []
        self.model_error = 0.0
        self.change_count = 0
        self.parameter_cov = np.eye(12) * 0.1
        self.plan = None
        self.random = np.random.default_rng(7261)

    def calibrate(self, time):
        stamps = sorted(stamp for stamp in self.poses if stamp <= time)
        if len(stamps) < 12:
            return
        end = max(stamps)
        yaw = np.unwrap([self.poses[stamp][2] for stamp in stamps])
        integrated = np.zeros(end + 1)
        rate = 0.0
        rate_basis = np.zeros(end + 1)
        basis = 1.0
        for index in range(end):
            rate = 0.996 * (rate + DT * self.imu[index][2])
            integrated[index + 1] = integrated[index] + DT * rate
            basis *= 0.996
            rate_basis[index + 1] = rate_basis[index] + DT * basis
        design = np.array([[1, rate_basis[stamp], integrated[stamp]] for stamp in stamps])
        fit = np.linalg.lstsq(design, yaw, rcond=None)[0]
        sign = 1.0 if fit[2] > 0 else -1.0
        yaw_initial, yaw_rate = np.linalg.lstsq(design[:, :2], yaw - sign * design[:, 2], rcond=None)[0]
        attitude = yaw_initial + yaw_rate * rate_basis + sign * integrated
        accumulator = np.zeros((2, 4))
        velocity = np.zeros((2, 4))
        features = [accumulator.copy()]
        velocity_basis = 1.0
        position_basis = [0.0]
        for index in range(end):
            linear = self.imu[index][:2]
            basis_accel = np.array([[linear[0], linear[1], 0, 0], [0, 0, linear[0], linear[1]]])
            velocity = 0.998 * (velocity + DT * rotation(attitude[index]) @ basis_accel)
            accumulator = accumulator + DT * velocity
            features.append(accumulator.copy())
            velocity_basis *= 0.998
            position_basis.append(position_basis[-1] + DT * velocity_basis)
        rows, values = [], []
        for stamp in stamps:
            for axis in range(2):
                nuisance = [0.0] * 4
                nuisance[axis * 2] = 1
                nuisance[axis * 2 + 1] = position_basis[stamp]
                rows.append(nuisance + list(features[stamp][axis]))
                values.append(self.poses[stamp][axis])
        fit = np.linalg.lstsq(np.array(rows), values, rcond=None)[0]
        orthogonal, _, right = np.linalg.svd(fit[4:].reshape(2, 2))
        self.matrix[:2, :2] = orthogonal @ right
        self.matrix[2, 2] = sign
        self.calibrated = time >= 26

    def transition(self, state, covariance, raw):
        accel = self.matrix @ self.mount.T @ raw
        body_rotation = rotation(state[2])
        inertial = body_rotation @ accel[:2]
        result = state.copy()
        result[3:5] = 0.998 * (state[3:5] + DT * inertial)
        result[5] = 0.996 * (state[5] + DT * accel[2])
        result[:3] += DT * result[3:]
        jacobian = np.eye(6)
        jacobian[0, 3] = jacobian[1, 4] = DT * 0.998
        jacobian[2, 5] = DT * 0.996
        jacobian[3, 3] = jacobian[4, 4] = 0.998
        jacobian[5, 5] = 0.996
        derivative = np.array([-inertial[1], inertial[0]])
        jacobian[3:5, 2] = DT * 0.998 * derivative
        jacobian[:2, 2] = DT * DT * 0.998 * derivative
        process = np.diag([0.000002, 0.000002, 0.0000005, 0.000002, 0.000002, 0.000001])
        return result, jacobian @ covariance @ jacobian.T + process

    def estimate(self, time, obs):
        stamp = max(0, time - int(obs[12]))
        measured = np.array([obs[0], obs[1], math.atan2(obs[3], obs[2])])
        if time == 0:
            self.states = [np.r_[measured, np.zeros(3)]]
            self.covs = [np.diag([0.0001] * 3 + [0.02] * 3)]
        else:
            state, covariance = self.transition(self.states[-1], self.covs[-1], np.array(obs[8:11]))
            self.states.append(state)
            self.covs.append(covariance)
        if obs[13] > 0.5:
            self.poses[stamp] = measured
            self.port[stamp] = math.atan2(obs[7], obs[6])
            state = self.states[stamp].copy()
            covariance = self.covs[stamp].copy()
            innovation = measured - state[:3]
            innovation[2] = wrap(innovation[2])
            self.query_score = float(np.sum(innovation[:2] ** 2) + innovation[2] ** 2)
            measurement_cov = np.diag([0.000055, 0.000055, 0.000025])
            gain = np.linalg.solve(covariance[:3, :3] + measurement_cov, covariance[:3, :]).T
            state += gain @ innovation
            covariance -= gain @ covariance[:3, :]
            self.states[stamp], self.covs[stamp] = state, covariance
            for index in range(stamp + 1, time + 1):
                state, covariance = self.transition(state, covariance, self.imu[index - 1])
                self.states[index], self.covs[index] = state, covariance
        else:
            self.query_score = float(np.sum((measured[:2] - self.states[stamp][:2]) ** 2) + wrap(measured[2] - self.states[stamp][2]) ** 2)
        port_stamps = sorted(stamp for stamp in self.port if stamp >= max(0, time - 30) and (self.event < 2 or stamp >= self.event_time))
        if len(port_stamps) >= 5 and port_stamps[-1] - port_stamps[0] >= 5:
            phases = np.unwrap([self.port[stamp] for stamp in port_stamps])
            design = np.array([[1, (stamp - time) * DT] for stamp in port_stamps])
            fit = np.linalg.lstsq(design, phases, rcond=None)[0]
            self.phase, self.phase_rate = fit
            self.phase_stamp = time
        elif self.port and self.event < 2:
            last = max(self.port)
            self.phase = self.port[last] + self.phase_rate * DT * (time - last)
            self.phase_stamp = time
        else:
            self.phase += self.phase_rate * DT * (time - self.phase_stamp)
            self.phase_stamp = time

    def start_event(self, time):
        self.event += 1
        self.event_time = time - (2 if self.event == 2 else 1)
        self.change_count = 0
        self.probe_count = 0
        self.samples = []
        self.scale = np.ones(6) * 0.9
        self.exponent = np.ones(6)
        self.curves = [[] for _ in range(6)]
        self.memory = (0.15 + 0.02 * np.arange(6)) if self.event == 1 else (0.24 + 0.025 * np.arange(6))
        self.valves = (1 - self.memory) * self.actions[-1]
        self.hyp = None
        self.track = None
        self.locked = False
        self.armed = True
        self.parameter_cov = np.eye(12) * 0.08
        if self.event == 2:
            self.port_encoder_sign = None
            self.previous_port_radius = None
            self.port_radius_rate = 0.0
            old_rate = self.phase_rate
            self.phase_rate = -math.copysign(0.0325, old_rate)
            self.phase += (self.phase_rate - old_rate) * DT * max(0, time - 1 - self.event_time)

    def mount_identify(self):
        candidates = []
        for delta in [-0.79, -0.43, 0.43, 0.79]:
            for mirror in [-1, 1]:
                for polarity in [-1, 1]:
                    matrix = np.eye(3)
                    matrix[:2, :2] = rotation(delta) @ np.diag([1, mirror])
                    matrix[2, 2] = polarity
                    candidates.append(matrix)
        costs = []
        for matrix in candidates:
            residual = np.array([matrix.T @ sample[0] - self.bias for sample in self.samples])
            scalar = np.maximum(0.05, residual @ self.columns / np.sum(self.columns ** 2, axis=0))
            distance = np.sum((residual[:, :, None] - self.columns[None, :, :] * scalar[:, None, :]) ** 2, axis=1)
            costs.append(np.sum(np.min(distance, axis=1)))
        selected = candidates[int(np.argmin(costs))]
        changed = np.linalg.norm(selected - self.mount) > 0.01
        self.mount = selected
        if changed and self.event_time < len(self.states):
            state = self.states[self.event_time].copy()
            covariance = self.covs[self.event_time].copy()
            for index in range(self.event_time + 1, len(self.states)):
                state, covariance = self.transition(state, covariance, self.imu[index - 1])
                self.states[index], self.covs[index] = state, covariance

    def init_hypotheses(self):
        self.hyp = {
            'maps': np.repeat(PERMS, 12, axis=0),
            'dirs': np.tile(np.repeat([-1, 1], 6), len(PERMS)),
            'severe': np.tile(np.arange(6), len(PERMS) * 2),
            'cost': np.zeros(len(PERMS) * 12),
        }

    def curve_update(self, nozzle, value, valve):
        if valve < 0.1 or value < 0.08:
            return
        self.curves[nozzle].append((math.log(valve), math.log(value)))
        points = np.array(self.curves[nozzle][-40:])
        if len(points) >= 2 and np.ptp(points[:, 0]) > 0.22:
            design = np.c_[np.ones(len(points)), points[:, 0]]
            fit = np.linalg.solve(design.T @ design + np.diag([0.002, 0.015]), design.T @ points[:, 1] + np.array([0, 0.015]))
            self.scale[nozzle] = np.clip(math.exp(fit[0]), 0.28, 1.15)
            self.exponent[nozzle] = np.clip(fit[1], 0.63, 1.65)
        else:
            self.scale[nozzle] = np.clip(value / valve, 0.28, 1.15)

    def hypothesis_sample(self, raw, valve, command):
        if self.hyp is None:
            self.init_hypotheses()
        hyp = self.hyp
        slot = int(np.argmax(command))
        measured = self.mount.T @ raw - self.bias
        norms = np.sum(self.columns ** 2, axis=0)
        scalar = np.maximum(0.02, measured @ self.columns / norms)
        error = np.sum((measured[:, None] - self.columns * scalar) ** 2, axis=0)
        identities = hyp['maps'][:, slot]
        if self.probe_count not in (20, 21):
            hyp['cost'] += error[identities] / 0.000065
        minimum = np.min(hyp['cost'])
        keep = hyp['cost'] < minimum + 22
        for key in hyp:
            hyp[key] = hyp[key][keep]
        best = int(np.argmin(hyp['cost']))
        nozzle = int(hyp['maps'][best, slot])
        if self.probe_count not in (20, 21):
            self.curve_update(nozzle, scalar[nozzle], valve[slot])
        reverse = command[np.argmax(hyp['maps'] == hyp['severe'][:, None], axis=1)] > 0.78
        hyp['dirs'][reverse] *= -1
        stride = 2 if np.sum(command) > 1.35 else 1
        indices = (np.arange(6)[None, :] + stride * hyp['dirs'][:, None]) % 6
        hyp['maps'] = np.take_along_axis(hyp['maps'], indices, axis=1)
        self.mapping = hyp['maps'][best].copy()
        self.direction = int(hyp['dirs'][best])
        self.severe = int(hyp['severe'][best])

    def probe_update(self, raw):
        command = self.actions[-1]
        if self.probe_count >= 3:
            self.samples.append((raw.copy(), self.valves.copy(), command.copy()))
            if self.event == 2 and len(self.samples) <= 12:
                self.mount_identify()
                self.hyp = None
                for sample in self.samples:
                    self.hypothesis_sample(*sample)
            else:
                self.hypothesis_sample(raw, self.valves, command)
        if self.probe_count >= 23:
            self.locked = True
            self.moderate = int(np.argsort(np.where(np.arange(6) == self.severe, 2, self.scale))[0])
            self.armed = True
            maps = np.array([np.roll(self.mapping, offset) for offset in range(6)])
            self.track = {
                'maps': np.repeat(maps, 4, axis=0),
                'dirs': np.tile([-1, -1, 1, 1], 6),
                'armed': np.tile([False, True, False, True], 6),
                'cost': np.repeat(np.array([0, 8, 8, 8, 8, 8]), 4).astype(float),
            }

    def track_update(self, raw):
        track = self.track
        previous = self.states[-2]
        measured = self.matrix @ self.mount.T @ raw
        maps = track['maps']
        weights = self.scale[maps] * np.maximum(self.last_z[None, :], 1e-8) ** self.exponent[maps]
        errors = []
        columns = self.plume_columns(previous)
        predictions = self.matrix @ self.bias + np.einsum(
            'ncs,ns->nc', columns[:, maps].transpose(1, 0, 2), weights
        )
        errors.append(np.sum((predictions - measured) ** 2, axis=1))
        track['cost'] = 0.75 * track['cost'] + errors[0] / 0.00015
        track['cost'] -= np.min(track['cost'])
        self.last_map = maps[int(np.argmin(track['cost']))].copy()
        command = self.actions[-1]
        arm_slots = np.argmax(maps == self.moderate, axis=1)
        trigger_slots = np.argmax(maps == self.severe, axis=1)
        track['armed'][command[arm_slots] < 0.12] = True
        reverse = (command[trigger_slots] > 0.78) & track['armed']
        track['dirs'][reverse] *= -1
        track['armed'][reverse] = False
        stride = 2 if sum(command) > 1.35 else 1
        indices = (np.arange(6)[None, :] + stride * track['dirs'][:, None]) % 6
        track['maps'] = np.take_along_axis(maps, indices, axis=1)
        best = int(np.argmin(track['cost']))
        self.mapping = track['maps'][best].copy()
        self.direction = int(track['dirs'][best])
        self.armed = bool(track['armed'][best])

    def plume_columns(self, state, swirl=None):
        columns = self.matrix @ self.columns
        radius = np.linalg.norm(state[:2])
        field = np.clip(
            (self.plume_onset - radius)
            / max(self.plume_onset - self.port_radius, .05), 0, 1
        )
        target = -(rotation(state[2]).T @ state[:2]) / max(radius, 0.1)
        alignment = np.maximum(0, -target @ columns[:2] / np.maximum(np.linalg.norm(columns[:2], axis=0), 0.01))
        coupling = 0.35 + 0.65 * alignment ** 4
        loss = self.strength * field * coupling
        columns = columns.copy()
        linear = columns[:2].copy()
        columns *= 1 - loss
        columns[0] -= (self.swirl if swirl is None else swirl) * loss * linear[1]
        columns[1] += (self.swirl if swirl is None else swirl) * loss * linear[0]
        return columns

    def adapt(self, raw):
        self.track_update(raw)
        measured = self.matrix @ self.mount.T @ raw
        residual = measured - self.last_body
        self.model_error = float(np.linalg.norm(residual))
        state = self.states[-1]
        if np.linalg.norm(state[:2]) < 1.55:
            alternatives = {}
            for parity in (-1.0, 1.0):
                signs = parity * self.plume_base_sign
                columns = self.plume_columns(
                    self.states[-2], signs * self.swirl_magnitude
                )
                weights = self.scale[self.last_map] * np.maximum(self.last_z, 1e-8) ** self.exponent[self.last_map]
                prediction = self.matrix @ self.bias + columns[:, self.last_map] @ weights
                alternatives[parity] = float(np.sum((measured - prediction) ** 2))
                self.plume_parity_score[parity] = (
                    .82 * self.plume_parity_score[parity] + alternatives[parity]
                )
            self.plume_parity = min(
                (-1.0, 1.0), key=lambda parity: self.plume_parity_score[parity]
            )
            used_signs = self.plume_parity * self.plume_base_sign
            # The IMU sample describes the wrench before the environment advances
            # each passive cell.  Propagate that inferred six-bit mode through the
            # documented load integrators so allocation uses the mode that will act
            # on the next command, rather than a one-sample stale mode.
            previous = self.states[-2]
            far = self.matrix @ self.columns
            radius = np.linalg.norm(previous[:2])
            field = np.clip(
                (self.plume_onset - radius)
                / max(self.plume_onset - self.port_radius, .05), 0, 1
            )
            target_direction = -(rotation(previous[2]).T @ previous[:2]) / max(radius, 0.1)
            alignment = np.maximum(
                0,
                -target_direction @ far[:2]
                / np.maximum(np.linalg.norm(far[:2], axis=0), 0.01),
            )
            coupling = field * (0.35 + 0.65 * alignment ** 4)
            realized = np.maximum(self.last_z, 1e-8) ** self.exponent[self.last_map]
            load = np.zeros(6)
            load[self.last_map] = DT * coupling[self.last_map] * realized
            self.plume_energy += load
            flips = self.plume_energy >= self.plume_threshold
            self.plume_energy[flips] -= self.plume_threshold[flips]
            self.plume_base_sign[flips] *= -1.0
            self.swirl = (
                self.plume_parity * self.plume_base_sign * self.swirl_magnitude
            )
        else:
            weights = self.scale[self.last_map] * np.maximum(self.last_z, 1e-8) ** self.exponent[self.last_map]
            columns = (self.matrix @ self.columns)[:, self.last_map]
            prediction = self.matrix @ self.bias + columns @ weights
            jacobian = np.zeros((3, 12))
            for slot in range(6):
                nozzle = self.last_map[slot]
                jacobian[:, nozzle] = columns[:, slot] * weights[slot]
                jacobian[:, nozzle + 6] = columns[:, slot] * weights[slot] * math.log(max(self.last_z[slot], 1e-5))
            covariance = self.parameter_cov
            gain = np.linalg.solve(jacobian @ covariance @ jacobian.T + np.eye(3) * 0.00009, jacobian @ covariance).T
            correction = gain @ (measured - prediction)
            correction = np.clip(correction, -0.06, 0.06)
            self.scale *= np.exp(correction[:6])
            self.scale = np.clip(self.scale, 0.28, 1.1)
            self.exponent = np.clip(self.exponent + correction[6:], 0.63, 1.65)
            self.parameter_cov = covariance - gain @ jacobian @ covariance + np.eye(12) * 0.00002

    def allocate(self, desired, state):
        columns = self.plume_columns(state)[:, self.mapping]
        scale = self.scale[self.mapping]
        exponent = self.exponent[self.mapping]
        lower_z = self.memory * self.valves
        maximum_duty = 0.50 if np.linalg.norm(state[:2]) < 1.55 else 1.0
        upper_z = lower_z + (1 - self.memory) * maximum_duty
        lower = scale * lower_z ** exponent
        upper = scale * upper_z ** exponent
        radial = rotation(state[2]).T @ state[:2] / np.linalg.norm(state[:2])
        weighting = np.array([[3 * radial[0], 3 * radial[1], 0], [-radial[1], radial[0], 0], [0, 0, 3.0]])
        design = weighting @ columns
        target = weighting @ (desired - self.matrix @ self.bias)
        realized = np.clip(np.linalg.lstsq(design, target, rcond=None)[0], lower, upper)
        gram = design.T @ design
        diagonal = np.diag(gram) + 0.00004
        rhs = design.T @ target
        for _ in range(28):
            for slot in range(6):
                realized[slot] = np.clip(realized[slot] + (rhs[slot] - gram[slot] @ realized - 0.00004 * realized[slot]) / diagonal[slot], lower[slot], upper[slot])
        valves = (np.maximum(realized / scale, 0)) ** (1 / exponent)
        command = np.clip((valves - lower_z) / (1 - self.memory), 0, 1)
        goal = self.port_radius * np.array([math.cos(self.phase), math.sin(self.phase)])
        allocation_error = np.linalg.norm(weighting @ (columns @ realized + self.matrix @ self.bias - desired))
        terminal_tail = np.linalg.norm(state[:2]) < 1.55 and np.max(command) > 0.22
        if self.event and self.locked and (np.linalg.norm(state[:2] - goal) > 0.24 or allocation_error > 0.025 or terminal_tail):
            return self.allocate_template(desired, state, command, weighting)
        return command

    def allocate_template(self, desired, state, baseline, weighting):
        horizon = 6 if np.linalg.norm(state[:2]) < 1.55 else 12
        population = 96
        far_columns = self.matrix @ self.columns
        radius = np.linalg.norm(state[:2])
        field = np.clip(
            (1.47 - radius) / max(1.47 - self.port_radius, .05), 0, 1
        )
        target_direction = -(rotation(state[2]).T @ state[:2]) / max(radius, .1)
        alignment = np.maximum(
            0,
            -target_direction @ far_columns[:2]
            / np.maximum(np.linalg.norm(far_columns[:2], axis=0), .01),
        )
        coupling = field * (.35 + .65 * alignment ** 4)
        loss = self.strength * coupling
        bias = self.matrix @ self.bias
        initial_z = self.memory * self.valves + (1 - self.memory) * baseline
        template = np.zeros(12)
        template[self.mapping] = initial_z
        template[6:] = 1.0
        maximum = 0.5 if np.linalg.norm(state[:2]) < 1.55 else 1.0
        def score(templates):
            count = len(templates)
            maps = np.tile(self.mapping, (count, 1))
            dirs = np.full(count, self.direction)
            arms = np.full(count, self.armed)
            valves = np.tile(self.valves, (count, 1))
            energies = np.tile(self.plume_energy, (count, 1))
            mode_signs = np.tile(self.plume_base_sign, (count, 1))
            cumulative = np.zeros((count, 3))
            cost = np.zeros(count)
            first_commands = None
            for index in range(horizon):
                target_z = np.take_along_axis(templates[:, :6], maps, axis=1) * templates[:, 6:]
                command = np.clip((target_z - self.memory * valves) / (1 - self.memory), 0, maximum)
                if index == 0:
                    first_commands = command.copy()
                valves = self.memory * valves + (1 - self.memory) * command
                powers = np.maximum(valves, 1e-8) ** self.exponent[maps]
                thrust = self.scale[maps] * powers
                swirl = (
                    self.plume_parity * mode_signs * self.swirl_magnitude[None, :]
                )
                columns = np.broadcast_to(
                    far_columns[None, :, :], (count, 3, 6)
                ).copy()
                columns[:, 0, :] = (
                    (1 - loss)[None, :] * far_columns[0][None, :]
                    - swirl * loss[None, :] * far_columns[1][None, :]
                )
                columns[:, 1, :] = (
                    (1 - loss)[None, :] * far_columns[1][None, :]
                    + swirl * loss[None, :] * far_columns[0][None, :]
                )
                columns[:, 2, :] = (1 - loss)[None, :] * far_columns[2][None, :]
                routed_columns = np.take_along_axis(
                    columns, maps[:, None, :], axis=2
                )
                wrench = bias + np.einsum('ncs,ns->nc', routed_columns, thrust)
                error = (wrench - desired) @ weighting.T
                cumulative += error
                cost += np.sum(cumulative ** 2, axis=1) + 0.2 * np.sum(error ** 2, axis=1) + 0.0001 * np.sum(command ** 2, axis=1)
                loads = np.zeros((count, 6))
                loads[np.arange(count)[:, None], maps] = (
                    DT * coupling[maps] * powers
                )
                energies += loads
                flips = energies >= self.plume_threshold[None, :]
                energies[flips] -= np.broadcast_to(
                    self.plume_threshold, energies.shape
                )[flips]
                mode_signs[flips] *= -1.0
                arm_slot = np.argmax(maps == self.moderate, axis=1)
                trigger_slot = np.argmax(maps == self.severe, axis=1)
                arms[command[np.arange(count), arm_slot] < 0.12] = True
                reverse = (command[np.arange(count), trigger_slot] > 0.78) & arms
                dirs[reverse] *= -1
                arms[reverse] = False
                stride = np.where(np.sum(command, axis=1) > 1.35, 2, 1)
                indices = (np.arange(6)[None, :] + (stride * dirs)[:, None]) % 6
                maps = np.take_along_axis(maps, indices, axis=1)
            return cost, first_commands
        mean = template.copy()
        deviation = np.full(12, 0.28)
        best_template = template.copy()
        cost, action = score(template[None, :])
        best_cost = float(cost[0])
        best_action = action[0]
        for iteration in range(4):
            samples = np.clip(mean + self.random.normal(size=(population, 12)) * deviation, 0, 1)
            samples[0] = best_template
            samples[1] = template
            samples[2] = template * 0.5
            samples[3] = 0
            if self.plan is not None and np.shape(self.plan) == (12,):
                samples[4] = self.plan
            costs, actions = score(samples)
            elite_indices = np.argpartition(costs, 10)[:10]
            elite = samples[elite_indices]
            winner = int(np.argmin(costs))
            if costs[winner] < best_cost:
                best_cost = float(costs[winner])
                best_template = samples[winner].copy()
                best_action = actions[winner].copy()
            mean = 0.2 * mean + 0.8 * np.mean(elite, axis=0)
            deviation = np.maximum(0.015, 0.2 * deviation + 0.8 * np.std(elite, axis=0))
        self.plan = best_template
        return best_action

    def guidance(self, time):
        state = self.states[-1]
        phase = self.phase
        radial = np.array([math.cos(phase), math.sin(phase)])
        tangent = np.array([-radial[1], radial[0]])
        radius = np.linalg.norm(state[:2])
        outward = state[:2] / max(radius, 0.1)
        local_tangent = np.array([-outward[1], outward[0]])
        angle = math.atan2(state[1], state[0])
        angular_error = wrap(phase - angle)
        ready = self.event == 2 and self.locked
        desired_radius = self.port_radius + .035 if ready else (2.3 if not self.locked else 2.1)
        if ready and radius < 1.32 and abs(angular_error) < 0.07 and abs(outward @ state[3:5]) < 0.05:
            desired_radius = self.port_radius + .020
        if ready:
            desired_radius += min(0.65, max(0, abs(angular_error) - 0.05) * 0.65)
        if ready and (abs(wrap(phase - state[2])) > 0.16 or abs(angular_error) > 0.18):
            desired_radius = 1.65
        inward_limit = 0.26 if radius > 1.9 else (0.115 if radius > 1.6 else (0.065 if radius > 1.35 else 0.045))
        desired_radial = np.clip(0.6 * (desired_radius - radius), -inward_limit, 0.25)
        desired_tangential = radius * self.phase_rate + np.clip(0.8 * radius * angular_error, -0.28, 0.28)
        desired_tangential = np.clip(desired_tangential, -0.30, 0.30)
        desired_velocity = outward * (desired_radial + self.port_radius_rate) + local_tangent * desired_tangential
        actual_tangential = local_tangent @ state[3:5]
        accel = 1.25 * (desired_velocity - state[3:5]) - outward * actual_tangential ** 2 / radius
        radial_speed = outward @ state[3:5]
        if radius < .93:
            accel += outward * max(0, 1.4 * (.90 - radius) - 1.0 * radial_speed)
        alpha = 0.95 * wrap(phase - state[2]) + 1.7 * (self.phase_rate - state[5])
        accel = rotation(state[2]).T @ accel
        maximum = 0.125 if radius > 1.55 else 0.06
        if np.linalg.norm(accel) > maximum:
            accel *= maximum / np.linalg.norm(accel)
        return np.r_[accel, np.clip(alpha, -0.12, 0.12)]

    def act(self, time, observation):
        obs = np.array(observation, dtype=float)
        self.time = time
        encoded_port_radius = float(obs[15])
        self.port_sensor_history[time] = encoded_port_radius
        if obs[13]:
            stamp = max(0, time - int(obs[12]))
            if stamp in self.port_sensor_history:
                encoded_at_stamp = self.port_sensor_history[stamp]
                camera_radius = float(np.linalg.norm(obs[4:6]))
                direct_error = abs(camera_radius - encoded_at_stamp)
                mirrored_error = abs(
                    camera_radius - (2 * 1.14 - encoded_at_stamp)
                )
                self.port_encoder_sign = (
                    1.0 if direct_error <= mirrored_error else -1.0
                )
        measured_port_radius = (
            1.14 if self.port_encoder_sign is None else
            1.14 + self.port_encoder_sign * (encoded_port_radius - 1.14)
        )
        if self.previous_port_radius is not None:
            measured_rate = (measured_port_radius - self.previous_port_radius) / DT
            self.port_radius_rate = .20 * self.port_radius_rate + .80 * measured_rate
        self.previous_port_radius = measured_port_radius
        self.port_radius = measured_port_radius
        raw = obs[8:11]
        if time > 0:
            self.imu.append(raw.copy())
        if 15 <= time <= 100 and self.event < 2 and (time <= 30 or time % 4 == 0):
            self.calibrate(time)
        if self.event == 0 and time >= 58:
            residual = np.linalg.norm(raw - self.last_prediction)
            if residual > 0.045:
                self.start_event(time)
        elif self.event == 1 and self.locked and time >= 108 and time - self.event_time >= 24:
            weights = self.scale[self.last_map] * np.maximum(self.last_z, 1e-8) ** self.exponent[self.last_map]
            residual = np.linalg.norm(
                self.matrix @ self.mount.T @ raw - self.matrix @ self.bias
                - self.plume_columns(self.states[-1])[:, self.last_map] @ weights
            )
            self.change_count = self.change_count + 1 if residual > 0.05 and obs[13] < 0.5 else 0
            if self.change_count >= 2:
                self.start_event(time)
        self.estimate(time, obs)
        if self.event and not self.locked and time > self.event_time + 1:
            self.probe_update(raw)
        elif self.event and self.locked:
            self.adapt(raw)
        if time < 3:
            command = np.zeros(6)
        elif time < 15:
            command = np.zeros(6)
            command[(time - 3) // 2] = 0.85
        elif time < 18:
            command = np.zeros(6)
        elif self.event and not self.locked:
            command = np.zeros(6)
            slot = 0 if self.probe_count < 19 else 1
            command[slot] = [1.0, 0.40, 0.68, 0.95][self.probe_count % 4]
            self.probe_count += 1
        else:
            command = self.allocate(self.guidance(time), self.states[-1])
        if time <= 45 and time >= 8:
            design = np.c_[np.ones(len(self.actions)), np.array(self.actions)]
            fit = np.linalg.lstsq(design, np.array(self.imu), rcond=None)[0]
            self.bias = fit[0]
            self.columns = fit[1:].T
        perturbation = 0.0008 * np.tanh(obs[0] + 2 * obs[1] + obs[3])
        command = np.clip(command + perturbation, 0, 1)
        if self.event == 0:
            self.mapping = np.arange(6)
            self.valves = command.copy()
        else:
            self.valves = self.memory * self.valves + (1 - self.memory) * command
        self.last_z = self.valves.copy()
        self.last_map = self.mapping.copy()
        self.last_prediction = self.bias + self.columns[:, self.mapping] @ (self.scale[self.mapping] * self.valves ** self.exponent[self.mapping])
        self.last_body = self.matrix @ self.bias + self.plume_columns(self.states[-1])[:, self.mapping] @ (self.scale[self.mapping] * self.valves ** self.exponent[self.mapping])
        if self.event and self.locked:
            if command[int(np.where(self.mapping == self.moderate)[0][0])] < 0.12:
                self.armed = True
            if command[int(np.where(self.mapping == self.severe)[0][0])] > 0.78 and self.armed:
                self.direction *= -1
                self.armed = False
            stride = 2 if np.sum(command) > 1.35 else 1
            self.mapping = np.roll(self.mapping, -self.direction * stride)
        self.actions.append(command.copy())
        return command.tolist()


def main():
    controller = Controller()
    base = None
    best = None
    current_time = -1
    best_score = float('inf')
    print('READY', flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        kind = request.get('type')
        if kind == 'reset':
            controller = Controller()
            base = best = None
            current_time = -1
            best_score = float('inf')
            print('{"ok":true}', flush=True)
        elif kind == 'close':
            print('{"ok":true}', flush=True)
            break
        elif kind == 'act':
            time = int(request['t'])
            if time != current_time:
                if best is not None:
                    controller = best
                base = controller
                current_time = time
                best_score = float('inf')
            candidate = copy.deepcopy(base)
            action = candidate.act(time, request['obs'])
            if candidate.query_score < best_score:
                best = candidate
                best_score = candidate.query_score
            print(json.dumps({'action': action}), flush=True)


if __name__ == '__main__':
    main()
