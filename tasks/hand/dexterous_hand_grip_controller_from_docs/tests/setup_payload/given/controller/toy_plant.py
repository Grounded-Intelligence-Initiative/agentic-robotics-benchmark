"""A toy plant for the self-test (check_controller.py). It is NOT a model of the hand.

Thirteen toy servos on a toy bus, one 1 ms tick at a time. Each toy servo is a rigid motor with a position loop:
its motor current is (Goal Position - Present Position) x P / 128 mA per count, clipped to Goal Current; its torque is
proportional to that motor current (0.0012 N m per mA); its Present Current register reads the motor current; a toy
servo turns at a speed proportional to its net torque. Toy servo 0 presses a linear spring once it is past 30 deg from
its home. Every other register of the control table reads its printed initial value or 0. The toy cell runs short
jobs (a ticket, "present", "test_lift" after a grasp flag, a short "transfer", "set_down", "release", "rest") so that
a controller's phase logic is exercised; nothing in it is scored, and its numbers are not the hand's, the specimens'
or the cell's.

    plant = ToyPlant(record)       record: a unit record (the format of given/unit/example_record.json)
    plant.write(fields) -> refused [[field, servo], ...]     the "write" dict of a step reply (applied next tick)
    plant.read(reads) -> {key: [13 ints]}                    the registers lying wholly inside the read blocks
    plant.tactile_newest(after_seq) -> None | (seq, t_capture_ms, t_arrival_ms, {module: bytes})
    plant.cell() -> the "cell" block of an observation;  plant.flags(grasped, decline);  plant.advance(n)
"""

MODULE_TAXELS = {"thumb": 127, "index": 52, "middle": 52, "ring": 52}
TORQUE_NM_PER_MA = 0.0012
SPRING_NM_PER_DEG = 0.01
CONTACT_DEG = 30.0
DEG_PER_COUNT = 360.0 / 4096
INITIAL = {"status_return_level": 2, "velocity_i_gain": 1800, "velocity_p_gain": 50, "position_p_gain": 2000,
           "goal_pwm": 885, "goal_current": 910, "present_input_voltage": 120, "present_temperature": 30}
# address, size, key, signed (the vendor's control table, RAM area)
RAM = ((64, 1, "torque_enable", False), (65, 1, "led", False), (68, 1, "status_return_level", False),
       (69, 1, "registered_instruction", False), (70, 1, "hardware_error_status", False),
       (76, 2, "velocity_i_gain", False), (78, 2, "velocity_p_gain", False), (80, 2, "position_d_gain", False),
       (82, 2, "position_i_gain", False), (84, 2, "position_p_gain", False), (88, 2, "feedforward_2nd_gain", False),
       (90, 2, "feedforward_1st_gain", False), (98, 1, "bus_watchdog", False), (100, 2, "goal_pwm", True),
       (102, 2, "goal_current", True), (104, 4, "goal_velocity", True), (108, 4, "profile_acceleration", False),
       (112, 4, "profile_velocity", False), (116, 4, "goal_position", True), (120, 2, "realtime_tick", False),
       (122, 1, "moving", False), (123, 1, "moving_status", False), (124, 2, "present_pwm", True),
       (126, 2, "present_current", True), (128, 4, "present_velocity", True), (132, 4, "present_position", True),
       (136, 4, "velocity_trajectory", True), (140, 4, "position_trajectory", True),
       (144, 2, "present_input_voltage", False), (146, 1, "present_temperature", False), (147, 1, "backup_ready", False))
PHASES = (("await_clear", 200), ("present", 3000), ("test_lift", 1000), ("transfer", 1000), ("set_down", 300),
          ("release", 500), ("rest", 500))
TICKET = {"job": 1, "class": "brittle", "code": "SH-1", "finish": "matte", "profile": "T0"}


class ToyPlant(object):
    def __init__(self, record, frame_hz=88.0):
        self.t = 0
        self.home = [int(s["home_offset"]) for s in sorted(record["servos"], key=lambda s: s["id"])]
        self.q = [0.0] * 13                     # deg from home
        self.qd = [0.0] * 13
        self.reg = {k: [INITIAL.get(k, 0)] * 13 for _a, _n, k, _s in RAM}
        self.reg["goal_position"] = list(self.home)
        self.mode = [5] * 13
        self.pending = []
        self.frame_hz = float(frame_hz)
        self.seq = 0
        self.frames = []
        self.job = 0
        self.phase = "handover"
        self.phase_t0 = 0
        self.shift = 1

    # ---------------------------------------------------------------------------------------- bus
    def write(self, fields):
        refused = []
        te = fields.get("torque_enable") or [None] * 13
        mode = fields.get("operating_mode") or [None] * 13
        for i in range(13):
            on_after = self.reg["torque_enable"][i] == 1 and te[i] != 0
            if mode[i] is not None and on_after:
                refused.append(["operating_mode", i])
        self.pending.append((dict(fields), {tuple(r) for r in refused}))
        return refused

    def _apply(self, fields, refused):
        te = fields.get("torque_enable")
        if te:
            for i, v in enumerate(te):
                if v == 0:
                    self.reg["torque_enable"][i] = 0
        mode = fields.get("operating_mode")
        if mode:
            for i, v in enumerate(mode):
                if v is not None and ("operating_mode", i) not in refused:
                    self.mode[i] = int(v)
                    self.reg["position_p_gain"][i], self.reg["position_i_gain"][i] = 2000, 0
                    self.reg["position_d_gain"][i] = 0
                    self.reg["profile_velocity"][i] = self.reg["profile_acceleration"][i] = 0
                    self.reg["goal_current"][i] = 910
        for k in ("position_d_gain", "position_i_gain", "position_p_gain", "goal_current", "profile_acceleration",
                  "profile_velocity", "goal_position"):
            vals = fields.get(k)
            if vals:
                for i, v in enumerate(vals):
                    if v is not None:
                        self.reg[k][i] = max(-910, min(910, int(v))) if k == "goal_current" else int(v)
        if te:
            for i, v in enumerate(te):
                if v == 1:
                    self.reg["torque_enable"][i] = 1

    def read(self, reads):
        out = {}
        for a, n, k, _s in RAM:
            if any(b <= a and a + n <= b + m for b, m in reads):
                out[k] = list(self.reg[k])
        return out

    def tactile_newest(self, after_seq):
        best = None
        for fr in self.frames:
            if fr[0] > after_seq and fr[2] <= self.t:
                best = fr
        return best

    # ---------------------------------------------------------------------------------------- the toy cell
    def cell(self):
        shown = self.phase not in ("handover", "pause", "end")
        return {"shift": self.shift, "job": self.job if shown else None, "phase": self.phase,
                "t_phase_ms": self.t - self.phase_t0, "ticket": dict(TICKET, job=self.job) if shown else None}

    def flags(self, grasped, decline):
        if self.phase == "present" and decline:
            self._enter("release")
        elif self.phase == "present" and grasped:
            self._enter("test_lift")

    def _enter(self, ph):
        self.phase, self.phase_t0 = ph, self.t

    def _timeline(self):
        dt = self.t - self.phase_t0
        if self.phase == "handover" and self.t >= 500:
            self.job = 1
            self._enter("await_clear")
            return
        for i, (ph, dur) in enumerate(PHASES):
            if self.phase == ph and dt >= dur:
                if ph == "present":
                    self._enter("release")
                elif ph == "rest":
                    self.job = self.job % 10 + 1
                    self._enter("await_clear")
                else:
                    self._enter(PHASES[i + 1][0])
                return

    # ---------------------------------------------------------------------------------------- physics
    def advance(self, n):
        for _ in range(int(n)):
            self.t += 1
            pend, self.pending = self.pending, []
            for f, r in pend:
                self._apply(f, r)
            for i in range(13):
                on = self.reg["torque_enable"][i] == 1 and self.mode[i] in (3, 4, 5)
                err = self.reg["goal_position"][i] - (self.home[i] + self.q[i] / DEG_PER_COUNT)
                cur = err * self.reg["position_p_gain"][i] / 128.0 if on else 0.0
                lim = abs(self.reg["goal_current"][i])
                cur = max(-lim, min(lim, cur))
                tau = TORQUE_NM_PER_MA * cur
                if i == 0 and self.q[i] > CONTACT_DEG:
                    tau -= SPRING_NM_PER_DEG * (self.q[i] - CONTACT_DEG)
                self.qd[i] = 2000.0 * tau                  # deg/s: a heavily damped toy
                self.q[i] += self.qd[i] / 1000.0
                self.reg["present_current"][i] = int(round(cur))
                self.reg["present_pwm"][i] = int(max(-885, min(885, round(cur))))
                self.reg["present_position"][i] = int(round(self.home[i] + self.q[i] / DEG_PER_COUNT))
                self.reg["present_velocity"][i] = int(round(self.qd[i] / 6.0 / 0.229))
                self.reg["realtime_tick"][i] = self.t % 32768
            while self.seq * 1000.0 / self.frame_hz <= self.t:
                tc = self.seq * 1000.0 / self.frame_hz
                frame = {m: bytes(6 + 3 * n) for m, n in MODULE_TAXELS.items()}
                self.frames = (self.frames + [(self.seq, tc, tc + 9.6, frame)])[-8:]
                self.seq += 1
            self._timeline()
