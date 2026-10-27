"""drive_bridge - the simulator side of sim_client's pipe, for supervised driving.

Runs as a child of the grader (or of the practice grader / the agent's own tools) under the
CARLA client interpreter, `/opt/carla/venv/bin/python` (Python 3.10). JSON lines:

    <- {"ready": true, "backend": ...}
    {"cmd": "reset", "spec": {"town", "kind", "seed"}}                 -> state
    {"cmd": "step", "mode": 0|1, "control": [throttle, steer, brake]}  -> state
    {"cmd": "close"}                                                    -> {"ok": true}

`state` holds what the supervisor may see (the front camera frame, ego speed, navigation,
the nominal control), a pre-rendered probe frame, and a privileged `truth` block (route
completion, infractions, pace, termination) that the grading side keeps to itself
(drive_protocol.agent_observation filters it out).

A spec may name a `variant`: a different staging of its kind, defined in `drive_variants`
next to this file. The practice copy ships without that module and stages every kind in
its standard form.

Backends:

* `carla` - CARLA 0.9.15 in synchronous mode at 20 Hz, one control period = 0.1 s. Routes
  and scenarios are generated from the spec's seed; the nominal planner is the stock
  `agents.navigation.behavior_agent.BehaviorAgent` ('normal' profile) following that route.
* `mock` - a stdlib kinematic world (a straight two-lane road) with the same state
  machine and kinds (standard forms only), for testing the grader, the wire and a
  supervisor's control flow on a machine without a GPU or CARLA.

Shared byte-identically by the grading stage and the agent payload.
"""

import argparse
import base64
import json
import math
import os
import random
import socket
import struct
import subprocess
import sys
import time
import zlib

CAM_X, CAM_Z = 1.5, 1.6   # front camera: 1.5 m ahead of the centre, 1.6 m up, level, 90 deg
DT = 0.05                 # simulator step
TICKS_PER_STEP = 2        # control period 0.1 s
HARSH_DECEL = 7.0         # m/s^2
BLOCKED_S = 25.0
MAX_EXCUSED_S = 90.0      # waiting excused per episode (red lights, required waits)
OFF_ROUTE_M = 15.0
TIME_PER_M, TIME_EXTRA_S = 1 / 5.0, 15.0   # time limit: route length / 5 m/s + 15 s
REF_FRACTION = 0.85       # reference pace: this fraction of the speed limit ...
REF_CAP_KMH = 50.0        # ... counting limits above the nominal planner's own cap as the cap
NAV_SPACING_M = 30.0      # navigation targets: about every 30 m and where the command changes
HERE = os.path.dirname(os.path.abspath(__file__))


def _emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def _log(message):
    sys.stderr.write("[drive_bridge] {}\n".format(message))
    sys.stderr.flush()


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Tracker(object):
    """Route progress, pace and infractions, identical for both backends."""

    def __init__(self, route_xy, kind, variant=None, hazard_ids=()):
        self.route = route_xy
        self.hazard_ids = set(hazard_ids)    # the staged scenario's own actors
        self.hazard_hits = 0
        self.length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route_xy, route_xy[1:]))
        self.index = 0
        self.time = 0.0
        self.time_limit = self.length * TIME_PER_M + TIME_EXTRA_S
        self.kind = kind
        self.variant = variant
        self.collisions = {"walker": 0, "vehicle": 0, "static": 0}
        self.red_lights = 0
        self.harsh = 0
        self._harsh_active = False
        self._still = 0.0
        self.excused = 0.0       # time held at a red light or a required wait: no blame
        self.reference_m = 0.0   # distance at the reference pace over the unexcused time
        self.travelled = 0.0
        self._last_xy = None
        self._last_hit = {}
        self.termination = ""

    def collision(self, category, other_id):
        if self.time - self._last_hit.get(other_id, -10.0) < 1.0:
            self._last_hit[other_id] = self.time
            return
        self._last_hit[other_id] = self.time
        self.collisions[category] += 1
        if other_id in self.hazard_ids:
            self.hazard_hits += 1
        if category == "walker":
            self.termination = "pedestrian_collision"

    def update(self, x, y, speed, prev_speed, dt, limit_kmh, excused=False):
        self.time += dt
        if self._last_xy is not None:
            self.travelled += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        best, best_d = self.index, float("inf")
        for i in range(self.index, min(len(self.route), self.index + 25)):
            d = math.hypot(self.route[i][0] - x, self.route[i][1] - y)
            if d < best_d:
                best, best_d = i, d
        if best_d < 4.0:
            self.index = best
        decel = (prev_speed - speed) / dt
        if decel > HARSH_DECEL and not self._harsh_active:
            self.harsh += 1
        self._harsh_active = decel > HARSH_DECEL
        if excused and speed < 0.1 and self.excused < MAX_EXCUSED_S:
            self.excused += dt       # paused: counts towards neither "blocked", timeout nor pace
        else:
            self._still = self._still + dt if speed < 0.1 else 0.0
            self.reference_m += REF_FRACTION * min(limit_kmh, REF_CAP_KMH) / 3.6 * dt
        nearest = min(math.hypot(px - x, py - y) for px, py in self.route[max(0, self.index - 5):
                                                                         self.index + 30])
        if self.termination:
            pass
        elif self.completion >= 0.97:
            self.termination = "completed"
        elif self._still >= BLOCKED_S:
            self.termination = "blocked"
        elif nearest > OFF_ROUTE_M:
            self.termination = "off_route"
        elif self.time - self.excused >= self.time_limit:
            self.termination = "timeout"

    @property
    def completion(self):
        return self.index / float(max(1, len(self.route) - 1))

    @property
    def pace(self):
        if self.reference_m <= 0.0:
            return 0.0
        return min(1.5, self.completion * self.length / self.reference_m)

    def truth(self):
        return {"done": bool(self.termination), "termination": self.termination,
                "kind": self.kind, "variant": self.variant,
                "route_completion": round(self.completion, 4),
                "route_length_m": round(self.length, 1),
                "collisions": dict(self.collisions), "red_lights": self.red_lights,
                "hazard_collisions": self.hazard_hits,
                "harsh_brakes": self.harsh, "time_s": round(self.time, 2),
                "excused_s": round(self.excused, 2), "pace": round(self.pace, 4),
                "mean_speed": round(self.travelled / max(self.time, 1e-6), 2)}


def junction_commands(route_xy, in_junction):
    """Per route point: `lanefollow`, or for the points inside a junction what the route
    does through it (`left` / `right` / `straight`, from its heading change)."""
    n = len(route_xy)
    commands = ["lanefollow"] * n
    i = 0
    while i < n:
        if not in_junction[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and in_junction[j + 1]:
            j += 1
        a0, a1 = route_xy[max(0, i - 3)], route_xy[max(0, i - 1)]
        b0, b1 = route_xy[min(n - 1, j + 1)], route_xy[min(n - 1, j + 3)]
        turn = math.degrees(_wrap(math.atan2(b1[1] - b0[1], b1[0] - b0[0]) -
                                  math.atan2(a1[1] - a0[1], a1[0] - a0[0])))
        command = "left" if turn < -30 else "right" if turn > 30 else "straight"
        for k in range(i, j + 1):
            commands[k] = command
        i = j + 1
    return commands


def navigation(route_xy, options, index, x, y, yaw):
    """The next sparse navigation target ahead of route index `index`, in the car's frame
    ([forward, right] metres), and its command. Targets sit about every NAV_SPACING_M and
    wherever the command changes (a junction's entry and exit), like a leaderboard route."""
    marks, run = [0], 0.0
    for i in range(1, len(route_xy)):
        run += math.hypot(route_xy[i][0] - route_xy[i - 1][0], route_xy[i][1] - route_xy[i - 1][1])
        if options[i] != options[i - 1] or run >= NAV_SPACING_M:
            marks.append(i)
            run = 0.0
    marks.append(len(route_xy) - 1)
    target = next((m for m in marks if m > index + 2), len(route_xy) - 1)
    tx, ty = route_xy[target]
    dx, dy = tx - x, ty - y
    c, s = math.cos(yaw), math.sin(yaw)
    return {"target": [c * dx + s * dy, -s * dx + c * dy], "command": options[target]}


# =========================================================================== PNG (mock)
def _png(width, height, rows, bit16=False):
    raw = bytearray()
    for row in rows:
        raw.append(0)
        if bit16:
            for value in row:
                raw += struct.pack(">H", max(0, min(65535, int(value))))
        else:
            for pixel in row:
                raw += bytes(pixel)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 16 if bit16 else 8, 0 if bit16 else 2,
                         0, 0, 0)
    return base64.b64encode(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) +
                            chunk(b"IDAT", zlib.compress(bytes(raw), 6)) +
                            chunk(b"IEND", b"")).decode("ascii")


# =========================================================================== mock backend
class MockBackend(object):
    """A straight two-lane road along +x (ego lane centred on y = 0, the opposite lane on
    y = -3.5), 200 m long. Obstacles are boxes; the pedestrian a 0.6 m box. The nominal
    planner has the same blind spots as the CARLA one: it ignores static props and
    crossing traffic, reacts to pedestrians only within 3 m, and waits forever behind a
    stopped vehicle."""

    W, H = 64, 32
    LANE = 3.5

    def __init__(self, args):
        self.args = args

    def reset(self, spec):
        rng = random.Random(spec["seed"])
        self.kind = spec["kind"]
        self.probe_rng = random.Random(spec["seed"] ^ 0x5EED)
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.applied = [0.0, 0.0, 0.0]
        self.boxes = []        # dicts: x, y, hx, hy, vx, vy, cat, id, trigger
        self.junction = None
        h = rng.uniform(55, 80)
        if self.kind == "construction_zone":
            for dy in (-1.2, 0.0, 1.2):
                self._box(h, dy, 0.3, 0.3, "static")
            self._box(h + 2.0, 0.0, 0.3, 1.5, "static")
        elif self.kind == "broken_down_vehicle":
            self._box(h, 0.0, 2.3, 1.0, "vehicle")
        elif self.kind == "occluded_pedestrian":
            self._box(h, 3.2, 2.5, 1.0, "vehicle", hazard=False)      # parked van at the kerb
            self._box(h + 3.5, 3.4, 0.3, 0.3, "walker", trigger=("ped", rng.uniform(2.5, 3.5)))
        elif self.kind in ("red_light_runner", "oncoming_left_turn"):
            side = 1 if self.kind == "red_light_runner" else -1
            self._box(h, 30.0 * side, 2.3, 1.0, "vehicle", trigger=("cross", 9.0 * side))
            self.junction = h
        self.route = [(float(i), 0.0) for i in range(0, 201)]
        self.options = ["lanefollow"] * len(self.route)
        if self.kind == "oncoming_left_turn":       # the mock road stays straight
            self.options[int(h):int(h) + 12] = ["left"] * 12
        elif self.kind == "red_light_runner":
            self.options[int(h):int(h) + 12] = ["straight"] * 12
        self.tracker = Tracker(self.route, self.kind, spec.get("variant"),
                               [b["id"] for b in self.boxes if b["hazard"]])
        self._still_nominal = 0.0
        return self._state()

    def _box(self, x, y, hx, hy, cat, trigger=None, hazard=True):
        self.boxes.append({"x": x, "y": y, "hx": hx, "hy": hy, "vx": 0.0, "vy": 0.0,
                           "cat": cat, "id": len(self.boxes) + 1, "trigger": trigger,
                           "active": False, "hazard": hazard})

    # -- nominal planner (with its blind spots) -----------------------------
    def _nominal(self):
        target = 8.0
        for b in self.boxes:
            ahead = b["x"] - b["hx"] - (self.x + 2.3)
            in_lane = abs(b["y"] - self.y) < b["hy"] + 1.2
            if b["cat"] == "vehicle" and in_lane and 0 < ahead < 25 and b["vx"] == 0 and b["vy"] == 0:
                target = min(target, max(0.0, (ahead - 6.0) * 0.8))
            if b["cat"] == "walker" and in_lane and 0 < ahead < 3.0:
                return [0.0, self._steer_to(0.0), 1.0]
        err = target - self.v
        throttle = max(0.0, min(1.0, 0.4 * err)) if err > 0 else 0.0
        brake = max(0.0, min(1.0, -0.3 * err)) if err < 0 else 0.0
        if target == 0.0 and self.v < 0.5:
            brake = 1.0
        return [throttle, self._steer_to(0.0), brake]

    def _check_hit(self, b):
        if abs(b["x"] - self.x) < b["hx"] + 2.3 and abs(b["y"] - self.y) < b["hy"] + 1.0:
            self.tracker.collision(b["cat"], b["id"])
            if b["cat"] != "walker":
                self.v = 0.0

    def _steer_to(self, lane_y):
        lookahead = max(4.0, self.v * 1.0)
        heading = math.atan2(lane_y - self.y, lookahead)
        return max(-1.0, min(1.0, 2.0 * _wrap(heading - self.yaw)))

    # -- stepping ------------------------------------------------------------
    def step(self, mode, control):
        nominal = self._nominal()
        throttle, steer, brake = nominal if mode == 0 else control
        self.applied = [throttle, steer, brake]
        for _ in range(TICKS_PER_STEP):
            prev = self.v
            accel = 3.0 * throttle - 8.0 * brake - 0.3
            self.v = max(0.0, self.v + accel * DT)
            self.yaw += self.v / 2.9 * math.tan(1.22 * steer) * DT   # 70 degree max wheel angle
            self.x += self.v * math.cos(self.yaw) * DT
            self.y += self.v * math.sin(self.yaw) * DT
            for b in self.boxes:
                trig = b["trigger"]
                if trig and not b["active"]:
                    if trig[0] == "ped" and b["x"] - self.x < self.v * 1.8 + 5:
                        b["active"], b["vy"] = True, -trig[1]
                    if trig[0] == "cross" and self.junction - self.x < self.v * 3.3 + 1:
                        b["active"], b["vy"] = True, -trig[1]
                if trig and trig[0] == "ped" and b["active"]:
                    b.setdefault("pause", 3.0)
                    if b["y"] <= 0.0 and b["pause"] > 0.0:
                        b["pause"] -= DT
                        continue_moving = False
                    else:
                        continue_moving = True
                    if not continue_moving:
                        self._check_hit(b)
                        continue
                b["x"] += b["vx"] * DT
                b["y"] += b["vy"] * DT
                self._check_hit(b)
            self.tracker.update(self.x, self.y, self.v, prev, DT, limit_kmh=30.0)
            if self.tracker.termination:
                break
        return self._state()

    # -- rendering -------------------------------------------------------------
    def _render(self):
        W, H = self.W, self.H
        f = W / 2.0
        rgb = [[(140, 180, 230) if r < H // 2 else (90, 90, 90) for _ in range(W)] for r in range(H)]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        colours = {"static": (240, 120, 20), "vehicle": (30, 60, 200), "walker": (200, 30, 160)}
        cx = self.x + CAM_X * math.cos(self.yaw)
        cy = self.y + CAM_X * math.sin(self.yaw)
        for b in sorted(self.boxes, key=lambda b: -math.hypot(b["x"] - cx, b["y"] - cy)):
            dx, dy = b["x"] - cx, b["y"] - cy
            fwd, right = c * dx + s * dy, -s * dx + c * dy
            if fwd < 1.0 or fwd > 60.0:
                continue
            # visible half width: the box's extent across the line of sight
            sight = math.atan2(right, fwd) + self.yaw
            half_w = abs(b["hy"] * math.cos(sight)) + abs(b["hx"] * math.sin(sight))
            u0, u1 = int(f + f * (right - half_w) / fwd), int(f + f * (right + half_w) / fwd)
            v0, v1 = int(H / 2 + f * (-0.2) / fwd), int(H / 2 + f * CAM_Z / fwd)
            for r in range(max(0, v0), min(H, v1 + 1)):
                for col in range(max(0, u0), min(W, u1 + 1)):
                    rgb[r][col] = colours[b["cat"]]
        return rgb

    def _state(self):
        rgb = self._render()
        W, H, f = self.W, self.H, self.W / 2.0
        probe = [row[:] for row in rgb]
        dist = self.probe_rng.uniform(8.0, 25.0)       # a pedestrian-sized blob on the road
        u = int(f + f * self.probe_rng.uniform(-1.0, 1.0) / dist)
        v1, v0 = int(H / 2 + f * CAM_Z / dist), int(H / 2 + f * (CAM_Z - 1.7) / dist)
        half = max(1, int(f * 0.3 / dist))
        for r in range(max(0, v0), min(H, v1 + 1)):
            for col in range(max(0, u - half), min(W, u + half + 1)):
                probe[r][col] = (200, 30, 160)
        nav = navigation(self.route, self.options, self.tracker.index, self.x, self.y, self.yaw)
        return {
            "frame": {"image": _png(W, H, rgb)},
            "probe": {"image": _png(W, H, probe)},
            "ego": {"speed": round(self.v, 3), "applied": [round(v, 3) for v in self.applied]},
            "navigation": nav,
            "nominal": {"control": [round(v, 3) for v in self._nominal()]},
            "truth": self.tracker.truth(),
        }

    def close(self):
        pass


# =========================================================================== CARLA backend
class CarlaBackend(object):
    def __init__(self, args):
        self.args = args
        root = args.carla_root
        for path in (os.path.join(root, "PythonAPI", "carla"),):
            if path not in sys.path:
                sys.path.insert(0, path)
        import carla  # noqa: F401
        import numpy  # noqa: F401
        import cv2  # noqa: F401
        self.carla, self.np, self.cv2 = carla, numpy, cv2
        self.port = args.port or _free_port()
        argv = [os.path.join(root, "CarlaUE4.sh"), "-RenderOffScreen", "-nosound",
                "-quality-level={}".format(args.quality), "-carla-rpc-port={}".format(self.port),
                "-carla-streaming-port=0"]
        self.server = subprocess.Popen(argv, cwd=root, stdout=sys.stderr, stderr=sys.stderr,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.time() + 300
        while True:
            if self.server.poll() is not None:
                raise RuntimeError("CARLA exited with code {}".format(self.server.returncode))
            # A client made before the server listens never recovers: make one per attempt.
            self.client = carla.Client("127.0.0.1", self.port)
            self.client.set_timeout(10.0)
            try:
                self.client.get_server_version()
                break
            except RuntimeError:
                if time.time() > deadline:
                    raise RuntimeError("CARLA did not answer within 300 s")
                time.sleep(2.0)
        self.client.set_timeout(60.0)
        self.tm = self.client.get_trafficmanager(self.port + 50)
        self.world = None
        self.town = None
        self.actors = []
        self.sensors = []
        _log("CARLA {} up on port {}".format(self.client.get_server_version(), self.port))

    # -- world setup ----------------------------------------------------------
    def _load(self, town):
        carla = self.carla
        if self.town != town:
            self.world = self.client.load_world(town)
            self.town = town
            self.light_times = {light.id: (light.get_green_time(), light.get_yellow_time(),
                                           light.get_red_time())
                                for light in self.world.get_actors().filter("traffic.traffic_light")}
        else:
            self.world = self.client.get_world()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(True)
        self.map = self.world.get_map()
        # A staged scenario stretches some lights' phases; restore the town's own timing.
        for light in self.world.get_actors().filter("traffic.traffic_light"):
            green, yellow, red = self.light_times.get(light.id, (None, None, None))
            if green is not None:
                light.set_green_time(green)
                light.set_yellow_time(yellow)
                light.set_red_time(red)
        self.world.reset_all_traffic_lights()
        self.world.tick()
        return carla

    def _cleanup(self):
        for sensor in self.sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass
        # Destroying Traffic Manager vehicles in synchronous mode can deadlock the server:
        # release them and leave synchronous mode first (`_load` turns it back on).
        for actor in self.actors:
            if actor.type_id.startswith("vehicle."):
                try:
                    actor.set_autopilot(False, self.tm.get_port())
                except RuntimeError:
                    pass
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(False)
        ids = [a.id for a in self.sensors + self.actors]
        if ids:
            self.client.apply_batch_sync([self.carla.command.DestroyActor(i) for i in ids], False)
        self.sensors, self.actors = [], []
        self.walker_triggers, self.scripted = [], []

    def reset(self, spec):
        from agents.navigation.behavior_agent import BehaviorAgent
        from agents.navigation.local_planner import RoadOption
        self.RoadOption = RoadOption
        if self.world is not None:
            self._cleanup()
        carla = self._load(spec["town"])
        rng = random.Random(spec["seed"])
        self.probe_rng = random.Random(spec["seed"] ^ 0x5EED)
        self.tm.set_random_device_seed(spec["seed"] & 0x7FFFFFFF)
        self.walker_triggers, self.scripted = [], []
        self.excused = lambda: False          # a staged required wait replaces this
        weathers = ["ClearNoon", "CloudyNoon", "WetNoon", "ClearSunset", "WetCloudyNoon",
                    "SoftRainNoon"]
        self.world.set_weather(getattr(carla.WeatherParameters, rng.choice(weathers)))
        builder = ScenarioBuilder(self, rng, spec["kind"], spec.get("variant"),
                                  distractors=int(spec.get("distractors", 0)))
        route, ego_tf = builder.build()
        hazard_ids = [a.id for a in builder.hazards]
        self.ego = self._spawn("vehicle.lincoln.mkz_2020", ego_tf, role="hero")
        self._attach_sensors()
        self.nominal = BehaviorAgent(self.ego, behavior="normal")
        self.nominal.set_global_plan(route, stop_waypoint_creation=True, clean_queue=True)
        self._background_traffic(rng, route)
        self.route_xy = [(wp.transform.location.x, wp.transform.location.y) for wp, _ in route]
        self.options = junction_commands(self.route_xy, [wp.is_junction for wp, _ in route])
        self.tracker = Tracker(self.route_xy, spec["kind"], spec.get("variant"), hazard_ids)
        self.applied = [0.0, 0.0, 0.0]
        self.prev_speed = 0.0
        self.red_candidate = None
        for _ in range(10):          # settle the vehicle and fill the sensor queues
            self.ego.apply_control(carla.VehicleControl(brake=1.0))
            self.world.tick()
        self._nominal_control = self.nominal.run_step()
        return self._state()

    def _spawn(self, blueprint, transform, role=None, physics=True):
        bp = self.world.get_blueprint_library().find(blueprint)
        if role and bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role)
        actor = self.world.try_spawn_actor(bp, transform)
        if actor is None:
            transform.location.z += 0.5
            actor = self.world.spawn_actor(bp, transform)
        if not physics:
            actor.set_simulate_physics(False)
        self.actors.append(actor)
        return actor

    def _attach_sensors(self):
        import queue
        carla = self.carla
        lib = self.world.get_blueprint_library()
        self.queues = {}
        size = self.args.image_size
        bp = lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size))
        bp.set_attribute("image_size_y", str(size // 2))
        bp.set_attribute("fov", "90")
        tf = carla.Transform(carla.Location(x=CAM_X, z=CAM_Z), carla.Rotation())
        sensor = self.world.spawn_actor(bp, tf, attach_to=self.ego)
        q = queue.Queue()
        sensor.listen(q.put)
        self.sensors.append(sensor)
        self.queues["front"] = q
        bp = lib.find("sensor.other.collision")
        collision = self.world.spawn_actor(bp, carla.Transform(), attach_to=self.ego)
        collision.listen(self._on_collision)
        self.sensors.append(collision)

    def _on_collision(self, event):
        other = event.other_actor
        type_id = getattr(other, "type_id", "") or ""
        category = ("walker" if type_id.startswith("walker") else
                    "vehicle" if type_id.startswith("vehicle") else "static")
        self.tracker.collision(category, getattr(other, "id", 0))

    def _background_traffic(self, rng, route):
        start = route[0][0].transform.location
        points = [p for p in self.map.get_spawn_points() if p.location.distance(start) > 40]
        rng.shuffle(points)
        lib = self.world.get_blueprint_library().filter("vehicle.*")
        cars = [bp for bp in lib if int(bp.get_attribute("number_of_wheels")) == 4]
        for tf in points[:self.args.background]:
            actor = self.world.try_spawn_actor(rng.choice(cars), tf)
            if actor is not None:
                actor.set_autopilot(True, self.tm.get_port())
                self.actors.append(actor)

    # -- stepping ----------------------------------------------------------------
    def step(self, mode, control):
        carla = self.carla
        throttle, steer, brake = self._control_tuple(self._nominal_control) if mode == 0 else control
        self.applied = [throttle, steer, brake]
        vc = carla.VehicleControl(throttle=float(throttle), steer=float(steer), brake=float(brake))
        for _ in range(TICKS_PER_STEP):
            for hook in self.scripted:
                hook()
            self.ego.apply_control(vc)
            self.world.tick()
            speed = self._speed()
            loc = self.ego.get_location()
            at_red = self._red_light_check(loc, speed)
            self.tracker.update(loc.x, loc.y, speed, self.prev_speed, DT,
                                limit_kmh=self.ego.get_speed_limit() or 30.0,
                                excused=at_red or self.excused())
            self.prev_speed = speed
            if self.tracker.termination:
                break
        self._nominal_control = self.nominal.run_step()
        return self._state()

    @staticmethod
    def _control_tuple(vc):
        return [float(vc.throttle), float(vc.steer), float(vc.brake)]

    def _speed(self):
        v = self.ego.get_velocity()
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _red_light_check(self, loc, speed):
        """Count red lights run; return whether the car is held at a red (or yellow) light.

        A light counts as run when the car enters the junction while that light is red, so
        pulling away the moment it turns green is not an infraction."""
        carla = self.carla
        at_red = False
        if self.ego.is_at_traffic_light():
            light = self.ego.get_traffic_light()
            if light is not None:
                self.red_candidate = (light, self.tracker.time)
                at_red = light.get_state() in (carla.TrafficLightState.Red,
                                               carla.TrafficLightState.Yellow)
        if not at_red:
            # The planner looks ahead and stops for a red before the car reaches the light's
            # trigger volume; that wait is just as legitimate.
            at_red = bool(self.nominal.traffic_light_manager())
        if self.red_candidate:
            light, seen = self.red_candidate
            if speed > 0.5 and self.map.get_waypoint(loc).is_junction:
                if light.get_state() == carla.TrafficLightState.Red:
                    self.tracker.red_lights += 1
                self.red_candidate = None
            elif self.tracker.time - seen >= 3.0:
                self.red_candidate = None
        return at_red

    # -- observation -------------------------------------------------------------
    def _latest(self, key):
        q, item = self.queues[key], None
        while not q.empty():
            item = q.get_nowait()
        if item is None:
            item = q.get(timeout=5.0)
        return item

    def _probe_frame(self, bgr):
        """The frame with a road user pasted on the road ahead (8-25 m, within about a lane
        width of the centre line), sized for its distance by the ground-plane geometry."""
        np, cv2, rng = self.np, self.cv2, self.probe_rng
        if not hasattr(self, "_sprites"):
            folder = os.path.join(HERE, "probe_sprites")
            names = sorted(os.listdir(folder)) if os.path.isdir(folder) else []
            self._sprites = [(n, cv2.imread(os.path.join(folder, n), cv2.IMREAD_UNCHANGED))
                             for n in names if n.endswith(".png")]
            self._sprites = [(n, img) for n, img in self._sprites if img is not None and img.shape[2] == 4]
        h, w = bgr.shape[:2]
        f = w / 2.0
        dist = rng.uniform(8.0, 25.0)
        lateral = rng.uniform(-1.5, 1.5)
        out = bgr.copy()
        if self._sprites:
            name, sprite = rng.choice(self._sprites)
            metres = 1.5 if name.startswith("vehicle") else 1.75     # sprite height in metres
            ph = max(4, int(f * metres / dist))
            pw = max(2, int(sprite.shape[1] * ph / float(sprite.shape[0])))
            sprite = cv2.resize(sprite, (pw, ph), interpolation=cv2.INTER_AREA)
        else:
            ph, pw = max(4, int(f * 1.75 / dist)), max(2, int(f * 0.5 / dist))
            sprite = np.zeros((ph, pw, 4), np.uint8)
            sprite[:, :, :3], sprite[:, :, 3] = 40, 255
        bottom = int(h / 2.0 + f * CAM_Z / dist)
        left = int(f + f * lateral / dist - pw / 2.0)
        y0, x0 = max(0, bottom - ph), max(0, left)
        y1, x1 = min(h, bottom), min(w, left + pw)
        if y1 <= y0 or x1 <= x0:
            return out
        part = sprite[y0 - (bottom - ph):y1 - (bottom - ph), x0 - left:x1 - left].astype(np.float64)
        alpha = part[:, :, 3:4] / 255.0
        region = out[y0:y1, x0:x1].astype(np.float64)
        out[y0:y1, x0:x1] = (alpha * part[:, :, :3] + (1.0 - alpha) * region).astype(np.uint8)
        return out

    def _state(self):
        np, cv2 = self.np, self.cv2
        image = self._latest("front")
        bgr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, :3]
        encode = lambda img: base64.b64encode(cv2.imencode(".png", img)[1].tobytes()).decode("ascii")  # noqa: E731
        tf = self.ego.get_transform()
        nav = navigation(self.route_xy, self.options, self.tracker.index, tf.location.x,
                         tf.location.y, math.radians(tf.rotation.yaw))
        return {
            "frame": {"image": encode(bgr)},
            "probe": {"image": encode(self._probe_frame(bgr))},
            "ego": {"speed": round(self._speed(), 3), "applied": [round(v, 3) for v in self.applied]},
            "navigation": nav,
            "nominal": {"control": [round(v, 3) for v in self._control_tuple(self._nominal_control)]},
            "truth": self.tracker.truth(),
        }

    def close(self):
        try:
            if self.world is not None:
                self._cleanup()
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
        except Exception:  # noqa: BLE001
            pass
        if self.server.poll() is None:
            try:
                os.killpg(self.server.pid, 9)
            except OSError:
                pass


class ScenarioBuilder(object):
    """Route + staged hazard for one kind, from the episode's rng. Everything is placed
    relative to map waypoints, so it works in any town with suitable roads; a draw that
    does not fit is redrawn (bounded). A variant (see the module docstring) restages the
    kind through `drive_variants.stage(builder, route, junction)`."""

    ROUTE_M = 180.0
    HAZARD_M = (55.0, 80.0)

    def __init__(self, backend, rng, kind, variant=None, distractors=0):
        self.b, self.rng, self.kind, self.variant = backend, rng, kind, variant
        self.distractors = distractors
        self.carla, self.map, self.world = backend.carla, backend.map, backend.world
        self.RoadOption = backend.RoadOption
        self.hazards = []       # the staged actors; colliding with one zeroes the episode
        self.variants = None
        if variant:
            try:
                sys.path.insert(0, HERE)
                import drive_variants
            except ImportError:
                raise RuntimeError("variant {!r} needs drive_variants, which this copy of the "
                                   "bridge does not have".format(variant))
            self.variants = drive_variants

    def build(self):
        for attempt in range(80):
            self.hazards = []
            spawned = len(self.b.actors)
            try:
                result = self._attempt()
            except (_Redraw, RuntimeError):   # RuntimeError: a spawn collided; draw again
                self._discard(self.b.actors[spawned:])
                del self.b.actors[spawned:]
                self.b.scripted = []
                continue
            if result is not None:
                return result
        raise RuntimeError("no placement for {} in {} after 80 draws".format(
            self.kind, self.b.town))

    def _discard(self, actors):
        if actors:
            self.b.client.apply_batch_sync([self.carla.command.DestroyActor(a.id) for a in actors], False)

    # -- route --------------------------------------------------------------------
    def _turn(self, entry):
        """Degrees turned through the junction starting at `entry` (left < 0 < right)."""
        wp, guard = entry, 0
        while wp.is_junction and guard < 40:
            nxt = wp.next(2.0)
            if not nxt:
                break
            wp, guard = nxt[0], guard + 1
        return math.degrees(_wrap(math.radians(wp.transform.rotation.yaw - entry.transform.rotation.yaw)))

    def _route(self, want_turn):
        rng = self.rng
        start = self.map.get_waypoint(rng.choice(self.map.get_spawn_points()).location)
        wps, opts, total, wp = [start], [self.RoadOption.LANEFOLLOW], 0.0, start
        hazard_junction = None
        while total < self.ROUTE_M:
            nexts = wp.next(2.0)
            if not nexts:
                raise _Redraw()
            if len(nexts) > 1:
                turns = [(self._turn(n), n) for n in nexts]
                if want_turn and hazard_junction is None and total >= self.HAZARD_M[0] - 15:
                    if want_turn == "left":
                        fit = [n for a, n in turns if a < -45]
                    else:
                        fit = [n for a, n in turns if abs(a) < 30]
                    if not fit:
                        raise _Redraw()
                    choice = rng.choice(fit)
                    hazard_junction = (len(wps), wp)
                else:
                    straight = [n for a, n in turns if abs(a) < 30]
                    choice = rng.choice(straight or [n for _, n in turns])
                angle = self._turn(choice)
                option = (self.RoadOption.LEFT if angle < -45 else
                          self.RoadOption.RIGHT if angle > 45 else self.RoadOption.STRAIGHT)
            else:
                choice = nexts[0]
                option = self.RoadOption.LANEFOLLOW if not choice.is_junction else opts[-1]
            total += choice.transform.location.distance(wp.transform.location)
            wps.append(choice)
            opts.append(option)
            wp = choice
        if want_turn and hazard_junction is None:
            raise _Redraw()
        return list(zip(wps, opts)), hazard_junction

    def _at_distance(self, route, metres):
        total = 0.0
        for i in range(1, len(route)):
            total += route[i][0].transform.location.distance(route[i - 1][0].transform.location)
            if total >= metres:
                return i, route[i][0]
        raise _Redraw()

    # -- kinds ---------------------------------------------------------------------
    def _attempt(self):
        carla, rng = self.carla, self.rng
        want = {"red_light_runner": "straight", "oncoming_left_turn": "left"}.get(self.kind)
        if self.variants is not None:
            want = self.variants.junction_wanted(self.variant, want)
        route, junction = self._route(want)
        ego_tf = route[0][0].transform
        ego_tf = carla.Transform(ego_tf.location + carla.Location(z=0.5), ego_tf.rotation)
        if self.variants is not None:
            self.variants.stage(self, route, junction)
            return route, ego_tf
        if self.kind in ("construction_zone", "broken_down_vehicle", "occluded_pedestrian"):
            _, hwp = self._at_distance(route, rng.uniform(*self.HAZARD_M))
            if hwp.is_junction or any(w.is_junction for w in hwp.previous(12.0) + hwp.next(12.0)):
                raise _Redraw()
            if self.kind != "occluded_pedestrian" and not self._passable(hwp):
                raise _Redraw()
            getattr(self, "_stage_" + self.kind)(hwp)
            return route, ego_tf
        index, entry = junction
        getattr(self, "_stage_" + self.kind)(route, index, entry)
        return route, ego_tf

    def _passable(self, hwp, back_m=30.0):
        """A drivable lane on the left at the hazard and over the approach to it, so a car
        stopped in front of the hazard can always pull out around it."""
        wps = [hwp] + [w for d in range(2, int(back_m) + 1, 2) for w in hwp.previous(float(d))[:1]]
        if len(wps) < int(back_m) // 2 + 1:
            return False
        for w in wps:
            left = w.get_left_lane()
            if w.is_junction or left is None or left.lane_type != self.carla.LaneType.Driving:
                return False
        return True

    def _offset(self, wp, forward, right, z=0.0):
        tf = wp.transform
        f, r = tf.get_forward_vector(), tf.get_right_vector()
        loc = tf.location + self.carla.Location(f.x * forward + r.x * right,
                                                f.y * forward + r.y * right, z)
        return self.carla.Transform(loc, tf.rotation)

    def _stage_construction_zone(self, wp):
        for lateral in (-1.2, 0.0, 1.2):
            self.hazards.append(self.b._spawn("static.prop.constructioncone",
                                              self._offset(wp, 0.0, lateral, 0.1)))
        self.hazards.append(self.b._spawn("static.prop.streetbarrier", self._offset(wp, 2.5, 0.0, 0.1)))
        self.hazards.append(self.b._spawn("static.prop.trafficwarning", self._offset(wp, 6.0, 0.0, 0.1)))

    def _stage_broken_down_vehicle(self, wp):
        lib = self.world.get_blueprint_library().filter("vehicle.*")
        cars = [bp.id for bp in lib if int(bp.get_attribute("number_of_wheels")) == 4]
        car = self.b._spawn(self.rng.choice(cars), self._offset(wp, 0.0, 0.0, 0.3))
        self.hazards.append(car)
        car.apply_control(self.carla.VehicleControl(hand_brake=True, brake=1.0))
        lights = self.carla.VehicleLightState
        car.set_light_state(lights(lights.LeftBlinker | lights.RightBlinker))

    def _stage_occluded_pedestrian(self, wp):
        carla, rng = self.carla, self.rng
        kerb = wp.lane_width / 2.0
        van = self.b._spawn("vehicle.volkswagen.t2", self._offset(wp, 0.0, kerb + 1.2, 0.3),
                            physics=False)
        van.set_simulate_physics(False)
        walkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        bp = rng.choice(walkers)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        spot = self._offset(wp, 3.5, kerb + 1.6, 1.0)
        walker = self.world.spawn_actor(bp, spot)
        self.b.actors.append(walker)
        self.hazards.append(walker)
        speed = rng.uniform(2.5, 3.5)
        right = wp.transform.get_right_vector()
        direction = carla.Vector3D(-right.x, -right.y, 0.0)
        to_centre = (kerb + 1.6) / speed          # walking time to the ego lane centre
        pause = 3.0                                # hesitates there, then walks on
        state = {"started": False, "t0": 0.0}

        def hook():
            ego = self.b.ego.get_location()
            ahead = walker.get_location().distance(ego)
            v = self.b._speed()
            if not state["started"] and ahead < v * 1.8 + 5.0:
                state["started"], state["t0"] = True, self.b.tracker.time
            if state["started"]:
                elapsed = self.b.tracker.time - state["t0"]
                moving = elapsed < to_centre or to_centre + pause <= elapsed < to_centre + pause + 4.0
                walker.apply_control(carla.WalkerControl(direction=direction,
                                                         speed=speed if moving else 0.0))
        self.b.scripted.append(hook)

    def _conflict_vehicle(self, entry, angle_range, straight_through):
        """A vehicle approaching the same junction from another arm."""
        carla, rng = self.carla, self.rng
        junction = None
        probe = entry
        for _ in range(20):
            nxt = probe.next(2.0)
            if not nxt:
                break
            probe = nxt[0]
            if probe.is_junction:
                junction = probe.get_junction()
                break
        if junction is None:
            raise _Redraw()
        ego_yaw = entry.transform.rotation.yaw
        pairs = []
        for start, end in junction.get_waypoints(carla.LaneType.Driving):
            rel = abs(math.degrees(_wrap(math.radians(start.transform.rotation.yaw - ego_yaw))))
            turn = abs(math.degrees(_wrap(math.radians(end.transform.rotation.yaw - start.transform.rotation.yaw))))
            if angle_range[0] <= rel <= angle_range[1] and (turn < 30 if straight_through else True):
                pairs.append((start, end))
        if not pairs:
            raise _Redraw()
        start, end = rng.choice(pairs)
        back = start.previous(25.0)
        if not back:
            raise _Redraw()
        tf = back[0].transform
        lib = self.world.get_blueprint_library().filter("vehicle.*")
        cars = [bp.id for bp in lib if int(bp.get_attribute("number_of_wheels")) == 4]
        car = self.b._spawn(rng.choice(cars), carla.Transform(tf.location + carla.Location(z=0.5), tf.rotation))
        self.hazards.append(car)
        exit_wp = end.next(20.0)
        return car, start, (exit_wp[0] if exit_wp else end), junction

    def _stage_red_light_runner(self, route, index, entry):
        from agents.navigation.basic_agent import BasicAgent
        carla = self.carla
        car, start, goal, junction = self._conflict_vehicle(entry, (60, 120), True)
        lights = self.world.get_traffic_lights_from_waypoint(entry, 40.0)
        for light in lights:
            light.set_state(carla.TrafficLightState.Green)
            light.set_green_time(120.0)
        for light in self.world.get_traffic_lights_from_waypoint(start, 40.0):
            light.set_state(carla.TrafficLightState.Red)
            light.set_red_time(120.0)
        self._drive_when(car, goal, entry, lead_s=2.6, speed_kmh=40.0,
                         opts={"ignore_traffic_lights": True, "ignore_vehicles": True},
                         agent_cls=BasicAgent)

    def _stage_oncoming_left_turn(self, route, index, entry):
        from agents.navigation.basic_agent import BasicAgent
        carla = self.carla
        car, start, goal, junction = self._conflict_vehicle(entry, (150, 180), True)
        for light in (self.world.get_traffic_lights_from_waypoint(entry, 40.0) +
                      self.world.get_traffic_lights_from_waypoint(start, 40.0)):
            light.set_state(carla.TrafficLightState.Green)
            light.set_green_time(120.0)
        self._drive_when(car, goal, entry, lead_s=3.0, speed_kmh=35.0,
                         opts={"ignore_traffic_lights": True, "ignore_vehicles": True},
                         agent_cls=BasicAgent)

    def _drive_when(self, car, goal, ego_entry, lead_s, speed_kmh, opts, agent_cls):
        carla = self.carla
        agent = agent_cls(car, target_speed=speed_kmh, opt_dict=opts)
        agent.set_destination(goal.transform.location)
        car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
        state = {"go": False}

        def hook():
            ego = self.b.ego.get_location()
            distance = ego.distance(ego_entry.transform.location)
            if not state["go"] and distance < self.b._speed() * lead_s + 4.0:
                state["go"] = True
            if state["go"]:
                car.apply_control(agent.run_step())
        self.b.scripted.append(hook)


class _Redraw(Exception):
    pass


def _free_port():
    for _ in range(50):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        if 2000 <= port < 59000 and port % 2 == 0:
            return port
    return 2000


# =========================================================================== main loop
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("carla", "mock"), default="carla")
    parser.add_argument("--carla-root", default="/opt/carla/server")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--image-size", type=int, default=320,
                        help="camera width in pixels; height is half of it")
    # Low crashes CARLA 0.9.15 (signal 11) right after a town load on recent drivers.
    parser.add_argument("--quality", default="Epic", choices=("Low", "Epic"))
    parser.add_argument("--background", type=int, default=6,
                        help="background traffic vehicles (CARLA backend)")
    args = parser.parse_args()

    reply_fd = os.dup(1)
    os.dup2(2, 1)
    sys.stdout = os.fdopen(reply_fd, "w", buffering=1)

    try:
        backend = (MockBackend if args.backend == "mock" else CarlaBackend)(args)
    except Exception as exc:  # noqa: BLE001
        _emit({"ready": False, "error": "{}: {}".format(type(exc).__name__, exc)})
        return 1
    _emit({"ready": True, "backend": args.backend})
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
                cmd = message.get("cmd")
                if cmd == "reset":
                    _emit(backend.reset(message["spec"]))
                elif cmd == "step":
                    _emit(backend.step(int(message["mode"]), message["control"]))
                elif cmd == "close":
                    _emit({"ok": True})
                    break
                else:
                    _emit({"error": "unknown command {!r}".format(cmd)})
            except Exception as exc:  # noqa: BLE001
                _log("error: {}: {}".format(type(exc).__name__, exc))
                _emit({"error": "{}: {}".format(type(exc).__name__, exc)})
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
