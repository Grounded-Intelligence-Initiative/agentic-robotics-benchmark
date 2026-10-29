"""sim_bridge - the simulator side of sim_client's pipe.

Runs as a child of the grader (or of the practice grader / the agent's own tools) under the
simulator interpreter, `/opt/aerialsim/venv/bin/python` (Python 3.10). Speaks JSON lines:

    <- {"ready": true, "backend": ...}
    {"cmd": "reset", "episode": {...}, "budget": 1000, "success_radius": 3.0} -> state
    {"cmd": "step", "action": "forward"}                                    -> state
    {"cmd": "close"}                                                        -> {"ok": true}

where `episode` names the map, the objects to place (asset, position, rotation) and the
candidate start poses, and `state` is

    {"frame": {"rgb": {cam: b64 PNG}, "depth": {cam: b64 16-bit PNG, centimetres},
               "pose": [x, y, z, qx, qy, qz, qw]},
     "distance_to_goal": m, "move_distance": m, "step": n, "done": bool, "success": bool,
     "collision": bool, "oracle_success": bool, "termination": "stop"|"collision"|"max_steps"|"",
     "start_index": the candidate start used}

and, when the episode asks for it (`"clearance": true`, the grader's privileged mode only),
`"clearance": {move: metres}`: how far each level or vertical move could go from the current
pose before touching geometry, swept the way a move is, up to CLEAR_REACH_M.

`state` is privileged: only `frame` (minus `pose`) and odometry derived from `pose` reach the
agent's controller, and that filtering happens on the grader side (`sar_protocol`).

Two backends:

* `airsim` - the real thing: a TravelUAV scene (Unreal Engine with the AirSim plugin)
  rendered offscreen, the targets spawned into it, a kinematic drone moved by swept
  teleports (see `AirSimBackend`).
* `mock` - a stdlib-only toy world with the same state machine, for testing the grader,
  the wire and the oracle's control flow on a machine without a GPU or Unreal.

Shared byte-identically by the grading stage and the agent payload.
"""

import argparse
import base64
import json
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import zlib

CAMERAS = ("front", "left", "right", "down")


def _emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def _log(message):
    sys.stderr.write("[sim_bridge] {}\n".format(message))
    sys.stderr.flush()


def _distance(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a[:3], b[:3])))


# =========================================================================== AirSim RPC
class RpcError(RuntimeError):
    pass


class AirSimRpc(object):
    """A minimal synchronous client for AirSim's RPC server (msgpack-rpc over TCP). The
    official `airsim` package drags in tornado 4 and an old msgpack-rpc whose decoding makes
    image calls take seconds; this speaks the same protocol with `msgpack` alone. Structs
    travel as maps keyed by AirSim's field names, as the official types send them."""

    def __init__(self, port, timeout=60.0):
        import msgpack
        self.msgpack = msgpack
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.unpacker = msgpack.Unpacker(raw=False, max_buffer_size=1 << 30)
        self.next_id = 0

    def call(self, method, *args):
        self.next_id += 1
        self.sock.sendall(self.msgpack.packb([0, self.next_id, method, list(args)],
                                             use_bin_type=True))
        return self.answer(method)

    def answer(self, method):
        """The answer to the last call; after a timeout, it can be waited for again."""
        while True:
            for kind, rid, error, result in self.unpacker:
                if kind == 1 and rid == self.next_id:
                    if error is not None:
                        raise RpcError("{}: {}".format(method, error))
                    return result
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise RpcError("{}: connection closed".format(method))
            self.unpacker.feed(chunk)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _airsim_pose(position, q_xyzw):
    return {"position": {"x_val": float(position[0]), "y_val": float(position[1]),
                         "z_val": float(position[2])},
            "orientation": {"w_val": float(q_xyzw[3]), "x_val": float(q_xyzw[0]),
                            "y_val": float(q_xyzw[1]), "z_val": float(q_xyzw[2])}}


def _yaw_quat(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# =========================================================================== real backend
class AirSimBackend(object):
    """TravelUAV's packaged Unreal scenes with the AirSim plugin. One scene runs at a time
    (missions arrive grouped by map). The drone is kinematic: every move is a teleport,
    swept in quarter-metre sub-steps, and the first sub-step that touches geometry ends the
    mission as a collision. Targets are spawned assets; nothing is drawn for the decoys."""

    DRONE = "Drone_1"
    CLEAR_REACH_M = 4.0
    SCRIPT = {"ModernCityMap": "ModernCityMap"}
    SUBSTEPS = 4
    # Cameras hang 1 m under the airframe, so neither props nor arms are in view: three
    # level ones (yaw, pitch) and one looking straight down.
    MOUNT_DOWN_M = 1.0
    MOUNTS = {"front": (0.0, 0.0), "left": (-90.0, 0.0), "right": (90.0, 0.0),
              "down": (0.0, -90.0)}

    def __init__(self, args):
        import cv2  # noqa: F401  (fail early, before Unreal starts)
        import numpy  # noqa: F401
        self.np, self.cv2 = numpy, cv2
        self.args = args
        self._private = tempfile.mkdtemp(prefix="airsim-")     # 0700
        self.ue = self.rpc = self.map = None
        self.spawned = []
        self.stale = None
        self.spawn_count = 0        # names are never reused: re-spawning a name can crash AirSim

    # -- scene lifecycle ----------------------------------------------------------
    def _settings(self, port):
        size = int(self.args.image_size)
        cameras = {}
        for name, (yaw, pitch) in self.MOUNTS.items():
            cameras[name] = {"X": 0.0, "Y": 0.0, "Z": self.MOUNT_DOWN_M, "Pitch": pitch, "Roll": 0, "Yaw": yaw,
                             "CaptureSettings": [{"ImageType": t, "Width": size, "Height": size,
                                                  "FOV_Degrees": 90} for t in (0, 2)]}
        return {"SettingsVersion": 1.2, "SimMode": "Multirotor", "ClockSpeed": 1,
                "PhysicsEngineName": "ExternalPhysicsEngine", "ApiServerPort": port,
                "ViewMode": "NoDisplay",
                "Vehicles": {self.DRONE: {"VehicleType": "SimpleFlight", "AutoCreate": True,
                                          "X": 0, "Y": 0, "Z": -2, "Cameras": cameras}}}

    def _launch(self, map_name):
        self._stop_scene()
        port = _free_port()
        path = os.path.join(self._private, "settings.json")
        with open(path, "w") as handle:
            json.dump(self._settings(port), handle)
        script = os.path.join(self.args.scenes_root, self.SCRIPT.get(map_name, map_name) + ".sh")
        if not os.path.isfile(script):
            raise RuntimeError("no scene {} under {}".format(map_name, self.args.scenes_root))
        argv = ["bash", script, "-RenderOffscreen", "-NoSound", "-NoVSync",
                "-GraphicsAdapter={}".format(self.args.gpu), "-settings=" + path]
        self.ue = subprocess.Popen(argv, stdout=sys.stderr, stderr=sys.stderr,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.time() + 300
        while True:
            if self.ue.poll() is not None:
                raise RuntimeError("scene {} exited with code {}".format(map_name, self.ue.returncode))
            try:
                rpc = AirSimRpc(port)
                rpc.call("ping")
                break
            except (OSError, RpcError):
                if time.time() > deadline:
                    raise RuntimeError("scene {} did not answer within 300 s".format(map_name))
                time.sleep(1.0)
        self.rpc, self.map, self.spawned, self.port = rpc, map_name, [], port
        _log("scene {} up on port {}".format(map_name, port))

    def _stop_scene(self):
        if self.rpc is not None:
            self.rpc.close()
        if self.ue is not None and self.ue.poll() is None:
            try:
                os.killpg(self.ue.pid, 9)
            except OSError:
                pass
            self.ue.wait()
        self.ue = self.rpc = self.map = None

    # -- episode ------------------------------------------------------------------
    def reset(self, episode, budget, success_radius):
        if self.map != episode["map_name"]:
            self._launch(episode["map_name"])
        rpc = self.rpc
        for name in self.spawned:
            rpc.call("simDestroyObject", name)
        self.spawned = []
        for obj in episode["objects"]:
            w, x, y, z = obj["rotation_wxyz"]
            self.spawn_count += 1
            name = "target_{}".format(self.spawn_count)
            got = rpc.call("simSpawnObject", name, obj["asset"],
                           _airsim_pose(obj["position"], [x, y, z, w]),
                           {"x_val": 1.0, "y_val": 1.0, "z_val": 1.0}, False, False)
            self.spawned.append(got or name)
        index, lift = self._choose_start(episode["start_candidates"])
        start = episode["start_candidates"][index]
        x, y, z = start[0], start[1], start[2] - lift
        self.goal = [float(v) for v in episode["goal_position"]]
        self.budget, self.radius = int(budget), float(success_radius)
        self.s = dict(x=x, y=y, z=z, yaw=_yaw_of(start[3:7]), step=0, move=0.0, done=False,
                      success=False, collision=False, term="", start_index=index)
        self._settle(self.s)
        self.s["oracle"] = _distance((x, y, z), self.goal) <= self.radius
        self.clearance = bool(episode.get("clearance"))
        return self._state()

    LIFTS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)

    def _choose_start(self, candidates):
        """(index, lift): the first candidate, raised by the smallest lift that leaves free
        space around it: 2 m sideways and 4 m below in the depth images, no geometry on a
        swept 6 m in any level direction, 40 m up (open sky, not a floor or a roof a
        storey up) or 4 m down, and 20 m clear in at least one level direction (a way
        out, not a courtyard or a light well). The sweep finds what depth does not (glass),
        and a candidate that fell inside a building, whose floors and walls are never that
        far away."""
        for lift in self.LIFTS:
            for index, start in enumerate(candidates):
                s = dict(x=start[0], y=start[1], z=start[2] - lift, yaw=_yaw_of(start[3:7]))
                self._place(s)
                depth = self._images()[1]
                sides = min(float(self.np.percentile(depth[c], 1)) for c in ("front", "left", "right"))
                below = float(self.np.percentile(depth["down"], 1))
                if sides > 2.0 and below > 4.0 and self._free(s):
                    if lift:
                        _log("start {} raised by {:.0f} m".format(index, lift))
                    return index, lift
        _log("no candidate start is clear even raised; using the first")
        return 0, 0.0

    def _free(self, s, level=6.0, up=40.0, down=4.0, out=20.0):
        """Whether swept moves from s stay clear: `level` m in each of eight level
        directions, `up` m up and `down` m down, and `out` m in at least one level
        direction, in half-metre steps."""
        r = 0.7071
        levels = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                  (r, r, 0), (r, -r, 0), (-r, r, 0), (-r, -r, 0)]
        for d, reach in [(d, level) for d in levels] + [((0, 0, -1), up), ((0, 0, 1), down)]:
            if not self._sweep(s, d, reach):
                return False
        return any(self._sweep(s, d, out) for d in levels)

    def _sweep(self, s, d, reach):
        """Whether a swept move of `reach` m from s along d stays clear."""
        clear = True
        steps = int(reach * 2)
        for k in range(1, steps + 1):
            f = reach * k / steps
            p = (s["x"] + d[0] * f, s["y"] + d[1] * f, s["z"] + d[2] * f)
            self._place(dict(x=p[0], y=p[1], z=p[2], yaw=s["yaw"]), collide=True)
            if self._hit(p):
                clear = False
                break
        self._settle(s)
        return clear

    def _contact(self):
        """The collision record AirSim holds, as (flag, where the drone was, object)."""
        info = self.rpc.call("simGetCollisionInfo", self.DRONE)
        at = info["position"]
        return bool(info["has_collided"]), (at["x_val"], at["y_val"], at["z_val"]), info.get("object_name")

    def _settle(self, s):
        """Put the drone back at s after sweeps that were not moves (start search,
        clearances), with collisions on: a placement that touches nothing is what clears
        AirSim's record, which otherwise keeps the sweep's last contact, and a later contact
        at the very same pose would read as that old one. Were s itself in contact, the
        record is remembered so it does not count against the next move."""
        self._place(s, collide=True)
        flag, at, _ = self._contact()
        self.stale = at if flag else None

    def _hit(self, want):
        """Whether the swept placement to `want` (x, y, z) touched geometry: a fresh collision
        record with the drone at most a sub-step short of `want` (a blocked sweep stops at
        the contact). The record's time stamp is no use, as the scene's clock stands still
        between the frames we ask for."""
        flag, at, self.hit_name = self._contact()
        return flag and at != self.stale and _distance(at, want) <= 0.6

    def _place(self, s, collide=False):
        self.rpc.call("simSetVehiclePose",
                      _airsim_pose((s["x"], s["y"], s["z"]), _yaw_quat(s["yaw"])),
                      not collide, self.DRONE)
        self._frame()

    FRAME_TIMEOUT = 10.0

    def _frame(self):
        """Advance the scene one frame. Now and then (about once in 50,000 calls) AirSim
        misses the frame it was to pause after: the scene runs on unpaused and the call
        never returns. Pausing the scene from a second connection lets it return; the
        stalled connection is kept open until it has, as AirSim crashes answering a call
        on a connection that is gone. The scene ran on meanwhile, so the frame was had."""
        self.rpc.sock.settimeout(self.FRAME_TIMEOUT)
        try:
            self.rpc.call("simContinueForFrames", 1)
        except socket.timeout:
            _log("frame call stalled; pausing the scene from a second connection")
            side = AirSimRpc(self.port, timeout=self.FRAME_TIMEOUT)
            try:
                side.call("simPause", True)
            finally:
                side.close()
            self.rpc.answer("simContinueForFrames")
        finally:
            self.rpc.sock.settimeout(60.0)

    @staticmethod
    def _deltas(yaw):
        c, n = math.cos(yaw), math.sin(yaw)
        return {"forward": (c, n, 0.0), "left": (n, -c, 0.0), "right": (-n, c, 0.0),
                "ascend": (0.0, 0.0, -1.0), "descend": (0.0, 0.0, 1.0)}

    def _clearances(self):
        """{move: free metres}: each move swept from the current pose in the same sub-steps
        as a real move, up to CLEAR_REACH_M. The drone is put back afterwards."""
        s, out = self.s, {}
        for move, d in self._deltas(s["yaw"]).items():
            self._settle(s)
            free = self.CLEAR_REACH_M
            for k in range(1, int(self.CLEAR_REACH_M * self.SUBSTEPS) + 1):
                f = float(k) / self.SUBSTEPS
                p = (s["x"] + d[0] * f, s["y"] + d[1] * f, s["z"] + d[2] * f)
                self._place(dict(x=p[0], y=p[1], z=p[2], yaw=s["yaw"]), collide=True)
                if self._hit(p):
                    free = float(k - 1) / self.SUBSTEPS
                    break
            out[move] = free
        self._settle(s)
        return out

    def step(self, action):
        s = self.s
        if s["done"]:
            return self._state()
        delta = self._deltas(s["yaw"]).get(action)
        if action == "rotl":
            s["yaw"] -= math.radians(15)
            self._place(s)
        elif action == "rotr":
            s["yaw"] += math.radians(15)
            self._place(s)
        elif delta is not None:
            for _ in range(self.SUBSTEPS):
                s["x"] += delta[0] / self.SUBSTEPS
                s["y"] += delta[1] / self.SUBSTEPS
                s["z"] += delta[2] / self.SUBSTEPS
                s["move"] += 1.0 / self.SUBSTEPS
                self._place(s, collide=True)
                if self._hit((s["x"], s["y"], s["z"])):
                    _log("collision with {} at ({:.1f}, {:.1f}, {:.1f})".format(
                        self.hit_name, s["x"], s["y"], s["z"]))
                    s.update(collision=True, success=False, oracle=False, done=True,
                             term="collision")
                    break
        if action != "stop":
            s["step"] += 1
        if not s["done"]:
            within = _distance((s["x"], s["y"], s["z"]), self.goal) <= self.radius
            s["oracle"] = s["oracle"] or within
            if action == "stop":
                s.update(success=within, done=True, term="stop")
            elif s["step"] >= self.budget:
                s.update(done=True, term="max_steps")
        return self._state()

    # -- observation -----------------------------------------------------------------
    def _images(self):
        np = self.np
        requests = []
        for cam in CAMERAS:
            requests.append({"camera_name": cam, "image_type": 0, "pixels_as_float": False,
                             "compress": False})
            requests.append({"camera_name": cam, "image_type": 2, "pixels_as_float": True,
                             "compress": False})
        responses = self.rpc.call("simGetImages", requests, self.DRONE, False)
        rgb, depth = {}, {}
        for i, cam in enumerate(CAMERAS):
            image, dist = responses[2 * i], responses[2 * i + 1]
            h, w = image["height"], image["width"]
            rgb[cam] = np.frombuffer(image["image_data_uint8"], np.uint8).reshape(h, w, -1)[:, :, :3]
            depth[cam] = np.asarray(dist["image_data_float"], np.float32).reshape(
                dist["height"], dist["width"])
        return rgb, depth

    def _state(self):
        np, cv2, size = self.np, self.cv2, int(self.args.image_size)
        rgb, depth = self._images()
        out_rgb, out_depth = {}, {}
        for cam in CAMERAS:
            image = cv2.resize(rgb[cam], (size, size), interpolation=cv2.INTER_AREA)  # BGR
            out_rgb[cam] = base64.b64encode(cv2.imencode(".png", image)[1].tobytes()).decode("ascii")
            metres = cv2.resize(depth[cam], (size, size), interpolation=cv2.INTER_NEAREST)
            metres = np.nan_to_num(metres, nan=0.0, posinf=655.35, neginf=0.0)
            centimetres = np.clip(np.round(metres * 100.0), 0, 65535).astype(np.uint16)
            out_depth[cam] = base64.b64encode(cv2.imencode(".png", centimetres)[1].tobytes()).decode("ascii")
        s = self.s
        return {
            "frame": {"rgb": out_rgb, "depth": out_depth,
                      "pose": [s["x"], s["y"], s["z"]] + _yaw_quat(s["yaw"])},
            "distance_to_goal": _distance((s["x"], s["y"], s["z"]), self.goal),
            "move_distance": s["move"], "step": s["step"], "done": s["done"],
            "success": s["success"], "collision": s["collision"],
            "oracle_success": s["oracle"], "termination": s["term"],
            "start_index": s["start_index"],
            "clearance": self._clearances() if self.clearance and not s["done"] else None,
        }

    def close(self):
        self._stop_scene()


def _yaw_of(q_xyzw):
    x, y, z, w = q_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _free_port():
    for _ in range(50):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        if port < 60000:
            return port
    return 41451


# =========================================================================== mock backend
def _png(width, height, rows, bit16=False):
    """A minimal PNG writer: rows are lists of pixel tuples (RGB) or ints (16-bit grey)."""
    raw = bytearray()
    for row in rows:
        raw.append(0)
        if bit16:
            for value in row:
                raw += struct.pack(">H", int(value))
        else:
            for pixel in row:
                raw += bytes(pixel)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 16 if bit16 else 8, 0 if bit16 else 2,
                         0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) +
            chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))


class MockBackend(object):
    """A flat world with the target drawn as a red square: enough to exercise every code
    path of the grader and the wire. Ground at z = 0 (NED: the drone flies at z < 0)."""

    SKY, GRASS = (135, 180, 230), (70, 120, 60)
    COLOURS = {"red": (220, 30, 30), "blue": (30, 30, 220), "yellow": (230, 220, 20),
               "green": (20, 200, 40), "magenta": (220, 30, 220), "cyan": (20, 210, 220),
               "orange": (240, 140, 10), "white": (250, 250, 250), "black": (5, 5, 5)}

    def __init__(self, args):
        self.size = min(int(args.image_size), 64)
        self.state = None

    def reset(self, episode, budget, success_radius):
        x, y, z, qx, qy, qz, qw = [float(v) for v in episode["start_candidates"][0]]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.goal = [float(v) for v in episode["goal_position"]]
        drawn = episode.get("mock_targets") or [{"position": self.goal,
                                                 "description": episode["description"]}]
        self.targets = []
        for item in drawn:
            words = item["description"].lower().split()
            colour = next((self.COLOURS[w] for w in words if w in self.COLOURS),
                          self.COLOURS["red"])
            self.targets.append(([float(v) for v in item["position"]], colour))
        self.budget, self.radius = int(budget), float(success_radius)
        self.clearance = bool(episode.get("clearance"))
        d = _distance([x, y, z], self.goal)
        self.s = dict(x=x, y=y, z=z, yaw=yaw, step=0, move=0.0, done=False, success=False,
                      collision=False, oracle=d <= self.radius, term="")
        return self._state()

    def step(self, action):
        s = self.s
        if s["done"]:
            return self._state()
        before = (s["x"], s["y"], s["z"])
        c, n = math.cos(s["yaw"]), math.sin(s["yaw"])
        if action == "forward":
            s["x"] += c; s["y"] += n          # noqa: E702
        elif action == "backward":
            s["x"] -= c; s["y"] -= n          # noqa: E702
        elif action == "left":
            s["x"] += n; s["y"] -= c          # noqa: E702
        elif action == "right":
            s["x"] -= n; s["y"] += c          # noqa: E702
        elif action == "ascend":
            s["z"] -= 1.0
        elif action == "descend":
            s["z"] += 1.0
        elif action == "rotl":
            s["yaw"] -= math.radians(15)
        elif action == "rotr":
            s["yaw"] += math.radians(15)
        if action != "stop":
            s["step"] += 1
        s["move"] += _distance(before, (s["x"], s["y"], s["z"]))
        d = _distance((s["x"], s["y"], s["z"]), self.goal)
        within = d <= self.radius
        s["oracle"] = s["oracle"] or within
        if s["z"] > -0.5:
            s.update(collision=True, success=False, oracle=False, done=True, term="collision")
        elif action == "stop":
            s.update(success=within, done=True, term="stop")
        elif s["step"] >= self.budget:
            s.update(done=True, term="max_steps")
        return self._state()

    def _render(self, cam):
        """Pinhole projection, 90 degree FOV, same mounting as the real drone's cameras."""
        size, s = self.size, self.s
        f = size / 2.0
        rows = [[self.SKY if r < size // 2 else self.GRASS for _ in range(size)]
                for r in range(size)]
        if cam == "down":       # flat ground at z = 0, camera 1 m under the body
            ground = int(max(0.0, -s["z"] - 1.0) * 100)
            depth = [[ground] * size for _ in range(size)]
        else:
            depth = [[0 if r < size // 2 else 2000 for _ in range(size)] for r in range(size)]
        c, n = math.cos(s["yaw"]), math.sin(s["yaw"])
        for goal, colour in self.targets:
            dx, dy, dz = goal[0] - s["x"], goal[1] - s["y"], goal[2] - s["z"]
            bf, br = c * dx + n * dy, -n * dx + c * dy          # body forward / right
            if cam == "down":
                cf, cr, cd = dz, br, -bf
            else:
                h = {"front": 0.0, "left": -math.pi / 2, "right": math.pi / 2}[cam]
                cf = math.cos(h) * bf + math.sin(h) * br
                cr = -math.sin(h) * bf + math.cos(h) * br
                cd = dz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if cf < 0.5 or abs(cr / cf) > 1 or abs(cd / cf) > 1 or dist > 60:
                continue
            col, row = int(f + f * cr / cf), int(f + f * cd / cf)
            half = max(1, int(size * 1.5 / max(dist, 1.0)))
            for r in range(max(0, row - half), min(size, row + half + 1)):
                for cc in range(max(0, col - half), min(size, col + half + 1)):
                    rows[r][cc] = colour
                    depth[r][cc] = int(dist * 100)
        return (base64.b64encode(_png(size, size, rows)).decode("ascii"),
                base64.b64encode(_png(size, size, depth, bit16=True)).decode("ascii"))

    def _state(self):
        s = self.s
        rgb, depth = {}, {}
        for cam in CAMERAS:
            rgb[cam], depth[cam] = self._render(cam)
        half = s["yaw"] / 2
        return {
            "frame": {"rgb": rgb, "depth": depth,
                      "pose": [s["x"], s["y"], s["z"], 0.0, 0.0, math.sin(half), math.cos(half)]},
            "distance_to_goal": _distance((s["x"], s["y"], s["z"]), self.goal),
            "move_distance": s["move"], "step": s["step"], "done": s["done"],
            "success": s["success"], "collision": s["collision"],
            "oracle_success": s["oracle"], "termination": s["term"], "start_index": 0,
            "clearance": ({"forward": 4.0, "left": 4.0, "right": 4.0, "ascend": 4.0,
                           "descend": max(0.0, min(4.0, -s["z"] - 0.5))}
                          if self.clearance else None),
        }

    def close(self):
        pass


# =========================================================================== main loop
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("airsim", "mock"), default="airsim")
    parser.add_argument("--scenes-root", default="/opt/aerialsim/scenes/closeloop_envs")
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--image-size", type=int, default=256)
    args = parser.parse_args()

    # Library chatter must never reach the pipe: keep the real stdout for replies only.
    reply_fd = os.dup(1)
    os.dup2(2, 1)
    sys.stdout = os.fdopen(reply_fd, "w", buffering=1)

    try:
        backend = (MockBackend if args.backend == "mock" else AirSimBackend)(args)
    except Exception as exc:  # noqa: BLE001
        _emit({"ready": False, "error": "{}: {}".format(type(exc).__name__, exc)})
        return 1
    _emit({"ready": True, "backend": args.backend, "image_size": args.image_size})
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
                cmd = message.get("cmd")
                if cmd == "reset":
                    _emit(backend.reset(message["episode"], message["budget"],
                                        message["success_radius"]))
                elif cmd == "step":
                    _emit(backend.step(message["action"]))
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
