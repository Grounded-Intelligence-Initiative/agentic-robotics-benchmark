"""sim_bridge - the simulator side of sim_client's pipe.

Runs as a child of the grader (or of the practice grader / the agent's own tools) under the
simulator interpreter, `/opt/aerialsim/venv/bin/python` (Python 3.10). Speaks JSON lines:

    <- {"ready": true, "backend": ...}
    {"cmd": "reset", "episode": {...}, "budget": 150, "success_radius": 3.0} -> state
    {"cmd": "step", "action": "forward"}                                    -> state
    {"cmd": "close"}                                                        -> {"ok": true}

where `episode` names the map, the objects to place (asset, position, rotation) and the
candidate start poses, and `state` is

    {"frame": {"rgb": {cam: b64 PNG}, "depth": {cam: b64 16-bit PNG, centimetres},
               "pose": [x, y, z, qx, qy, qz, qw]},
     "distance_to_goal": m, "move_distance": m, "step": n, "done": bool, "success": bool,
     "collision": bool, "oracle_success": bool, "termination": "stop"|"collision"|"max_steps"|"",
     "start_index": the candidate start used}

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
        self.rpc, self.map, self.spawned = rpc, map_name, []
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
        index = self._choose_start(episode["start_candidates"])
        start = episode["start_candidates"][index]
        x, y, z = start[:3]
        self.goal = [float(v) for v in episode["goal_position"]]
        self.budget, self.radius = int(budget), float(success_radius)
        self.s = dict(x=x, y=y, z=z, yaw=_yaw_of(start[3:7]), step=0, move=0.0, done=False,
                      success=False, collision=False, term="", start_index=index)
        self._place(self.s)
        self.s["oracle"] = _distance((x, y, z), self.goal) <= self.radius
        self.last_hit = self.rpc.call("simGetCollisionInfo", self.DRONE)["time_stamp"]
        return self._state()

    def _choose_start(self, candidates):
        """The first candidate with free space around it (2 m sideways, 4 m below)."""
        for index, start in enumerate(candidates):
            self._place(dict(x=start[0], y=start[1], z=start[2], yaw=_yaw_of(start[3:7])))
            depth = self._images()[1]
            sides = min(float(self.np.percentile(depth[c], 1)) for c in ("front", "left", "right"))
            below = float(self.np.percentile(depth["down"], 1))
            if sides > 2.0 and below > 4.0:
                return index
            _log("start {} blocked: {:.1f} m to the sides, {:.1f} m below".format(index, sides, below))
        _log("no candidate start is clear; using the first")
        return 0

    def _place(self, s, collide=False):
        self.rpc.call("simSetVehiclePose",
                      _airsim_pose((s["x"], s["y"], s["z"]), _yaw_quat(s["yaw"])),
                      not collide, self.DRONE)
        self.rpc.call("simContinueForFrames", 1)

    def step(self, action):
        s = self.s
        if s["done"]:
            return self._state()
        c, n = math.cos(s["yaw"]), math.sin(s["yaw"])
        delta = {"forward": (c, n, 0.0), "left": (n, -c, 0.0), "right": (-n, c, 0.0),
                 "ascend": (0.0, 0.0, -1.0), "descend": (0.0, 0.0, 1.0)}.get(action)
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
                info = self.rpc.call("simGetCollisionInfo", self.DRONE)
                if info["has_collided"] and info["time_stamp"] != self.last_hit:
                    _log("collision with {} at ({:.1f}, {:.1f}, {:.1f})".format(
                        info.get("object_name"), s["x"], s["y"], s["z"]))
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
