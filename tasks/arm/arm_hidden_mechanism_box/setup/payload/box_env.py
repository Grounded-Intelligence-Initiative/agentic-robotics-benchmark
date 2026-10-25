"""Hidden-mechanism box: the task's robot environment (scene, robot, controller, sensors, credit).

This file is byte-identical in your dev sim, in the lab service and in the grader. What differs
between them is only the BOX that is loaded (a box spec = an MJCF body fragment + a stage list).
Your dev sim ships one practice box (`practice_box.json`); the lab and the grader load the hidden
boxes, whose specs you never see.

Frames and units
----------------
World frame of the LIBERO kitchen tabletop scene: the table top is the plane z = 0.80, the table
spans x, y in [-0.40, 0.40]. The Panda base stands at (-0.56, 0, 0.80) facing +x. Metres, radians,
seconds. Quaternions are (w, x, y, z).

Main entry points
-----------------
    env = BoxEnv(box_spec)                  # box_spec = json.load(open("practice_box.json"))
    obs = env.reset(seed)                   # any int seed: box placement jitter + sensor noise
    obs, done, info = env.step(action)      # 10 Hz
    env.episode_score()                     # credit of the finished episode (see `episode_score`)
    env.gt()                                # ground truth of the loaded box (dev use only)

Action (a list of 8 numbers):
    [dx, dy, dz, drx, dry, drz, grip, cams]
    dx..dz:   target displacement of the gripper site in the world frame (m; the vector is clipped
              to length 0.03);
    drx..drz: a world-frame rotation vector (rad; clipped to length 0.2) applied to the current
              gripper orientation; an operational-space controller tracks the resulting target;
    grip:     -1 = fully open (80 mm) ... +1 = fully closed; linear in between;
    cams:     integer bit mask of the cameras whose RGB image the NEXT observation carries
              (1 = left, 2 = right, 4 = wrist; 0 = none). Images cost wall time.
"""

import json
import math
import os
import threading

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
import mujoco  # noqa: E402

# ----------------------------------------------------------------------------------------------
# Public constants
# ----------------------------------------------------------------------------------------------
TIMESTEP = 0.002
SUBSTEPS = 50                    # -> 10 Hz control
CONTROL_DT = TIMESTEP * SUBSTEPS
MAX_STEPS = 2500                 # control steps per episode (250 s simulated)
MAX_DPOS = 0.03
MAX_DROT = 0.2
TABLE_Z = 0.80
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.45, -0.45, TABLE_Z + 0.005])
WORKSPACE_HI = np.array([0.45, 0.45, TABLE_Z + 0.55])
BOX_NOMINAL = np.array([0.04, 0.0])      # nominal box centre (x, y) on the table
BOX_JITTER_XY = np.array([0.04, 0.05])  # uniform +- per episode
BOX_JITTER_YAW = math.radians(25.0)     # uniform +- per episode (yaw 0: the box front faces -x)
HOLD_STEPS = 3                   # a stage (and its blockers) must stay unlocked this many control steps
FINAL_HOLD_STEPS = 3
RGB_NOISE = 2.0                  # 8-bit sigma
WRENCH_NOISE = 0.3               # N (torque: 0.03 N m)
INSTAB_ROBOT_QVEL = 8.0          # rad/s: an episode with faster robot joints ends as a fault
INSTAB_PART_QVEL = 30.0          # rad/s or m/s on any box joint
HOME_Q = np.array([0.0, -0.55, 0.0, -2.45, 0.0, 1.95, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])

CAMERAS = {
    # name: (position, look-at target, fovy deg, width, height); fixed cameras never move
    "left": ((-0.42, 0.42, 1.30), (0.0, 0.0, 0.86), 42.0, 320, 240),
    "right": ((-0.42, -0.42, 1.30), (0.0, 0.0, 0.86), 42.0, 320, 240),
}
WRIST = {"fovy": 75.0, "width": 256, "height": 192}
CAMERA_NAMES = ("left", "right", "wrist")


class EnvError(Exception):
    pass


_GL_LOCK = threading.RLock()


def _gl_release():
    """Release the shared offscreen GL context from this thread (EGL), so any thread can render next."""
    if os.environ.get("MUJOCO_GL", "egl").lower() != "egl":
        return
    import mujoco.egl as mjegl
    from mujoco.egl import egl_ext as EGL
    if mjegl.EGL_DISPLAY is not None:
        EGL.eglMakeCurrent(mjegl.EGL_DISPLAY, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)


class ActionError(ValueError):
    pass


# ----------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------
def look_at(pos, target, up=(0, 0, 1)):
    """Rotation whose columns are the MuJoCo camera axes (x right, y up, z backwards)."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, dtype=float), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def axis_angle_mat(v):
    v = np.asarray(v, dtype=float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def robosuite_assets():
    """robosuite's model assets (Panda + gripper) installed in the image; robosuite is not imported."""
    import importlib.util
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        raise EnvError("robosuite assets not installed")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def libero_assets():
    """LIBERO scene assets (Hugging Face `lerobot/libero-assets`), installed in the image."""
    for p in (os.environ.get("LIBERO_ASSETS", ""), "/opt/libero-assets"):
        if p and os.path.isfile(os.path.join(p, "scenes", "libero_kitchen_tabletop_base_style.xml")):
            return p
    raise EnvError("LIBERO assets not found")


def camera_intrinsics(fovy, width, height):
    f = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


def fixed_camera_matrices(name):
    """(K, T_world_cam) of a fixed camera, OpenCV convention (x right, y down, z forward)."""
    pos, tgt, fovy, w, h = CAMERAS[name]
    R = look_at(pos, tgt) @ np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return camera_intrinsics(fovy, w, h), T


def parse_action(a):
    """Validate an action against the public format; returns (move[6], grip, render tuple)."""
    if not isinstance(a, (list, tuple)) or len(a) != 8:
        raise ActionError("action must be a list of 8 numbers")
    out = []
    for v in a:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) or abs(v) > 1e3:
            raise ActionError("action must be a list of 8 numbers")
        out.append(float(v))
    cams = out[7]
    if cams != int(cams) or not 0 <= cams <= 7:
        raise ActionError("action must be a list of 8 numbers")
    cams = int(cams)
    render = tuple(n for i, n in enumerate(CAMERA_NAMES) if cams & (1 << i))
    return np.array(out[:6]), float(np.clip(out[6], -1.0, 1.0)), render


def encode_image(img):
    """The JSON form of an image (identical to the lab service's): raw RGB bytes, row-major."""
    import base64
    img = np.ascontiguousarray(img, dtype=np.uint8)
    return {"dtype": "uint8", "shape": list(img.shape), "b64": base64.b64encode(img.tobytes()).decode("ascii")}


def decode_image(d):
    import base64
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def to_wire(obs):
    """Observation with numpy arrays -> JSON-ready dict (images via `encode_image`)."""
    out = {}
    for k, v in obs.items():
        if k in CAMERA_NAMES:
            out[k] = encode_image(v)
        elif isinstance(v, np.ndarray):
            out[k] = v.tolist()
        else:
            out[k] = v
    return out


OBS_KEYS = ("t", "box", "q", "dq", "ee_pos", "ee_quat", "gripper", "wrench", "T_world_wrist", "left", "right", "wrist")


# ----------------------------------------------------------------------------------------------
# Model assembly
# ----------------------------------------------------------------------------------------------
def box_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def build_model(box_spec, box_pose):
    """Compile the LIBERO kitchen tabletop + Panda + the box at box_pose = (x, y, yaw)."""
    lib = libero_assets()
    spec = mujoco.MjSpec.from_file(os.path.join(lib, "scenes", "libero_kitchen_tabletop_base_style.xml"))
    spec.option.timestep = TIMESTEP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 2.0
    spec.visual.global_.offwidth = 320
    spec.visual.global_.offheight = 240
    spec.visual.quality.shadowsize = 0
    spec.visual.quality.offsamples = 0
    spec.compiler.autolimits = True
    wb = spec.worldbody
    # robot pedestal (visual only)
    g = wb.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.pos = [ROBOT_BASE[0], 0.0, TABLE_Z / 2]
    g.size = [0.12, 0.12, TABLE_Z / 2]
    g.rgba = [0.25, 0.25, 0.27, 1]
    g.contype = 0
    g.conaffinity = 0
    g.group = 1
    for name, (pos, tgt, fovy, _w, _h) in CAMERAS.items():
        c = wb.add_camera()
        c.name = name
        c.pos = list(pos)
        c.quat = list(quat_from_mat(look_at(pos, tgt)))
        c.fovy = fovy
    assets = robosuite_assets()
    robot = mujoco.MjSpec.from_file(os.path.join(assets, "robots", "panda", "robot.xml"))
    grip = mujoco.MjSpec.from_file(os.path.join(assets, "grippers", "panda_gripper.xml"))
    for cam in list(robot.cameras):
        if cam.name == "robotview":
            robot.delete(cam)
    robot.body("right_hand").add_frame().attach_body(grip.body("right_gripper"), "g_", "")
    wb.add_frame(pos=list(ROBOT_BASE)).attach_body(robot.body("base"), "r_", "")
    bx = mujoco.MjSpec.from_string(
        '<mujoco><compiler angle="radian" autolimits="true"/><worldbody>{}</worldbody></mujoco>'.format(box_spec["xml"]))
    x, y, yaw = box_pose
    wb.add_frame(pos=[x, y, TABLE_Z], quat=list(box_quat(yaw))).attach_body(bx.body("box"), "b_", "")
    return spec.compile()


# ----------------------------------------------------------------------------------------------
# Environment
# ----------------------------------------------------------------------------------------------
class BoxEnv(object):
    _gl = None

    def __init__(self, box_spec, render=True, max_steps=MAX_STEPS, box_id="practice", early_end=True):
        self.box_spec = box_spec
        self.box_id = box_id
        self.early_end = early_end
        self.meta = box_spec["meta"]
        self.render_enabled = render
        self.max_steps = int(max_steps)
        self.model = None
        self._con = None

    # ------------------------------------------------------------------ episode
    def reset(self, seed):
        self.seed = int(seed) & (2 ** 64 - 1)
        # separate streams: the box placement depends on the seed only, never on how many images or
        # wrench readings were drawn before (sensor noise has its own stream)
        place = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 7])
        self.rng = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 11])
        jx, jy = place.uniform(-1, 1, 2) * BOX_JITTER_XY
        yaw = place.uniform(-1, 1) * BOX_JITTER_YAW
        self.box_pose = (float(BOX_NOMINAL[0] + jx), float(BOX_NOMINAL[1] + jy), float(yaw))
        self.model = build_model(self.box_spec, self.box_pose)
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_state()
        self.t = 0
        self.done = False
        self.fault = None
        self.achieved = [False] * len(self.stages)
        self.hold = [0] * len(self.stages)
        self.max_q = [0.0] * len(self.stages)
        self.render_next = ("left", "right", "wrist")
        if self.render_enabled:
            self._init_render()
        return self.observation()

    def _index(self):
        m = self.model
        self.jq = np.array([m.joint("r_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint("r_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator("r_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = [m.actuator("r_g_gripper_finger_joint1").id, m.actuator("r_g_gripper_finger_joint2").id]
        self.gq = [m.joint("r_g_finger_joint1").qposadr[0], m.joint("r_g_finger_joint2").qposadr[0]]
        self.site = m.site("r_g_grip_site").id
        self.ft_force = m.sensor("r_g_force_ee").adr[0]
        self.ft_torque = m.sensor("r_g_torque_ee").adr[0]
        self.ft_site = m.site("r_g_ft_frame").id
        self.wrist_cam = m.camera("r_eye_in_hand").id
        self.cam_id = {n: m.camera(n).id for n in CAMERAS}
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))
        self.stages = []
        for s in self.meta["stages"]:
            j = m.joint("b_" + s["joint"])
            self.stages.append((s, int(j.qposadr[0])))
        ids = [s["id"] for s, _qa in self.stages]
        self.stage_blockers = [[ids.index(b) for b in s.get("blockers", []) if b in ids] for s, _qa in self.stages]
        self.box_dofs = np.array([int(m.joint("b_" + jn).dofadr[0]) for jn in self.meta["joints"]])
        self.box_qadr = np.array([int(m.joint("b_" + jn).qposadr[0]) for jn in self.meta["joints"]])

    def _init_state(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self.jq] = HOME_Q
        d.qpos[self.gq[0]], d.qpos[self.gq[1]] = 0.04, -0.04
        mujoco.mj_forward(m, d)
        self.grip_target = 0.04
        self.q_null = NULLSPACE_Q.copy()
        # settle with the arm held
        q_hold = d.qpos[self.jq].copy()
        for _ in range(150):
            tau = 200.0 * (q_hold - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
            d.ctrl[self.act] = tau
            d.ctrl[self.gact] = [0.04, -0.04]
            mujoco.mj_step(m, d)
        self.target_pos = d.site_xpos[self.site].copy()
        self.target_R = d.site_xmat[self.site].reshape(3, 3).copy()

    # ------------------------------------------------------------------ control
    def _osc_torque(self):
        m, d = self.model, self.data
        jp, jr = self._jp, self._jr
        mujoco.mj_jacSite(m, d, jp, jr, self.site)
        J = np.vstack([jp, jr])[:, self.jv]
        mujoco.mj_fullM(m, d, self._M)
        Mq = self._M[np.ix_(self.jv, self.jv)]
        R = d.site_xmat[self.site].reshape(3, 3)
        ep = self.target_pos - d.site_xpos[self.site]
        Re = self.target_R @ R.T
        eo = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        v = J @ d.qvel[self.jv]
        kp, ko = 400.0, 200.0
        acc = np.concatenate([kp * ep - 2.0 * math.sqrt(kp) * v[:3], ko * eo - 2.0 * math.sqrt(ko) * v[3:]])
        Minv = np.linalg.inv(Mq)
        Lam = np.linalg.inv(J @ Minv @ J.T + 1e-2 * np.eye(6))
        tau = J.T @ (Lam @ acc) + d.qfrc_bias[self.jv]
        Jbar = Minv @ J.T @ Lam
        N = np.eye(7) - J.T @ Jbar.T
        qn = d.qpos[self.jq]
        dq = d.qvel[self.jv]
        tau += N @ (5.0 * (self.q_null - qn) - 6.0 * dq)
        tau -= 0.5 * dq
        over = np.abs(dq) - 1.5
        tau -= np.where(over > 0, 30.0 * over * np.sign(dq), 0.0)
        return tau

    def step(self, action):
        """Apply one 10 Hz action. Returns (obs, done, info). Raises ActionError on a malformed action."""
        if self.done:
            raise EnvError("episode already finished")
        mv, g, render = parse_action(action)
        d = self.data
        dp = mv[:3]
        n = np.linalg.norm(dp)
        if n > MAX_DPOS:
            dp = dp * (MAX_DPOS / n)
        dr = mv[3:]
        n = np.linalg.norm(dr)
        if n > MAX_DROT:
            dr = dr * (MAX_DROT / n)
        cur_p = d.site_xpos[self.site].copy()
        cur_R = d.site_xmat[self.site].reshape(3, 3).copy()
        self.target_pos = np.clip(cur_p + dp, WORKSPACE_LO, WORKSPACE_HI)
        self.target_R = axis_angle_mat(dr) @ cur_R
        self.grip_target = 0.04 * (1.0 - g) / 2.0
        self.render_next = render
        self._physics_step()
        self.t += 1
        self._post_step()
        return self.observation(), self.done, self.info()

    def _physics_step(self):
        m, d = self.model, self.data
        gw = [self.grip_target, -self.grip_target]
        for _ in range(SUBSTEPS):
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = gw
            mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.fault = "non-finite state"

    def _post_step(self):
        d = self.data
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif np.max(np.abs(d.qvel[self.box_dofs])) > INSTAB_PART_QVEL:
                self.fault = "box part speed"
        if self.fault is not None:
            self.done = True
            return
        rel = [self._in_release(s, float(d.qpos[qa])) for s, qa in self.stages]
        ok = self._chain_ok(rel)
        for i, (s, qa) in enumerate(self.stages):
            q = float(d.qpos[qa])
            self.max_q[i] = max(self.max_q[i], q)
            # chain-aware credit: a stage counts only while it and every part upstream of it in the chain
            # (its blockers, their blockers, ...) are in their own credit positions
            if ok[i]:
                self.hold[i] += 1
                need = FINAL_HOLD_STEPS if s["id"] == "final" else HOLD_STEPS
                if self.hold[i] >= need:
                    self.achieved[i] = True
            else:
                self.hold[i] = 0
        if (self.early_end and all(self.achieved)) or self.t >= self.max_steps:
            self.done = True

    def _chain_ok(self, rel):
        """Transitive chain test: ok[i] = rel[i] and ok[j] for every blocker j of stage i."""
        ok = [None] * len(rel)

        def f(i, depth=0):
            if ok[i] is None:
                ok[i] = bool(rel[i]) and depth < 32 and all(f(j, depth + 1) for j in self.stage_blockers[i])
            return ok[i]

        for i in range(len(rel)):
            f(i)
        return ok

    @staticmethod
    def _in_release(s, q):
        win = s.get("window")
        return (win[0] <= q <= win[1]) if win else (q >= s["threshold"])

    def episode_score(self):
        """Credit of the episode (0 after a fault): half for the box's part stages (fraction done),
        half for the opening (lid / drawer) itself. A stage is done once it has stayed in its unlocked
        position for HOLD_STEPS control steps while every part upstream of it in the chain (its blockers,
        their blockers, ...) stayed in that part's own unlocked position (window parts inside their window,
        other parts past their threshold)."""
        if self.fault is not None:
            return 0.0
        parts = [a for (s, _q), a in zip(self.stages, self.achieved) if s["id"] != "final"]
        final = [a for (s, _q), a in zip(self.stages, self.achieved) if s["id"] == "final"]
        f = 1.0 if (final and final[0]) else 0.0
        if not parts:
            return f
        return 0.5 * sum(1.0 for a in parts if a) / len(parts) + 0.5 * f

    def info(self):
        return {"t": self.t, "fault": self.fault}

    # ------------------------------------------------------------------ sensing
    def _init_render(self):
        m = self.model
        with _GL_LOCK:
            if BoxEnv._gl is None:
                BoxEnv._gl = mujoco.GLContext(320, 240)
            BoxEnv._gl.make_current()
            try:
                if self._con is not None:
                    self._con.free()
                self._con = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_100)
                mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._con)
            finally:
                _gl_release()
        self._scn = mujoco.MjvScene(m, maxgeom=4000)
        self._opt = mujoco.MjvOption()
        self._opt.sitegroup[:] = 0
        self._opt.geomgroup[:] = 0
        self._opt.geomgroup[1] = 1
        self._pert = mujoco.MjvPerturb()
        self._cam = mujoco.MjvCamera()
        self._cam.type = mujoco.mjtCamera.mjCAMERA_FIXED

    def close(self):
        if self._con is not None:
            with _GL_LOCK:
                BoxEnv._gl.make_current()
                try:
                    self._con.free()
                finally:
                    _gl_release()
            self._con = None

    def render(self, cam):
        m, d = self.model, self.data
        if cam == "wrist":
            W, H, cid = WRIST["width"], WRIST["height"], self.wrist_cam
        else:
            W, H, cid = CAMERAS[cam][3], CAMERAS[cam][4], self.cam_id[cam]
        self._cam.fixedcamid = cid
        vp = mujoco.MjrRect(0, 0, W, H)
        rgb = np.zeros((H, W, 3), np.uint8)
        mujoco.mjv_updateScene(m, d, self._opt, self._pert, self._cam, mujoco.mjtCatBit.mjCAT_ALL, self._scn)
        with _GL_LOCK:
            BoxEnv._gl.make_current()
            try:
                mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._con)
                mujoco.mjr_render(vp, self._scn, self._con)
                mujoco.mjr_readPixels(rgb, None, vp, self._con)
            finally:
                _gl_release()
        rgb = rgb[::-1]
        noise = self.rng.normal(0.0, RGB_NOISE, rgb.shape)
        return np.clip(rgb.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    def wrist_camera_matrices(self):
        """(K, T_world_cam) of the wrist camera now, OpenCV convention."""
        d = self.data
        R = d.cam_xmat[self.wrist_cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = d.cam_xpos[self.wrist_cam]
        return camera_intrinsics(WRIST["fovy"], WRIST["width"], WRIST["height"]), T

    def observation(self):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        Rft = d.site_xmat[self.ft_site].reshape(3, 3)
        f = Rft @ d.sensordata[self.ft_force:self.ft_force + 3] + self.rng.normal(0, WRENCH_NOISE, 3)
        tq = Rft @ d.sensordata[self.ft_torque:self.ft_torque + 3] + self.rng.normal(0, WRENCH_NOISE * 0.1, 3)
        obs = {"t": int(self.t),
               "box": self.box_id,
               "q": np.round(d.qpos[self.jq], 6),
               "dq": np.round(d.qvel[self.jv], 6),
               "ee_pos": np.round(d.site_xpos[self.site], 6),
               "ee_quat": np.round(quat_from_mat(R), 6),
               "gripper": round(float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]]), 6),
               "wrench": np.round(np.concatenate([f, tq]), 4),
               "T_world_wrist": np.round(self.wrist_camera_matrices()[1], 6)}
        empty = np.zeros((0, 0, 3), np.uint8)
        for cam in CAMERA_NAMES:
            obs[cam] = self.render(cam) if (self.render_enabled and cam in self.render_next) else empty
        return obs

    def perturbed_observation(self, obs, rng):
        """A probe copy for the grader's lock: small joint-reading offsets (up to 0.001 rad) with the
        gripper pose and wrist-camera pose recomputed consistently, and a small wrench offset."""
        q = np.asarray(obs["q"], float) + np.array([rng.uniform(-0.001, 0.001) for _ in range(7)])
        f = self.fk
        f.qpos[:] = self.data.qpos
        f.qpos[self.jq] = q
        mujoco.mj_kinematics(self.model, f)
        mujoco.mj_camlight(self.model, f)
        out = dict(obs)
        out["q"] = np.round(q, 6)
        out["ee_pos"] = np.round(f.site_xpos[self.site], 6)
        out["ee_quat"] = np.round(quat_from_mat(f.site_xmat[self.site].reshape(3, 3)), 6)
        R = f.cam_xmat[self.wrist_cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = f.cam_xpos[self.wrist_cam]
        out["T_world_wrist"] = np.round(T, 6)
        w = np.asarray(obs["wrench"], float).copy()
        w[:3] += np.array([rng.uniform(-0.2, 0.2) for _ in range(3)])
        out["wrench"] = np.round(w, 4)
        return out

    # ------------------------------------------------------------------ ground truth (dev only)
    def gt(self):
        d = self.data
        return {"box_pose": list(self.box_pose),
                "stages": [{"id": s["id"], "kind": s["kind"], "q": float(d.qpos[qa]), "threshold": s["threshold"],
                            "achieved": a} for (s, qa), a in zip(self.stages, self.achieved)],
                "joints": {jn: float(d.qpos[qa]) for jn, qa in zip(self.meta["joints"], self.box_qadr)},
                "t": self.t, "fault": self.fault}
