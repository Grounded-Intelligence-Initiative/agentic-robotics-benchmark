"""One instrument's case: the instrument, its calibration recording and a request.

Shared by the grader (hidden instruments, from OS-entropy seeds) and tests/gen/make_payload.py
(the agent's example instruments, from fixed seeds). A case folder, as the agent's program
receives it:

  <instrument_dir>/request.json              trajectories to compensate
  <instrument_dir>/calibration/motors.csv    the drive's log of the calibration recording
  <instrument_dir>/calibration/endoscope_pose.json   the pose the robot's kinematics reports
  <instrument_dir>/calibration/endoscope/    frames.csv, left/*.jpg, right/*.jpg

The calibration recording is what the robot records right after an instrument is mounted:
an operator moves the wrist about for two minutes with the stock firmware while the stereo
endoscope films it.
"""

import json
import math
import os

import numpy as np

import plant as P
import trajectories as TR

CAMERA_RATE = 15.0
CAL_SECONDS = 120.0
# The jaws are rigid: they never pass through each other. The drive re-engages an instrument
# (new motor zero offsets) until the stock firmware's calibration motion keeps the jaws at
# least this far apart (rad), so the plant, which has no jaw contact, stays physical.
GRIP_FLOOR = math.radians(2.0)
MAX_ENGAGEMENTS = 200


def _rotation(axis, angle):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]), eye


def random_view(rng):
    """The endoscope looks at the wrist from the side, a little off the -y direction, rolled
    about its viewing axis (a level view would put the shaft's edges along the pixel columns)."""
    phi = math.radians(-90.0 + rng.choice([-1.0, 1.0]) * rng.uniform(28.0, 52.0))
    r = rng.uniform(33.0, 39.0)
    eye = np.array([r * math.cos(phi), r * math.sin(phi), rng.uniform(1.0, 7.0)])
    target = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(3.0, 4.6)])
    R, eye = look_at(eye, target)
    roll = math.radians(rng.choice([-1.0, 1.0]) * rng.uniform(12.0, 35.0))
    c, s = math.cos(roll), math.sin(roll)
    return R @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]), eye


def _clearance(R, eye, a0, a1, ra, b0, b1, rb, n=24):
    """Smallest gap (pixels) between two capsules' images (segments a0-a1, b0-b1, radii mm)."""
    def img(p0, p1, r):
        P = p0[None, :] + np.linspace(0.0, 1.0, n)[:, None] * (p1 - p0)[None, :]
        q = (P - eye) @ R
        z = np.maximum(q[:, 2], 1.0)
        return np.column_stack([210.0 * q[:, 0] / z, 210.0 * q[:, 1] / z]), 210.0 * r / z
    A, rA = img(a0, a1, ra)
    B, rB = img(b0, b1, rb)
    d = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2) - rA[:, None] - rB[None, :]
    return float(d.min())


def random_occluder(rng, R, eye):
    """A second instrument in the field, held still by the assistant arm: its tip near the
    wrist in the image, 15 to 60 px off the wrist's middle, its shaft leaving the view on that
    side, between a quarter nearer the endoscope and a quarter further than the wrist. Its pose
    is in the (first) instrument's frame; its jaws are open by a random angle. It may hide the
    wrist, but keeps 12 px clear of the instrument's shaft in the image (the shaft is what the
    endoscope's pose is told from)."""
    wrist = np.array([0.0, 0.0, 5.0])
    d = wrist - eye
    q = R.T @ d                                    # the wrist's middle in camera coordinates
    for _ in range(100):
        depth = float(d @ R[:, 2]) * rng.uniform(0.75, 1.25)
        a = rng.uniform(0.0, 2.0 * math.pi)
        off = rng.uniform(15.0, 60.0)
        ray = np.array([math.cos(a) * off / 210.0, math.sin(a) * off / 210.0, 1.0])
        ray[:2] += q[:2] / q[2]
        tip = eye + R @ (ray * depth)
        # the shaft leaves the view outwards, tilted in depth by up to 35 degrees
        b = a + rng.uniform(-0.6, 0.6)
        tilt = math.radians(rng.uniform(-35.0, 35.0))
        out = R @ np.array([math.cos(b) * math.cos(tilt), math.sin(b) * math.cos(tilt),
                            math.sin(tilt)])
        z = -out                                   # the instrument's z points to its tip
        x = np.cross(rng.normal(size=3), z)
        x /= np.linalg.norm(x)
        Ro = np.column_stack([x, np.cross(z, x), z])
        p = tip - Ro @ np.array([0.0, 0.0, 9.1 + 9.65])
        gap = _clearance(R, eye, np.array([0.0, 0.0, -60.0]), np.array([0.0, 0.0, 2.0]), 4.4,
                         p + Ro @ np.array([0.0, 0.0, -80.0]), tip, 4.4)
        if gap >= 12.0:
            break
    yaw, grip = rng.uniform(-0.4, 0.4), rng.uniform(0.0, 0.9)
    return dict(R=Ro.tolist(), p=p.tolist(),
                angles=[rng.uniform(-0.5, 0.5), yaw + 0.5 * grip, yaw - 0.5 * grip])


def calibration_recording(rng, inst, seconds=CAL_SECONDS):
    """Motor log, endoscope frames (camera clock and true joint angles) of a calibration
    recording, plus the endoscope's true and reported poses and the video's faults."""
    pyg = TR.generate(rng, seconds)
    commands = P.nominal_commands(P.pyg_to_joints(pyg))
    scale_r = 1.0 + 0.03 * rng.standard_normal(3)
    scale_tau = 1.0 + 0.03 * rng.standard_normal(3)
    for _ in range(MAX_ENGAGEMENTS):
        joints, motors, sub_j, _, end = P.simulate(inst, commands, scale_r, scale_tau,
                                                   record_sub=True, return_state=True)
        if np.min(sub_j[:, 1] - sub_j[:, 2]) >= GRIP_FLOOR:
            break
        # the jaws would close onto each other: the drive re-engages the instrument
        inst.q0 = P.engagement_offsets(rng)
    else:
        raise RuntimeError("no engagement keeps the jaws apart")
    n = len(commands)
    t = np.arange(n) * P.DT
    enc = motors + rng.normal(0.0, 2e-5, motors.shape)
    motor_rows = np.column_stack([t, np.clip(commands, -P.MOTOR_LIMIT, P.MOTOR_LIMIT), enc])
    sub_t = np.arange(len(sub_j)) * (P.DT / P.SUB)
    # frames: drive time of each (strobed) exposure; about 1 % lost singly, plus a burst
    t_first = 0.35 + rng.uniform(0.0, 0.05)
    frames = np.arange(t_first, t[-1] - 0.3, 1.0 / CAMERA_RATE)
    frames = frames + rng.normal(0.0, 2e-4, frames.shape)
    keep = rng.random(len(frames)) > 0.01
    b0 = rng.uniform(10.0, seconds - 12.0)
    keep &= ~((frames > b0) & (frames < b0 + rng.uniform(0.5, 2.0)))
    frames = frames[keep]
    angles = np.column_stack([np.interp(frames, sub_t, sub_j[:, i]) for i in range(3)])
    offset = rng.uniform(0.2, 1.3)                 # camera clock = drive clock + offset
    stamps = frames + offset
    # smoke from the energy device now and then: the view hazes over for a few seconds
    smoke = []
    for _ in range(int(rng.integers(1, 4))):
        s0 = rng.uniform(5.0, seconds - 8.0)
        smoke.append([s0 + offset, s0 + offset + rng.uniform(1.5, 4.0), rng.uniform(0.3, 0.6)])
    R, pos = random_view(rng)
    R_rep = _rotation(rng.normal(size=3), math.radians(rng.uniform(0.5, 1.5))) @ R
    pos_rep = pos + rng.normal(0.0, 0.6, 3)
    tissue = int(rng.integers(0, 1 << 30))
    spec = dict(stamps=stamps.tolist(), angles=angles.tolist(), R=R.tolist(), pos=pos.tolist(),
                tissue=tissue, noise_seed=int(rng.integers(0, 1 << 30)), smoke=smoke,
                l1=inst.l1, l2=inst.l2,
                # from a stream of its own, so the draws after it stay what they were
                occluder=random_occluder(np.random.default_rng([tissue, 7]), R, pos))
    # end: the wrist's state when the recording stops (the episodes start from it)
    return dict(motor_rows=motor_rows, spec=spec, R_rep=R_rep, pos_rep=pos_rep, offset=offset,
                end=end)


def make_case(rng, n_episodes, episode_seconds, spread_r=0.03, spread_tau=0.03):
    inst = P.random_instrument(rng)
    rec = calibration_recording(rng, inst)
    episodes = []
    for i in range(n_episodes):
        pyg = TR.generate(rng, episode_seconds)
        episodes.append(dict(
            id="ep_{:02d}_{:06x}".format(i, int(rng.integers(0, 1 << 24))),
            pyg=np.round(pyg, 6),
            scale_r=1.0 + spread_r * rng.standard_normal(3),
            scale_tau=1.0 + spread_tau * rng.standard_normal(3)))
    return dict(inst=inst, rec=rec, episodes=episodes)


def write_case(folder, case):
    """Everything but the video (which render_frames.render_spec writes from case['rec']['spec'])."""
    cal = os.path.join(folder, "calibration")
    os.makedirs(cal, exist_ok=True)
    with open(os.path.join(cal, "motors.csv"), "w") as f:
        f.write("t_s,cmd_pitch,cmd_jaw1,cmd_jaw2,enc_pitch,enc_jaw1,enc_jaw2\n")
        for r in case["rec"]["motor_rows"]:
            f.write("{:.3f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(*r))
    with open(os.path.join(cal, "endoscope_pose.json"), "w") as f:
        json.dump({"frame": "instrument", "units": "mm",
                   "R_left_camera": [[round(float(v), 6) for v in row]
                                     for row in case["rec"]["R_rep"]],
                   "p_left_camera": [round(float(v), 4) for v in case["rec"]["pos_rep"]]},
                  f, indent=1)
    with open(os.path.join(folder, "request.json"), "w") as f:
        json.dump({"dt_s": P.DT, "columns": ["pitch", "yaw", "grip"],
                   "episodes": [{"id": e["id"], "desired": e["pyg"].tolist()}
                                for e in case["episodes"]]}, f)
