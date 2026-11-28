"""Fit the torso-relative right-grasp FSQ grid from robot kinematics and the public G1 encoder."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from io import BytesIO
from pathlib import Path
import struct
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from sonic_sim.decoder import DEFAULT_Q
from sonic_sim.encoder import ReferenceEncoder

ARM = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
)
SPACING = 0.035
AXES = (
    np.arange(-0.04, 0.451, SPACING),
    np.arange(-0.45, -0.009, SPACING),
    np.arange(-0.27, 0.251, SPACING),
)
GRID = np.stack(np.meshgrid(*AXES, indexing="ij"), axis=-1)


class ArrayMember(ZipInfo):
    """NumPy's historical local ZIP64 size field on an otherwise ZIP 2.0 member."""

    def FileHeader(self, zip64=None):
        # ZipFile writes this header twice to a seekable output. The second pass
        # knows the compressed size. The central directory carries 32-bit sizes.
        self.extra = struct.pack("<HHQQ", 1, 16, self.file_size, self.compress_size)
        try:
            return super().FileHeader(zip64=False)
        finally:
            self.extra = b""


def write_grid(output, **arrays):
    """Write reproducible NPY payloads and the released six-member ZIP layout."""
    names = ("origin", "spacing", "centers", "jac", "rest", "rest_jac")
    if set(arrays) != set(names):
        raise ValueError("expected the six Cartesian grid fields")
    with ZipFile(output, "w", compression=ZIP_DEFLATED, allowZip64=False) as archive:
        for name in names:
            data = BytesIO()
            np.save(data, arrays[name], allow_pickle=False)
            member = ArrayMember(name + ".npy", date_time=(1980, 1, 1, 0, 0, 0))
            member.create_system = 3
            member.external_attr = 0o600 << 16
            archive.writestr(
                member, data.getvalue(), compress_type=ZIP_DEFLATED, compresslevel=6
            )


class Calibration:
    def __init__(self, assets):
        self.encoder = ReferenceEncoder(assets / "policy/model_encoder.onnx")
        self.model = mujoco.MjModel.from_xml_path(
            str(assets / "menagerie_g1/g1_with_hands.xml")
        )
        self.data = mujoco.MjData(self.model)
        joints = [self.model.joint("right_" + name + "_joint").id for name in ARM]
        self.qadr = self.model.jnt_qposadr[joints]
        self.vadr = self.model.jnt_dofadr[joints]
        self.limits = self.model.jnt_range[joints]
        self.wrist = self.model.body("right_wrist_yaw_link").id
        self.data.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        self.data.qpos[self.qadr] = DEFAULT_Q[22:]
        mujoco.mj_kinematics(self.model, self.data)
        self.origin = self.data.body("torso_link").xpos.copy()
        self.qref = np.tile(DEFAULT_Q, (10, 1))
        self.velocity = np.zeros_like(self.qref)

    def arm_ik(self, goal, rotation):
        data = self.data
        data.qpos[self.qadr] = DEFAULT_Q[22:]
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros_like(jp)
        for _ in range(22):
            mujoco.mj_kinematics(self.model, data)
            mujoco.mj_comPos(self.model, data)
            measured = data.xmat[self.wrist].reshape(3, 3)
            point = data.xpos[self.wrist] + measured @ [0.115, 0.048, 0.0]
            orientation = 0.5 * np.cross(measured.T, rotation.T).sum(axis=0)
            posture = DEFAULT_Q[22:] - data.qpos[self.qadr]
            error = np.concatenate((goal - point, 0.10 * orientation, 0.012 * posture))
            mujoco.mj_jac(self.model, data, jp, jr, point, self.wrist)
            jac = np.vstack(
                (jp[:, self.vadr], 0.10 * jr[:, self.vadr], 0.012 * np.eye(7))
            )
            delta = np.linalg.solve(jac.T @ jac + 0.00018 * np.eye(7), jac.T @ error)
            data.qpos[self.qadr] = np.clip(
                data.qpos[self.qadr] + delta / max(1.0, abs(delta).max() / 0.22),
                self.limits[:, 0],
                self.limits[:, 1],
            )
            if abs(delta).max() < 0.007:
                break
        return data.qpos[self.qadr].copy()

    def label(self, arm, base_rotvec):
        self.qref[:, 22:] = arm
        anchor = Rotation.from_rotvec(-base_rotvec).as_matrix()[:, :2].reshape(6)
        codes = self.encoder.codes_g1(
            self.qref, self.velocity, np.broadcast_to(anchor, (10, 6))
        )
        return codes.astype(np.float32)

    def base_jacobian(self, arm):
        return [
            (self.label(arm, rv) - self.label(arm, -rv)) / 0.24
            for rv in 0.12 * np.eye(3)
        ]

    def probe(self, point):
        goal = self.origin + point
        zero = np.zeros(3)
        arm = self.arm_ik(goal, np.eye(3))
        center = self.label(arm, zero)
        jac = []
        for rv in 0.12 * np.eye(3):
            plus = self.arm_ik(goal, Rotation.from_rotvec(rv).as_matrix())
            minus = self.arm_ik(goal, Rotation.from_rotvec(-rv).as_matrix())
            jac.append((self.label(plus, zero) - self.label(minus, zero)) / 0.24)
        return center, jac + self.base_jacobian(arm)


def initialize(assets):
    global worker
    worker = Calibration(assets)


def probe(point):
    return worker.probe(point)


def regenerate(assets, output, workers):
    with ProcessPoolExecutor(
        workers, initializer=initialize, initargs=(assets,)
    ) as pool:
        centers, jac = zip(*pool.map(probe, GRID.reshape(-1, 3), chunksize=8))
    model = Calibration(assets)
    rest = model.label(DEFAULT_Q[22:], np.zeros(3))
    rest_jac = model.base_jacobian(DEFAULT_Q[22:])
    shape = GRID.shape[:3]
    write_grid(
        output,
        origin=GRID[0, 0, 0],
        spacing=np.float64(SPACING),
        centers=np.asarray(centers, np.uint8).reshape(*shape, 64),
        jac=np.asarray(jac, np.float16).reshape(*shape, 6, 64),
        rest=rest,
        rest_jac=rest_jac,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=Path("/opt/sonic/assets"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    regenerate(args.assets, args.out, args.workers)
