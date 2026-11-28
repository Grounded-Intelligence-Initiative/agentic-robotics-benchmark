"""Direct, observation-conditioned G1 token control for a right-hand Cartesian waypoint."""

import numpy as np
from scipy.spatial.transform import Rotation


_CORNERS = np.stack(
    np.meshgrid([0, 1], [0, 1], [0, 1], indexing="ij"), axis=-1
).reshape(8, 3)


def _rotation(quat):
    q = np.asarray(quat)
    return Rotation.from_quat(q[[1, 2, 3, 0]])


class DirectTokenController:
    """Predict FSQ codes from Cartesian error, measured torso transform and pelvis tilt.

    The saved workspace stores no episode trajectories. Its spatial grid and
    derivatives were fitted by independent release-encoder probes offline.
    """

    def __init__(self, model_path):
        with np.load(model_path, allow_pickle=False) as fields:
            self._load_spatial_model(fields)
            self.rest = fields["rest"]
            self.rest_jac = fields["rest_jac"]
        self.integral = np.zeros(3)
        self.smooth_codes = self.rest.copy()

    def _load_spatial_model(self, fields):
        self.origin = fields["origin"]
        self.spacing = float(fields["spacing"])
        self.centers = fields["centers"]
        self.jac = fields["jac"]

    def _predict(self, point, rotations):
        # Trilinear interpolation of the eight local token maps.
        hi = np.asarray(self.centers.shape[:3]) - 1
        u = np.clip((point - self.origin) / self.spacing, 0, hi)
        lo = np.minimum(np.floor(u).astype(int), hi - 1)
        weights = np.where(_CORNERS, u - lo, 1 - u + lo).prod(axis=1)
        cells = tuple((lo + _CORNERS).T)
        local_models = self.centers[cells] + np.einsum(
            "r,crk->ck", rotations, self.jac[cells]
        )
        return weights @ local_models

    def command(self, observation, target_world_xyz=None, *, wrist_quat=None):
        """Return one legal 64-coordinate FSQ action. Make a new controller on reset."""
        base_rv = _rotation(observation["base_quat"]).as_rotvec()
        if target_world_xyz is None:
            predicted = self.rest + base_rv @ self.rest_jac
            self.integral.fill(0)
        else:
            goal = np.asarray(target_world_xyz, dtype=float)
            measured = np.asarray(observation["right_hand_pos"], dtype=float)
            if goal.shape != (3,) or not np.isfinite(goal).all():
                raise ValueError("target_world_xyz must be a finite position")
            self.integral += 0.008 * np.clip(goal - measured, -0.05, 0.05)
            np.clip(self.integral, -0.052, 0.052, out=self.integral)
            torso = _rotation(observation["torso_quat"])
            point = torso.inv().apply(goal + self.integral - observation["torso_pos"])
            wrist = Rotation.identity() if wrist_quat is None else _rotation(wrist_quat)
            wrist_rv = (torso.inv() * wrist).as_rotvec()
            # The finite-difference rotational model is local to an upright posture.
            rotations = np.clip(np.r_[wrist_rv, base_rv], -0.3, 0.3)
            predicted = self._predict(point, rotations)
        # Slew in latent space on abrupt waypoint transitions.
        self.smooth_codes += np.clip(predicted - self.smooth_codes, -0.34, 0.34)
        code = np.clip(np.rint(self.smooth_codes), 0, 31).astype(np.uint8)
        return {"mode": "codes", "codes": code.tolist()}
