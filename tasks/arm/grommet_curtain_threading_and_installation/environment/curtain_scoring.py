"""Score a six-eyelet curtain from the final simulator state."""

import numpy as np
from scipy.spatial.transform import Rotation


EYELET_COUNT = 6
MIN_UPWARD_LOAD = 0.2  # N per bracket; the bare rod weighs about 2.4 N.
GRIPPER_CLEARANCE = 0.05  # m from the rod axis, steel rings and cloth.


def score_curtain(
    rod_pose, eyelet_poses, *, rod_half_length, eyelet_inner_radius, hook_half_span,
    hook_thickness, bracket_loads, grippers_clear,
):
    """Use world-space xyzw poses and eyelet indices ordered along the cloth."""
    rod_pose = np.asarray(rod_pose)
    eyelet_poses = np.asarray(eyelet_poses)
    bracket_loads = np.asarray(bracket_loads)
    if rod_pose.shape != (7,) or eyelet_poses.shape != (EYELET_COUNT, 7) or bracket_loads.shape != (2,):
        raise ValueError("Expected one rod, six ordered eyelets and two bracket loads")
    if not all(np.isfinite(data).all() for data in (rod_pose, eyelet_poses, bracket_loads)):
        raise ValueError("Curtain scoring observations must be finite")

    origin = rod_pose[:3]
    axis = Rotation.from_quat(rod_pose[3:]).apply([0, 1, 0])
    centers = eyelet_poses[:, :3]
    normals = Rotation.from_quat(eyelet_poses[:, 3:]).apply([0, 0, 1])
    facing = normals @ axis
    plane_offset = np.einsum("ij,ij->i", centers - origin, normals)
    axial = np.zeros(EYELET_COUNT)
    crosses_plane = np.abs(facing) >= 0.2
    np.divide(plane_offset, facing, out=axial, where=crosses_plane)
    intersections = origin + axial[:, None] * axis
    radial = np.linalg.norm(intersections - centers, axis=1)
    threaded = crosses_plane & (np.abs(axial) < rod_half_length) & (radial < eyelet_inner_radius)
    rings = np.flatnonzero(threaded)
    rings = rings[np.argsort(axial[rings])]
    by_direction = (_valid_prefix(rings, facing), _valid_prefix(rings[::-1], facing))
    correct = max(by_direction, key=len)

    between = threaded & (np.abs(intersections[:, 1]) < hook_half_span - hook_thickness / 2)
    on_both_brackets = bool(np.all(bracket_loads >= MIN_UPWARD_LOAD))
    installed = bool(grippers_clear and on_both_brackets and between.any())
    points = len(correct)
    return {
        "score": (points + 4 * installed) / 10,
        "threading_score": points / 10,
        "installation_score": 0.4 if installed else 0.0,
        "correct_eyelets": correct,
        "threading_counts_by_direction": [len(prefix) for prefix in by_direction],
        "eyelets_on_rod": int(threaded.sum()),
        "eyelets_between_hooks": int(between.sum()),
        "rod_on_both_brackets": on_both_brackets,
        "installed": installed,
    }


def _valid_prefix(rings, facing):
    if not len(rings) or rings[0] not in (0, EYELET_COUNT - 1):
        return []
    start = int(rings[0])
    direction = 1 if start == 0 else -1
    first_face = np.signbit(facing[start])
    correct = []
    for offset, ring in enumerate(rings):
        expected_face = first_face ^ bool(offset % 2)
        if ring != start + direction * offset or np.signbit(facing[ring]) != expected_face:
            break
        correct.append(int(ring))
    return correct


class IsaacCurtainScorer:
    """Read the final poses, seat contacts and fingertip clearance from Isaac Sim."""

    def __init__(self, scene, views, *, dt, rod_half_length, eyelet_inner_radius, hook_half_span, hook_thickness):
        self._dt = dt
        self._geometry = dict(
            rod_half_length=rod_half_length, eyelet_inner_radius=eyelet_inner_radius,
            hook_half_span=hook_half_span, hook_thickness=hook_thickness,
        )
        self._physics, self._rings, self._cloth = views
        self._rod = scene["rod"]
        self._robot = scene["mobile_fr3"]
        ring_names = [path.rsplit("/", 1)[-1] for path in self._rings.prim_paths]
        self._ring_indices = [ring_names.index(f"ring_{i}") for i in range(EYELET_COUNT)]
        self._finger_indices = [
            i for i, name in enumerate(self._robot.body_names)
            if "robotiq_85" in name and name.endswith("_finger_tip_link")
        ]
        if len(self._finger_indices) != 4:
            raise ValueError("Expected four Mobile FR3 Duo gripper fingertips")
        self._brackets = self._physics.create_rigid_contact_view(
            "/World/envs/env_0/Rod/body",
            ["/World/LeftSeat/geometry/mesh", "/World/RightSeat/geometry/mesh"],
        )
        if (self._brackets.sensor_count, self._brackets.filter_count) != (1, 2):
            raise ValueError("Expected rod contact observations at both bracket seats")

    def measure(self):
        rod_pose = self._rod.data.root_pose_w.torch[0].cpu().numpy()
        eyes = self._rings.get_transforms().cpu().numpy()[self._ring_indices]
        loads = self._brackets.get_contact_force_matrix(dt=self._dt)[0, :, 2].cpu().numpy()
        clear = bool(np.all(loads >= MIN_UPWARD_LOAD)) and self._grippers_clear(rod_pose, eyes)
        return score_curtain(rod_pose, eyes, bracket_loads=loads, grippers_clear=clear, **self._geometry)

    def _grippers_clear(self, rod_pose, eyes):
        fingers = self._robot.data.body_link_pos_w.torch[0, self._finger_indices].cpu().numpy()
        axis = Rotation.from_quat(rod_pose[3:]).apply([0, 1, 0])
        delta = fingers - rod_pose[:3]
        end = self._geometry["rod_half_length"]
        to_rod = delta - np.outer(np.clip(delta @ axis, -end, end), axis)
        if np.any(np.linalg.norm(to_rod, axis=1) < GRIPPER_CLEARANCE):
            return False
        nodes = self._cloth.get_simulation_nodal_positions()[0].cpu().numpy()
        curtain = np.concatenate((eyes[:, :3], nodes))
        distances = np.linalg.norm(fingers[:, None, :] - curtain, axis=2)
        return bool(np.all(distances >= GRIPPER_CLEARANCE))
