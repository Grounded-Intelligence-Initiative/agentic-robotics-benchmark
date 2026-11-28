"""Isaac Lab scene for assembling a curtain on the floor and lifting it onto wall brackets."""

from math import cos, radians, sin, sqrt
from pathlib import Path

import isaaclab.sim as sim_utils
import omni.physics.tensors as tensors
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim.utils import get_current_stage_id
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from numpy.random import default_rng


ASSETS = Path(__file__).resolve().parent / "simulation_assets"
PHYSICS_DT = 1 / 240
MOBILE_MODEL_NAME = "mobile_fr3_duo_with_vision_and_manipulation_kit"
MOBILE_MODEL = ASSETS / MOBILE_MODEL_NAME / (MOBILE_MODEL_NAME + ".usda")
ROBOT_X = -0.57
ROBOT_Z = 0.009
TURN_TO_RIGHT = (0, 0, -sqrt(0.5), sqrt(0.5))
ASSET_QUARTER_TURN = (0, 0, sqrt(0.5), sqrt(0.5))
PROP_TRANSLATION_LIMIT = 0.10
PROP_YAW_LIMIT_DEG = 10
SPINE_TRAVEL = 0.827
WORK_AREA_WIDTH = 4.8
WALL_HEIGHT = 2.4384
WALL_FRONT = 0.66
HOOK_X = 0.50
HOOK_HEIGHT = 2.1336
HOOK_HALF_SPAN = 0.3395
HOOK_THICKNESS = 0.010
SLOT_WIDTH = 0.015
SLOT_TOP = HOOK_HEIGHT + 0.016
ROD_HALF_LENGTH = 0.3905
ROD_RADIUS = 0.006
EYELET_INNER_RADIUS = 0.02032
EYELET_COUNT = 6


def fixed_box(path, center, size, color):
    return AssetBaseCfg(
        prim_path=path,
        init_state=AssetBaseCfg.InitialStateCfg(pos=center),
        spawn=sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
    )


@configclass
class CurtainSceneCfg(InteractiveSceneCfg):
    floor = fixed_box("/World/Floor", (-1.00, 0, -0.05), (5, WORK_AREA_WIDTH, 0.10), (0.64, 0.57, 0.47))
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=1250))
    mobile_fr3 = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/MobileFR3Duo",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(MOBILE_MODEL),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                solver_position_iteration_count=16, solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(ROBOT_X, 0, ROBOT_Z),
            rot=TURN_TO_RIGHT,
            joint_pos={
                "franka_spine_vertical_joint": 0.0,
                ".*_fr3v2_1_joint1": 0.0, ".*_fr3v2_1_joint2": -0.55,
                ".*_fr3v2_1_joint3": 0.0, ".*_fr3v2_1_joint4": -1.90,
                ".*_fr3v2_1_joint5": 0.0, ".*_fr3v2_1_joint6": 2.0,
                ".*_fr3v2_1_joint7": 0.70,
            },
        ),
        actuators={
            "chassis": ImplicitActuatorCfg(
                joint_names_expr=["tmrv0_2_joint_.*", "caster_.*_joint"],
                stiffness=160.0, damping=35.0,
            ),
            "spine": ImplicitActuatorCfg(
                joint_names_expr=["franka_spine_vertical_joint"],
                joint_effort_limit=900.0, joint_velocity_limit=0.28,
                stiffness=45000.0, damping=2600.0,
            ),
            "shoulders": ImplicitActuatorCfg(
                joint_names_expr=[".*_fr3v2_1_joint[1-4]"],
                joint_effort_limit=87.0, stiffness=420.0, damping=70.0,
            ),
            "wrists": ImplicitActuatorCfg(
                joint_names_expr=[".*_fr3v2_1_joint[5-7]"],
                joint_effort_limit=12.0, stiffness=160.0, damping=28.0,
            ),
            "grippers": ImplicitActuatorCfg(
                joint_names_expr=[".*_robotiq_85_.*_(knuckle|finger_tip)_joint"],
                joint_effort_limit=8.0, joint_velocity_limit=0.5, stiffness=24.0, damping=2.0,
            ),
        },
    )
    rod = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Rod",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(ROBOT_X, -0.62, 0.0105), rot=ASSET_QUARTER_TURN),
        spawn=sim_utils.UsdFileCfg(usd_path=str(ASSETS / "rod.usda")),
    )
    curtain = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Curtain",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(ROBOT_X, -0.72, 0.0018), rot=ASSET_QUARTER_TURN),
        spawn=sim_utils.UsdFileCfg(usd_path=str(ASSETS / "curtain.usda")),
    )


def apply_initial_jitter(scene_cfg, seed):
    """Sample a shared in-plane offset and yaw for the rod and curtain."""
    rng = default_rng(seed)
    dx, dy = rng.uniform(-PROP_TRANSLATION_LIMIT, PROP_TRANSLATION_LIMIT, size=2)
    half_yaw = radians(90 + rng.uniform(-PROP_YAW_LIMIT_DEG, PROP_YAW_LIMIT_DEG)) / 2
    rotation = (0.0, 0.0, sin(half_yaw), cos(half_yaw))
    for asset in (scene_cfg.rod, scene_cfg.curtain):
        x, y, z = asset.init_state.pos
        asset.init_state.pos = (float(x + dx), float(y + dy), z)
        asset.init_state.rot = rotation


def add_window(scene_cfg):
    front = WALL_FRONT
    half_width = 0.31
    side_width = WORK_AREA_WIDTH / 2 - half_width
    side_center = half_width + side_width / 2
    bottom, top = 0.8128, 2.032
    middle = (bottom + top) / 2
    span = top - bottom
    wall = (0.67, 0.64, 0.58)
    trim = (0.87, 0.87, 0.84)
    glass = (0.20, 0.40, 0.54)
    pieces = (
        ("WallBelow", (front + .05, 0, bottom / 2), (.10, WORK_AREA_WIDTH, bottom), wall),
        ("WallAbove", (front + .05, 0, (top + WALL_HEIGHT) / 2), (.10, WORK_AREA_WIDTH, WALL_HEIGHT - top), wall),
        ("WallLeft", (front + .05, side_center, middle), (.10, side_width, span), wall),
        ("WallRight", (front + .05, -side_center, middle), (.10, side_width, span), wall),
        ("WindowGlass", (front + .068, 0, middle), (.010, .62, span), glass),
        ("WindowHead", (front + .024, 0, top - .018), (.095, .64, .036), trim),
        ("WindowSill", (front + .010, 0, bottom + .016), (.135, .665, .042), trim),
        ("WindowLeft", (front + .024, half_width - .019, middle), (.095, .038, span), trim),
        ("WindowRight", (front + .024, -half_width + .019, middle), (.095, .038, span), trim),
        ("WindowStile", (front + .041, 0, middle), (.042, .031, span), trim),
        ("WindowRail", (front + .041, 0, middle), (.042, .62, .030), trim),
    )
    for name, center, size, color in pieces:
        setattr(scene_cfg, name, fixed_box("/World/" + name, center, size, color))


def make_scene(log_dir, *, seed=None):
    """Spawn the nominal layout, or a reproducible perturbed layout with a seed."""
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            device="cuda:0", dt=PHYSICS_DT, render_interval=12, use_fabric=True,
            log_dir=str(log_dir), physics=PhysxCfg(solver_type=1),
        ),
    )
    scene_cfg = CurtainSceneCfg(num_envs=1, env_spacing=2)
    if seed is not None:
        apply_initial_jitter(scene_cfg, seed)
    steel = (0.15, 0.17, 0.18)
    for side, y in (("Left", HOOK_HALF_SPAN), ("Right", -HOOK_HALF_SPAN)):
        seat_z = HOOK_HEIGHT - ROD_RADIUS
        lip_height = SLOT_TOP - seat_z
        for part, center, size in (
            ("Seat", (HOOK_X, y, seat_z - 0.006), (0.031, HOOK_THICKNESS, 0.012)),
            ("NearLip", (HOOK_X - SLOT_WIDTH / 2 - 0.004, y, seat_z + lip_height / 2),
             (0.008, HOOK_THICKNESS, lip_height)),
            ("FarLip", (HOOK_X + SLOT_WIDTH / 2 + 0.004, y, seat_z + lip_height / 2),
             (0.008, HOOK_THICKNESS, lip_height)),
            ("WallMount", ((HOOK_X + WALL_FRONT) / 2, y, seat_z - 0.006),
             (WALL_FRONT - HOOK_X + 0.010, HOOK_THICKNESS, 0.012)),
        ):
            setattr(scene_cfg, side + part, fixed_box("/World/" + side + part, center, size, steel))
    add_window(scene_cfg)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    robot = scene["mobile_fr3"]
    position = robot.data.default_joint_pos.torch
    robot.write_root_link_pose_to_sim_index(root_pose=robot.data.default_root_pose.torch)
    robot.write_joint_position_to_sim_index(position=position)
    robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch)
    robot.actuators.target_command.set_position_index(value=position)
    scene.write_data_to_sim()
    return sim, scene


def curtain_views():
    physics = tensors.create_simulation_view("torch", stage_id=get_current_stage_id())
    physics.set_subspace_roots("/")
    root = "/World/envs/env_0/Curtain"
    return (
        physics,
        physics.create_rigid_body_view(root + "/ring_*"),
        physics.create_surface_deformable_body_view(root + "/Cloth"),
    )


def make_scorer(scene):
    """Bind a fresh scorer to this episode's live physics views."""
    from curtain_scoring import IsaacCurtainScorer

    return IsaacCurtainScorer(
        scene, curtain_views(),
        rod_half_length=ROD_HALF_LENGTH, eyelet_inner_radius=EYELET_INNER_RADIUS,
        hook_half_span=HOOK_HALF_SPAN, hook_thickness=HOOK_THICKNESS, dt=PHYSICS_DT,
    )
