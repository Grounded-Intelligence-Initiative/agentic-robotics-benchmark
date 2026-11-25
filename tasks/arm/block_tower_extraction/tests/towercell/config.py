"""The cell's fixed numbers: layout, the tower's nominal geometry, the robot's tools, sensors, budgets and the score.

Everything here is public: tests/review/make_given.py writes `public_config()` into the agent's
given/cell/cell_config.json. The hidden per-tower values (each block's height, friction, mass and placement offsets,
the tower's placement on the turntable) are drawn in tower.py from the tower's seed, inside the ranges of `RANGES`
(the agent's documents state the same ranges).

Units: metres, seconds, radians, kilograms, newtons. World frame: the robot's base at the origin, z up; the tower
stands in front of the robot along +x.
"""

import copy
import math

TICK_S = 0.01           # one control tick
SUBSTEPS = 10           # physics steps per tick (dt 1 ms)
PHYSICS_DT = TICK_S / SUBSTEPS

CONFIG = {
    "tick_s": TICK_S,
    "physics_dt_s": PHYSICS_DT,
    "block": {"length_m": 0.075, "width_m": 0.025, "height_m": 0.015, "mass_kg_nominal": 0.0165},
    "tower": {
        "levels": 18,
        "slot_pitch_m": 0.0265,
        "turntable_axis_xy": [0.65, 0.0],
        "base_z": 0.115,
        "level0_yaw_rad": 0.0,
        "slot_order_note": "slot s of level k sits at (s - 1) * slot_pitch along the level's lateral axis "
                           "(-sin(yaw), cos(yaw)); level k's blocks lie along yaw_k = level0_yaw + k * pi / 2 "
                           "(at turntable angle 0)",
        "turntable_top_half_m": 0.030,
        "turntable_speed_rad_s": 0.6,
        "turntable_accel_rad_s2": 0.6,
    },
    "staging_post": {"xy": [0.42, 0.28], "top_z": 0.115, "half_xy": [0.015, 0.012]},
    "robot": {
        "home_q": [0.0, -0.5, 0.0, -2.3, 0.0, 1.8, 0.785],
        "tcp_offset_m": 0.1034,
        "pusher": {"x0_m": 0.02, "length_m": 0.12, "radius_m": 0.003, "dz_above_tcp_m": 0.014},
        "max_speed_m_s": 0.5,
        "max_rot_rad_s": 2.0,
        "joint_speed_cap_rad_s": 1.5,
        "gripper_max_opening_m": 0.08,
        "grip_force_max_N": 40.0,
        "ft_noise_N": 0.02,
        "ft_torque_noise_Nm": 0.002,
    },
    "cameras": {
        "left": {"pos": [0.38, 0.40, 0.62], "target": [0.65, 0.0, 0.30], "size": [480, 360], "fovy_deg": 40.0,
                 "depth_range_m": [0.2, 2.0]},
        "right": {"pos": [0.38, -0.40, 0.62], "target": [0.65, 0.0, 0.30], "size": [480, 360], "fovy_deg": 40.0,
                  "depth_range_m": [0.2, 2.0]},
        "wrist": {"size": [320, 240], "fovy_deg": 60.0, "depth_range_m": [0.04, 1.0],
                  "hand_pos": [0.065, 0.0, 0.055], "hand_look_at": [0.12, 0.0, 0.30]},
        "rgb_noise_sigma": 2.0,
        "depth_noise_m": 0.0005,
    },
    "episode": {"duration_s": 2400.0},
    "budgets": {
        "frames_per_episode": 1500,
        "program_time_factor": 0.25,
        "program_time_extra_s": 300.0,
        "privileged_program_time_factor": 0.5,
        "silence_s": 20.0,
        "start_s": 120.0,
        "max_ticks_per_step": 1000,
        "frame_price_s": 0.2,
        "call_price_s": 0.005,
        "episode_wall_cap_s": 2700.0,
    },
    "score": {
        "n_cap": 30,
        "fall_dz_m": 0.005,
        "fall_tilt_deg": 5.0,
        "removed_beyond_m": 0.040,
        "place_lateral_tol_m": 0.004,
        "place_axial_tol_m": 0.006,
        "place_yaw_tol_deg": 5.0,
        "place_z_tol_m": 0.0015,
        "rest_speed_m_s": 0.005,
        "final_settle_s": 3.0,
    },
}

# hidden per-tower values are drawn inside these ranges (tower.py); public
RANGES = {
    "block_height_bearing_m": [0.0149, 0.0151],
    "loose_block_gap_m": [0.00012, 0.00030],
    "block_friction": [0.30, 0.50],
    "block_mass_kg": [0.0145, 0.0185],
    "block_axial_offset_m": [-0.002, 0.002],
    "block_lateral_offset_m": [-0.0003, 0.0003],
    "block_yaw_offset_deg": [-0.6, 0.6],
    "tower_offset_on_turntable_m": [-0.003, 0.003],
    "tower_yaw_offset_deg": [-1.5, 1.5],
    "level_patterns": "per level, the blocks that carry the levels above: middle only, both sides, middle + one side, "
                      "or all three; the others are shorter by the loose gap",
}


def config():
    return copy.deepcopy(CONFIG)


def n_cap(cfg=None):
    return int((cfg or CONFIG)["score"]["n_cap"])


def public_config():
    c = config()
    c["ranges"] = copy.deepcopy(RANGES)
    return c


def level_yaw(k, cfg=None):
    cfg = cfg or CONFIG
    return cfg["tower"]["level0_yaw_rad"] + (math.pi / 2 if k % 2 else 0.0)
