"""The engine of an episode, drawn from its seed (cell internals).

Everything a seed decides is here: the engine's pose on its pallet, the belt's physics (flatwise stiffness within the
droop class, mass, length within +-0.5 % of nominal, belt-pulley friction), the tensioner's spring (rate and preload
+-15 %), damper and pivot friction, the accessories' drag, the belt's drape on its peg, and the sensors' per-episode
draws (the F/T biases, the cameras' colour). The physics draws use one stream in a fixed order (the development
reference's seeds reproduce the same engines); the sensor draws use a second stream.
"""

import math

import numpy as np

from . import drive as dr

__all__ = ["draw_layout", "HIDDEN_RANGES", "PEG_DRAPE"]

HIDDEN_RANGES = {
    "mu": (0.6, 0.9),                       # belt-pulley friction (grooves and the tensioner's face); 0.5-0.6 dropped
                                            # 2026-11-09: correct fits slipped > 3 % at the joint-extreme corners
    "bend_EI_scale": (0.6, 1.2),            # x nominal 0.012 N m^2 (the cell's chain: a 200 mm free end held level
                                            # droops 32-71 mm); 1.2-1.7 dropped 2026-11-09: correct fits slipped > 2.5 %
                                            # (3.4-4.7 % at EI 1.4-1.7x) at the soft-spring, long-belt, high-drag corners
    "mass_scale": (0.9, 1.1),               # x nominal 0.24 kg/m
    "len_scale": (0.995, 1.005),            # x the nominal belt length
    "spring_scale": (0.85, 1.15),           # rate and preload, each
    "damper": (0.7, 1.3),                   # N m s/rad
    "pivot_friction": (0.04, 0.12),         # N m
    "drag": (0.2, 0.4),                     # N m per accessory during the run-in
    "dpos": 0.004, "yaw_deg": 1.0, "roll_deg": 0.5,
}
# the belt hangs on its peg as a loop: an arc over the peg, two spans, a rounded bottom of radius U(0.05, 0.09) m
PEG_DRAPE = {"bottom_r": (0.05, 0.09), "start": 2000, "depth": 0.06}
FT_BIAS = {"force_N": 1.0, "torque_Nm": 0.05}


def draw_physics(rng):
    """The hidden physics of one engine (same order as the development reference's draw)."""
    T = dr.VARIANTS["A"]["tensioner"]
    H = HIDDEN_RANGES
    return {
        "mu": float(rng.uniform(*H["mu"])),
        "bend_EI": float(dr.BELT["bend_EI"] * rng.uniform(*H["bend_EI_scale"])),
        "mass_per_m": float(dr.BELT["mass_per_m"] * rng.uniform(*H["mass_scale"])),
        "len_scale": float(rng.uniform(*H["len_scale"])),
        "t_rate": float(T["rate"] * rng.uniform(*H["spring_scale"])),
        "t_tau_closed": float(T["tau_closed"] * rng.uniform(*H["spring_scale"])),
        "t_damper": float(rng.uniform(*H["damper"])),
        "t_friction": float(rng.uniform(*H["pivot_friction"])),
        "drag_WP": float(rng.uniform(*H["drag"])), "drag_ALT": float(rng.uniform(*H["drag"])),
        "dpos": [float(x) for x in rng.uniform(-H["dpos"], H["dpos"], 3)],
        "yaw": float(math.radians(rng.uniform(-H["yaw_deg"], H["yaw_deg"]))),
        "roll": float(math.radians(rng.uniform(-H["roll_deg"], H["roll_deg"]))),
    }


def draw_layout(seed):
    seed = int(seed)
    rng = np.random.default_rng(seed)
    L = {"seed": seed, "variant": "A"}
    L["prm"] = draw_physics(rng)
    L["drape"] = {"bottom_r": float(rng.uniform(*PEG_DRAPE["bottom_r"])), "start": int(rng.integers(0, PEG_DRAPE["start"]))}
    srng = np.random.default_rng([seed & 0xFFFFFFFF, 0xBE17C])
    L["ft_bias"] = {s: [float(v) for v in np.concatenate([srng.uniform(-FT_BIAS["force_N"], FT_BIAS["force_N"], 3),
                                                          srng.uniform(-FT_BIAS["torque_Nm"], FT_BIAS["torque_Nm"], 3)])]
                    for s in ("L", "R")}
    L["noise_seed"] = int(srng.integers(0, 2 ** 31 - 1))
    L["engine_id"] = "EN-%05d" % (seed % 100000)
    return L
