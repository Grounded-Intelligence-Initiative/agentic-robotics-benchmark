"""The public numbers of the grip-controller acceptance, one dict per topic.

Every number here is printed in the controller author's documents (or follows from a printed one) and the grader uses
it with the same value. Nothing here is a property of a particular hand unit or specimen beyond its printed range.
Units are in the key names (_mm, _deg, _s, _ms, _kg, _n, _v, _ma, _c, _hz); ranges are (low, high), both included.
The register table itself (addresses, sizes, initial values, printed ranges) is names.REGISTERS.
"""

# ---------------------------------------------------------------------------------------------------------------
# The servo actuator, as its vendor prints it
# ---------------------------------------------------------------------------------------------------------------
SERVO = {
    "resolution_counts_per_rev": 4096,
    "position_deg_per_count": 360.0 / 4096,
    "velocity_rpm_per_unit": 0.229,
    "current_ma_per_unit": 1.0,
    "pwm_percent_per_unit": 0.113,
    "voltage_v_per_unit": 0.1,
    "temperature_c_per_unit": 1,
    "profile_acceleration_rev_per_min2_per_unit": 214.577,
    "pwm_limit_full": 885,                       # 885 = 100 % output
    "gain_divisors": {"position_p_gain": 128, "position_i_gain": 65536, "position_d_gain": 16,
                      "velocity_p_gain": 128, "velocity_i_gain": 65536,
                      "feedforward_1st_gain": 4, "feedforward_2nd_gain": 4},
    "stall_rows": ({"supply_v": 9.0, "torque_nm": 0.85, "current_a": 0.61},
                   {"supply_v": 11.1, "torque_nm": 1.03, "current_a": 0.80},
                   {"supply_v": 12.0, "torque_nm": 1.12, "current_a": 0.88}),
    "standby_current_ma": 17,
    "input_voltage_v": (6.5, 12.0),
    "recommended_supply_v": 11.1,
}

# what a write of Operating Mode resets, as the vendor prints it ("pwm_limit" and "current_limit" stand for the
# servo's own PWM Limit and Current Limit)
MODE_CHANGE_RESET = {
    "position_p_gain": 2000, "position_i_gain": 0, "position_d_gain": 0,
    "profile_velocity": 0, "profile_acceleration": 0,
    "goal_pwm": "pwm_limit", "goal_current": "current_limit",
}

# the EEPROM settings of every delivered unit (each servo; in the unit's record)
UNIT_EEPROM = {"operating_mode": 5, "current_limit": 910, "temperature_limit": 70, "shutdown": 52,
               "return_delay": 0, "pwm_limit": 885}

# the servo firmware as observed on the bench
FIRMWARE = {
    "loop_hz": 1000,
    "overload_observation": "about 2 s at full output",
    "reboot_available": False,
}

# ---------------------------------------------------------------------------------------------------------------
# The range sheet: how a unit may differ from the documents (basis: measured / published / our estimate)
# ---------------------------------------------------------------------------------------------------------------
RANGE_SHEET = {
    "supply_v": {"range": (11.1, 12.4), "basis": ("published", "measured")},
    "home_offset_record_error_deg": {"range": (-0.7, 0.7), "basis": ("measured",)},
    "joint_friction_nm": {"range": (0.015, 0.04), "basis": ("measured", "our estimate")},
    "reflected_inertia_kg_m2": {"range": (3.0e-3, 4.5e-3), "basis": ("our estimate",)},
    "stall_torque_vs_curve_frac": {"range": (-0.08, 0.08), "basis": ("our estimate",)},
    "standby_current_ma": {"range": (12.6, 16.5), "basis": ("measured",)},
    "start_temperature_c": {"range": (37, 48), "basis": ("measured",)},
    "tactile_frame_hz": {"range": (83, 91), "basis": ("published", "measured")},
}

# ---------------------------------------------------------------------------------------------------------------
# Tactile modules: frame encoding and transport
# ---------------------------------------------------------------------------------------------------------------
TACTILE = {
    "scale": "uncalibrated",
    "resultant_bytes": 6,                        # x, y, z as int16 little-endian, in the module frame
    "resultant_n_per_lsb": 0.01,
    "taxel_bytes": 3,                            # Fx int8, Fy int8, Fz uint8
    "taxel_n_per_step": 0.1,
    "fz_wrap_reads": 255,                        # one step below zero reads 0xFF
    "fz_saturation_reads": 255,
    "fxy_saturation": (-128, 127),
    "frame_bytes": 880,
    "baud": 921600,
    "bits_per_byte": 10,
    "arrival_after_capture_ms": 9.6,             # the frame's transfer time, rounded up to 0.1 ms
    "rezero_rule": "Any contact present at call time becomes the new zero",
    "zero_residual_steps": (-1, 1),
}

# ---------------------------------------------------------------------------------------------------------------
# Specimens and labels
# ---------------------------------------------------------------------------------------------------------------
SPECIMENS = {
    "width_mm": (35, 45),                        # along the squeeze axis
    "lateral_mm": 50,
    "height_mm": 50,
    "pad_line_below_top_mm": 10,
    "soft_k_n_per_mm": (1, 10),
    "mass_range_kg": {"SH-1": (0.040, 0.700), "SH-2": (0.068, 0.700), "SH-3": (0.117, 0.700),
                      "SP": (0.030, 0.700), "FL": (0.150, 0.700)},
    "finish_mu": {"matte": (0.40, 0.55), "rough": (0.55, 0.70)},
}
LABEL_RULE = {"carry_max_kg": 0.20, "decline_min_kg": 0.50}    # nothing in between is presented

# ---------------------------------------------------------------------------------------------------------------
# Placement, the arm's motion, the timeline
# ---------------------------------------------------------------------------------------------------------------
PLACEMENT = {"offset_radius_mm": 2.0, "yaw_deg": 3.0, "clearance_mm": 2.0}
# each nest: a plate under the specimen and four walls of this height round its pocket
NEST_GEOMETRY = {"wall_height_mm": 3.5}

MOTION = {
    "translation_law": "minimum-jerk",
    "translation_peak_mps2": 6.0,
    "test_lift": {"rise_mm": 8, "rise_s": 0.5, "hold_s": 2.5},
    "lower": {"drop_mm": 8, "s": 0.5},
    "set_down": {"drop_mm": 8, "s": 0.5},
    "transfer": {"up_m": 0.15, "x_m": 0.25, "y_m": 0.25, "dwell_s": 0.3},
    "tilt_rate_deg_s": 10,
    "tilt_ramp_s": 0.1,
    "tracking_scale": (0.9, 1.1),
}
# each transfer profile: the tilt axis in the cell frame (z up) and its angle; T0 has no tilt
PROFILE_TILT = {"T0": None, "R30": {"axis": (1, 0, 0), "deg": 30}, "P30": {"axis": (0, 1, 0), "deg": 30}}

TIMELINE = {
    "shift1_opens_s": 3.0,
    "clear_window_s": 2.0,
    "grasp_window_s": 3.0,
    "test_lift_s": 3.0,
    "lower_s": 0.5,
    "set_down_s": 0.5,
    "release_window_s": 1.0,
    "rest_s": 2.0,
    "pause_s": 10.0,
    "end_s": 1.0,
    "jobs_per_shift": 10,
    "shifts_per_unit": 2,
    "units": 2,
}

# ---------------------------------------------------------------------------------------------------------------
# The controller interface and its budgets
# ---------------------------------------------------------------------------------------------------------------
INTERFACE = {
    "init_wall_s": 60,
    "setup_cpu_s": 60,
    "setup_wall_s": 120,
    "period_ms": (5, 20),
    "read_blocks_max": 4,
    "read_area": (64, 147),                      # first and last address a read block may cover
    "command_effective_ticks": 1,                # a command written at tick k takes effect at tick k + 1
    "read_sample_ticks": 2,                      # the reads given to a step are sampled at tick k + 2
    "step_cpu_end_s": 1.0,
    "step_wall_end_s": 5.0,
    "unit_reply_wall_s": 3600,
    "reply_line_max_bytes": 1048576,
    "on_time_min_fraction": 0.9,
    "size_cap_mb": 80,
    "entries_max": 10000,                        # files and folders of the controller folder, in all
    "depth_max": 12,                             # folder levels below the controller folder (a file in it: level 1)
    "source_file_max_mb": 4,                     # each .py file of the controller folder
    "audit_cpu_s": 300,                          # the static checks of the controller's sources (CPU)
    "audit_wall_s": 600,                         # the static checks of the controller's sources (wall clock)
    "process_cpu_s": 7200,                       # CPU of each process of the controller, in all (prlimit --cpu)
    "stderr_log_max_mb": 5,                      # the controller's log per unit (stderr and fd 1); the rest is dropped
    "end_wait_s": 5,                             # after the end request: the processes are counted once none is left
                                                 # or this many seconds after the request, whichever comes first
}
# the bus timing rule: effective period = max(period_ms, ceil(sync_write_ms + sum over read blocks of
# (turnaround_ms + (request_bytes + servos x (status_overhead_bytes + length)) x ms_per_byte)))
BUS = {"baud": 4000000, "sync_write_ms": 1.5, "turnaround_ms": 1.0, "request_bytes": 27, "status_overhead_bytes": 11,
       "ms_per_byte": 0.0025, "servos": 13}

# ---------------------------------------------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------------------------------------------
SCORE = {
    "slip_band_mm": (1.5, 6.0),                  # full at the first, zero at the second
    "rot_band_deg": (3.0, 12.0),
    "release_no_contact_ms": 50,
    "release_window_s": 1.0,
    "release_push_mm": 3.0,
    "crush_run_ms": 10,
    "crush_impact_window_ms": 150,
    "drop_slip_mm": 20.0,
    "drop_no_contact_ms": 50,
    "lifted_min_above_nest_mm": 4.0,
    "seated_max_above_nest_mm": 1.0,
    "decline_rest_check_s": 1.0,
    "shift_base_weight": 0.7,
    "shift_j_weight": 0.3,
}

SESSION = {"wall_s": 21600, "cpus": 4, "memory_mb": 8192, "gpus": 0, "network": False,
           "verifier_timeout_s": 13500,           # the whole acceptance (verify_time_terms); reaching it: no score
           "host_runtime_mb": 2048}               # the grader's and the unit's share of memory_mb (measured 1,348 MB)

# The inputs of the acceptance's time limit beside the printed budgets (verify_time_terms): the two units
# run one after the other, each at the worst case of every printed budget, plus the plant's own time, the fixed steps
# and a margin. The plant's pace is an allowance above the slowest measured unit (1.21 CPU s per simulated s, the
# reference over a whole unit on a loaded host), for a controller that keeps more contact than the reference.
VERIFY_TIME = {
    "plant_cpu_s_per_sim_s": 2.0,
    "unit_overhead_s": 30,                      # a unit's worker start and model build, the close and the clean-up
    "lockdown_s": 11,                           # the verify-start kill (8 s at most), then stopping what outlived it (3 s)
    "other_fixed_s": 60,                        # the givens' hashes, the hidden set, the records and the verdict
    "margin": 1.35,
    "round_to_s": 100,
}


def move_time_s(distance_m):
    """Duration of a minimum-jerk move whose acceleration peak is the printed one (peak = 10/sqrt(3) d / T^2)."""
    return (10.0 / 3.0 ** 0.5 * abs(distance_m) / MOTION["translation_peak_mps2"]) ** 0.5


def tilt_time_s(deg):
    """Duration of a tilt: the angle at the printed rate, plus one ramp at each end."""
    return deg / MOTION["tilt_rate_deg_s"] + 2 * MOTION["tilt_ramp_s"]


def transfer_steps(profile):
    """The transfer of a profile as steps [{"kind", "t0_s", "dur_s", ...}] from the start of the transfer, in the
    pick nest's frame (z up): "move" steps carry "delta_m", "tilt" steps "axis" and "deg" (signed), "dwell" none."""
    tr = MOTION["transfer"]
    tilt = PROFILE_TILT[profile]
    steps, t = [], 0.0

    def add(kind, dur, **kw):
        nonlocal t
        steps.append(dict(kind=kind, t0_s=t, dur_s=dur, **kw))
        t += dur
    add("move", move_time_s(tr["up_m"]), delta_m=(0.0, 0.0, tr["up_m"]))
    if tilt:
        add("tilt", tilt_time_s(tilt["deg"]), axis=tilt["axis"], deg=tilt["deg"])
    add("move", move_time_s(tr["x_m"]), delta_m=(tr["x_m"], 0.0, 0.0))
    add("move", move_time_s(tr["y_m"]), delta_m=(0.0, tr["y_m"], 0.0))
    add("dwell", tr["dwell_s"])
    if tilt:
        add("tilt", tilt_time_s(tilt["deg"]), axis=tilt["axis"], deg=-tilt["deg"])
    add("move", move_time_s(tr["up_m"]), delta_m=(0.0, 0.0, -tr["up_m"]))
    return steps


def transfer_duration_s(profile):
    s = transfer_steps(profile)[-1]
    return s["t0_s"] + s["dur_s"]


def unit_timeline_max_s():
    """The longest timeline of one unit (s): the hand-over, every job of both shifts taking every window to its end with
    the longest transfer profile, the pause between the shifts and the end."""
    T = TIMELINE
    transfer = max(transfer_duration_s(p) for p in PROFILE_TILT)
    job = (T["clear_window_s"] + T["grasp_window_s"] + T["test_lift_s"] + transfer + T["set_down_s"]
           + T["release_window_s"] + T["rest_s"])
    n = T["shifts_per_unit"]
    return T["shift1_opens_s"] + n * T["jobs_per_shift"] * job + (n - 1) * T["pause_s"] + T["end_s"]


def verify_time_terms():
    """The arithmetic of SESSION["verifier_timeout_s"]: per unit the import, the setup, the controller's own reply time
    and one more step, the plant's time over the longest timeline and the unit's overhead; the fixed steps (the
    lockdown, the static checks, the rest); units x unit + fixed, times the margin, rounded up."""
    I, V = INTERFACE, VERIFY_TIME
    sim = unit_timeline_max_s()
    plant = sim * V["plant_cpu_s_per_sim_s"]
    unit = (I["init_wall_s"] + I["setup_wall_s"] + I["unit_reply_wall_s"] + I["step_wall_end_s"] + plant
            + V["unit_overhead_s"])
    fixed = V["lockdown_s"] + I["audit_wall_s"] + V["other_fixed_s"]
    raw = fixed + TIMELINE["units"] * unit
    step = V["round_to_s"]
    total = int(-(-(raw * V["margin"]) // step) * step)
    return {"unit_sim_s": sim, "unit_plant_s": plant, "unit_s": unit, "fixed_s": fixed, "raw_s": raw, "total_s": total}


def verify_time_strings():
    """The terms as the manifest's comment prints them ({vt_...} placeholders of task.toml)."""
    t, I, V = verify_time_terms(), INTERFACE, VERIFY_TIME

    def n(x):
        return format(int(round(x)), ",")
    return {"vt_init_s": n(I["init_wall_s"]), "vt_setup_s": n(I["setup_wall_s"]),
            "vt_reply_s": n(I["unit_reply_wall_s"]), "vt_step_s": n(I["step_wall_end_s"]),
            "vt_unit_sim_s": n(t["unit_sim_s"]), "vt_pace": "%g" % V["plant_cpu_s_per_sim_s"],
            "vt_unit_plant_s": n(t["unit_plant_s"]), "vt_unit_overhead_s": n(V["unit_overhead_s"]),
            "vt_unit_s": n(t["unit_s"]), "vt_lockdown_s": n(V["lockdown_s"]), "vt_audit_s": n(I["audit_wall_s"]),
            "vt_other_s": n(V["other_fixed_s"]), "vt_fixed_s": n(t["fixed_s"]),
            "vt_units": str(TIMELINE["units"]), "vt_raw_s": n(t["raw_s"]), "vt_margin": "%g" % V["margin"],
            "vt_round_s": n(V["round_to_s"]), "vt_total_s": n(t["total_s"])}


def period_ms(period, reads):
    """The effective cycle period in ms for a requested period and read blocks [[address, length], ...]."""
    b = BUS
    units = round(b["sync_write_ms"] / b["ms_per_byte"])          # exact: work in units of one byte time
    for _a, n in reads:
        units += round(b["turnaround_ms"] / b["ms_per_byte"]) + b["request_bytes"] \
            + b["servos"] * (b["status_overhead_bytes"] + int(n))
    per_ms = round(1.0 / b["ms_per_byte"])
    return max(int(period), -(-units // per_ms))
