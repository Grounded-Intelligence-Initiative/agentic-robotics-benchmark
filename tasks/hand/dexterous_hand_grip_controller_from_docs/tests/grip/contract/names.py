"""Names shared by the grader, the plant, the cell and the documents of the grip-controller acceptance.

One name per thing. Every string here may reach a document the controller's author reads, so the hand, its makers
and the vendors of its bought parts are named only by the neutral names and part numbers below. Joint names are
"<digit> <axis>" with a space (as printed in the documents and used in the model); link names are "<digit>_<link>"
with an underscore; each link has one mesh file "<link>.stl".
"""
from collections import namedtuple

# ---------------------------------------------------------------------------------------------------------------
# The hand and its bought parts
# ---------------------------------------------------------------------------------------------------------------
HAND = {"name": "the hand", "class_phrase": "a four-digit robot hand", "digits_phrase": "three fingers and a thumb"}

PARTS = {
    "servo": {"name": "servo actuator", "part_number": "VKR-4LBA", "vendor": "Vakrell Motion", "short": "servo",
              "count": 13},
    "finger_module": {"name": "fingertip tactile module", "part_number": "ZCP-F561", "short": "pad", "count": 3},
    "thumb_module": {"name": "thumb tactile module", "part_number": "ZCP-DG1Z", "short": "pad", "count": 1},
    "interface_board": {"name": "tactile interface board", "part_number": "ZCP-384F", "count": 1},
    "bus_board": {"name": "bus distribution board", "part_number": "BWD-RS8C", "count": 1},
    "hub": {"name": "power and bus hub board", "part_number": "JQT-NLP9", "short": "hub", "count": 1},
    "converter": {"name": "USB bus converter", "short": "converter", "count": 1},
}

# ---------------------------------------------------------------------------------------------------------------
# Digits, servos, joints, links, meshes
# ---------------------------------------------------------------------------------------------------------------
DIGITS = ("thumb", "index", "middle", "ring")
DIGIT_NAMES = {"thumb": "thumb", "index": "index finger", "middle": "middle finger", "ring": "ring finger"}
FINGERS = ("index", "middle", "ring")
FINGER_AXES = ("abad", "mcp", "pip")            # driven by a servo each
THUMB_AXES = ("roll", "side", "mp", "ip")        # driven by a servo each

Servo = namedtuple("Servo", "id joint digit axis")
# servo id -> joint (the ids are the bus ids of the delivered hand)
SERVOS = (
    Servo(0, "thumb ip", "thumb", "ip"),
    Servo(1, "thumb mp", "thumb", "mp"),
    Servo(2, "thumb side", "thumb", "side"),
    Servo(3, "thumb roll", "thumb", "roll"),
    Servo(4, "index pip", "index", "pip"),
    Servo(5, "index mcp", "index", "mcp"),
    Servo(6, "index abad", "index", "abad"),
    Servo(7, "middle pip", "middle", "pip"),
    Servo(8, "middle mcp", "middle", "mcp"),
    Servo(9, "middle abad", "middle", "abad"),
    Servo(10, "ring pip", "ring", "pip"),
    Servo(11, "ring mcp", "ring", "mcp"),
    Servo(12, "ring abad", "ring", "abad"),
)
N_SERVOS = len(SERVOS)
SERVO_JOINTS = tuple(s.joint for s in SERVOS)
SERVO_ID = {s.joint: s.id for s in SERVOS}
# the joints no servo drives: each finger's last joint and its four-bar rocker (both follow the pip axis)
PASSIVE_JOINTS = tuple("%s %s" % (f, a) for f in FINGERS for a in ("dip", "rocker"))
JOINTS = SERVO_JOINTS + PASSIVE_JOINTS

# links: the palm (root, carrying the finger and thumb bases) and the moving links of each digit
FINGER_LINKS = ("abad", "pitch", "pip", "dip", "rocker")
THUMB_LINKS = ("roll", "side", "mp", "ip")
LINKS = ("palm",) + tuple("%s_%s" % (f, k) for f in FINGERS for k in FINGER_LINKS) \
    + tuple("thumb_%s" % k for k in THUMB_LINKS)
# the link each joint moves (child) and the link it is mounted on (parent)
JOINT_CHILD = {}
JOINT_PARENT = {}
for _f in FINGERS:
    JOINT_PARENT["%s abad" % _f], JOINT_CHILD["%s abad" % _f] = "palm", "%s_abad" % _f
    JOINT_PARENT["%s mcp" % _f], JOINT_CHILD["%s mcp" % _f] = "%s_abad" % _f, "%s_pitch" % _f
    JOINT_PARENT["%s pip" % _f], JOINT_CHILD["%s pip" % _f] = "%s_pitch" % _f, "%s_pip" % _f
    JOINT_PARENT["%s dip" % _f], JOINT_CHILD["%s dip" % _f] = "%s_pip" % _f, "%s_dip" % _f
    JOINT_PARENT["%s rocker" % _f], JOINT_CHILD["%s rocker" % _f] = "%s_pitch" % _f, "%s_rocker" % _f
JOINT_PARENT["thumb roll"], JOINT_CHILD["thumb roll"] = "palm", "thumb_roll"
JOINT_PARENT["thumb side"], JOINT_CHILD["thumb side"] = "thumb_roll", "thumb_side"
JOINT_PARENT["thumb mp"], JOINT_CHILD["thumb mp"] = "thumb_side", "thumb_mp"
JOINT_PARENT["thumb ip"], JOINT_CHILD["thumb ip"] = "thumb_mp", "thumb_ip"
del _f
MESH_FILES = {link: link + ".stl" for link in LINKS}
# the four bars of a finger's loop, by the link that carries each span (pin to pin)
FOURBAR_SPANS = ("pitch", "pip", "dip", "rocker")


def hull_name(link, k):
    """Name of convex piece k of a link: the mesh asset name in the model and the stem of its STL file."""
    if link not in LINKS:
        raise ValueError("not a link: %r" % (link,))
    return "%s_h%02d" % (link, int(k))


def site_name(link, what):
    """Name of a site on a link (what: a short neutral word, e.g. "pad")."""
    if link not in LINKS:
        raise ValueError("not a link: %r" % (link,))
    return "%s_%s" % (link, what)


# ---------------------------------------------------------------------------------------------------------------
# Tactile modules ("pads")
# ---------------------------------------------------------------------------------------------------------------
MODULES = ("thumb", "index", "middle", "ring")
MODULE_TAXELS = {"thumb": 127, "index": 52, "middle": 52, "ring": 52}
MODULE_PART = {"thumb": "thumb_module", "index": "finger_module", "middle": "finger_module", "ring": "finger_module"}
MODULE_LINK = {"thumb": "thumb_ip", "index": "index_dip", "middle": "middle_dip", "ring": "ring_dip"}
GRASP_MODULES = ("thumb", "index", "middle")     # the taught three-pad grasp: thumb against index and middle

# ---------------------------------------------------------------------------------------------------------------
# The cell: specimens, finishes, profiles, phases, base segments
# ---------------------------------------------------------------------------------------------------------------
CLASSES = ("brittle", "soft", "full_load")
CLASS_NAMES = {"brittle": "brittle shell", "soft": "soft pack", "full_load": "full-load block"}
CODES = {"brittle": ("SH-1", "SH-2", "SH-3"), "soft": ("SP",), "full_load": ("FL",)}
ALL_CODES = tuple(c for k in CLASSES for c in CODES[k])
CODE_CLASS = {c: k for k in CLASSES for c in CODES[k]}
FINISHES = ("matte", "rough")
PROFILES = ("T0", "R30", "P30")
LABELS = ("carry", "decline")
NESTS = ("pick", "place")

PHASES = ("handover", "rest", "await_clear", "present", "test_lift", "lower", "transfer", "set_down", "release",
          "pause", "end")
SET_BASE_KINDS = ("rest", "test_lift", "lower", "transfer", "set_down", "return")

# ---------------------------------------------------------------------------------------------------------------
# Outcomes
# ---------------------------------------------------------------------------------------------------------------
EVENT_KINDS = ("crush_static", "crush_impact", "drop_slip", "drop_contact", "shutdown_overheat", "shutdown_overload")
JOB_LOSS_KINDS = ("not_presented", "not_grasped", "not_lifted", "not_seated", "release_late", "release_push",
                  "decline_missed", "decline_wrong", "voided")
PARTIAL_LOSS_KINDS = ("band_slip", "band_rot")
CONTROLLER_FAULT_KINDS = ("controller_ended", "late_cycles")
LOSS_KINDS = EVENT_KINDS + JOB_LOSS_KINDS + PARTIAL_LOSS_KINDS + CONTROLLER_FAULT_KINDS
# the kind an observation reports after an event that ended a shift
OBS_EVENT_KINDS = ("crush", "drop", "shutdown")
OBS_EVENT_OF = {k: k.split("_")[0] for k in EVENT_KINDS}

# the acceptance's pass conditions on the controller (each a 0 / 1 in the score)
GATES = ("controller_file", "size", "import_audit", "setup", "on_time", "no_late_write", "clean", "network")

# The static import audit of the controller's sources (grader.runtime.netaudit matches exactly these names; the
# controller interface prints them from here). Modules a source may not import, nor any submodule of one:
AUDIT_MODULES = ("socket", "_socket", "ssl", "_ssl", "urllib", "urllib2", "urllib3", "http", "httplib", "requests",
                 "httpx", "aiohttp", "ftplib", "smtplib", "imaplib", "poplib", "nntplib", "telnetlib", "websocket",
                 "websockets", "paramiko", "pycurl", "xmlrpc", "asyncio", "socketserver", "grpc", "zmq",
                 "huggingface_hub", "boto3", "botocore", "wget", "ctypes", "_ctypes", "cffi", "selectors",
                 "multiprocessing.connection", "multiprocessing.managers", "logging.handlers", "subprocess", "pty",
                 "_posixsubprocess")
# Calls that start another program: these names as attributes of the modules of AUDIT_PROCESS_MODULES (under the
# module's own name or any other name bound to it: `import os as o`, `o = os`, `sys.modules["os"]`, `__import__("os")`;
# also through getattr(), vars() or __dict__ with the name as a string), and these names imported from those modules
# (`from os import system`; `from os import *` imports them all). A function or method of the controller's own that
# has such a name is no hit.
AUDIT_PROCESS_CALLS = ("system", "popen", "execv", "execve", "execvp", "execvpe", "execl", "execle", "execlp",
                       "execlpe", "spawnv", "spawnve", "spawnvp", "spawnvpe", "spawnl", "spawnle", "spawnlp",
                       "spawnlpe", "posix_spawn", "posix_spawnp", "fork_exec")
AUDIT_PROCESS_MODULES = ("os", "posix", "subprocess", "_posixsubprocess")
# A module loaded under a computed name: one of these calls, __import__ as the built-in (or importlib.__import__), the
# others as attributes of AUDIT_IMPORT_MODULES or of a module inside it (importlib.util, importlib.machinery, ...) or of
# an object reached from one, under any name bound to it, or imported from one; called with a first argument that is
# not a string literal (or used other than as a call or an assignment to a name): a hit, except AUDIT_LITERAL_IMPORT_
# CALLS; called with a string literal that names a refused module (or a module inside one): that module's import.
AUDIT_IMPORT_CALLS = ("__import__", "import_module", "find_spec", "spec_from_file_location", "load_module")
AUDIT_LITERAL_IMPORT_CALLS = ("spec_from_file_location", "load_module")
AUDIT_IMPORT_MODULES = ("importlib",)
# Computed code: exec / eval / compile as the built-ins (the plain name where the file does not bind it itself; an
# attribute of AUDIT_BUILTINS_MODULES, or of __builtins__, under any name bound to it; imported from it), called with a
# first argument that is not a string literal (or used other than as a call or an assignment to a name). A function,
# method or attribute of the controller's own, or another module's function, that only has such a name is no hit.
AUDIT_CODE_CALLS = ("exec", "eval", "compile")
AUDIT_BUILTINS_MODULES = ("builtins",)

UNITS = (1, 2)
SHIFTS = (1, 2)
_SHIFT_KEYS = ("S", "J", "carry_mean", "events_crush", "events_drop", "events_shutdown")
_UNIT_KEYS = ("on_time_fraction", "late_cycles", "controller_ended", "wall_s")
# the only keys the verifier publishes among its metrics (aggregates; no per-job value)
METRIC_KEYS = (
    ("ratio", "anchor_recorded", "reward_exact", "acceptance_score", "contract_ok")
    + tuple("gate_" + g for g in GATES)
    + tuple("u%ds%d_%s" % (u, s, k) for u in UNITS for s in SHIFTS for k in _SHIFT_KEYS)
    + tuple("u%d_%s" % (u, k) for u in UNITS for k in _UNIT_KEYS)
    + ("privileged_mode", "oracle_token_mismatch", "network_imports", "hidden_set", "root_payload_ok",
       "controller_no_network", "remote_sockets_seen", "net_outbound_events", "stray_processes",
       "runner_processes_killed", "runner_files_removed", "units_in_turn", "plant_faults", "agent_processes_killed",
       "agent_processes_left", "agent_processes_running",
       "agent_processes_frozen", "verify_wall_s", "runner_oom_adj_killed", "oom_floor_active", "stderr_bytes_dropped")
)

# unit serials: "HU-" and five characters of the part-number alphabet
SERIAL_ALPHABET = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ"
SERIAL_PATTERN = r"^HU-[0-9A-HJ-NP-Z]{5}$"

# ---------------------------------------------------------------------------------------------------------------
# The servo's control table, as the vendor prints it
# ---------------------------------------------------------------------------------------------------------------
# address, size in bytes, key (the vendor's data name in snake case), the vendor's data name, signed (two's
# complement), the printed unit, area, access, the printed initial value (None where the table prints "-"), the
# printed range (text as printed), and the numeric bounds a written value must lie in (None where none is printed).
# Symbolic ranges take the referenced EEPROM limit's printed maximum; Goal Position takes the range the vendor prints
# for current-based position control mode.
Register = namedtuple("Register", "address size key name signed unit area access initial range_text lo hi")

REGISTERS = (
    Register(11, 1, "operating_mode", "Operating Mode", False, "-", "EEPROM", "RW", 3, "0 ~ 16", 0, 16),
    Register(64, 1, "torque_enable", "Torque Enable", False, "-", "RAM", "RW", 0, "0 ~ 1", 0, 1),
    Register(65, 1, "led", "LED", False, "-", "RAM", "RW", 0, "0 ~ 1", 0, 1),
    Register(68, 1, "status_return_level", "Status Return Level", False, "-", "RAM", "RW", 2, "0 ~ 2", 0, 2),
    Register(69, 1, "registered_instruction", "Registered Instruction", False, "-", "RAM", "R", 0, "0 ~ 1", 0, 1),
    Register(70, 1, "hardware_error_status", "Hardware Error Status", False, "-", "RAM", "R", 0, "-", None, None),
    Register(76, 2, "velocity_i_gain", "Velocity I Gain", False, "-", "RAM", "RW", 1800, "0 ~ 16,383", 0, 16383),
    Register(78, 2, "velocity_p_gain", "Velocity P Gain", False, "-", "RAM", "RW", 50, "0 ~ 16,383", 0, 16383),
    Register(80, 2, "position_d_gain", "Position D Gain", False, "", "RAM", "RW", 0, "0 ~ 16,383", 0, 16383),
    Register(82, 2, "position_i_gain", "Position I Gain", False, "", "RAM", "RW", 0, "0 ~ 16,383", 0, 16383),
    Register(84, 2, "position_p_gain", "Position P Gain", False, "", "RAM", "RW", 2000, "0 ~ 16,383", 0, 16383),
    Register(88, 2, "feedforward_2nd_gain", "Feedforward 2nd Gain", False, "", "RAM", "RW", 0, "0 ~ 16,383", 0,
             16383),
    Register(90, 2, "feedforward_1st_gain", "Feedforward 1st Gain", False, "", "RAM", "RW", 0, "0 ~ 16,383", 0,
             16383),
    Register(98, 1, "bus_watchdog", "Bus Watchdog", False, "20 [msec]", "RAM", "RW", 0, "1 ~ 127", 1, 127),
    Register(100, 2, "goal_pwm", "Goal PWM", True, "0.113 [%]", "RAM", "RW", None,
             "-PWM Limit(36) ~ PWM Limit(36)", -885, 885),
    Register(102, 2, "goal_current", "Goal Current", True, "1.0 [mA]", "RAM", "RW", None,
             "-Current Limit(38) ~ Current Limit(38)", -910, 910),
    Register(104, 4, "goal_velocity", "Goal Velocity", True, "0.229 [rev/min]", "RAM", "RW", None,
             "-Velocity Limit(44) ~ Velocity Limit(44)", -2047, 2047),
    Register(108, 4, "profile_acceleration", "Profile Acceleration", False, "214.577 [rev/min2] 1 [msec]", "RAM",
             "RW", 0, "0 ~ 32,767", 0, 32767),
    Register(112, 4, "profile_velocity", "Profile Velocity", False, "0.229 [rev/min]", "RAM", "RW", 0, "0 ~ 32,767",
             0, 32767),
    Register(116, 4, "goal_position", "Goal Position", True, "1 [pulse]", "RAM", "RW", None,
             "Min Position Limit(52) ~ Max Position Limit(48)", -1048575, 1048575),
    Register(120, 2, "realtime_tick", "Realtime Tick", False, "1 [msec]", "RAM", "R", None, "0 ~ 32,767", 0, 32767),
    Register(122, 1, "moving", "Moving", False, "-", "RAM", "R", 0, "0 ~ 1", 0, 1),
    Register(123, 1, "moving_status", "Moving Status", False, "-", "RAM", "R", 0, "-", None, None),
    Register(124, 2, "present_pwm", "Present PWM", True, "0.113 [%]", "RAM", "R", None, "-", None, None),
    Register(126, 2, "present_current", "Present Current", True, "1.0 [mA]", "RAM", "R", None, "-", None, None),
    Register(128, 4, "present_velocity", "Present Velocity", True, "0.229 [rev/min]", "RAM", "R", None, "-", None,
             None),
    Register(132, 4, "present_position", "Present Position", True, "1 [pulse]", "RAM", "R", None, "-", None, None),
    Register(136, 4, "velocity_trajectory", "Velocity Trajectory", True, "0.229 [rev/min]", "RAM", "R", None, "-",
             None, None),
    Register(140, 4, "position_trajectory", "Position Trajectory", True, "1 [pulse]", "RAM", "R", None, "-", None,
             None),
    Register(144, 2, "present_input_voltage", "Present Input Voltage", False, "0.1 [V]", "RAM", "R", None, "-", None,
             None),
    Register(146, 1, "present_temperature", "Present Temperature", False, "1 [℃]", "RAM", "R", None, "-", None,
             None),
    Register(147, 1, "backup_ready", "Backup Ready", False, "-", "RAM", "R", None, "0 ~ 1", 0, 1),
)
REGISTER = {r.key: r for r in REGISTERS}
RAM_FIRST, RAM_END = 64, 148            # a read block lies inside [RAM_FIRST, RAM_END)

# the EEPROM entries the servos use: six are in the unit's record (record key -> control-table entry); the others
# hold the vendor's printed initial values on every unit
EepromEntry = namedtuple("EepromEntry", "address size key name unit initial range_text")
EEPROM_ENTRIES = (
    EepromEntry(9, 1, "return_delay_time", "Return Delay Time", "2 [μsec]", 250, "0 ~ 254"),
    EepromEntry(11, 1, "operating_mode", "Operating Mode", "-", 3, "0 ~ 16"),
    EepromEntry(24, 4, "moving_threshold", "Moving Threshold", "0.229 [rev/min]", 10, "0 ~ 1,023"),
    EepromEntry(31, 1, "temperature_limit", "Temperature Limit", "1 [℃]", 70, "0 ~ 100"),
    EepromEntry(36, 2, "pwm_limit", "PWM Limit", "0.113 [%]", 885, "0 ~ 885"),
    EepromEntry(38, 2, "current_limit", "Current Limit", "1.0 [mA]", 910, "0 ~ 910"),
    EepromEntry(44, 4, "velocity_limit", "Velocity Limit", "0.229 [rev/min]", 280, "0 ~ 2,047"),
    EepromEntry(48, 4, "max_position_limit", "Max Position Limit", "1 [pulse]", 3072, "0 ~ 4,095"),
    EepromEntry(52, 4, "min_position_limit", "Min Position Limit", "1 [pulse]", 1024, "0 ~ 4,095"),
    EepromEntry(63, 1, "shutdown", "Shutdown", "-", 52, "-"),
)
RECORD_EEPROM_KEYS = {"operating_mode": "operating_mode", "current_limit": "current_limit",
                      "temperature_limit": "temperature_limit", "shutdown": "shutdown",
                      "return_delay": "return_delay_time", "pwm_limit": "pwm_limit"}
# the entries outside the record: the vendor's initial value on every unit (control-table key -> value)
EEPROM_DEFAULTS = {e.key: e.initial for e in EEPROM_ENTRIES if e.key not in RECORD_EEPROM_KEYS.values()}
# every servo's direction as delivered (+1: a positive joint angle reads as rising Present Position)
SERVO_DIRECTION = (1,) * N_SERVOS

# the fields a step reply may write, in the order they are applied within one reply (see the contract): torque off
# first, then the operating mode, then the RAM fields by ascending address, then torque on
WRITE_FIELDS = ("torque_enable", "operating_mode", "position_d_gain", "position_i_gain", "position_p_gain",
                "goal_current", "profile_acceleration", "profile_velocity", "goal_position")
WRITE_ADDRESS = {k: REGISTER[k].address for k in WRITE_FIELDS}
OPERATING_MODES = {0: "current control", 1: "velocity control", 3: "position control",
                   4: "extended position control", 5: "current-based position control", 16: "PWM control"}
HARDWARE_ERROR_BITS = {"input_voltage": 0x01, "overheating": 0x04, "electrical_shock": 0x10, "overload": 0x20}


def registers_in(reads):
    """The REGISTERS entries lying wholly inside the read blocks [[address, length], ...], in address order."""
    out = []
    for r in REGISTERS:
        if r.area != "RAM":
            continue
        for a, n in reads:
            if a <= r.address and r.address + r.size <= a + n:
                out.append(r)
                break
    return out
