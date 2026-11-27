"""The contract gate of the unit's description files: unit.xml and calibration.yaml in the submission folder.

One module, used unchanged by the grader and by the format checker beside it (check_submission.py). It knows nothing
about the unit: every rule below is a rule of the contract, printed with its numbers in acceptance.txt, and every
reference value is either a number of the contract or the given model file's own (its 26 link bodies, 25 hinges,
free joint and 26 mesh files, as MuJoCo 3.14.0 compiles them).

    result = check(submission_dir, given)
        given:  the folder that holds the given model's 26 mesh files, or a mapping {mesh file name: bytes}, or an
                object whose `assets` attribute is such a mapping
        result.ok           True when no rule failed
        result.failures     [Failure(code, where, message)], every rule that failed (codes: RULES)
        result.reasons      [(code, "where: message")], the same list in pairs
        result.model        the compiled unit.xml (mujoco.MjModel, compiled with the given meshes), or None
        result.calib        calibration.yaml as loaded (a dict), or None
        result.calibration  {"sign": (25,), "home": (25,), "ankle_map": {side: {"pitch": (19, 19), "roll": ...}}}
                            as NumPy arrays, or None
        result.info         counts, masses and site placements read from the compiled model

Order. The files (present, regular files, within their caps) -> unit.xml's static rules (no DOCTYPE or ENTITY
declaration, the element whitelist, the compiler, mesh, texture and attribute rules, the caps) -> calibration.yaml
(UTF-8; no &-label, *-reference or !-tag; one document read by yaml.safe_load; the schema unit-calibration/1) ->
only when every static rule of unit.xml held: the compile (MuJoCo 3.14.0, the given meshes, no error, no warning) ->
the fixed tree, the three sites, the masses, finite numbers. The grader adds one rule of its own, passed in as
`late_entries`: an entry of the submission folder written after the session ended zeroes the gate.

Nothing submitted is executed. unit.xml is parsed as data (expat), its compiler's meshdir, assetdir and texturedir
are dropped and each mesh is named by its file name only, and MuJoCo compiles that text from a string with the given
meshes in memory: whatever folder a submission's mesh paths point at, the given copies are the ones compiled.
calibration.yaml is read by PyYAML's pure-Python safe loader. Imports: the standard library, NumPy, PyYAML, MuJoCo.
"""

from __future__ import annotations

import math
import os
import stat as _stat
import tempfile
import xml.etree.ElementTree as ET
import xml.parsers.expat as _expat
from collections.abc import Mapping

import numpy as np
import yaml

GATE_VERSION = "unit-description-gate/1.0.0"
MUJOCO_VERSION = "3.14.0"                      # the compiler the contract names

UNIT_XML = "unit.xml"
CALIBRATION_YAML = "calibration.yaml"
MAX_XML_BYTES = 1024 * 1024                    # 1 MiB
MAX_YAML_BYTES = 256 * 1024                    # 256 KiB
MAX_BODIES = 64                                # bodies besides the world
MAX_GEOMS = 512
MAX_SITES = 64
MASS_RANGE_KG = (20.0, 60.0)                   # the total mass, inclusive
TOL_M = 1e-6                                   # positions of the fixed tree
TOL_RAD = 1e-6                                 # orientations, axes and refs of the fixed tree

# calibration.yaml, schema unit-calibration/1
SCHEMA_NAME = "unit-calibration/1"
SLOTS = ("L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ankle_A", "L_ankle_B",
         "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ankle_A", "R_ankle_B",
         "L_shoulder_pitch", "L_shoulder_roll", "L_shoulder_yaw", "L_elbow", "L_wrist_yaw",
         "R_shoulder_pitch", "R_shoulder_roll", "R_shoulder_yaw", "R_elbow", "R_wrist_yaw",
         "waist_yaw", "neck_yaw", "neck_pitch")
HOME_BOUND_RAD = 6.3                           # |home_rad| <= 6.3
AXES = ("A", "B")                              # row index: motor A; column index: motor B
EXTENT_RAD = 0.787                             # both axes from -0.787 to 0.787 rad
NODES = 19                                     # nodes per axis
NODE_BOUND_RAD = 1.6                           # |pitch|, |roll| <= 1.6 at every node
NODE_STEP_RAD = 2 * EXTENT_RAD / (NODES - 1)
NODE_VALUES_RAD = tuple(-EXTENT_RAD + i * NODE_STEP_RAD for i in range(NODES))
SIDES = ("left", "right")

IMU_SITE = "imu"
SOLE_SITES = (("left_sole", "left_ankle_roll_link"), ("right_sole", "right_ankle_roll_link"))

# ---- BEGIN GENERATED BLOCK: the given model's fixed tree (do not edit by hand) ----
# The 26 link bodies: (name, parent, position in the parent's frame (m), orientation (w, x, y, z)).
LINK_BODIES = (
    ('pelvis_link', 'world', (0.0, 0.0, 0.63),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_hip_pitch_link', 'pelvis_link', (-0.0519919979, 0.06849986203, -0.044045),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_hip_roll_link', 'left_hip_pitch_link', (0.0422501, 0.04000027594, 0.0),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_hip_yaw_link', 'left_hip_roll_link', (-0.04225, 0.0, -0.052),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_knee_link', 'left_hip_yaw_link', (0.0, 0.0, -0.19564),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_ankle_pitch_link', 'left_knee_link', (0.0, 0.0, -0.2946623361),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_ankle_roll_link', 'left_ankle_pitch_link', (-0.0015, 0.0, -0.009999992),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_hip_pitch_link', 'pelvis_link', (-0.0519919979, -0.06850013797, -0.044045),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_hip_roll_link', 'right_hip_pitch_link', (0.0422501, -0.04000027594, 0.0),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_hip_yaw_link', 'right_hip_roll_link', (-0.04225, 0.0, -0.052),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_knee_link', 'right_hip_yaw_link', (0.0, 0.0, -0.19564),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_ankle_pitch_link', 'right_knee_link', (0.0, 0.0, -0.2946623361),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_ankle_roll_link', 'right_ankle_pitch_link', (-0.0015, 0.0, -0.009999992),
     (1.0, 0.0, 0.0, 0.0)),
    ('waist_yaw_link', 'pelvis_link', (-0.0519919979, 0.0, 0.074755),
     (1.0, 0.0, 0.0, 0.0)),
    ('neck_yaw_link', 'waist_yaw_link', (-0.01659877455, 0.0, 0.37817729),
     (1.0, 0.0, 0.0, 0.0)),
    ('neck_pitch_link', 'neck_yaw_link', (0.0, 0.0, 0.03735),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_shoulder_pitch_link', 'waist_yaw_link', (-0.02621282535, -0.0965, 0.2611404833),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_shoulder_roll_link', 'right_shoulder_pitch_link', (5e-05, -0.06535322324, 6.334135041e-05),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_shoulder_yaw_link', 'right_shoulder_roll_link', (0.0, -0.0007840587182, -0.1289963672),
     (1.0, 0.0, 0.0, 0.0)),
    ('right_elbow_link', 'right_shoulder_yaw_link', (0.0, 0.0004006795283, -0.09860555673),
     (0.9396857289717903, 1.3350542699599211e-11, 0.34203275988973203, -0.0020789249549375897)),
    ('right_wrist_yaw_link', 'right_elbow_link', (0.106, -0.0009999815286, 0.0),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_shoulder_pitch_link', 'waist_yaw_link', (-0.02621282535, 0.0965, 0.2611404833),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_shoulder_roll_link', 'left_shoulder_pitch_link', (5e-05, 0.06535322324, 6.334135041e-05),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_shoulder_yaw_link', 'left_shoulder_roll_link', (0.0, 0.0007840587182, -0.1289963672),
     (1.0, 0.0, 0.0, 0.0)),
    ('left_elbow_link', 'left_shoulder_yaw_link', (0.0, -0.0004006795283, -0.09860555673),
     (0.939680598306951, -5.5867843300413276e-12, 0.3420468548025302, 0.0020790106270153793)),
    ('left_wrist_yaw_link', 'left_elbow_link', (0.106, 0.0009999815286, 0.0),
     (1.0, 0.0, 0.0, 0.0)),
)
# The 25 hinges: (name, body, position in the body's frame (m), axis in the body's frame, ref (rad)).
HINGES = (
    ('left_hip_pitch_joint', 'left_hip_pitch_link', (0.0, 0.0, 0.0),
     (0.0, 1.0, 0.0), 0.0),
    ('left_hip_roll_joint', 'left_hip_roll_link', (0.0, 0.0, 0.0),
     (1.0, 0.0, 0.0), 0.0),
    ('left_hip_yaw_joint', 'left_hip_yaw_link', (0.0, 0.0, 0.0),
     (0.0, 0.0, -1.0), 0.0),
    ('left_knee_joint', 'left_knee_link', (0.0, 0.0, 0.0),
     (0.0, 1.0, 0.0), 0.0),
    ('left_ankle_pitch_joint', 'left_ankle_pitch_link', (0.0, 0.0, 0.0),
     (0.0, 1.0, 0.0), 0.0),
    ('left_ankle_roll_joint', 'left_ankle_roll_link', (0.0, 0.0, 0.0),
     (-1.0, 0.0, 0.0), 0.0),
    ('right_hip_pitch_joint', 'right_hip_pitch_link', (0.0, 0.0, 0.0),
     (0.0, -1.0, 0.0), 0.0),
    ('right_hip_roll_joint', 'right_hip_roll_link', (0.0, 0.0, 0.0),
     (1.0, 0.0, 0.0), 0.0),
    ('right_hip_yaw_joint', 'right_hip_yaw_link', (0.0, 0.0, 0.0),
     (0.0, 0.0, -1.0), 0.0),
    ('right_knee_joint', 'right_knee_link', (0.0, 0.0, 0.0),
     (0.0, -1.0, 0.0), 0.0),
    ('right_ankle_pitch_joint', 'right_ankle_pitch_link', (0.0, 0.0, 0.0),
     (0.0, -1.0, 0.0), 0.0),
    ('right_ankle_roll_joint', 'right_ankle_roll_link', (0.0, 0.0, 0.0),
     (-1.0, 0.0, 0.0), 0.0),
    ('waist_yaw_joint', 'waist_yaw_link', (0.0, 0.0, 0.0),
     (0.0, 0.0, 1.0), 0.0),
    ('neck_yaw_joint', 'neck_yaw_link', (0.0, 0.0, 0.0),
     (0.0, 0.0, 1.0), 0.0),
    ('neck_pitch_joint', 'neck_pitch_link', (0.0, 0.0, 0.0),
     (0.0, -1.0, 0.0), 0.0),
    ('right_shoulder_pitch_joint', 'right_shoulder_pitch_link', (0.0, 0.0, 0.0),
     (0.0, -1.0, 0.0), 0.0),
    ('right_shoulder_roll_joint', 'right_shoulder_roll_link', (0.0, 0.0, 0.0),
     (-1.0, 0.0, 0.0), 0.0),
    ('right_shoulder_yaw_joint', 'right_shoulder_yaw_link', (0.0, 0.0, 0.0),
     (0.0, -0.006078033455903102, -0.999981528584058), 0.0),
    ('right_elbow_joint', 'right_elbow_link', (0.0, 0.0, 0.0),
     (0.0, 0.999981528584058, -0.006078033455903102), -0.785398),
    ('right_wrist_yaw_joint', 'right_wrist_yaw_link', (0.0, 0.0, 0.0),
     (1.0, 0.0, 0.0), 0.0),
    ('left_shoulder_pitch_joint', 'left_shoulder_pitch_link', (0.0, 0.0, 0.0),
     (0.0, 1.0, 0.0), 0.0),
    ('left_shoulder_roll_joint', 'left_shoulder_roll_link', (0.0, 0.0, 0.0),
     (-1.0, 0.0, 0.0), 0.0),
    ('left_shoulder_yaw_joint', 'left_shoulder_yaw_link', (0.0, 0.0, 0.0),
     (0.0, 0.006078009292187985, -0.9999815287309282), 0.0),
    ('left_elbow_joint', 'left_elbow_link', (0.0, 0.0, 0.0),
     (0.0, -0.999981528584058, -0.006078033455903102), 0.785398),
    ('left_wrist_yaw_joint', 'left_wrist_yaw_link', (0.0, 0.0, 0.0),
     (1.0, 0.0, 0.0), 0.0),
)
FREE_JOINT_BODY = 'pelvis_link'
# The given model's 26 mesh files (the only mesh files a submission may name).
MESH_FILES = (
    'left_ankle_pitch_link.STL', 'left_ankle_roll_link.STL', 'left_elbow_link.STL',
    'left_hip_pitch_link.STL', 'left_hip_roll_link.STL', 'left_hip_yaw_link.STL',
    'left_knee_link.STL', 'left_shoulder_pitch_link.STL', 'left_shoulder_roll_link.STL',
    'left_shoulder_yaw_link.STL', 'left_wrist_yaw_link.STL', 'neck_pitch_link.STL',
    'neck_yaw_link.STL', 'pelvis_link.STL', 'right_ankle_pitch_link.STL',
    'right_ankle_roll_link.STL', 'right_elbow_link.STL', 'right_hip_pitch_link.STL',
    'right_hip_roll_link.STL', 'right_hip_yaw_link.STL', 'right_knee_link.STL',
    'right_shoulder_pitch_link.STL', 'right_shoulder_roll_link.STL', 'right_shoulder_yaw_link.STL',
    'right_wrist_yaw_link.STL', 'waist_yaw_link.STL',
)
# ---- END GENERATED BLOCK ----

LINK_NAMES = tuple(b[0] for b in LINK_BODIES)
HINGE_NAMES = tuple(h[0] for h in HINGES)

# ------------------------------------------------------------------------------------------------ the whitelist
# unit.xml is checked element by element against this policy before anything is compiled. It is a shared MJCF
# whitelist policy with this contract's changes: the elements below; option with any attribute; no actuator, tendon
# or equality; textures built in only; meshes only from the given model's files; the dynamics bands of the shared
# policy are off (this grader runs no simulation).
ANY = "*"                                      # any child element (sensor types)
CHILDREN = {                                   # element -> the child elements allowed inside it
    None: frozenset({"mujoco"}),
    "mujoco": frozenset({"compiler", "option", "visual", "statistic", "default", "asset", "worldbody", "contact",
                         "sensor", "keyframe"}),
    "option": frozenset({"flag"}),
    "visual": frozenset({"global", "quality", "headlight", "map", "scale", "rgba"}),
    "default": frozenset({"default", "mesh", "material", "joint", "geom", "site", "camera", "light"}),
    "asset": frozenset({"mesh", "material", "texture"}),
    "material": frozenset({"layer"}),
    "worldbody": frozenset({"body", "geom", "site", "camera", "light"}),
    "body": frozenset({"body", "inertial", "joint", "freejoint", "geom", "site", "camera", "light"}),
    "contact": frozenset({"exclude"}),
    "sensor": ANY,
    "keyframe": frozenset({"key"}),
}
IGNORED_ELEMENTS = ("visual (and its children)", "statistic", "camera", "light", "sensor (and its children)",
                    "material", "texture (built in only)", "keyframe", "option (any attribute)", "contact/exclude")
READ_ELEMENTS = ("mujoco", "compiler", "default", "asset/mesh (the given model's 26 mesh files only)", "worldbody",
                 "body", "inertial", "joint", "freejoint", "geom", "site")
FORBIDDEN = frozenset({"include", "plugin", "extension", "instance", "custom", "numeric", "text", "tuple", "equality",
                       "tendon", "actuator", "flex", "flexcomp", "deformable", "skin", "hfield", "composite",
                       "replicate", "frame", "attach", "size", "pair"})
COMPILER_ATTRS = frozenset({"angle", "meshdir", "texturedir", "assetdir", "autolimits", "inertiafromgeom",
                            "balanceinertia", "eulerseq", "strippath", "coordinate", "usethread",
                            "exactmeshinertia", "inertiagrouprange"})
COMPILER_ANGLE = "radian"
COMPILER_IGNORED = ("meshdir", "assetdir", "texturedir")
REFUSED_ATTRS = {                              # element -> attributes refused unless false / 0
    "compiler": ("fusestatic", "discardvisual", "alignfree"),
    "freejoint": ("align",),
    "body": ("mocap", "gravcomp"),
    "geom": ("fluidshape", "fluidcoef"),
}
_OFF_VALUES = ("false", "0", "0.0")
MESH_DATA_ATTRS = ("vertex", "normal", "texcoord", "face", "builtin", "params")
TEXTURE_FILE_ATTRS = ("file", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback")
TEXTURE_BUILTINS = ("gradient", "checker", "flat")

# ------------------------------------------------------------------------------------------------ the rules
RULES = {
    "files.folder": "The submission folder exists and is a folder, not a symbolic link.",
    "files.missing": ("unit.xml and calibration.yaml lie in the submission folder, each a regular file (not a "
                      "symbolic link, a folder or a special file). No other file of the folder is read."),
    "files.size": "unit.xml has at most 1048576 bytes (1 MiB); calibration.yaml at most 262144 bytes (256 KiB).",
    "files.late_write": ("Nothing in the submission folder was written after the session ended: the grader's own "
                         "check of each entry's change and modification times against the moment the grading stage "
                         "was set up (the checker cannot run it)."),
    "xml.declaration": "unit.xml is UTF-8 text with no DOCTYPE and no ENTITY declaration.",
    "xml.parse": "unit.xml is well-formed XML and its root element is <mujoco>.",
    "xml.forbidden": ("No element of the forbidden list appears anywhere: " + ", ".join(sorted(FORBIDDEN)) +
                      " (pair: the contact pair)."),
    "xml.unlisted": "Every element is on the whitelist and sits where the whitelist allows it.",
    "xml.compiler": ('Some <compiler> states angle="radian" and none states another unit; a compiler attribute is '
                     "one of: " + ", ".join(sorted(COMPILER_ATTRS)) + " (meshdir, assetdir and texturedir are "
                     "ignored)."),
    "xml.refused": ("These attributes are refused unless false or 0: fusestatic, discardvisual and alignfree on "
                    "<compiler>, align on <freejoint>, mocap and gravcomp on <body>, fluidshape and fluidcoef on "
                    "<geom>."),
    "xml.mesh": ("Every <mesh> of the assets has a file attribute whose file name (its last path component) is one "
                 "of the given model's 26 mesh files; none is given by inline data (vertex, normal, texcoord, face) "
                 "or built in (builtin, params). The grader compiles its own copy of the named file."),
    "xml.texture": "Every <texture> is built in (builtin gradient, checker or flat) and names no file.",
    "xml.caps": "unit.xml has at most 64 bodies besides the world, 512 geoms and 64 sites.",
    "model.compile": ("unit.xml compiles in MuJoCo 3.14.0 without an error (with the given meshes, its meshdir, "
                      "assetdir and texturedir dropped)."),
    "model.warning": "The compile raises no warning (MuJoCo warns, for example, when the XML contains a NaN).",
    "tree.bodies": ("The given model's 26 link bodies are present, each by its name and with the given model's "
                    "parent. Further bodies may be added; they carry no joint."),
    "tree.frames": ("Each link body's position (within 1e-6 m) and orientation (within 1e-6 rad) in its parent's "
                    "frame equal the given model's."),
    "tree.joints": ("There are exactly 26 joints: the given model's 25 hinges, each by its name, a hinge and on its "
                    "given body, and one free joint, on pelvis_link."),
    "tree.hinges": ("Each hinge's axis (within 1e-6 rad), its position, the point its axis passes through (within "
                    "1e-6 m), and its ref (within 1e-6 rad) equal the given model's."),
    "sites.imu": "A site named imu lies on a body of the unit: pelvis_link, a link below it, or a body fixed to one.",
    "sites.sole": ("Sites left_sole and right_sole are of type box, on left_ankle_roll_link and right_ankle_roll_link "
                   "or on a body fixed below that link (no joint in between)."),
    "mass.total": "The total mass is at least 20 kg and at most 60 kg.",
    "mass.link": ("Each of the 26 link bodies has a mass above 0 kg. A body added below a link may have no mass of "
                  "its own."),
    "numbers.finite": ("Every compiled number the grader reads is finite: each body's mass, centre of mass, inertia "
                       "frame and principal inertias, and the position, orientation and size of the sites imu, "
                       "left_sole and right_sole."),
    "yaml.encoding": "calibration.yaml is UTF-8 text.",
    "yaml.properties": ("calibration.yaml uses no &-label, no *-reference and no !-tag (YAML's node labels, "
                        "references to them and type tags)."),
    "yaml.parse": "calibration.yaml is one YAML document that yaml.safe_load reads.",
    "cal.top": "The document is a mapping of exactly three keys: schema, home_table, ankle_map.",
    "cal.schema": "schema is the text unit-calibration/1.",
    "cal.home_table": "home_table is a list of exactly 25 rows, row i for firmware slot i.",
    "cal.row": ("Row i is a mapping of exactly slot, actuator, sign, home_rad: slot is the integer i, actuator slot "
                "i's name, sign the integer 1 or -1, home_rad a finite number with |home_rad| <= 6.3."),
    "cal.ankle_map": ("ankle_map is a mapping of exactly left and right; each foot a mapping of exactly axes, extent, "
                      "nodes, pitch and roll."),
    "cal.foot": ("Per foot: axes is [A, B]; extent is [[-0.787, 0.787], [-0.787, 0.787]]; nodes is 19; pitch and "
                 "roll are each 19 rows of 19 finite numbers with |value| <= 1.6 (rad)."),
    "gate.internal": "The gate ran to its end (an unexpected error inside it is reported under this code).",
}


class GateSetupError(RuntimeError):
    """The gate cannot run: a given mesh is missing or unreadable. A fault of the setup, never of the submission."""


class Failure(tuple):
    """(code, where, message)"""
    __slots__ = ()

    def __new__(cls, code: str, where: str, message: str):
        if code not in RULES:
            raise ValueError(f"unknown rule code {code!r}")
        return tuple.__new__(cls, (code, where, message))

    code = property(lambda self: self[0])
    where = property(lambda self: self[1])
    message = property(lambda self: self[2])

    def __repr__(self):
        return f"Failure({self[0]!r}, {self[1]!r}, {self[2]!r})"


class GateResult:
    def __init__(self):
        self.failures = []
        self.model = None
        self.calib = None
        self.calibration = None
        self.info = {"gate_version": GATE_VERSION}

    @property
    def ok(self) -> bool:
        return not self.failures

    @property
    def reasons(self) -> list:
        return [(f.code, f"{f.where}: {f.message}" if f.where else f.message) for f in self.failures]

    def codes(self) -> list:
        return sorted({f.code for f in self.failures})

    def fail(self, code: str, where: str, message: str) -> None:
        self.failures.append(Failure(code, where, message))


# ================================================================================================ the entry point
def check(submission_dir, given, late_entries=None) -> GateResult:
    """Every rule of the contract on one submission folder. Raises GateSetupError only for the given meshes."""
    res = GateResult()
    try:
        _check(res, os.fspath(submission_dir), _mesh_reader(given), late_entries)
    except GateSetupError:
        raise
    except Exception as exc:  # noqa: BLE001 - whatever a file holds, the gate answers with a verdict
        res.model = res.calib = res.calibration = None
        res.fail("gate.internal", "", f"{type(exc).__name__}: {str(exc)[:300]}")
    return res


def _check(res: GateResult, sub: str, read_mesh, late_entries) -> None:
    raw_xml, raw_yaml = _read_files(res, sub)
    if late_entries:
        names = [e[0] if isinstance(e, (tuple, list)) else str(e) for e in late_entries]
        res.fail("files.late_write", "", f"{len(names)} entr{'y' if len(names) == 1 else 'ies'} written after the "
                 f"session ended: " + ", ".join(names[:5]) + (" ..." if len(names) > 5 else ""))
    root = lines = None
    if raw_xml is not None:
        root, lines = _xml_static(res, raw_xml)
    if raw_yaml is not None:
        _yaml_part(res, raw_yaml)
    if root is None:
        return
    text, assets = _rewrite_for_compile(root, read_mesh)
    model = _compile(res, text, assets)
    if model is None:
        return
    res.model = model
    _check_model(res, model)


# ================================================================================================ the files
def _read_files(res: GateResult, sub: str):
    try:
        st = os.lstat(sub)
    except OSError:
        res.fail("files.folder", sub, "the submission folder does not exist")
        return None, None
    if _stat.S_ISLNK(st.st_mode) or not _stat.S_ISDIR(st.st_mode):
        res.fail("files.folder", sub, "the submission folder is not a folder (or is a symbolic link)")
        return None, None
    out = []
    for name, cap in ((UNIT_XML, MAX_XML_BYTES), (CALIBRATION_YAML, MAX_YAML_BYTES)):
        p = os.path.join(sub, name)
        try:
            st = os.lstat(p)
        except OSError:
            res.fail("files.missing", name, "missing from the submission folder")
            out.append(None)
            continue
        if not _stat.S_ISREG(st.st_mode):
            kind = "a symbolic link" if _stat.S_ISLNK(st.st_mode) else (
                "a folder" if _stat.S_ISDIR(st.st_mode) else "not a regular file")
            res.fail("files.missing", name, f"is {kind}, not a regular file")
            out.append(None)
            continue
        if st.st_size > cap:
            res.fail("files.size", name, f"{st.st_size} bytes, more than {cap}")
            out.append(None)
            continue
        with open(p, "rb") as f:
            data = f.read(cap + 1)
        if len(data) > cap:
            res.fail("files.size", name, f"more than {cap} bytes")
            out.append(None)
            continue
        res.info[name.replace(".", "_") + "_bytes"] = len(data)
        out.append(data)
    return out[0], out[1]


def _mesh_reader(given):
    """a function: mesh file name -> bytes, from a folder, a mapping or an object with `assets`"""
    if given is None:
        raise GateSetupError("no source of the given meshes")
    if isinstance(given, (str, bytes, os.PathLike)):
        folder = os.fsdecode(given)
        if not os.path.isdir(folder):
            raise GateSetupError(f"the folder of the given meshes does not exist: {folder}")

        def read(name):
            p = os.path.join(folder, name)
            try:
                with open(p, "rb") as f:
                    return f.read()
            except OSError as exc:
                raise GateSetupError(f"the given mesh {name} cannot be read: {exc}") from None
        return read
    assets = getattr(given, "assets", given)
    if not isinstance(assets, Mapping):
        raise GateSetupError("the given meshes must be a folder, a mapping of file name to bytes, or an object with "
                             "an assets mapping")

    def read_map(name):
        if name not in assets:
            raise GateSetupError(f"the given mesh {name} is not in the mapping")
        return bytes(assets[name])
    return read_map


# ================================================================================================ unit.xml, static
def _parse_xml(raw: bytes):
    """(root element, {id(element): line}) by expat; DOCTYPE and ENTITY declarations raise ValueError"""
    p = _expat.ParserCreate()
    stack, lines, keep = [], {}, []
    top = []

    def start(tag, attrs):
        el = ET.Element(tag, attrs)
        keep.append(el)
        lines[id(el)] = p.CurrentLineNumber
        if stack:
            stack[-1].append(el)
        else:
            top.append(el)
        stack.append(el)

    def end(tag):
        stack.pop()

    def refuse(*args):
        raise ValueError("a DOCTYPE or ENTITY declaration")

    p.StartElementHandler = start
    p.EndElementHandler = end
    p.StartDoctypeDeclHandler = refuse
    p.EntityDeclHandler = refuse
    p.Parse(raw, True)
    if len(top) != 1:
        raise _expat.ExpatError("no root element")
    return top[0], lines


def _where(el, lines) -> str:
    nm = el.attrib.get("name")
    line = lines.get(id(el))
    return f"<{el.tag}{' name=' + repr(nm) if nm else ''}>" + (f" line {line}" if line else "")


def _xml_static(res: GateResult, raw: bytes):
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        res.fail("xml.declaration", UNIT_XML, f"not UTF-8 text (byte {exc.start})")
        return None, None
    if "<!DOCTYPE" in text or "<!ENTITY" in text:
        res.fail("xml.declaration", UNIT_XML, "holds a DOCTYPE or ENTITY declaration")
        return None, None
    try:
        root, lines = _parse_xml(raw)
    except ValueError as exc:
        res.fail("xml.declaration", UNIT_XML, f"holds {exc}")
        return None, None
    except _expat.ExpatError as exc:
        res.fail("xml.parse", UNIT_XML, f"not well-formed XML: {exc}")
        return None, None
    if root.tag != "mujoco":
        res.fail("xml.parse", UNIT_XML, f"the root element is <{root.tag}>, not <mujoco>")
        return None, None
    n_before = len(res.failures)
    counts = {"body": 0, "geom": 0, "site": 0}
    radian = []
    stack = [(root, None, False)]                  # depth first, document order (children pushed in reverse)
    while stack:
        el, parent_tag, in_default = stack.pop()
        kids = _visit(res, el, parent_tag, lines, counts, radian, in_default)
        stack.extend((c, el.tag, in_default or el.tag == "default") for c in reversed(kids))
    if not radian:
        res.fail("xml.compiler", UNIT_XML, 'no <compiler angle="radian">: the angle unit must be stated as radian')
    res.info["xml_counts"] = dict(counts)
    for tag, cap in (("body", MAX_BODIES), ("geom", MAX_GEOMS), ("site", MAX_SITES)):
        if counts[tag] > cap:
            res.fail("xml.caps", UNIT_XML, f"{counts[tag]} <{tag}> elements, more than {cap}")
    if len(res.failures) > n_before:
        return None, None
    return root, lines


def _visit(res, el, parent_tag, lines, counts, radian, in_default) -> list:
    """the rules of one element; returns the children to visit"""
    tag = el.tag
    where = _where(el, lines)
    if tag in FORBIDDEN:
        res.fail("xml.forbidden", where, f"<{tag}> is forbidden")
        return []
    allowed = CHILDREN.get(parent_tag, frozenset())
    if parent_tag is not None and allowed is not ANY and tag not in allowed:
        res.fail("xml.unlisted", where, f"<{tag}> is not allowed inside <{parent_tag}>")
        return []
    if parent_tag is None and tag != "mujoco":
        res.fail("xml.unlisted", where, f"<{tag}> is not allowed at the top")
        return []
    a = el.attrib
    if not in_default and tag in counts:
        counts[tag] += 1
    if tag == "compiler":
        for k, v in a.items():
            if k in REFUSED_ATTRS["compiler"]:
                continue
            if k not in COMPILER_ATTRS:
                res.fail("xml.compiler", where, f"the compiler attribute {k} is not allowed")
        ang = a.get("angle")
        if ang is not None:
            if ang.strip() != COMPILER_ANGLE:
                res.fail("xml.compiler", where, f'angle="{ang}": the angle unit must be radian')
            else:
                radian.append(where)
    for k in REFUSED_ATTRS.get(tag, ()):
        if k in a and a[k].strip().lower() not in _OFF_VALUES:
            res.fail("xml.refused", where, f'{k}="{a[k]}" is refused')
    if tag == "mesh" and parent_tag == "asset":
        bad = [k for k in MESH_DATA_ATTRS if k in a]
        if bad:
            res.fail("xml.mesh", where, "a mesh given by " + ", ".join(bad) + " (only the given mesh files)")
        f = a.get("file")
        if f is None:
            if not bad:
                res.fail("xml.mesh", where, "a mesh without a file attribute (only the given mesh files)")
        elif _file_name(f) not in MESH_FILES:
            res.fail("xml.mesh", where, f"file {f!r}: {_file_name(f)!r} is not one of the given model's mesh files")
    if tag == "texture" and parent_tag == "asset":
        files = [k for k in TEXTURE_FILE_ATTRS if k in a]
        if files:
            res.fail("xml.texture", where, "a texture from a file (" + ", ".join(files) + "): built in only")
        elif a.get("builtin", "none").strip() not in TEXTURE_BUILTINS:
            res.fail("xml.texture", where, f"builtin={a.get('builtin')!r}: one of gradient, checker, flat")
    if parent_tag == "sensor":
        for c in el:
            res.fail("xml.unlisted", _where(c, lines), f"<{c.tag}> is not allowed inside the sensor <{tag}>")
        return []
    return list(el)


def _file_name(f: str) -> str:
    return f.replace("\\", "/").rstrip("/").split("/")[-1]


def _rewrite_for_compile(root, read_mesh):
    """the XML text MuJoCo compiles (compiler folders dropped, meshes by file name) and its mesh bytes"""
    root = _copy(root)
    for c in root.iter("compiler"):
        for k in COMPILER_IGNORED:
            c.attrib.pop(k, None)
    assets = {}
    for asset in root.findall("asset"):
        for me in asset.findall("mesh"):
            name = _file_name(me.attrib["file"])
            me.set("file", name)
            if name not in assets:
                assets[name] = read_mesh(name)
    return ET.tostring(root, encoding="unicode"), assets


def _copy(el):
    top = ET.Element(el.tag, dict(el.attrib))
    stack = [(el, top)]
    while stack:
        src, dst = stack.pop()
        for c in src:
            new = ET.SubElement(dst, c.tag, dict(c.attrib))
            stack.append((c, new))
    return top


# ================================================================================================ the compile
def _compile(res: GateResult, text: str, assets: dict):
    import mujoco
    res.info["mujoco_version"] = mujoco.__version__
    warnings, error, model = [], None, None
    previous = mujoco.get_mju_user_warning()
    saved = os.dup(2)
    tmp = tempfile.TemporaryFile(mode="w+b")
    os.dup2(tmp.fileno(), 2)
    try:
        mujoco.set_mju_user_warning(lambda msg: warnings.append(str(msg)))
        try:
            model = mujoco.MjModel.from_xml_string(text, assets)
        except Exception as exc:  # noqa: BLE001 - every compile error is a failed rule
            error = str(exc)
        finally:
            mujoco.set_mju_user_warning(previous)
    finally:
        os.dup2(saved, 2)
        os.close(saved)
        tmp.seek(0)
        captured = tmp.read().decode("utf-8", "replace")
        tmp.close()
    if "WARNING" in captured.upper():
        warnings.append(captured.strip()[:400])
    if error is not None:
        res.fail("model.compile", UNIT_XML, error.strip().replace("\n", " | ")[:600])
        return None
    if warnings:
        res.fail("model.warning", UNIT_XML, " | ".join(w.strip()[:200] for w in warnings[:5]))
        return None
    return model


# ================================================================================================ the compiled model
def _angle_quat(q1, q2) -> float:
    """rotation angle (rad) between two orientations given as unit quaternions (w, x, y, z)"""
    a = np.asarray(q1, float)
    b = np.asarray(q2, float)
    w = a[0] * b[0] + a[1:] @ b[1:]
    v = a[0] * b[1:] - b[0] * a[1:] - np.cross(a[1:], b[1:])
    return 2.0 * math.atan2(float(np.linalg.norm(v)), abs(float(w)))


def _angle_vec(u, v) -> float:
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    return math.atan2(float(np.linalg.norm(np.cross(u, v))), float(u @ v))


def _link_of(m, b: int, names: list):
    """(nearest link body at or above body b, or None for the world; True when every body on the way is jointless)"""
    jointless = True
    while b > 0 and names[b] not in LINK_NAMES:
        if int(m.body_jntnum[b]) != 0:
            jointless = False
        b = int(m.body_parentid[b])
    return (names[b] if b > 0 else None), jointless


def _check_model(res: GateResult, m) -> None:
    import mujoco
    B, J, S = mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_SITE
    names = [mujoco.mj_id2name(m, B, i) or "" for i in range(m.nbody)]
    jnames = [mujoco.mj_id2name(m, J, j) or "" for j in range(m.njnt)]
    info = res.info
    info.update({"bodies": int(m.nbody) - 1, "geoms": int(m.ngeom), "sites": int(m.nsite), "joints": int(m.njnt)})
    for n, cap, what in ((m.nbody - 1, MAX_BODIES, "bodies besides the world"), (m.ngeom, MAX_GEOMS, "geoms"),
                         (m.nsite, MAX_SITES, "sites")):
        if n > cap:
            res.fail("xml.caps", UNIT_XML, f"the compiled model has {n} {what}, more than {cap}")

    # finite numbers the grader reads
    site_ids = {s: int(mujoco.mj_name2id(m, S, s)) for s in (IMU_SITE,) + tuple(x[0] for x in SOLE_SITES)}
    arrays = [("body mass", m.body_mass), ("body centre of mass", m.body_ipos), ("body inertia frame", m.body_iquat),
              ("body inertia", m.body_inertia), ("body position", m.body_pos), ("body orientation", m.body_quat),
              ("joint position", m.jnt_pos), ("joint axis", m.jnt_axis), ("joint ref", m.qpos0)]
    for s, k in site_ids.items():
        if k >= 0:
            arrays += [(f"site {s} position", m.site_pos[k]), (f"site {s} orientation", m.site_quat[k]),
                       (f"site {s} size", m.site_size[k])]
    bad = [nm for nm, arr in arrays if not np.all(np.isfinite(np.asarray(arr, float)))]
    if bad:
        res.fail("numbers.finite", UNIT_XML, "not finite: " + ", ".join(bad))
        return

    # the fixed tree: bodies
    present = {}
    for name, parent, pos, quat in LINK_BODIES:
        b = int(mujoco.mj_name2id(m, B, name))
        if b < 0:
            res.fail("tree.bodies", f"body {name}", "missing")
            continue
        present[name] = b
        p = int(m.body_parentid[b])
        pname = "world" if p == 0 else names[p]
        if pname != parent:
            res.fail("tree.bodies", f"body {name}", f"its parent is {pname}, the given model's is {parent}")
            continue
        dp = float(np.linalg.norm(np.asarray(m.body_pos[b], float) - np.asarray(pos)))
        dq = _angle_quat(m.body_quat[b], quat)
        if not dp <= TOL_M:
            res.fail("tree.frames", f"body {name}", f"its position differs from the given model's by {dp:.3g} m")
        if not dq <= TOL_RAD:
            res.fail("tree.frames", f"body {name}", f"its orientation differs from the given model's by {dq:.3g} rad")

    # joints
    free = [j for j in range(m.njnt) if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)]
    if len(free) != 1:
        res.fail("tree.joints", "free joint", f"{len(free)} free joints; exactly one, on {FREE_JOINT_BODY}")
    elif names[int(m.jnt_bodyid[free[0]])] != FREE_JOINT_BODY:
        res.fail("tree.joints", "free joint", f"on {names[int(m.jnt_bodyid[free[0]])]}, not on {FREE_JOINT_BODY}")
    for j in range(m.njnt):
        if j not in free and jnames[j] not in HINGE_NAMES:
            res.fail("tree.joints", f"joint {jnames[j] or '#' + str(j)}",
                     f"not a joint of the given model (on body {names[int(m.jnt_bodyid[j])]})")
    if m.njnt != 26 and len(free) == 1:
        res.fail("tree.joints", UNIT_XML, f"{m.njnt} joints; exactly 26 (25 hinges and the free joint)")
    for name, body, pos, axis, ref in HINGES:
        j = int(mujoco.mj_name2id(m, J, name))
        if j < 0:
            res.fail("tree.joints", f"joint {name}", "missing")
            continue
        if int(m.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            res.fail("tree.joints", f"joint {name}", "not a hinge")
            continue
        jb = names[int(m.jnt_bodyid[j])]
        if jb != body:
            res.fail("tree.joints", f"joint {name}", f"on body {jb}, the given model's is {body}")
            continue
        da = _angle_vec(m.jnt_axis[j], axis)
        dp = float(np.linalg.norm(np.asarray(m.jnt_pos[j], float) - np.asarray(pos)))
        dr = abs(float(m.qpos0[int(m.jnt_qposadr[j])]) - ref)
        if not da <= TOL_RAD:
            res.fail("tree.hinges", f"joint {name}", f"its axis differs from the given model's by {da:.3g} rad")
        if not dp <= TOL_M:
            res.fail("tree.hinges", f"joint {name}", f"its position differs from the given model's by {dp:.3g} m")
        if not dr <= TOL_RAD:
            res.fail("tree.hinges", f"joint {name}", f"its ref differs from the given model's by {dr:.3g} rad")

    # sites
    k = site_ids[IMU_SITE]
    if k < 0:
        res.fail("sites.imu", f"site {IMU_SITE}", "missing")
    else:
        link, jointless = _link_of(m, int(m.site_bodyid[k]), names)
        if link is None or not jointless:
            res.fail("sites.imu", f"site {IMU_SITE}", f"on body {names[int(m.site_bodyid[k])] or 'world'}, which is "
                     "not part of the unit")
        else:
            info["imu"] = {"body": names[int(m.site_bodyid[k])], "link": link}
    info["soles"] = {}
    for site, link_name in SOLE_SITES:
        k = site_ids[site]
        if k < 0:
            res.fail("sites.sole", f"site {site}", "missing")
            continue
        if int(m.site_type[k]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            res.fail("sites.sole", f"site {site}", "not of type box")
        link, jointless = _link_of(m, int(m.site_bodyid[k]), names)
        if link != link_name or not jointless:
            res.fail("sites.sole", f"site {site}", f"on body {names[int(m.site_bodyid[k])] or 'world'}: not on "
                     f"{link_name} or a body fixed below it")
        else:
            info["soles"][site] = {"body": names[int(m.site_bodyid[k])], "link": link,
                                   "half_size_m": [float(x) for x in m.site_size[k]]}

    # masses
    total = float(np.sum(m.body_mass))
    info["total_mass_kg"] = total
    if not MASS_RANGE_KG[0] <= total <= MASS_RANGE_KG[1]:
        res.fail("mass.total", UNIT_XML, f"{total:.3f} kg, outside {MASS_RANGE_KG[0]:g} to {MASS_RANGE_KG[1]:g} kg")
    for name, b in present.items():
        if not float(m.body_mass[b]) > 0.0:
            res.fail("mass.link", f"body {name}", f"mass {float(m.body_mass[b]):.3g} kg")
    extra = [names[b] for b in range(1, m.nbody) if names[b] not in LINK_NAMES]
    info["added_bodies"] = len(extra)


# ================================================================================================ calibration.yaml
def check_calibration(raw: bytes) -> GateResult:
    """calibration.yaml's rules alone, on the file's bytes: the code check() runs on that file"""
    res = GateResult()
    if len(raw) > MAX_YAML_BYTES:
        res.fail("files.size", CALIBRATION_YAML, f"{len(raw)} bytes, more than {MAX_YAML_BYTES}")
    else:
        _yaml_part(res, bytes(raw))
    return res


def _yaml_part(res: GateResult, raw: bytes) -> None:
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        res.fail("yaml.encoding", CALIBRATION_YAML, f"not UTF-8 text (byte {exc.start})")
        return
    props = []
    try:
        for tok in yaml.scan(text, Loader=yaml.SafeLoader):
            if isinstance(tok, yaml.ScalarToken):
                continue
            mk = tok.start_mark
            ch = mk.buffer[mk.pointer] if mk.buffer is not None else text[mk.index:mk.index + 1]
            if ch in "&*!":
                props.append(f"{ch} at line {mk.line + 1} column {mk.column + 1}")
    except yaml.YAMLError as exc:
        res.fail("yaml.parse", CALIBRATION_YAML, "not YAML: " + " ".join(str(exc).split())[:300])
        return
    except RecursionError:
        res.fail("yaml.parse", CALIBRATION_YAML, "nested too deeply to read")
        return
    if props:
        res.fail("yaml.properties", CALIBRATION_YAML, f"{len(props)} node propert{'y' if len(props) == 1 else 'ies'}"
                 ": " + "; ".join(props[:5]) + (" ..." if len(props) > 5 else ""))
        return
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        res.fail("yaml.parse", CALIBRATION_YAML, "yaml.safe_load: " + " ".join(str(exc).split())[:300])
        return
    except RecursionError:
        res.fail("yaml.parse", CALIBRATION_YAML, "nested too deeply to read")
        return
    n_before = len(res.failures)
    parsed = validate_calibration(doc, res)
    if len(res.failures) == n_before:
        res.calib = doc
        res.calibration = parsed


def _is_int(v) -> bool:
    """an integer in the schema's sense: an int, or a float with no fractional part (never a boolean)"""
    if isinstance(v, bool):
        return False
    return isinstance(v, int) or (isinstance(v, float) and math.isfinite(v) and v.is_integer())


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _num_hint(v) -> str:
    if isinstance(v, str):
        try:
            float(v)
            return " (YAML read it as text: write a number with a dot and a signed exponent, e.g. 1.0e-05)"
        except ValueError:
            return ""
    return ""


def _desc(v) -> str:
    s = repr(v)
    return s if len(s) <= 60 else s[:57] + "..."


def _keys(res, obj, where, want, code) -> bool:
    if not isinstance(obj, dict):
        res.fail(code, where, f"must be a mapping of {', '.join(want)}, not {type(obj).__name__}")
        return False
    ok = True
    unknown = [k for k in obj if k not in want]
    missing = [k for k in want if k not in obj]
    if unknown:
        res.fail(code, where, "unknown key" + ("s " if len(unknown) > 1 else " ") + ", ".join(map(_desc, unknown)))
        ok = False
    if missing:
        res.fail(code, where, "missing key" + ("s " if len(missing) > 1 else " ") + ", ".join(missing))
        ok = False
    return ok


def _const_equal(a, b) -> bool:
    """equality as the schema's const keyword has it: numbers by value, booleans never equal to numbers"""
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_const_equal(x, y) for x, y in zip(a, b))
    if isinstance(a, str) or isinstance(b, str):
        return isinstance(a, str) and isinstance(b, str) and a == b
    if _is_num(a) and _is_num(b):
        return a == b
    return False


def validate_calibration(doc, res: GateResult | None = None):
    """the schema unit-calibration/1 on a loaded document; returns the parsed arrays, or None when a rule failed"""
    res = res if res is not None else GateResult()
    n0 = len(res.failures)
    if not _keys(res, doc, "", ("schema", "home_table", "ankle_map"), "cal.top"):
        if not isinstance(doc, dict):
            return None
    if isinstance(doc, dict) and "schema" in doc and not _const_equal(doc["schema"], SCHEMA_NAME):
        res.fail("cal.schema", "schema", f"{_desc(doc['schema'])}, not {SCHEMA_NAME}")
    sign = np.zeros(len(SLOTS))
    home = np.zeros(len(SLOTS))
    ht = doc.get("home_table") if isinstance(doc, dict) else None
    if isinstance(doc, dict) and "home_table" in doc:
        if not isinstance(ht, list):
            res.fail("cal.home_table", "home_table", f"must be a list of 25 rows, not {type(ht).__name__}")
        else:
            if len(ht) != len(SLOTS):
                res.fail("cal.home_table", "home_table", f"{len(ht)} rows; exactly 25, one per firmware slot")
            for i, row in enumerate(ht[:len(SLOTS)]):
                w = f"home_table[{i}]"
                if not _keys(res, row, w, ("slot", "actuator", "sign", "home_rad"), "cal.row"):
                    if not isinstance(row, dict):
                        continue
                if "slot" in row and not (_is_int(row["slot"]) and row["slot"] == i):
                    res.fail("cal.row", w + ".slot", f"{_desc(row['slot'])}: must be the integer {i}")
                if "actuator" in row and not (isinstance(row["actuator"], str) and row["actuator"] == SLOTS[i]):
                    res.fail("cal.row", w + ".actuator", f"{_desc(row['actuator'])}: row {i} is {SLOTS[i]}")
                if "sign" in row:
                    s = row["sign"]
                    if _is_int(s) and s in (1, -1):
                        sign[i] = float(s)
                    else:
                        res.fail("cal.row", w + ".sign", f"{_desc(s)}: must be the integer 1 or -1")
                if "home_rad" in row:
                    h = row["home_rad"]
                    if not _is_num(h):
                        res.fail("cal.row", w + ".home_rad", f"{_desc(h)}: must be a number{_num_hint(h)}")
                    elif not (isinstance(h, int) or math.isfinite(h)):
                        res.fail("cal.row", w + ".home_rad", f"{_desc(h)}: must be finite")
                    elif not -HOME_BOUND_RAD <= h <= HOME_BOUND_RAD:
                        res.fail("cal.row", w + ".home_rad", f"{_desc(h)}: |home_rad| must be at most 6.3 (rad)")
                    else:
                        home[i] = float(h)
    maps = {}
    am = doc.get("ankle_map") if isinstance(doc, dict) else None
    if isinstance(doc, dict) and "ankle_map" in doc:
        if _keys(res, am, "ankle_map", SIDES, "cal.ankle_map") or isinstance(am, dict):
            for side in SIDES:
                if side not in am:
                    continue
                maps[side] = _validate_foot(res, am[side], f"ankle_map.{side}")
    if len(res.failures) > n0:
        return None
    return {"sign": sign, "home": home, "ankle_map": maps}


def _validate_foot(res, foot, w):
    if not _keys(res, foot, w, ("axes", "extent", "nodes", "pitch", "roll"), "cal.ankle_map"):
        if not isinstance(foot, dict):
            return None
    if "axes" in foot and not _const_equal(foot["axes"], list(AXES)):
        res.fail("cal.foot", w + ".axes", f"{_desc(foot['axes'])}: must be [A, B]")
    ext = [[-EXTENT_RAD, EXTENT_RAD], [-EXTENT_RAD, EXTENT_RAD]]
    if "extent" in foot and not _const_equal(foot["extent"], ext):
        res.fail("cal.foot", w + ".extent", f"{_desc(foot['extent'])}: must be [[-0.787, 0.787], [-0.787, 0.787]]")
    if "nodes" in foot and not _const_equal(foot["nodes"], NODES):
        res.fail("cal.foot", w + ".nodes", f"{_desc(foot['nodes'])}: must be 19")
    out = {}
    for key in ("pitch", "roll"):
        if key not in foot:
            continue
        g = foot[key]
        gw = f"{w}.{key}"
        if not isinstance(g, list) or len(g) != NODES:
            res.fail("cal.foot", gw, "must be a list of 19 rows" + (f", not {len(g)}" if isinstance(g, list) else
                                                                     f", not {type(g).__name__}"))
            continue
        arr = np.zeros((NODES, NODES))
        bad, shape_bad = [], False
        for i, r in enumerate(g):
            if not isinstance(r, list) or len(r) != NODES:
                res.fail("cal.foot", f"{gw}[{i}]", "must be a list of 19 numbers" + (
                    f", not {len(r)}" if isinstance(r, list) else f", not {type(r).__name__}"))
                shape_bad = True
                continue
            for j, v in enumerate(r):
                if not _is_num(v) or not (isinstance(v, int) or math.isfinite(v)) or \
                        not -NODE_BOUND_RAD <= v <= NODE_BOUND_RAD:
                    bad.append((i, j, v))
                else:
                    arr[i, j] = float(v)
        if bad:
            i, j, v = bad[0]
            why = "must be a finite number" + _num_hint(v) if not _is_num(v) or not (
                isinstance(v, int) or math.isfinite(v)) else "|value| must be at most 1.6 (rad)"
            res.fail("cal.foot", gw, f"{len(bad)} of 361 values are not finite numbers within 1.6 rad; the first, "
                     f"[{i}][{j}] = {_desc(v)}: {why}")
        if not bad and not shape_bad:
            out[key] = arr
    return out


# ================================================================================================ helpers
def ankle_map_value(pitch, roll, a: float, b: float):
    """the foot's (pitch, roll) at motor coordinates (A, B) from one foot's 19 x 19 nodes, as the contract reads the
    map: bilinear inside a cell; outside the extent the bilinear formula of the nearest edge cell, extended"""
    h = NODE_STEP_RAD
    i = min(max(int(math.floor((a + EXTENT_RAD) / h)), 0), NODES - 2)
    j = min(max(int(math.floor((b + EXTENT_RAD) / h)), 0), NODES - 2)
    s = (a - NODE_VALUES_RAD[i]) / h
    t = (b - NODE_VALUES_RAD[j]) / h
    out = []
    for g in (pitch, roll):
        g = np.asarray(g, float)
        out.append(float((1 - s) * (1 - t) * g[i, j] + s * (1 - t) * g[i + 1, j] + (1 - s) * t * g[i, j + 1]
                         + s * t * g[i + 1, j + 1]))
    return out[0], out[1]


def rule_lines() -> list:
    """the rules as the acceptance sheet prints them: 'code: sentence'"""
    return [f"{code}: {text}" for code, text in RULES.items()]


def whitelist_lines() -> list:
    """the element whitelist of unit.xml as the acceptance sheet prints it"""
    places = []
    for parent, kids in CHILDREN.items():
        if parent is None:
            continue
        places.append(f"  inside <{parent}>: " + ("any sensor element (none inside it)" if kids is ANY else
                                                 ", ".join(f"<{k}>" for k in sorted(kids))))
    return ([
        "Allowed and not read by the grader: " + ", ".join(IGNORED_ELEMENTS) + ".",
        "Allowed and read: " + ", ".join(READ_ELEMENTS) + ".",
        "Forbidden anywhere: " + ", ".join(sorted(FORBIDDEN)) + "; and every element not listed below.",
        "Where each element may stand (any other place is refused):",
    ] + places + [
        'Compiler: angle="radian" must be stated; the attributes allowed: ' + ", ".join(sorted(COMPILER_ATTRS)) +
        "; meshdir, assetdir and texturedir are allowed and ignored (the grader compiles the given meshes by file "
        "name); fusestatic, discardvisual and alignfree are refused.",
        "Refused attributes (unless false or 0): " + "; ".join(f"<{t}> " + ", ".join(v) for t, v in
                                                               REFUSED_ATTRS.items()) + ".",
        "Meshes: a file attribute naming one of the given model's 26 mesh files (any folder: the file name decides); "
        "no inline data (" + ", ".join(MESH_DATA_ATTRS[:4]) + ") and no built-in mesh (builtin, params).",
        "Textures: built in only (" + ", ".join(TEXTURE_BUILTINS) + "); no file attribute.",
        "Every other attribute of an allowed element is free (no band on friction, contact or solver parameters: "
        "nothing is simulated).",
        f"Caps: {MAX_BODIES} bodies besides the world, {MAX_GEOMS} geoms, {MAX_SITES} sites; {MAX_XML_BYTES} bytes.",
    ])
