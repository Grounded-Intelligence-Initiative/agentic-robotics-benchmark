"""contract: the names and conventions of the hand model that the grader reads (standard library only).

The same file ships with the format checker and with the grader. Nothing in it depends on the unit that was
photographed.

Frames and units
  palm frame  the frame of every given placement: x across the palm, y from the wrist towards the fingertips, z from
              the palm side (negative z) towards the back of the hand. Metres, radians, kilograms. The body "palm" is a
              jointless child of the world body; every position the grader reads is expressed in the frame of that
              body.
  q = 0       all 16 hinges at zero = the pose of the given placements.
  counts      4096 per turn of a servo's output.
Digits, hinges and bodies
  The chain of each digit runs from the palm outwards (CHAINS). Each hinge sits in the body of its own name. The 13
  hinges driven by a servo are listed in the order of the servo ids 0 .. 12 (MOTOR_ORDER); the last hinge of each
  finger (index_dip, middle_dip, ring_dip) has no servo and is coupled to the hinge before it.
Pads
  The site <digit>_pad sits on the sensing surface of the digit's tactile module, rigid on the digit's last body (or on
  a jointless body below it); its +z axis (PAD_NORMAL_AXIS) is the outward normal of that surface.
Sense of a finger's last hinge
  At q = 0, turning index_dip (middle_dip, ring_dip) alone by DIP_SENSE_STEP_RAD moves the finger's pad. If that move
  points to the palm side (a positive dot product with PALM_SIDE_DIR of the finger) the model flexes that hinge with
  negative angles; otherwise its range is read in the opposite sense.
"""

CONTRACT_VERSION = "hand-description-contract/1"

DIGITS = ("thumb", "index", "middle", "ring")
FINGERS = ("index", "middle", "ring")

# the 13 servo-driven hinges, in the order of the servo ids 0 .. 12
MOTOR_ORDER = (
    "thumb_ip", "thumb_mp", "thumb_side", "thumb_roll",
    "index_pip", "index_mcp", "index_abad",
    "middle_pip", "middle_mcp", "middle_abad",
    "ring_pip", "ring_mcp", "ring_abad",
)
MOTOR_IDS = tuple(range(len(MOTOR_ORDER)))
MOTOR_ID = {j: i for i, j in enumerate(MOTOR_ORDER)}
DRIVEN = MOTOR_ORDER

# chains from the palm outwards
CHAINS = {
    "thumb": ("thumb_roll", "thumb_side", "thumb_mp", "thumb_ip"),
    "index": ("index_abad", "index_mcp", "index_pip", "index_dip"),
    "middle": ("middle_abad", "middle_mcp", "middle_pip", "middle_dip"),
    "ring": ("ring_abad", "ring_mcp", "ring_pip", "ring_dip"),
}
CONTRACT_JOINTS = tuple(j for d in DIGITS for j in CHAINS[d])
DIGIT_OF_JOINT = {j: d for d in DIGITS for j in CHAINS[d]}
DRIVEN_BY_DIGIT = {d: tuple(j for j in MOTOR_ORDER if DIGIT_OF_JOINT[j] == d) for d in DIGITS}
DIP_JOINTS = tuple(CHAINS[f][3] for f in FINGERS)
DIP_OF_FINGER = {f: CHAINS[f][3] for f in FINGERS}
PIP_OF_FINGER = {f: CHAINS[f][2] for f in FINGERS}

ROOT_BODY = "palm"
BODY_OF_JOINT = {j: j for j in CONTRACT_JOINTS}
FIRST_BODY = {d: CHAINS[d][0] for d in DIGITS}
LAST_BODY = {d: CHAINS[d][-1] for d in DIGITS}
# extra hinged bodies (a closed chain) are allowed only under these names, below the finger's first body
LOOP_BODIES = {f: tuple("%s_loop%d" % (f, k) for k in (1, 2, 3, 4)) for f in FINGERS}

PAD_SITES = {d: d + "_pad" for d in DIGITS}
PAD_NORMAL_AXIS = (0.0, 0.0, 1.0)

ACTUATORS = {"act_" + j: j for j in MOTOR_ORDER}
ACTUATOR_OF_JOINT = {j: "act_" + j for j in MOTOR_ORDER}

# the sense rule of a finger's last hinge
DIP_SENSE_STEP_RAD = -0.05
PALM_SIDE_DIR = {f: (0.0, 0.0, -1.0) for f in FINGERS}

COUNTS_PER_TURN = 4096

# the finger whose pad the pinch requests pair with the thumb pad
PINCH_FINGERS = ("index",)

# largest total number of mesh triangles in a submitted model, counted per mesh asset (room for every given part
# mesh once plus convex collision hulls, with half as much again)
TRI_CAP = 5000000
