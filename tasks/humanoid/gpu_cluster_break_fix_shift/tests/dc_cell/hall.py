"""The data hall's survey (cell internals): every dimension of the pod, its racks, the GPU nodes, the leaf switches,
the containment, the overhead services and the service area, in one place. The cell builds its model from these
numbers and the site documents (the DCIM floor plan and rack elevations given to the agent) are printed from them.

Frames: the hall's frame has x east, y north, z up (metres, the floor at z = 0). Each rack has its own frame: origin at
the middle of its front edge on the floor, x to the right of a person facing its front, y into the rack (front to
rear), z up. Row A's fronts face south and row B's north, both onto the contained cold aisle between them (y -0.6 ..
0.6); the hot aisles lie behind the rows. Racks are numbered 01..05 from west to east in both rows; 01 is the row's
network rack, 02..05 hold one GPU node each.
"""

from __future__ import annotations

import math

U = 0.04445                     # one rack unit
RACK_W, RACK_D, RACK_H = 0.600, 1.200, 2.265
RACK_UNITS = 48
U1_Z = 0.100                    # the bottom of U1 above the floor (plinth and levelling feet)
FRONT_RAIL_Y = 0.100            # front mounting rails behind the rack's front edge
REAR_RAIL_Y = 1.090
EIA_W = 0.4826                  # 19-inch panel width (ears included)
INNER_W = 0.450                 # between the rails

COLD_AISLE_HALF = 0.600         # the cold aisle: y -0.6 .. 0.6
HOT_AISLE_W = 1.200
X0 = 0.0                        # the pod's west end (rack 01's west side)
N_RACKS = 5
ROWS = ("A", "B")


def u_z(u):
    """Height of the bottom of rack unit `u` (1-based) above the floor."""
    return U1_Z + (u - 1) * U


def rack_frame(row, idx):
    """(origin (x, y, z), yaw) of a rack's frame in the hall: idx 1..5 from west to east."""
    xc = X0 + (idx - 0.5) * RACK_W
    if row == "A":
        return (xc, COLD_AISLE_HALF, 0.0), 0.0          # front at y = 0.6, rack extends north (+y)
    return (xc, -COLD_AISLE_HALF, 0.0), math.pi           # front at y = -0.6, rack extends south


def rack_name(row, idx):
    return f"{row}{idx:02d}"


def is_network(idx):
    return idx == 1


# ------------------------------------------------------------------------------------------------ the GPU node
NODE_U = (8, 15)                # U8 .. U15 (8U)
NODE_W, NODE_D = 0.440, 0.897
NODE_Z0 = u_z(NODE_U[0])        # 0.41115
NODE_Z1 = u_z(NODE_U[1] + 1)    # 0.76675
NODE_H = NODE_Z1 - NODE_Z0
NODE_Y0 = FRONT_RAIL_Y          # the chassis front (behind the ears)
NODE_Y1 = NODE_Y0 + NODE_D      # the chassis rear
CPU_TRAY_H = 2 * U              # the lower 2U: drive bays and the control panel
GPU_TRAY_Z0 = NODE_Z0 + CPU_TRAY_H
GPU_TRAY_H = NODE_Z1 - GPU_TRAY_Z0
BEZEL = {"w": 0.404, "h": 0.258, "t": 0.028, "gap": 0.004}   # over the GPU tray's fan wall
FAN_MODULES = 5                 # dual-rotor hot-swap fan modules across the GPU tray's front
DRIVE_BAYS = (2, 4)             # rows x columns of U.2 bays in the CPU tray's front
PSUS = 6                        # 4 + 2 redundant, alternating feeds A / B, in one row at the rear bottom
PSU_W, PSU_H, PSU_D = 0.0725, 0.0405, 0.185
OSFP_PORTS = 8                  # one flat-top OSFP per GPU at the rear top (rail k <- GPU k)
TRAY_SERVICE_TRAVEL = 0.74      # how far the GPU tray slides out to its lock stop
GPU_GRID = (2, 4)               # GPU modules in the tray: rows (front, back) x columns
GPU_MODULE = {"w": 0.094, "d": 0.118, "base": 0.010, "fins_h": 0.095}

AUX_2U = (28, 32)               # two 2U storage / CPU nodes per GPU rack (out of the robot's reach; never ticketed)
MGMT_U = 40                     # the rack's 1U out-of-band management switch (front, blue patch cords)
PATCH_U = (40, 42)              # the network racks' 1U fibre patch panels (front)

# ------------------------------------------------------------------------------------------------ the network racks
LEAF_U = (16, 18, 20, 22)       # four 1U leaf switches per network rack, ports facing the hot aisle (rear)
LEAF_DEPTH = 0.62
LEAF_PORTS = 32


def leaf_name(row, k):
    """Leaf switch k (0..3) of row `row`'s network rack; rail = 4 * (row == 'B') + k."""
    return f"leaf-{row.lower()}01-{k + 1}"


def rail_of_leaf(row, k):
    return (0 if row == "A" else 4) + k


# ------------------------------------------------------------------------------------------------ containment and overhead
CONTAIN_TOP = RACK_H + 0.010
DOOR_X = (X0 - 0.03, X0 + N_RACKS * RACK_W + 0.03)
FIBER_TRAY = {"z": 2.55, "w": 0.30, "h": 0.10}
BUSWAY = {"z": 2.78, "w": 0.16, "h": 0.13}
BASKET = {"z": 2.95, "w": 0.40, "h": 0.06}
LIGHT_Z = 3.30
CEILING_Z = 3.60
LIGHT_ROWS_Y = (0.0, 2.4, -2.4)
HALL = {"x": (-4.6, 6.2), "y": (-6.4, 6.4)}
BACKGROUND_ROWS = (("C", 3.0, 1), ("D", -3.0, -1))    # (name, y of the rear face, +1 extends north / -1 south)

# ------------------------------------------------------------------------------------------------ the service area
CAGE = {"x": (-4.2, -2.2), "y": (-2.2, 2.2), "h": 2.2}
BENCH = {"x": (-1.85, -0.95), "y": (2.0, 2.75), "h": 0.90}
DOCK = (-1.2, -0.9, 0.0)
ROBOT_START = (-1.0, 0.0, 0.0)
