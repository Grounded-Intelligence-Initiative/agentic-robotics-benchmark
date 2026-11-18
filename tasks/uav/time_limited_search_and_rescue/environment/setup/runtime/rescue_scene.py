"""3D industrial rescue yard. Only the forecast paint varies with the route."""
from functools import lru_cache
import xml.etree.ElementTree as ET

from pbr_scene import prop
from scene_assets import asset_root, geom, materials, robot_meshes, vec

LANES = (-2.7, -.9, .9, 2.7)
PAD = (0., -5.25)
BOARD = (0., -3.28)
HEIGHT_BOARD = (1., -2.12)
BOUNDS = ((-3.39, 3.39), (-5.70, 8.10), (.39, 2.55))
DOWN_WIDTH = 3.5
DOWN_OFFSET = .20
GATE_Y = .53
GATE_HALF_WIDTH = .44
GATE_CEILING = 1.43
HEIGHT_GATE_Y = 2.72
HEIGHT_GATE_ZONE = .61
HEIGHT_HALF_WIDTH = .49
# Indexed by forecast dot count minus one: (x, visible opening bottom/top,
# safe UAV center range including clearance for the loaded pod).
PASSAGES = ((.92, .03, 1.12, .57, .92), (-.92, 1.52, 2.62, 1.98, 2.42))
CRATES = [(-.75, 5.13, .37, .46, .60), (.75, 6.06, .37, .46, .68)]

DARK = (.085, .106, .118, 1)
STEEL = (.33, .38, .41, 1)
WHITE = (.91, .92, .87, 1)
HOT = (.83, .075, .025, 1)
AMBER = (.96, .70, .08, 1)
COOL = (.018, .92, .55, 1)
BLUE = (.06, .30, .78, 1)
ORANGE = (1., .29, .016, 1)


@lru_cache(maxsize=1)
def template():
    root = ET.fromstring("""
    <mujoco model="civilian rescue yard">
        <compiler angle="degree" autolimits="true"/>
        <option timestep=".005" integrator="implicitfast"/>
        <visual>
          <global offwidth="256" offheight="256"/>
          <headlight ambient=".53 .53 .53" diffuse=".25 .25 .25" specular="0 0 0"/>
          <map znear=".001" zfar="6"/>
        </visual>
        <worldbody>
          <light directional="true" pos="-3 -4 7" dir=".3 .48 -1"
                 diffuse=".64 .63 .60" castshadow="false"/>
          <geom name="floor" type="plane" size="5 7 1" friction=".9 .02 .005"/>
          <body name="uav" gravcomp="1">
            <inertial pos="0 0 0" mass=".8" diaginertia=".012 .012 .015"/>
            <joint name="x" type="slide" axis="1 0 0" damping="9"/>
            <joint name="y" type="slide" axis="0 1 0" damping="9"/>
            <joint name="z" type="slide" axis="0 0 1" damping="9"/>
            <camera name="down" pos="0 .20 -.145" projection="orthographic" fovy="3.5"/>
            <camera name="forward" pos="0 .17 .045" xyaxes="1 0 0 0 .05 .99875" fovy="106"/>
          </body>
          <body name="pod" pos="0 2 .074">
            <freejoint name="pod_free"/>
            <geom name="sled_collision" type="box" size=".145 .255 .072" mass=".15"
                  rgba=".105 .115 .12 1" friction=".95 .04 .01"/>
          </body>
        </worldbody>
        <equality><weld name="clamp" body1="uav" body2="pod" active="false"
                        solref=".008 1" relpose="0 0 -.31 1 0 0 0"/></equality>
        <actuator>
          <position name="mx" joint="x" kp="190" kv="16"/>
          <position name="my" joint="y" kp="190" kv="16"/>
          <position name="mz" joint="z" kp="190" kv="16"/>
        </actuator>
    </mujoco>""")
    materials(root)
    w = root.find("worldbody")
    w.find("geom[@name='floor']").set("material", "floor_mat")
    uav = w.find("body[@name='uav']")
    robot_meshes(root, uav, "drone")
    geom(uav, "gripper_stem", "cylinder", (0, 0, -.115), (.031, .075), DARK)
    for side in (-1, 1):
        geom(uav, f"gripper_jaw{side}", pos=(side*.053, 0, -.207),
             size=(.025, .050, .025), rgba=STEEL)

    body = w.find("body[@name='pod']")
    geom(body, "pod_shell", pos=(0, 0, .056), size=(.155, .26, .050), rgba=ORANGE)
    geom(body, "mannequin_torso", "ellipsoid", (0, .027, .105), (.086, .109, .038), AMBER)
    geom(body, "mannequin_head", "sphere", (0, .155, .107), (.057,), AMBER)
    for side in (-1, 1):
        geom(body, f"mannequin_leg{side}", "capsule",
             size=(.031,), fromto=vec((side*.041, -.045, .11, side*.054, -.204, .106)),
             rgba=AMBER)
        geom(body, f"mannequin_arm{side}", "capsule",
             size=(.023,), fromto=vec((side*.064, .074, .105, side*.112, -.042, .10)),
             rgba=AMBER)
    geom(body, "grapple_crossbar", pos=(0, 0, .136), size=(.142, .025, .015), rgba=WHITE)

    for side in (-1, 1):
        geom(w, f"boundary{side}", pos=(side*3.88, 1.23, 1.34),
             size=(.13, 7.30, 1.34), material="brick_mat")
        geom(w, f"boundary_cap{side}", pos=(side*3.88, 1.23, 2.68),
             size=(.16, 7.3, .04), rgba=STEEL)
        for y in (-4.8, -2.4, 0, 2.4, 4.8, 7.2):
            geom(w, f"pier{side}_{y}", pos=(side*3.90, y, 1.02),
                 size=(.19, .13, 1.02), rgba=STEEL)
    geom(w, "south_wall", pos=(0, -6.07, .58), size=(3.92, .12, .58), material="brick_mat")
    geom(w, "rear_wall", pos=(0, 8.53, 1.46), size=(3.92, .12, 1.46), material="brick_mat")
    for i in range(6):
        geom(w, f"rear_cladding{i}", pos=(i*1.3-3.25, 8.40, 1.99),
             size=(.43, .025, .43), rgba=(.22, .35, .41, 1))
    # Licensed, locally staged Poly Haven scans form the two service cabinets.
    for i, (x, y, sx, sy, h) in enumerate(CRATES):
        prop(root, asset_root(), "utility_box_01", f"service_cabinet{i}", (x, y, 0),
             (2*sx, 2*sy, h))

    geom(w, "evac_pad", pos=(*PAD, .004), size=(.57, .54, .004), rgba=BLUE)
    for x in (-.41, .41):
        geom(w, f"pad_upright{x}", pos=(x, PAD[1], .010), size=(.028, .44, .001), rgba=WHITE)
    geom(w, "pad_crosspiece", pos=(*PAD, .010), size=(.41, .028, .001), rgba=WHITE)
    geom(w, "forecast_base", pos=(*BOARD, .047), size=(1.02, .31, .047), rgba=DARK)
    for i, x in enumerate((-0.72, -.24, .24, .72)):
        geom(w, f"forecast_tile{i}", pos=(x, BOARD[1], .106), size=(.202, .23, .011), rgba=AMBER)
        for j in range(i+1):
            geom(w, f"forecast_dot{i}_{j}", "cylinder",
                 (x+(j-i/2)*.088, BOARD[1], .120), (.026, .004), DARK)
    geom(w, "height_forecast_base", pos=(*HEIGHT_BOARD, .047), size=(.63, .32, .047), rgba=BLUE)
    for i, dx in enumerate((-.30, .30)):
        x, y = HEIGHT_BOARD
        geom(w, f"height_tile{i}", pos=(x+dx, y, .106), size=(.23, .24, .011), rgba=AMBER)
        for j in range(i+1):
            geom(w, f"height_dot{i}_{j}", "cylinder", (x+dx+(j-i/2)*.15, y, .120),
                 (.041, .004), DARK)

    # Red/yellow baffles are always hot. All four openings have the same local
    # geometry and markings; none of these geoms encode the episodic forecast.
    for i, x in enumerate((-3.6, -1.8, 0., 1.8, 3.6)):
        geom(w, f"baffle{i}", pos=(x, 0, .81), size=(.26, .37, .81), rgba=HOT)
        for j in range(5):
            geom(w, f"baffle_band{i}_{j}", pos=(x, 0, .16+j*.30),
                 size=(.265, .375, .055), rgba=AMBER)
    geom(w, "lintel", pos=(0, 0, 1.72), size=(3.85, .29, .13), rgba=DARK)
    for i, x in enumerate(LANES):
        for side in (-1, 1):
            geom(w, f"lane_plate{i}_{side}", pos=(x, side*.70, .005),
                 size=(.43, .13, .004), rgba=DARK)
            for j in range(i+1):
                geom(w, f"lane_dot{i}_{side}_{j}", "cylinder",
                     (x+(j-i/2)*.142, side*.70, .013), (.044, .005), WHITE)
        for offset in (-.62, .62):
            geom(w, f"hot_edge{i}_{offset}", pos=(x+offset, 0, .008),
                 size=(.018, .54, .008), rgba=HOT)

    # The transverse bulkhead is solid from wall to wall, with two genuinely
    # empty windows in different rows/columns. No sign at this bulkhead varies.
    def bulkhead(name, lo_x, hi_x, lo_z, hi_z, rgba=STEEL):
        geom(w, name, pos=((lo_x+hi_x)/2, HEIGHT_GATE_Y, (lo_z+hi_z)/2),
             size=((hi_x-lo_x)/2, .30, (hi_z-lo_z)/2), rgba=rgba)

    bulkhead("height_sill", -3.90, 3.90, 0., .03)
    bulkhead("height_middle", -3.90, 3.90, 1.12, 1.52, DARK)
    bulkhead("height_roof", -3.90, 3.90, 2.62, 2.93, DARK)
    for i, (x, bottom, top, _, _) in enumerate(PASSAGES):
        bulkhead(f"bulkhead{i}_left", -3.90, x-.75, bottom, top)
        bulkhead(f"bulkhead{i}_right", x+.75, 3.90, bottom, top)
        for side in (-1, 1):
            y = HEIGHT_GATE_Y+side*.308
            for dx in (-.78, .78):
                geom(w, f"window{i}_jamb{side}_{dx}", pos=(x+dx, y, (bottom+top)/2),
                     size=(.045, .015, (top-bottom)/2), rgba=AMBER)
            z = 1.32 if i == 0 else 2.77
            for j in range(i+1):
                geom(w, f"window{i}_dot{side}_{j}", "sphere",
                     (x+(j-i/2)*.22, y+side*.016, z), (.077,), WHITE)
            y = HEIGHT_GATE_Y+side*1.07
            geom(w, f"window{i}_lead{side}", pos=(x, y, .005),
                 size=(.35, .13, .004), rgba=DARK)
            for j in range(i+1):
                geom(w, f"window{i}_lead_dot{side}_{j}", "cylinder",
                     (x+(j-i/2)*.16, y, .012), (.048, .004), WHITE)
    for i, x in enumerate((-3.46, -2.78, -2.10, 2.10, 2.78, 3.46)):
        for side in (-1, 1):
            geom(w, f"bulkhead_warning{i}_{side}", pos=(x, HEIGHT_GATE_Y+side*.308, 1.32),
                 size=(.24, .016, .055), rgba=HOT, euler="0 30 0")
    # Center a landmark between the gates, and delineate the far search yard.
    geom(w, "staging_line", pos=(0, 1.14, .004), size=(1.45, .035, .003), rgba=WHITE)
    for y in (3.77, 5.69, 7.59):
        for x in (-2.4, 0., 2.4):
            geom(w, f"search_grid{x}_{y}", pos=(x, y, .002),
                 size=(.65, .016, .002), rgba=(.48, .51, .49, 1))
    return ET.tostring(root, encoding="unicode")
