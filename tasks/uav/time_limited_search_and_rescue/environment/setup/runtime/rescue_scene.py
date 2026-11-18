"""3D industrial rescue yard. The forecast board is the only route-dependent mesh."""
from functools import lru_cache
import xml.etree.ElementTree as ET

from pbr_scene import prop
from scene_assets import asset_root, geom, materials, robot_meshes, vec

LANES = (-2.7, -.9, .9, 2.7)
PAD = (0., -5.25)
BOARD = (0., -3.28)
BOUNDS = ((-3.39, 3.39), (-5.70, 5.50), (.39, 1.88))
DOWN_WIDTH = 3.5
DOWN_OFFSET = .20
GATE_Y = .53
GATE_HALF_WIDTH = .44
GATE_CEILING = 1.43
CRATES = [(-.75, 2.62, .37, .46, .60), (.75, 3.46, .37, .46, .68)]

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
            <camera name="forward" pos="0 .17 .045" xyaxes="1 0 0 0 .47 .8827" fovy="72"/>
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
        geom(w, f"boundary{side}", pos=(side*3.88, -.03, .76),
             size=(.13, 5.99, .76), material="brick_mat")
        geom(w, f"boundary_cap{side}", pos=(side*3.88, -.03, 1.54),
             size=(.16, 6.0, .04), rgba=STEEL)
        for y in (-4.8, -2.4, 0, 2.4, 4.8):
            geom(w, f"pier{side}_{y}", pos=(side*3.90, y, 1.02),
                 size=(.19, .13, 1.02), rgba=STEEL)
    for y in (-6.07, 5.84):
        geom(w, f"endwall{y}", pos=(0, y, .58), size=(3.92, .12, .58), material="brick_mat")
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
    for y in (1.31, 3.18, 4.99):
        for x in (-2.4, 0., 2.4):
            geom(w, f"search_grid{x}_{y}", pos=(x, y, .002),
                 size=(.65, .016, .002), rgba=(.48, .51, .49, 1))
    return ET.tostring(root, encoding="unicode")
