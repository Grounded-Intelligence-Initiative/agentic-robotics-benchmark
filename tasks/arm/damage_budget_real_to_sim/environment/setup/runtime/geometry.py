"""Trusted scene construction. Do not stage this module in the agent workspace."""

EXTENT = 4.0
SIZE = 256


def header():
    return """<mujoco>
    <option timestep="0.01" gravity="0 0 -9.81"/>
    <visual><global offwidth="256" offheight="256"/><quality shadowsize="1024"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.15 0.15 0.15"/></visual>
    <default><geom friction="0.8 0.02 0.002" solref="0.02 1"/></default>
    <worldbody>
    <light pos="0 0 8" dir="0 0 -1" diffuse="0.4 0.4 0.4" castshadow="false"/>
    <camera name="top" pos="0 0 4.5" projection="orthographic" fovy="8"/>
    <camera name="side" pos="0 -12 2" xyaxes="1 0 0 0 0 1"
        projection="orthographic" fovy="8"/>
    <geom name="floor" type="plane" size="8 8 .1" rgba=".16 .18 .20 1"/>
    """


def box(name, position, size, color, collision=True):
    return (
        f'<geom name="{name}" type="box" pos="{position}" size="{size}" '
        f'rgba="{color}" contype="{int(collision)}" conaffinity="{int(collision)}"/>'
    )


def packing_stop(goal, side, dx):
    """Captive transverse shipping bolt: an unactuated slide in a bearing."""
    x, y = goal[0]+dx, goal[1]
    bearing_y = y+side*.52
    sleeve = box("fixture_bearing", f"{x} {bearing_y} .232", ".112 .064 .232", ".38 .37 .33 1")
    for i, s in enumerate((-1, 1)):
        sleeve += box(f"fixture_cheek{i}", f"{x+s*.089} {bearing_y} .545",
                      ".014 .064 .081", ".45 .44 .37 1")
    sleeve += box("fixture_keeper", f"{x} {bearing_y} .636", ".106 .064 .01", ".45 .44 .37 1")
    return sleeve, f"""
    <body name="packing_stop" pos="{x} {y} .544">
      <joint name="stop_slide" type="slide" axis="0 {side} 0" limited="true"
             range="0 .73" damping="2" frictionloss="1.2"/>
      <geom name="stop_bar" type="box" pos="0 {side*.28} 0" size=".070 .58 .076"
            mass=".32" rgba=".56 .51 .36 1"/>
      <geom name="stop_lug" type="box" pos="0 {side*.82} .157" size=".105 .054 .085"
            mass=".12" rgba=".93 .008 .86 1"/>
    </body>"""


def shipping_sleeve(block, slot_x, half_gap, front, back_side, turn, draw, travel):
    """A covered feed lane ending in a one-sided transverse release pocket."""
    x, y = block
    back = back_side*.255
    mid_y = y+back_side*(.211-front)/2
    sheet = (f'<geom name="slip_sheet" type="box" pos="{x+turn*travel/2} {y-back_side*draw/2} .001" '
             f'size="{.86+travel/2} {draw/2+.39} .005" priority="1" friction=".14 .003 .0003" '
             'rgba=".45 .52 .54 1"/>')
    # The continuous toe retains the shell. Gaps between the uprights expose
    # the end marks both at reset (+/- .57) and after the signed bay offset.
    sheet += box("source_spine", f"{x} {y+back} .034", ".82 .043 .034", ".02 .77 .85 1")
    for i, (left, right) in enumerate(((-.82,-.68),(-.46,-.30),(.30,.46),(.68,.82))):
        sheet += box(f"source_back_stile{i}", f"{x+(left+right)/2} {y+back} .186",
                     f"{(right-left)/2} .043 .118", ".02 .77 .85 1")
    for i, (left, right) in enumerate(((-.46, slot_x-half_gap), (slot_x+half_gap, .46))):
        sheet += box(f"source_hood{i}", f"{x+(left+right)/2} {mid_y} .240",
                     f"{(right-left)/2} {(.211+front)/2} .035", ".83 .52 .16 1")
        sheet += box(f"source_hood_outer{i}", f"{x+(-.75,.75)[i]} {mid_y} .240",
                     f".07 {(.211+front)/2} .035", ".83 .52 .16 1")
        sheet += box(f"source_guide{i}", f"{x+(-.772,.772)[i]} {mid_y} .103",
                     f".022 {(.211+front)/2} .103", ".47 .47 .44 1")
    # Coordinates in the fixture frame: +u turns towards the open bay and
    # +v draws away from the rear rail. Every guide face is a collision box.
    def plate(name, u0, u1, v0, v1, z0, z1, color):
        return box(name, f"{x+turn*(u0+u1)/2} {y-back_side*(v0+v1)/2} {(z0+z1)/2}",
                   f"{(u1-u0)/2} {(v1-v0)/2} {(z1-z0)/2}", color)

    steel, violet = ".18 .24 .28 1", ".54 .045 .90 1"
    far_y, far_x = draw+.17+.09, travel+.69+.09
    sheet += plate("key_spine", -.82, -.75, .15, far_y+.06, .006, .228, steel)
    sheet += plate("key_lip", -.82, travel-.69-.08, .15, far_y+.06, .195, .232, ".055 .13 .43 1")
    sheet += plate("key_entrance", .75, .81, .15, draw-.17-.09, .006, .148, steel)
    sheet += plate("key_inner_rail", .75, far_x+.06, draw-.17-.15, draw-.17-.09, .006, .065, steel)
    sheet += plate("key_front_fence", -.82, far_x+.06, far_y, far_y+.06, .006, .061, violet)
    sheet += plate("key_outer_fence", far_x, far_x+.06, draw-.17-.15, far_y, .006, .116, violet)
    return sheet


def level_cradle(goal, tare, spring, slider_mass):
    """Unactuated weighbeam and a self-holding rider with a two-sided cup pocket."""
    x, y = goal
    iron, metal, gauge = ".22 .26 .27 1", ".46 .50 .51 1", ".99 .89 .012 1"
    mounting = box("fixture_plinth", f"{x} {y} .145", ".12 .26 .145", iron)
    mounting += box("fixture_bearing_cap", f"{x} {y} .31", ".078 .29 .055", metal, False)
    # A fixed datum sits just below the moving sight flags in the front view.
    mounting += box("fixture_datum", f"{x} {y-2.36} .485", ".86 .018 .012", ".40 .45 .46 1", False)
    parts = box("tray", "0 0 .040", ".79 .245 .030", ".03 .18 .94 1")
    parts += box("balance_rail", "0 -1.87 .006", ".87 .061 .032", metal)
    parts += box("balance_gauge_beam", "0 -2.33 .152", ".76 .03 .024", iron, False)
    for i, s in enumerate((-1, 1)):
        parts += box(f"tray_pad{i}", f"{s*.46} 0 .085", ".18 .245 .015", ".03 .18 .94 1")
        parts += box(f"balance_arm{i}", f"{s*.54} -1.13 -.056", ".037 1.18 .035", iron)
        parts += box(f"balance_flag{i}", f"{s*.73} -2.34 .180", ".072 .043 .048", gauge, False)
    fork = box("balance_rider", "0 0 .066", ".18 .136 .038", ".42 .34 .19 1")
    for i, s in enumerate((-1,1)):
        fork += box(f"balance_jaw{i}", f"{s*.146} 0 .218", ".030 .125 .137", ".52 .46 .33 1")
    fork += box("balance_index_arm", "0 -.151 .095", ".041 .126 .024", metal, False)
    fork += box("balance_index", "0 -.258 .106", ".074 .053 .030", ".99 .19 .012 1", False)
    moving = f"""
      <body name="balance_cradle" pos="{x} {y} .33">
        <joint name="cradle_pitch" type="hinge" axis="0 1 0" range="-14 14"
               limited="true" stiffness="{spring}" damping="8" armature=".16"/>
        <inertial pos="{tare/3.3} -.48 -.025" mass="3.3" diaginertia="2.1 1.3 2.1"/>
        {parts}
        <body name="balance_weight" pos="0 -1.87 0">
          <joint name="rider_slide" type="slide" axis="1 0 0" limited="true"
                 range="-.71 .71" damping="5" frictionloss="9"/>
          <inertial pos="0 0 .13" mass="{slider_mass}" diaginertia=".075 .080 .065"/>
          {fork}
        </body>
      </body>"""
    return mounting, moving


def arm_scene(block, goal, obstacle, gains, mass, com, cap_proof, *, cassette, stop=None, shipping=None, cradle=None):
    """Physical fixture and composite cartridge with a bounded passive pitch joint."""
    x, y = block
    gx, gy = goal
    ox, oy = obstacle
    direction, radius_a, radius_b = cassette
    shell, insert = .26, mass-.26
    insert_x = mass*com/insert
    ix = shell*(.17**2+.08**2)/3 + insert*2*.04**2/3
    shift = shell*com**2 + insert*(insert_x-com)**2
    iy = shell*(.69**2+.08**2)/3 + insert*2*.04**2/3 + shift
    iz = shell*(.69**2+.17**2)/3 + insert*2*.04**2/3 + shift
    fixture = "".join(
        (box(f"tray_pad{i}", f"{gx+sign*.46} {gy} .415", ".18 .245 .015", ".03 .18 .94 1")
         if cradle is None else "")
        + box(f"fixture_roof{i}", f"{gx} {gy+sign*.205} 1.127", ".79 .095 .413",
              ".53 .29 .10 1")
        for i, sign in enumerate((-1, 1)))
    fixture += "".join(
        box(f"fixture_post{i}{j}", f"{gx+dx} {gy+dy} .57", ".032 .031 .57", ".25 .23 .21 1")
        for i, dx in enumerate((-.72, .72)) for j, dy in enumerate((-.305, .305)))
    sleeve, bolt = packing_stop(goal, *stop) if stop is not None else ("", "")
    deck, rocker = ((box("tray", f"{gx} {gy} .20", ".79 .245 .20", ".03 .18 .94 1"), "")
                    if cradle is None else level_cradle(goal, *cradle))
    xml = header().replace('gravity="0 0 -9.81"', 'gravity="0 0 -9.81" integrator="implicitfast"')
    xml = xml.replace("<worldbody>", f"""
    <custom>
      <numeric name="cup_capacity_n" data="20"/>
      <numeric name="cap_proof_n" data="{cap_proof}"/>
      <numeric name="impact_limit_n" data="65"/>
      <numeric name="scuff_limit_n" data="29"/>
    </custom><worldbody>""")
    return xml + f"""
    {deck}
    {fixture}
    {sleeve}
    {shipping_sleeve(block, *shipping) if shipping is not None else ""}
    {box("obstacle", f"{ox} {oy} .285", ".14 .22 .285", ".6 .6 .6 1")}
    {box("gantry", "-2.7 0 1.8", ".08 2.3 .08", ".45 .5 .55 1", False)}
    <body name="agent">
      <joint name="ax" type="slide" axis="1 0 0" damping="12"/>
      <joint name="ay" type="slide" axis="0 1 0" damping="12"/>
      <joint name="az" type="slide" axis="0 0 1" damping="12"/>
      <inertial pos="0 0 0" mass=".47" diaginertia=".004 .004 .004"/>
      <body name="tool">
        <joint name="pitch" type="hinge" axis="0 1 0" stiffness="13"
               damping=".72" limited="true" range="-24 24" armature=".003"/>
        <geom name="cup" type="cylinder" size=".080 .030" mass=".04" rgba="1 .02 .02 1"/>
      </body>
    </body>
    <body name="object" pos="{x} {y} {.086 if shipping is not None else .08}">
      <freejoint/>
      <inertial pos="{com} 0 0" mass="{mass}" diaginertia="{ix} {iy} {iz}"/>
      <geom name="fragile" type="box" size=".69 .17 .08" rgba=".11 .32 .20 1"/>
    </body>
    {bolt}
    {rocker}
    </worldbody>
    <tendon>
      <fixed name="sum_belt"><joint joint="ax" coef="1"/><joint joint="ay" coef="1"/></fixed>
      <fixed name="diff_belt"><joint joint="ax" coef="1"/><joint joint="ay" coef="-1"/></fixed>
    </tendon>
    <actuator>
      <position tendon="sum_belt" gear="{direction/radius_a}" kp="{gains[0]*radius_a**2/2}"/>
      <position tendon="diff_belt" gear="{direction/radius_b}" kp="{gains[1]*radius_b**2/2}"/>
      <position joint="az" kp="{gains[2]}"/>
    </actuator>
    <equality><weld name="grasp" body1="tool" body2="object" active="false"
        solref=".02 1" torquescale="2"/></equality>
    </mujoco>"""


def pixel_to_world(u, v):
    return ((u + .5) / SIZE * 2 * EXTENT - EXTENT,
            EXTENT - (v + .5) / SIZE * 2 * EXTENT)
