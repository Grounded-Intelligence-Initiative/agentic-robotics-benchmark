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
    <camera name="top" pos="0 0 4.5" orthographic="true" fovy="8"/>
    <camera name="side" pos="0 -12 2" xyaxes="1 0 0 0 0 1"
        orthographic="true" fovy="8"/>
    <geom name="floor" type="plane" size="8 8 .1" rgba=".16 .18 .20 1"/>
    """


def box(name, position, size, color, collision=True):
    return (
        f'<geom name="{name}" type="box" pos="{position}" size="{size}" '
        f'rgba="{color}" contype="{int(collision)}" conaffinity="{int(collision)}"/>'
    )


def arm_scene(block, goal, obstacle, gains=(120., 120., 120.)):
    """A Cartesian suction arm; callers supply the scene they reconstructed."""
    x, y = block
    gx, gy = goal
    ox, oy = obstacle
    return header() + f"""
    {box("tray", f"{gx} {gy} .025", ".28 .28 .025", "0 .1 1 1", False)}
    {box("obstacle", f"{ox} {oy} .3", ".17 .3 .3", ".6 .6 .6 1")}
    {box("gantry", "-2.7 0 1.8", ".08 2.3 .08", ".45 .5 .55 1", False)}
    <body name="agent" pos="0 0 0">
      <joint name="ax" type="slide" axis="1 0 0" damping="12"/>
      <joint name="ay" type="slide" axis="0 1 0" damping="12"/>
      <joint name="az" type="slide" axis="0 0 1" damping="12"/>
      <geom name="cup" type="cylinder" size=".10 .04" mass=".5" rgba="1 .02 .02 1"/>
      <geom type="capsule" fromto="0 0 .06 0 0 .7" size=".035" mass=".01"
            rgba=".6 .65 .7 1" contype="0" conaffinity="0"/>
    </body>
    <body name="object" pos="{x} {y} .12">
      <freejoint/>
      <geom name="fragile" type="box" size=".12 .12 .12" mass=".12" rgba="0 1 .1 1"/>
    </body>
    </worldbody>
    <actuator>
      <position joint="ax" kp="{gains[0]}"/><position joint="ay" kp="{gains[1]}"/>
      <position joint="az" kp="{gains[2]}"/>
    </actuator>
    <equality><weld name="grasp" body1="agent" body2="object" active="false"
        solref=".02 1"/></equality>
    </mujoco>"""


def pixel_to_world(u, v):
    return ((u + .5) / SIZE * 2 * EXTENT - EXTENT,
            EXTENT - (v + .5) / SIZE * 2 * EXTENT)
