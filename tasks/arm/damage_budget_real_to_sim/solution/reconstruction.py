"""Independent approximate scene estimator used only by the visual reference."""
import numpy as np
from scipy.ndimage import label

from coordinates import pixel_to_world


def create_scene(top, block, tray, gains=(120., 120., 120.)):
    # Isolate the neutral obstacle, not the cool-tinted workbench or overhead rails.
    # PBR daylight gives the matte white block a slight warm cast. The metallic
    # workbench is darker; do not classify it by neutral hue alone.
    gray = (top.max(axis=2)-top.min(axis=2) < 35) & (top.mean(axis=2) > 215)
    gray[:51, :] = gray[205:, :] = False
    gray[:, :51] = gray[:, 205:] = False
    labels, count = label(gray)
    if count == 0:
        raise ValueError("no visual obstacle component")
    candidates = []
    for region in range(1, count+1):
        ys, xs = np.where(labels == region)
        width, height = np.ptp(xs)+1, np.ptp(ys)+1
        if 5 <= width <= 50 and 5 <= height <= 50 and .25 <= width/height <= 4:
            candidates.append((len(xs), region))
    if not candidates:
        raise ValueError("no bounded visual obstacle component")
    region = max(candidates)[1]
    ys, xs = np.where(labels == region)
    obstacle = pixel_to_world(xs.mean(), ys.mean())
    half_x, half_y = max(.03, np.ptp(xs)/64), max(.03, np.ptp(ys)/64)
    bx, by = block
    tx, ty = tray
    ox, oy = obstacle
    # A Cartesian surrogate reconstructed from the visible family of robot,
    # not a copy/load of the protected scene. Dynamics are approximate prior values.
    return f"""<mujoco>
      <option timestep=".01" gravity="0 0 -9.81"/>
      <default><geom friction=".8 .02 .002" solref=".02 1"/></default>
      <worldbody>
        <geom type="plane" size="5 5 .1"/>
        <geom type="box" pos="{ox} {oy} .30" size="{half_x} {half_y} .30"/>
        <geom type="box" pos="{tx} {ty} .025" size=".28 .28 .025" contype="0" conaffinity="0"/>
        <body name="agent">
          <joint name="x" type="slide" axis="1 0 0" damping="12"/>
          <joint name="y" type="slide" axis="0 1 0" damping="12"/>
          <joint name="z" type="slide" axis="0 0 1" damping="12"/>
          <geom type="cylinder" size=".10 .04" mass=".50"/>
        </body>
        <body name="object" pos="{bx} {by} .12">
          <freejoint/><geom type="box" size=".12 .12 .12" mass=".12"/>
        </body>
      </worldbody>
      <actuator><position joint="x" kp="{gains[0]}"/><position joint="y" kp="{gains[1]}"/>
        <position joint="z" kp="{gains[2]}"/></actuator>
      <equality><weld body1="agent" body2="object" active="false" solref=".02 1"/></equality>
    </mujoco>"""


def identify_response(client, initial, scene):
    """Estimate actuator gains from safe visual probes; no protected state is read."""
    import mujoco
    from scipy.optimize import least_squares
    from policy import xyz
    from predict import rollout

    actions, observed = [], []
    obs = initial
    for axis in range(3):
        for value in (1., 1., -1., -1., 0., 0.):
            action = [0., 0., 0., -1.]
            action[axis] = value
            obs = client.act(action)
            actions.append(action)
            observed.append(xyz(obs))
    model = mujoco.MjModel.from_xml_string(scene)
    observed = np.asarray(observed)

    def residual(gains):
        model.actuator_gainprm[:, 0] = gains
        model.actuator_biasprm[:, 1] = -gains
        predicted = np.asarray(rollout(model, actions))[:, :3]
        return (predicted-observed).ravel()

    fit = least_squares(residual, np.full(3, 120.), bounds=(25., 260.), max_nfev=50)
    return obs, fit.x, {"probe_actions": len(actions), "gains": fit.x.tolist(),
                        "fit_rmse": float(np.sqrt(np.mean(fit.fun**2)))}
