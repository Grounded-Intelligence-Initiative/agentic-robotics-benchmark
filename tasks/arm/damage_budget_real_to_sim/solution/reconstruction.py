"""Camera-only identification of drive gains and ballast in the explored scene."""
import json
import math

import numpy as np
from scipy.ndimage import label, uniform_filter
from scipy.optimize import least_squares

from arm_policy import (ArmController, HOME, MARK, PIVOT, SHEET_TOP, back_orientation,
                        fiducials, pitch_pose, tray_location)
from policy import decode
from belt_drive import CassetteProbe, side_marker

LOAD_RATINGS = {"light": (.80, .94, 13.), "heavy": (1.43, 1.55, 20.)}
SHORT_SWEEP = (1., 1., -1., -1., 0., 0.)
# Cross the same heights at speed in both directions, unloaded at home. The
# short single rise scarcely separates vertical lag from raster error.
VERTICAL_TRAVERSE = (-1.,)*4 + (1.,)*7 + (-1.,)*6 + (1.,)*3 + (0.,)*2
PX = 32.
U, V = np.meshgrid((np.arange(256)+.5)/PX-4, 4-(np.arange(256)+.5)/PX)


def peak_offset(left, middle, right):
    curvature = left-2*middle+right
    return float(np.clip(.5*(left-right)/curvature, -.5, .5)) if curvature < -1e-10 else 0.


def obstacle_location(top):
    # Match the bright 0.28 x 0.44 m top against its surround. The bench
    # itself can be bright and neutral and is much larger than the block.
    gray = top.mean(axis=2)
    footprint = (2*round(.22*PX)+1, 2*round(.14*PX)+1)
    score = uniform_filter(gray, footprint)-uniform_filter(gray, tuple(x+6 for x in footprint))
    roi = (U > -.64) & (U < -.07) & (V > -.77) & (V < -.05)
    y, x = np.unravel_index(np.argmax(np.where(roi, score, -np.inf)), score.shape)
    dx = peak_offset(*score[y, x-1:x+2])
    dy = peak_offset(*score[y-1:y+2, x])
    return np.array([U[y, x]+dx/PX, V[y, x]-dy/PX])


def source_from_sweep(tops):
    # The rail is stationary; the bridge that hides its end moves in X.
    surfaces = [np.minimum(im[:, :, 1], im[:, :, 2]).astype(float)-im[:, :, 0] for im in tops]
    color = np.maximum.reduce(surfaces)
    roi = (U < -.55) & (U > -2.65) & (V > -1.25) & (V < .05)
    labels, n = label((color > 55) & roi)
    if n == 0:
        raise ValueError("source rail not visible across the calibration sweep")
    rail = labels == (np.argmax(np.bincount(labels.ravel())[1:])+1)
    if np.ptp(U[rail]) < 1.35:
        raise ValueError("source rail ends not resolved")
    weight = np.maximum(color-30., 0.)*rail
    x, y = (U[rail].min()+U[rail].max())/2, (V*weight).sum()/weight.sum()
    votes = [s for im in tops if (s := back_orientation(im, x, y)) is not None]
    if not votes or abs(sum(votes)) <= len(votes)/2:
        raise ValueError("shipping-sleeve opening not resolved")
    return np.array([x, y-np.sign(sum(votes))*.255])


def shell_center_x(obs):
    # The sleeve sight windows expose both marks to the side camera. A
    # clipped mark is a poor ruler; keep the rail estimate if no pair resolves.
    marks = fiducials(obs, side=True, wide=True)
    if marks is not None and abs(marks[1, 0]-marks[0, 0]-2*MARK) <= 1/PX:
        return float(marks[:, 0].mean())
    return None


def fixture_center(obs):
    tray = tray_location(obs)
    if tray is None:
        raise ValueError("fixture not visible at reset")
    r, g, b = decode(obs).astype(float).transpose(2, 0, 1)
    blue = (b-1.35*np.maximum(r, g) > 25) & (abs(V-tray[1]) < .16) & (abs(U-tray[0]) < 1.)
    # The bolt masks a strip of blue at a variable X station. The midpoint
    # of the two visible deck edges does not depend on that missing area.
    midpoints = []
    for row in blue[blue.any(axis=1)]:
        xs = U[0, row]
        if xs[-1]-xs[0] >= 1.58-2/PX:
            midpoints.append((xs[0]+xs[-1])/2)
    if midpoints:
        tray[0] = np.median(midpoints)
    return tray


def create_scene(block, tray, obstacle, gains, mass=.87, com=0., cap_proof=13., *, cassette, on_sheet=False):
    """Build MJCF from camera poses and the equipment's nominal dimensions."""
    bx, by = block
    tx, ty = tray
    ox, oy = obstacle
    gx, gy, gz = gains
    sign, ra, rb = cassette
    shell, insert = .26, mass-.26
    slug = com*mass/insert
    i_x = shell*(.17**2+.08**2)/3 + insert*2*.04**2/3
    shift = shell*com**2 + insert*(slug-com)**2
    i_y = shell*(.69**2+.08**2)/3 + insert*2*.04**2/3 + shift
    i_z = shell*(.69**2+.17**2)/3 + insert*2*.04**2/3 + shift
    sheet = (f'<geom type="box" pos="{bx} {by} .001" size=".86 .76 .005" '
             'priority="1" friction=".14 .003 .0003"/>') if on_sheet else ""
    return f"""<mujoco>
      <option timestep=".01" gravity="0 0 -9.81" integrator="implicitfast"/>
      <default><geom friction=".8 .02 .002" solref=".02 1"/></default>
      <custom>
        <numeric name="cup_capacity_n" data="20"/><numeric name="cap_proof_n" data="{cap_proof}"/>
        <numeric name="impact_limit_n" data="65"/><numeric name="scuff_limit_n" data="29"/>
      </custom>
      <worldbody>
        <geom type="plane" size="8 8 .1"/>
        {sheet}
        <geom type="box" pos="{tx} {ty} .20" size=".79 .245 .20"/>
        <geom type="box" pos="{tx-.46} {ty} .415" size=".18 .245 .015"/>
        <geom type="box" pos="{tx} {ty-.205} 1.127" size=".79 .095 .413"/>
        <geom type="box" pos="{tx+.46} {ty} .415" size=".18 .245 .015"/>
        <geom type="box" pos="{tx} {ty+.205} 1.127" size=".79 .095 .413"/>
        <geom type="box" pos="{tx-.72} {ty-.305} .57" size=".032 .031 .57"/>
        <geom type="box" pos="{tx-.72} {ty+.305} .57" size=".032 .031 .57"/>
        <geom type="box" pos="{tx+.72} {ty-.305} .57" size=".032 .031 .57"/>
        <geom type="box" pos="{tx+.72} {ty+.305} .57" size=".032 .031 .57"/>
        <geom type="box" pos="{ox} {oy} .285" size=".14 .22 .285"/>
        <body name="agent">
          <joint name="x" type="slide" axis="1 0 0" damping="12"/>
          <joint name="y" type="slide" axis="0 1 0" damping="12"/>
          <joint name="z" type="slide" axis="0 0 1" damping="12"/>
          <inertial pos="0 0 0" mass=".47" diaginertia=".004 .004 .004"/>
          <body name="tool">
            <joint name="pitch" type="hinge" axis="0 1 0" stiffness="13" damping=".72"
                   limited="true" range="-24 24" armature=".003"/>
            <geom type="cylinder" size=".08 .03" mass=".04"/>
          </body>
        </body>
        <body name="object" pos="{bx} {by} {.08+(SHEET_TOP if on_sheet else 0.)}">
          <freejoint/>
          <inertial pos="{com} 0 0" mass="{mass}" diaginertia="{i_x} {i_y} {i_z}"/>
          <geom name="fragile" type="box" size=".69 .17 .08"/>
        </body>
      </worldbody>
      <tendon>
        <fixed name="A"><joint joint="x" coef="1"/><joint joint="y" coef="1"/></fixed>
        <fixed name="B"><joint joint="x" coef="1"/><joint joint="y" coef="-1"/></fixed>
      </tendon>
      <actuator>
        <position tendon="A" gear="{sign/ra}" kp="{gx*ra**2/2}"/>
        <position tendon="B" gear="{sign/rb}" kp="{gy*rb**2/2}"/>
        <position joint="z" kp="{gz}"/>
      </actuator>
      <equality><weld body1="tool" body2="object" active="false"
                      solref=".02 1" torquescale="2"/></equality>
    </mujoco>"""


def identify_response(client, initial, tray, obstacle, *, vertical_sweep=SHORT_SWEEP):
    """Fit drive gains from relative tracks, eliminating each camera's fixed offset."""
    import mujoco
    from model_rollout import rollout

    probe = CassetteProbe()
    actions, tops, side = [], [decode(initial)], [side_marker(decode(initial, 1))]
    obs, command = initial, HOME.copy()
    shell_xs = []

    def sample(action, radius):
        nonlocal obs
        obs = client.act(action)
        actions.append(action)
        tops.append(decode(obs))
        side.append(side_marker(decode(obs, 1), command, radius))
        if (x := shell_center_x(obs)) is not None:
            shell_xs.append(x)

    while (action := probe.action(obs)) is not None:
        sample(action, .80)
    planar = len(actions)
    for value in vertical_sweep:
        command[2] += .14*value
        sample([0., 0., value, 0.], .26)
    block = source_from_sweep(tops)
    if shell_xs:
        block[0] = np.median(shell_xs)
    model = mujoco.MjModel.from_xml_string(
        create_scene(block, tray, obstacle, [120., 120., 74.], cassette=probe.cassette))
    observed = np.asarray(side)
    observed[planar+1:, 0] = np.nan
    observed[:planar, 1] = np.nan
    valid = np.isfinite(observed)
    if np.any(valid.sum(axis=0) < 4):
        raise ValueError("too few observed free-space poses")

    def residual(gains):
        _, ra, rb = probe.cassette
        kp = gains*np.array([ra**2/2, rb**2/2, 1.])
        model.actuator_gainprm[:, 0] = kp
        model.actuator_biasprm[:, 1] = -kp
        predicted = np.vstack((HOME, np.asarray(rollout(model, actions))[:, :3]))
        error = predicted[:, [0, 2]]-observed
        error -= np.nanmean(error, axis=0)
        return np.r_[error[valid], (gains-[115., 115., 74.])*.00003]

    fit = least_squares(residual, [120., 120., 74.],
                        bounds=([55., 55., 60.], [175., 175., 88.]), max_nfev=40)
    residual(fit.x)
    free_z = np.r_[HOME[2], np.asarray(rollout(model, actions))[:, 2]]
    diagnostics = {"drive_fit_rmse_m": float(np.sqrt(np.mean(fit.fun[:-3]**2))),
                   "marker_height_bias_m": float(np.nanmedian(observed[:, 1]-free_z)),
                   "drive_samples_xz": valid.sum(axis=0).tolist(), "cassette": list(probe.cassette),
                   "pulse_dx_m": list(probe.shifts), "source_reset_xy_m": block.tolist()}
    return obs, block, fit.x, diagnostics


def fit_load(block, tray, obstacle, gains, start, actions, samples, tool_samples, estimate, cartridge, cassette):
    import mujoco
    from arm_dynamics import ArmDynamics, pose_row

    idx, observed = zip(*samples)
    tool_idx = [i for i, _ in tool_samples]
    tool_z = np.array([z for _, z in tool_samples])
    observed = np.array(observed)
    # The side-to-top x registration uses relative displacement.
    observed[:, 0] -= observed[0, 0]
    units = np.array([.75, 1., .69])
    low, high, cap_proof = LOAD_RATINGS[cartridge]

    def residual(estimate):
        model = mujoco.MjModel.from_xml_string(
            create_scene(block, tray, obstacle, gains, *estimate, cap_proof, cassette=cassette, on_sheet=True))
        data = mujoco.MjData(model)
        arm = ArmDynamics(model, data)
        data.qpos[:3] = start
        arm.set_command(start)
        mujoco.mj_forward(model, data)
        poses = []
        for action in actions:
            arm.step(action)
            poses.append(pose_row(data)[[3, 5, 6, 2]])
        poses = np.asarray(poses)
        rows = poses[list(idx), :3]
        rows[:, 0] -= rows[0, 0]
        # Vertical spring deflection directly weighs the load independently
        # of the model's pickup height and the shell's pitch fiducials.
        return np.r_[((rows-observed)*units).ravel(), 2*(poses[tool_idx, 3]-tool_z)]

    fit = least_squares(residual, estimate, bounds=([low, -.45], [high, .45]),
                        diff_step=.002, max_nfev=30, loss="soft_l1", f_scale=.009)
    return fit.x, float(np.sqrt(np.mean(fit.fun**2)))


def reconstruct(client, initial, out, *, controller=None):
    tray, obstacle = fixture_center(initial), obstacle_location(decode(initial))
    obs, block, gains, diagnostics = identify_response(client, initial, tray, obstacle,
                                                       vertical_sweep=VERTICAL_TRAVERSE)
    ctrl = ArmController() if controller is None else controller
    ctrl.cassette = tuple(diagnostics["cassette"])
    actions, samples, tool_samples = [], [], []
    start = pickup = None
    estimate, fit_error = np.array([.87, 0.]), None
    while not obs["done"]:
        previous = ctrl.command.copy()
        action = ctrl.action(obs)
        if ctrl.stage == "shift" and start is None:
            # Fresh settled shell pose after the supported tip. Fit only the
            # observed pickup/proof segment, not a hypothetical sleeve motion.
            start, pickup = previous, ctrl.block.copy()
            if (x := shell_center_x(obs)) is not None:
                pickup[0] = x
        if ctrl.stage == "proof" and ctrl.ticks >= 2:
            measured = pitch_pose(obs, ctrl.baseline, .08+SHEET_TOP, wide=True)
            if measured is not None:
                samples.append((len(actions)-1, measured))
        if ctrl.stage in ("proof", "lift") and ctrl.ticks >= 2 and start is not None:
            aim = previous - [0., 0., .13]
            _, top = side_marker(decode(obs, 1), aim)
            if np.isfinite(top):
                tool_samples.append((len(actions)-1, top-diagnostics["marker_height_bias_m"]))
        elif ctrl.stage == "align" and fit_error is None and ctrl.proof_pose is not None and len(samples) >= 10:
            low, high, _ = LOAD_RATINGS[ctrl.cartridge]
            initial_mass = float(np.clip(ctrl.sag*gains[2]/9.81, low, high))
            angle = ctrl.proof_pose[2]
            com = ctrl.offset+(12.45*angle/(9.81*initial_mass)
                               + PIVOT*math.sin(angle))/math.cos(angle)
            estimate, fit_error = fit_load(pickup, tray, obstacle, gains, start, actions, samples, tool_samples,
                                           [initial_mass, np.clip(com, -.45, .45)], ctrl.cartridge, ctrl.cassette)
        obs = client.act(action)
        if start is not None:
            actions.append(action)
    cap_proof = LOAD_RATINGS[ctrl.cartridge or "light"][2]
    (out/"scene.xml").write_text(create_scene(block, tray, obstacle, gains, *estimate, cap_proof,
                                              cassette=ctrl.cassette))
    (out/"identification.json").write_text(json.dumps({
        "drive_gains": gains.tolist(), **diagnostics,
        "mass_kg": float(estimate[0]), "com_x_m": float(estimate[1]),
        "cartridge": ctrl.cartridge, "cap_proof_n": cap_proof,
        "proof_pose_x_z_pitch": None if ctrl.proof_pose is None else ctrl.proof_pose.tolist(),
        "proof_fit_rmse_m": fit_error, "probe_samples": len(samples), "loaded_carriage_samples": len(tool_samples),
    }, indent=2))
    return obs
