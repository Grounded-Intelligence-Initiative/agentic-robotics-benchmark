"""The hidden world of the swarm localization task: the warehouse, the eight drones' and six
anchors' hardware (fixed per unit), flight generation and the sensor models.

Used by the grader (fresh hidden flights at verify time, from OS-entropy seeds) and by
tests/gen/make_flights.py (the calibration and practice flights the agent receives). This file
is in tests/, absent while the agent works.

Sensors per drone:
* UWB two-way ranging (TWR) to the other drones and to six wall anchors, scheduled in TDMA
  slots. A measured range is the true distance plus both nodes' antenna-delay biases, both
  nodes' antenna-pattern biases (a function of the 3-D direction of the other node in the
  node's own body frame: degree-1 and degree-2 spherical harmonics, i.e. an antenna offset that
  turns with the drone, an azimuthal second harmonic and the elevation dependence of a chip
  antenna above a carbon frame), the chipset's received-power bias (a function of the
  distance, shared by every radio), each drone radio's thermal drift (its delay follows the
  board temperature: warm or cold at take-off, heated by the motors and ESCs, cooled by the
  airflow in fast flight; per-flight amounts, a per-unit time constant), noise, and, when the
  straight line between the nodes passes through a rack, a
  positive non-line-of-sight excess with more noise, growing with the length of path inside
  each rack at a rate set by what that rack holds in this flight. A few ranges are gross multipath
  outliers; some slots are lost.
* Downward optical flow + ToF: horizontal body-frame velocity (per-unit scale error, noise,
  invalid over the low-texture rack tops), and the distance to whatever is below (the floor,
  or a rack top).
* Gyro z: yaw rate plus a per-flight bias. There is no usable magnetometer in a steel
  warehouse, and each drone is placed on its pad by hand with an unknown heading.
"""

import math

import numpy as np

# --- the warehouse ---------------------------------------------------------------------- #
HALL = (0.0, 40.0, 0.0, 24.0, 0.0, 9.0)               # x0, x1, y0, y1, z0, z1 (m)
# racks: axis-aligned boxes (x0, x1, y0, y1, z0, z1)
RACKS = [
    (4.0, 34.0, 3.0, 4.2, 0.0, 6.0),
    (4.0, 34.0, 7.6, 8.8, 0.0, 6.0),
    (4.0, 34.0, 12.2, 13.4, 0.0, 6.0),
    (4.0, 34.0, 16.8, 18.0, 0.0, 6.0),
    (4.0, 20.0, 21.0, 22.2, 0.0, 4.5),
]
ANCHORS = np.array([
    [0.3, 0.3, 8.2], [39.7, 0.3, 8.0], [39.7, 23.7, 8.3],
    [0.3, 23.7, 7.9], [20.0, 0.3, 3.0], [20.0, 23.7, 3.1],
])
N_DRONES = 8
PADS = np.array([[36.0 + 1.2 * (i % 2), 4.0 + 2.2 * (i // 2), 0.0] for i in range(N_DRONES)])

# --- per-unit hardware truth (fixed: the same units fly every flight) -------------------- #
# antenna-delay bias (m of range), drones then anchors
DELAY = np.array([0.212, 0.087, 0.301, 0.145, 0.248, 0.059, 0.176, 0.333,
                  0.121, 0.268, 0.094, 0.187, 0.230, 0.155])
# antenna pattern: bias = a * cos(az_body - phi) + c2 * cos(2 (az_body - phi2)), drones only
PAT_A = np.array([0.085, 0.062, 0.104, 0.071, 0.093, 0.066, 0.088, 0.058])
PAT_PHI = np.array([0.4, -1.9, 2.7, 1.1, -0.6, 2.2, -2.8, 0.9])
PAT_C2 = np.array([0.036, 0.048, 0.027, 0.041, 0.033, 0.052, 0.030, 0.044])
PAT_PHI2 = np.array([1.3, -0.2, 0.8, -1.4, 2.5, 0.3, -2.1, 1.7])
# elevation terms (u = unit vector to the other node in the body frame, z up):
# + z * u_z + zz * (3 u_z^2 - 1) / 2 + 2 u_z (xz * u_x + yz * u_y)
PAT_Z = np.array([0.014, -0.011, 0.028, 0.009, 0.083, -0.006, 0.047, 0.060])
PAT_ZZ = np.array([0.068, 0.074, 0.053, 0.062, 0.095, 0.052, 0.066, 0.054])
PAT_XZ = np.array([0.023, 0.038, 0.006, 0.006, 0.045, -0.035, -0.011, 0.012])
PAT_YZ = np.array([0.014, -0.041, 0.007, -0.041, -0.039, 0.040, -0.013, -0.031])
# received-power bias of the chipset: POWER_BIAS * ln(distance / POWER_REF) (m)
POWER_BIAS = 0.05
POWER_REF = 8.0
# board temperature: a drone radio's delay drifts with its board's temperature (m of range above
# a cold board). Per flight and drone: the drift at take-off W0 (a board still warm from the
# charger) and the drift the board settles at in a hover WH (the day's air temperature, the
# battery's state); in fast flight the airflow carries away up to WARM_COOL of the heating.
# The drift follows its equilibrium with the unit's thermal time constant WARM_TAU (s).
WARM_TAU = np.array([54.0, 31.0, 44.0, 38.0, 40.0, 55.0, 36.0, 52.0])
WARM_W0 = (0.0, 0.07)
WARM_WH = (0.08, 0.20)
WARM_COOL = 0.6
WARM_V = 1.6                     # m/s at which the cooling saturates
FLOW_SCALE = np.array([1.062, 0.951, 1.034, 0.968, 1.081, 0.927, 1.015, 0.989])
TOF_OFFSET = np.array([0.031, -0.022, 0.044, 0.012, -0.035, 0.027, -0.009, 0.038])

RANGE_SIGMA = 0.035
NLOS_SIGMA = 0.07
NLOS_BASE = 0.10                 # m of excess range for any blocked path
# m of excess per metre of path inside a rack: it depends on what is stored on its shelves
# (steel parts, cardboard, empty bays), which changes from flight to flight, rack by rack
NLOS_PER_M = (0.15, 0.50)
OUTLIER_P = 0.012
LOSS_P = 0.08
SLOT_S = 0.010                   # TDMA slot: 100 ranges per second for the whole network
FLOW_RATE = 50.0
FLOW_SIGMA = 0.04
GYRO_SIGMA = 0.004
TOF_SIGMA = 0.01


def segment_hits_box(p, q, box):
    """Whether the segment p->q passes through an axis-aligned box (slab test)."""
    t0, t1 = 0.0, 1.0
    d = q - p
    for a in range(3):
        lo, hi = box[2 * a], box[2 * a + 1]
        if abs(d[a]) < 1e-12:
            if p[a] < lo or p[a] > hi:
                return False
            continue
        ta, tb = (lo - p[a]) / d[a], (hi - p[a]) / d[a]
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def blocked(p, q):
    return any(segment_hits_box(p, q, b) for b in RACKS)


def length_inside_racks(p, q):
    """Length (m) of the segment p->q that lies inside each rack box."""
    total = np.zeros(len(RACKS))
    d = q - p
    L = float(np.linalg.norm(d))
    for r, b in enumerate(RACKS):
        t0, t1 = 0.0, 1.0
        ok = True
        for a in range(3):
            lo, hi = b[2 * a], b[2 * a + 1]
            if abs(d[a]) < 1e-12:
                if p[a] < lo or p[a] > hi:
                    ok = False
                    break
                continue
            ta, tb = (lo - p[a]) / d[a], (hi - p[a]) / d[a]
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                ok = False
                break
        if ok:
            total[r] = (t1 - t0) * L
    return total


def surface_below(x, y):
    """Height of what is directly below (x, y): a rack top or the floor."""
    h = 0.0
    for b in RACKS:
        if b[0] <= x <= b[1] and b[2] <= y <= b[3]:
            h = max(h, b[5])
    return h


def inside_rack(p, margin=0.35):
    for b in RACKS:
        if (b[0] - margin <= p[0] <= b[1] + margin and b[2] - margin <= p[1] <= b[3] + margin
                and p[2] <= b[5] + margin):
            return True
    return False


# --- flights ------------------------------------------------------------------------------ #
AISLE_Y = [1.5, 5.9, 10.5, 15.1, 19.5, 23.0]
FLIGHT_LEVELS = [1.2, 2.0, 3.0, 4.2, 7.2, 7.8]


def _route(rng, start, n_legs):
    """Waypoints that keep out of the racks: aisle runs at rack height, cross-overs above."""
    pts = [start.copy()]
    pos = start.copy()
    # take off
    pos = pos + np.array([0.0, 0.0, rng.uniform(1.0, 2.0)])
    pts.append(pos.copy())
    for _ in range(n_legs):
        kind = rng.choice(["aisle", "over", "hover"], p=[0.55, 0.3, 0.15])
        if kind == "aisle":
            y = rng.choice(AISLE_Y) + rng.uniform(-0.3, 0.3)
            z = rng.choice(FLIGHT_LEVELS[:4]) + rng.uniform(-0.2, 0.2)
            x_end = rng.uniform(2.0, 38.0)
            # go up and over if the aisle change would cross a rack
            if pos[2] < 6.6 and any(_crosses_rack_y(pos[1], y, b) for b in RACKS):
                pts.append(np.array([pos[0], pos[1], 7.2 + rng.uniform(0, 0.5)]))
                if 4.0 <= pos[0] <= 34.0:
                    pts.append(np.array([pos[0], y, 7.2 + rng.uniform(0, 0.5)]))
                else:
                    pts.append(np.array([pos[0], y, pts[-1][2]]))
            pts.append(np.array([pts[-1][0], y, z]))
            pts.append(np.array([x_end, y, z]))
        elif kind == "over":
            z = rng.uniform(6.9, 8.0)
            pts.append(np.array([pos[0], pos[1], z]))
            pts.append(np.array([rng.uniform(3.0, 37.0), rng.uniform(1.0, 23.0), z]))
        else:
            pts.append(pts[-1].copy())
        pos = pts[-1].copy()
    return pts


def _crosses_rack_y(y0, y1, b):
    lo, hi = min(y0, y1), max(y0, y1)
    return hi >= b[2] and lo <= b[3]


def _fly(rng, waypoints, t_total, dt):
    """Constant-speed legs with smoothed corners; yaw turns towards the direction of travel
    most of the time (sometimes the drone crabs)."""
    speeds = rng.uniform(0.6, 1.6, len(waypoints))
    # sit on the pad for 2 s first, so the smoothing below leaves the start on the pad
    pos = [waypoints[0]] * int(2.0 / dt)
    for a, b, v in zip(waypoints[:-1], waypoints[1:], speeds):
        d = np.linalg.norm(b - a)
        n = max(2, int(math.ceil(d / v / dt))) if d > 1e-6 else int(rng.uniform(1.0, 3.0) / dt)
        s = np.linspace(0.0, 1.0, n + 1)[1:]
        pos.extend(a[None, :] + (b - a)[None, :] * s[:, None])
    pos = np.array(pos)
    n_total = int(round(t_total / dt))
    if len(pos) < n_total:
        pos = np.concatenate([pos, np.repeat(pos[-1:], n_total - len(pos), axis=0)])
    pos = pos[:n_total]
    # smooth (moving average twice, ~1 s)
    k = int(0.8 / dt)
    ker = np.ones(k) / k
    for _ in range(2):
        padded = np.concatenate([np.repeat(pos[:1], k, 0), pos, np.repeat(pos[-1:], k, 0)])
        pos = np.stack([np.convolve(padded[:, i], ker, mode="same")[k:-k] for i in range(3)], 1)
    return pos


def generate_flight(rng, t_total=150.0, dt=0.01):
    """Truth for one flight of all drones: positions (D, N, 3), yaw (D, N), t (N,)."""
    n = int(round(t_total / dt))
    t = np.arange(n) * dt
    P = np.empty((N_DRONES, n, 3))
    Y = np.empty((N_DRONES, n))
    for i in range(N_DRONES):
        for _attempt in range(50):
            wps = _route(rng, PADS[i], int(rng.integers(5, 9)))
            # stagger take-offs
            delay = int(rng.uniform(0.0, 8.0) / dt)
            path = _fly(rng, wps, t_total, dt)
            path = np.concatenate([np.repeat(path[:1], delay, 0), path])[:n]
            if not any(inside_rack(p) for p in path[::20]):
                break
        P[i] = path
        vel = np.gradient(path, dt, axis=0)
        sp = np.linalg.norm(vel[:, :2], axis=1)
        course = np.arctan2(vel[:, 1], vel[:, 0])
        yaw0 = rng.uniform(-math.pi, math.pi)
        yaw = np.empty(n)
        yaw[0] = yaw0
        crab = rng.uniform(-0.6, 0.6)
        for k in range(1, n):
            target = course[k] + crab if sp[k] > 0.25 else yaw[k - 1]
            err = (target - yaw[k - 1] + math.pi) % (2 * math.pi) - math.pi
            yaw[k] = yaw[k - 1] + np.clip(err, -1.2 * dt, 1.2 * dt)
        Y[i] = yaw
    return dict(t=t, P=P, Y=Y, dt=dt)


# --- sensors ----------------------------------------------------------------------------- #
def _pattern(i, d, yaw):
    """Antenna-pattern bias of drone i's radio towards the world-frame vector d, the drone at
    heading yaw (level flight)."""
    c, s = math.cos(yaw), math.sin(yaw)
    n = float(np.linalg.norm(d))
    ux, uy, uz = (c * d[0] + s * d[1]) / n, (-s * d[0] + c * d[1]) / n, d[2] / n
    h2 = ux * ux + uy * uy                       # cos^2 of the elevation
    az = math.atan2(uy, ux)
    return (PAT_A[i] * math.sqrt(h2) * math.cos(az - PAT_PHI[i])
            + PAT_C2[i] * h2 * math.cos(2.0 * (az - PAT_PHI2[i]))
            + PAT_Z[i] * uz + PAT_ZZ[i] * 0.5 * (3.0 * uz * uz - 1.0)
            + 2.0 * uz * (PAT_XZ[i] * ux + PAT_YZ[i] * uy))


def board_drift(rng, truth):
    """Each drone radio's delay drift over the flight (D, N), m: on the pad it keeps its
    take-off value; airborne it relaxes towards the hover equilibrium, less the cooling of the
    airflow at speed."""
    P, dt = truth["P"], truth["dt"]
    w0 = rng.uniform(WARM_W0[0], WARM_W0[1], N_DRONES)
    wh = rng.uniform(WARM_WH[0], WARM_WH[1], N_DRONES)
    out = np.empty(P.shape[:2])
    for i in range(N_DRONES):
        sp = np.linalg.norm(np.gradient(P[i], dt, axis=0), axis=1)
        flying = np.maximum.accumulate(P[i, :, 2] > P[i, 0, 2] + 0.02)
        target = np.where(flying, wh[i] * (1.0 - WARM_COOL * np.minimum(sp / WARM_V, 1.0)), w0[i])
        a = dt / WARM_TAU[i]
        w = w0[i]
        for k in range(len(sp)):
            w += (target[k] - w) * a
            out[i, k] = w
    return out


def node_pos(truth, node, k):
    if node < N_DRONES:
        return truth["P"][node, k]
    return ANCHORS[node - N_DRONES]


def range_schedule():
    pairs = [(i, j) for i in range(N_DRONES) for j in range(i + 1, N_DRONES)]
    pairs += [(i, N_DRONES + a) for i in range(N_DRONES) for a in range(len(ANCHORS))]
    return pairs


def simulate_sensors(rng, truth):
    """Measured logs of one flight. Returns dict with 'uwb' rows
    (t, initiator, responder, range or nan) and per-drone 'flow' (t, vx_b, vy_b, valid, tof)
    and 'gyro' (t, wz)."""
    t, P, Y, dt = truth["t"], truth["P"], truth["Y"], truth["dt"]
    n = len(t)
    pairs = range_schedule()
    order = rng.permutation(len(pairs))
    uwb = []
    slot = 0
    k_slot = int(round(SLOT_S / dt))
    drift = board_drift(rng, truth)
    stock = rng.uniform(NLOS_PER_M[0], NLOS_PER_M[1], len(RACKS))
    for k in range(0, n, k_slot):
        a, b = pairs[order[slot % len(pairs)]]
        slot += 1
        if slot % len(pairs) == 0:
            order = rng.permutation(len(pairs))
        if rng.random() < LOSS_P:
            continue
        pa, pb = node_pos(truth, a, k), node_pos(truth, b, k)
        d = pb - pa
        dist = float(np.linalg.norm(d))
        meas = dist + DELAY[a] + DELAY[b] + POWER_BIAS * math.log(max(dist, 0.5) / POWER_REF)
        if a < N_DRONES:
            meas += _pattern(a, d, Y[a, k]) + drift[a, k]
        if b < N_DRONES:
            meas += _pattern(b, -d, Y[b, k]) + drift[b, k]
        inside = length_inside_racks(pa, pb)
        if inside.sum() > 0.0:
            # through the shelving: an excess that grows with the material crossed, rack by
            # rack, varying with where along the shelves the path goes
            excess = (NLOS_BASE + float(stock @ inside)) * float(np.clip(1.0 + 0.2 * rng.standard_normal(), 0.5, 1.6))
            meas += excess + rng.normal(0.0, NLOS_SIGMA)
        else:
            meas += rng.normal(0.0, RANGE_SIGMA)
        if rng.random() < OUTLIER_P:
            meas += rng.uniform(1.0, 6.0)
        uwb.append((t[k], a, b, meas))
    flow, gyro = [], []
    kf = int(round(1.0 / FLOW_RATE / dt))
    gyro_bias = rng.normal(0.0, 0.006, N_DRONES)
    for i in range(N_DRONES):
        vel = np.gradient(P[i], dt, axis=0)
        wz = np.gradient(np.unwrap(Y[i]), dt)
        rows, grows = [], []
        for k in range(0, n, kf):
            c, s = math.cos(Y[i, k]), math.sin(Y[i, k])
            vb = np.array([c * vel[k, 0] + s * vel[k, 1], -s * vel[k, 0] + c * vel[k, 1]])
            below = surface_below(P[i, k, 0], P[i, k, 1])
            height = P[i, k, 2] - below
            valid = below == 0.0 and height > 0.25
            if valid:
                v = FLOW_SCALE[i] * vb + rng.normal(0.0, FLOW_SIGMA * (1 + 0.3 * height), 2)
            else:
                v = np.array([np.nan, np.nan])
            tof = height + TOF_OFFSET[i] + rng.normal(0.0, TOF_SIGMA) if height < 8.0 else np.nan
            rows.append((t[k], v[0], v[1], tof))
            grows.append((t[k], wz[k] + gyro_bias[i] + rng.normal(0.0, GYRO_SIGMA)))
        flow.append(np.array(rows))
        gyro.append(np.array(grows))
    return dict(uwb=np.array(uwb), flow=flow, gyro=gyro)
