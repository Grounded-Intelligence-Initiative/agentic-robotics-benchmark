"""V3 stabilized planar transport model; no leg or free-payload simulation.

Geometry is sampled without solver/model outcomes. The service aisle is a constructive
feasibility witness, not a path given to an agent (or a requirement in the success test).
"""
import math
import random

FIELD = 3.8
DT = .01
SUBSTEPS = 20
GOAL_TOLERANCE = .12
YAW_TOLERANCE = math.radians(7)
STOP_SPEED = .055
STOP_YAW = .07


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(x, lo, hi):
    return min(hi, max(lo, x))


def overlaps(pose, half, rect, margin=0.):
    """Exact separating-axis OBB/AABB test, including touching as collision."""
    x, y, yaw = pose[:3]
    a, b = half[0] + margin, half[1] + margin
    cx, cy, hx, hy = rect
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = cx-x, cy-y
    return (abs(dx) <= hx+a*abs(c)+b*abs(s)
            and abs(dy) <= hy+a*abs(s)+b*abs(c)
            and abs(dx*c+dy*s) <= a+hx*abs(c)+hy*abs(s)
            and abs(-dx*s+dy*c) <= b+hx*abs(s)+hy*abs(c))


def free(pose, half, obstacles, margin=0.):
    return not any(overlaps(pose, half, r, margin) for r in obstacles)


def accelerations(yaw, velocity, steer, control, params):
    """Force/mass and torque/inertia for mj_step, under a shared traction circle.

    The supplied stabilizer supplies lateral force, not a position constraint.
    Steering/yaw lag and lateral slip persist; this is not foot-contact physics.
    """
    vx, vy, omega = velocity
    c, s = math.cos(yaw), math.sin(yaw)
    v, slip = vx*c+vy*s, -vx*s+vy*c
    throttle, steering = control
    steer += DT * (steering*params['steer_max']-steer)/params['steer_tau']
    requested_a = (throttle*params['speed']-v)/params['speed_tau']
    requested_w = v*math.tan(steer)/params['wheelbase']
    limit = params['traction']*9.81
    lateral = clip(v*requested_w, -.85*limit, .85*limit)
    target_w = lateral/v if abs(v) > 1e-7 else 0.
    lateral_a = v*omega-slip/params['lateral_tau']
    norm = math.hypot(requested_a, lateral_a)
    scale = min(1., limit/max(norm, 1e-12))
    a, b = requested_a*scale, lateral_a*scale
    return (a*c-b*s, a*s+b*c, (target_w-omega)/params['yaw_tau']), steer


def step(state, control, params):
    """Free-space predictor only. The authoritative world uses MuJoCo mj_step.

    State is x,y,yaw,vx,vy,yaw_rate,steer. No contacts are predicted here.
    """
    x, y, yaw, vx, vy, omega, steer = state
    acc, steer = accelerations(yaw, (vx, vy, omega), steer, control, params)
    vx, vy, omega = (v+DT*a for v, a in zip((vx, vy, omega), acc))
    return (x+DT*vx, y+DT*vy, wrap(yaw+DT*omega), vx, vy, omega, steer)


def pulse(state, control, params):
    states = []
    for _ in range(SUBSTEPS):
        state = step(state, control, params)
        states.append(state)
    return states


def docked(state, goal):
    return (math.hypot(state[0]-goal[0], state[1]-goal[1]) < GOAL_TOLERANCE
            and abs(wrap(state[2]-goal[2])) < YAW_TOLERANCE
            and math.hypot(state[3], state[4]) < STOP_SPEED and abs(state[5]) < STOP_YAW)


class Site:
    """Seed-frozen generative policy v3, with no rejection based on any rollout."""
    def __init__(self, seed):
        self.seed = int(seed)
        rng = random.Random(self.seed ^ 0xC8050003)
        self.rotation = rng.randrange(4)*math.pi/2
        self.side = rng.choice((-1, 1))
        wall = rng.uniform(-.15, .15)
        service = rng.uniform(1.8, 2.6)
        short = rng.uniform(-.65, -.40)
        widths = (rng.uniform(1.22, 1.38), rng.uniform(.86, .94), rng.uniform(.46, .50))
        openings = [(service-widths[0]/2, service+widths[0]/2),
                    (short-widths[1]/2, short+widths[1]/2),
                    (-2.65-widths[2]/2, -2.65+widths[2]/2)]
        partition, cursor = [], -FIELD
        for lo, hi in sorted(openings)+[(FIELD, FIELD)]:
            partition.append((wall, (cursor+lo)/2, .12, (lo-cursor)/2))
            cursor = hi
        # Width admits the carrier sideways, but length cannot rotate into this
        # dogleg. A scout (or a circular/XY approximation) can take the shortcut.
        self.dogleg_width = rng.uniform(.84, .90)
        baffle_x = wall+.24+self.dogleg_width
        crates = [(baffle_x, short, .12, .92),
                  (rng.uniform(-1.25, -1.05), rng.uniform(-.15, .10), .23, .32),
                  (rng.uniform(-1.0, -.8), -2.8, .25, .25)]
        self.partition = [self.rect(r) for r in partition]
        self.crates = [self.rect(r) for r in crates]
        self.carrier_half = (rng.uniform(.82, .91), rng.uniform(.31, .35))
        self.carrier_mass = rng.uniform(7., 10.)
        self.carrier_start = self.point((rng.uniform(-2.6, -2.2), rng.uniform(-1.95, -1.3)))
        self.scout_start = self.point((-2.35, -2.55))
        self.goal = self.point((rng.uniform(2.2, 2.6), rng.uniform(-1.95, -1.3)))
        self.carrier_yaw = self.heading(math.pi/2)
        self.goal_yaw = self.heading(-math.pi/2)
        self.wall_x, self.wide = wall, openings[0]
        self.service_y = service
        self.look_seed = rng.randrange(2**62)
        self.dynamics = dict(speed=rng.uniform(.85, 1.05), speed_tau=self.carrier_mass/rng.uniform(20., 25.),
                             yaw_tau=rng.uniform(.16, .25), steer_tau=rng.uniform(.18, .28),
                             steer_max=rng.uniform(.76, .85), wheelbase=rng.uniform(.56, .62),
                             traction=rng.uniform(.10, .16), lateral_tau=rng.uniform(.10, .18))

    def point(self, p):
        x, y = p[0], p[1]*self.side
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return (c*x-s*y, s*x+c*y)

    def heading(self, yaw):
        return wrap(self.rotation+self.side*yaw)

    def rect(self, r):
        x, y = self.point(r[:2])
        return (x, y, r[2], r[3]) if round(self.rotation/(math.pi/2)) % 2 == 0 else (x, y, r[3], r[2])

    def boundary(self):
        t = .06
        return [(0., FIELD+t, FIELD+2*t, t), (0., -FIELD-t, FIELD+2*t, t),
                (FIELD+t, 0., t, FIELD+2*t), (-FIELD-t, 0., t, FIELD+2*t)]

    def obstacles(self):
        return self.partition+self.crates+self.boundary()

    def witness(self):
        """Reviewer geometry witness only: continuous centreline with 0.85 m turns."""
        # Constructed before running a controller; never used for acceptance or scoring.
        radius = .85
        # Inverse transform gives the actual jittered start/goal x coordinates.
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        sx = c*self.carrier_start[0]+s*self.carrier_start[1]
        gx = c*self.goal[0]+s*self.goal[1]
        sy = self.side*(-s*self.carrier_start[0]+c*self.carrier_start[1])
        gy = self.side*(-s*self.goal[0]+c*self.goal[1])
        points = []
        def line(a, b, yaw):
            n = max(1, math.ceil(math.dist(a, b)/.025))
            for k in range(n):
                p = tuple(a[j]+(b[j]-a[j])*k/n for j in (0, 1))
                points.append((*self.point(p), self.heading(yaw)))
        line((sx, sy), (sx, self.service_y-radius), math.pi/2)
        for k in range(65):
            a = math.pi-k/64*math.pi/2
            p = (sx+radius+radius*math.cos(a), self.service_y-radius+radius*math.sin(a))
            points.append((*self.point(p), self.heading(a-math.pi/2)))
        line((sx+radius, self.service_y), (gx-radius, self.service_y), 0.)
        for k in range(65):
            a = math.pi/2-k/64*math.pi/2
            p = (gx-radius+radius*math.cos(a), self.service_y-radius+radius*math.sin(a))
            points.append((*self.point(p), self.heading(a-math.pi/2)))
        line((gx, self.service_y-radius), (gx, gy), -math.pi/2)
        return points+[(*self.goal, self.goal_yaw)]
