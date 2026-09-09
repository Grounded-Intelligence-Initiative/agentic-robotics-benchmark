"""Hidden case families. Random parameters are generated in verifier memory only."""

FAMILIES = ("free_flight", "impulses", "zero_gravity", "elastic", "inelastic",
            "rest_and_lift", "many_impacts", "variable_steps")


def reset(config, position, velocity):
    return {"type": "reset", "config": config,
            "state": {"position": position, "velocity": velocity}}


def step(dt, jx=0, jy=0):
    return {"type": "step", "dt": dt, "impulse": [jx, jy]}


def make_case(rng, family, steps=40):
    g = rng.uniform(0.5, 20)
    plate = rng.uniform(-2, 2)
    radius = rng.uniform(0.01, 0.5)
    mass = rng.uniform(0.25, 5)
    e = rng.uniform(0.2, 0.85)
    x = rng.uniform(-5, 5)
    y = plate + radius + rng.uniform(0.2, 4)
    vx, vy = rng.uniform(-3, 3), rng.uniform(-3, 3)
    if family in ("free_flight", "impulses"):
        plate = None
        y = rng.uniform(-5, 15)
    elif family == "zero_gravity":
        g = 0
        vy = rng.uniform(-5, -0.5)
    elif family == "elastic":
        e = 1
    elif family == "rest_and_lift":
        e = 0
        y, vy = plate + radius, 0
    elif family == "many_impacts":
        g = 20
        y, vy = plate + radius + rng.uniform(0.001, 0.01), 0
        e = rng.choice([1, 0.5, 0.95])
    config = {"gravity": g, "mass": mass, "radius": radius,
              "restitution": e, "plate_y": plate}
    requests = [reset(config, [x, y], [vx, vy])]
    for t in range(steps):
        dt = rng.uniform(0.01, 0.4)
        jx = jy = 0
        if family in ("impulses", "variable_steps") and rng.random() < 0.5:
            jx, jy = rng.uniform(-5, 5), rng.uniform(-5, 5)
        if family == "rest_and_lift" and t in (3, 10, 25):
            jy = rng.choice([-1, 1]) * rng.uniform(0.5, 5)
        if family in ("variable_steps", "many_impacts"):
            dt = rng.choice([0, 0.001, 0.02, 0.7, 2])
        requests.append(step(dt, jx, jy))
    return requests


def edge_cases():
    base = {"gravity": 2, "mass": 1, "radius": 0.125,
            "restitution": 1, "plate_y": -0.125}
    return [
        [reset(base, [0, 1], [0, 0]), step(1), step(1), step(2)],
        [reset(base, [0, 0], [1, 0]), step(0, 0, -2), step(1), step(1)],
        [reset(dict(base, restitution=0), [0, 1], [1, 0]),
         step(2), step(2), step(0, 0, 2), step(0.5)],
        [reset(dict(base, restitution=0.5), [0, 0.01], [0, 0]),
         step(2), step(2), step(0, 0, 1), step(0.1)],
        [reset(dict(base, gravity=0), [0, 1], [0, -1]),
         step(1), step(1), step(0, 1, 2)],
        [reset(dict(base, plate_y=None, gravity=0, mass=2), [1, 2], [3, 4]),
         step(0, 2, -4), step(0.5), step(0.25, -2, 4)],
    ]


def suite(rng, count=8, steps=40):
    cases = [(name, make_case(rng, name, steps))
             for _ in range(count) for name in FAMILIES]
    # Shuffled family order prevents the submission from relying on a fixed curriculum.
    for i in range(len(cases)-1, 0, -1):
        j = rng.seed() % (i+1)
        cases[i], cases[j] = cases[j], cases[i]
    cases.extend(("edge_cases", case) for case in edge_cases())
    return cases
