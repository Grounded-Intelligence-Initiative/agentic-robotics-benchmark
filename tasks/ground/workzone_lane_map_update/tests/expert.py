"""Physical geometry for reverse curves and a temporary two-branch fork."""

from math import ceil, cos, pi


def smoothstep(t):
    return t * t * (3 - 2 * t)


def curve(knots, station):
    for (start, x0), (end, x1) in zip(knots, knots[1:]):
        if station <= end:
            t = (station - start) / (end - start)
            return x0 + (x1 - x0) * smoothstep(t)
    return knots[-1][1]


def pinches(rng, side):
    # Jersey barriers intrude from opposite sides on the approach and at the fork.
    return [(rng.uniform(.35, .39), rng.uniform(.060, .074), side, rng.uniform(.86, 1.04)),
            (rng.uniform(.74, .81), rng.uniform(.055, .071), -side, rng.uniform(.80, 1.04))]


def expert_site(prior, layout, side, rng):
    """Return the painted rails, physical edges and reference route for each signal state."""
    distance = layout["length"]
    lane_width = layout["lane_width"]
    nodes = prior["nodes"]
    fork = rng.uniform(.54, .60)
    common = [(0, 0), (.095, 0), (rng.uniform(.23, .27), rng.uniform(3.55, 4.25)),
              (rng.uniform(.32, .35), rng.uniform(4.55, 5.20)),
              (rng.uniform(.43, .47), rng.uniform(1.90, 2.55)),
              (fork, rng.uniform(3.35, 4.24))]
    path_knots = {
        "through_merge": [(fork + .14, rng.uniform(.55, 1.00)),
                          (rng.uniform(.82, .86), rng.uniform(1.70, 2.35)), (.96, 0), (1, 0)],
        "alternate_merge": [(fork + .12, rng.uniform(5.85, 6.50)),
                            (rng.uniform(.80, .84), rng.uniform(3.60, 4.18)),
                            (.96, abs(nodes["alternate_merge"][0])), (1, abs(nodes["alternate_merge"][0]))],
    }
    widths = [(0, lane_width), (.13, lane_width), (.34, rng.uniform(3.55, 3.90)),
              (.48, rng.uniform(3.36, 3.70)), (.68, rng.uniform(3.55, 4.10)),
              (.87, rng.uniform(3.38, 3.76)), (1, lane_width)]
    narrow = pinches(rng, side * rng.choice((-1, 1)))
    count = ceil(distance / .48)
    corridors = {}
    for name, branch in path_knots.items():
        # Use the same early blockage, then a distinct barrier span on each branch.
        stations = [narrow[0], narrow[1] if name == "through_merge" else
                    (narrow[1][0] - .038, narrow[1][1], -narrow[1][2], narrow[1][3])]
        paint = {"left": [], "right": []}
        edges = {"left": [], "right": []}
        barriers = {"left": [], "right": []}
        path = []
        for i in range(count + 1):
            frac = i / count
            y = frac * distance
            x = side * curve(common + branch, frac)
            width = curve(widths, frac)
            actual = []
            for key, sign in (("left", -1), ("right", 1)):
                offset = sign * width / 2
                paint[key].append((x + offset, y))
                intrude = 0
                has_barrier = False
                for middle, radius, which, depth in stations:
                    normalized = abs(frac - middle) / radius
                    if sign == which and normalized < 1:
                        intrude = depth * (1 + cos(pi * normalized)) / 2
                        has_barrier = True
                point = (x + offset - sign * intrude, y)
                edges[key].append(point)
                barriers[key].append(point if has_barrier else None)
                actual.append(point[0])
            path.append(((actual[0] + actual[1]) / 2, y))
        paths = [path[i] for i in range(0, len(path) - 1, 2)]
        corridors[name] = {"paint": paint, "edges": edges, "barriers": barriers,
                           "centerline": [*paths, nodes[name]]}

    # Early cones show the through closure. Follow the downstream geometry from the paint and barriers.
    cones = [(x + direction * lane_width / 2, frac * distance)
             for frac in (.12, .20, .27) for x in (side * curve(common, frac),) for direction in (-1, 1)]
    cones.extend((x, distance * .302) for x in (-1.35, -.45, .45, 1.35))
    return corridors, cones
