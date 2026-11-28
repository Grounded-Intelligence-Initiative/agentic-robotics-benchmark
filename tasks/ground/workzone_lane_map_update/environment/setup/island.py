"""A single physical traffic island upstream of a separately signalled exit."""

from math import ceil, cos, pi

from expert import curve
from geometry import cross_section


def separation_hole(a, b):
    """The island fills the gap between the two inner painted edges."""
    left, right = [], []
    for i, ((xa, y), (xb, _)) in enumerate(zip(a, b)):
        gap = xb - xa
        if i and (gap > 0) != (last_gap > 0):
            t = last_gap / (last_gap - gap)
            tip = (a[i-1][0] + t * (xa - a[i-1][0]), a[i-1][1] + t * (y - a[i-1][1]))
            left.append(tip)
            right.append(tip)
        if gap > 0:
            left.append((xa, y))
            right.append((xb, y))
        last_gap = gap
    return left + list(reversed(right[1:-1]))


def island_site(prior, layout, variant, side, open_side, rng):
    length = layout["length"]
    width = layout["lane_width"]
    spread = rng.uniform(2.55, 2.90)
    split = [(0, 0), (.12, 0), (rng.uniform(.34,.37), 1), (rng.uniform(.53,.56), 1),
             (.70, 0), (1, 0)]
    drift = [(0, 0), (.08, 0), (.22, rng.uniform(1.85,2.65)), (.32, rng.uniform(.3,.7)),
             (.45, rng.uniform(.85,1.45)), (.58, rng.uniform(-.15,.25)),
             (.70, rng.uniform(1.85,2.4)), (.83, rng.uniform(.5,1.35)), (.97, 0), (1, 0)]
    ripple = [(0,0), (.17,0), (.32,rng.uniform(-.07,.07)), (.52,rng.uniform(-.07,.07)),
              (.68,rng.uniform(-.14,.14)), (.87,0), (1,0)]
    widths = [(0,width), (.29,rng.uniform(3.50,3.85)), (.46,rng.uniform(3.50,3.85)),
              (.63,rng.uniform(3.55,3.9)), (1,width)]
    exit_shape = [(0,0), (.70,0), (.83,rng.uniform(.82,.98)), (.90,rng.uniform(.68,.79)), (.985,1), (1,1)]
    control = variant == "shoulder_work"
    opened = "alternate_merge" if variant == "changed_connection" else "through_merge"
    sides = ("left", "right")
    signs = {"left": -1, "right": 1}
    closed = next(key for key in sides if key != open_side)
    n = ceil(length / .46)
    stations = [i / n for i in range(n + 1)]
    pinches = {key: [(rng.uniform(.36,.40), rng.uniform(.036,.048), rng.uniform(.51,.67)),
                     (rng.uniform(.51,.54), rng.uniform(.036,.050), rng.uniform(.44,.65))] for key in sides}
    rails, references = {}, {}
    for dest in ("through_merge", "alternate_merge"):
        for arm in sides:
            rails[dest, arm] = edges = {"left": [], "right": []}
            references[dest, arm] = mid = []
            for t in stations:
                center = (side * (spread * curve(split,t) + curve(ripple,t) + (0 if control else curve(drift,t)))
                          + signs[arm] * spread * curve(split,t) + prior["nodes"][dest][0] * curve(exit_shape,t))
                half = curve(widths,t) / 2
                edges["left"].append((center - half, t * length))
                edges["right"].append((center + half, t * length))
                depth = max((d * (1 + cos(pi*(t-m)/r))/2 for m,r,d in pinches[arm] if abs(t-m) < r), default=0)
                mid.append((center - signs[arm]*depth/2, t * length))

    arm_edges = {key: rails[opened,key] for key in sides}
    inner = {arm: arm_edges[arm][sides[1-i]] for i,arm in enumerate(sides)}
    outer = {arm: arm_edges[arm][arm] for arm in sides}
    island = separation_hole(inner["left"], inner["right"])
    fixtures = [{"kind": "island", "poly": island}]
    for arm in sides:
        for m,r,d in pinches[arm]:
            run = [(x,y) for x,y in outer[arm] if abs(y/length-m) < r]
            edge = [(x-signs[arm]*d*(1+cos(pi*(y/length-m)/r))/2,y) for x,y in run]
            back = [(x+signs[arm]*.18,y) for x,y in reversed(run)]
            fixtures.append({"kind":"rail", "poly":edge+back})
    gate = rng.uniform(.422,.483)*length
    lo, hi = (cross_section(arm_edges[closed][key],gate) for key in sides)
    fixtures.append({"kind":"gate", "poly":[(lo-.16,gate-.18),(hi+.16,gate-.18),
                                           (hi+.16,gate+.18),(lo-.16,gate+.18)]})
    # The same portable barrier sits lengthwise along the island on the open side.
    line = [(cross_section(inner[open_side],y),y) for y in (gate-(hi-lo)/2, gate+(hi-lo)/2)]
    fixtures.append({"kind":"parallel_bar", "poly":[(x+signs[open_side]*.12,y) for x,y in line]
                     + [(x-signs[open_side]*.24,y) for x,y in reversed(line)]})
    geometry = {"outer":outer, "fixtures":fixtures, "gate_y":gate,
                "island_span":[cross_section(inner["left"],gate),cross_section(inner["right"],gate)],
                "open_side":open_side, "exit":opened}
    paths = {dest: [list(p) for p in references[dest,open_side][::2]] for dest in ("through_merge","alternate_merge")}
    for dest,path in paths.items():
        if path[-1][1] < length:
            path.append(prior["nodes"][dest])
    # Both exits share the entire island. Draw its rails once and the extra outlet only after separation.
    painted = [rails["through_merge",arm] for arm in sides]
    extra = rails["alternate_merge","left"]
    painted.append({key:[p for p in edge if p[1] >= .69*length] for key,edge in extra.items()})
    cones = [(cross_section(outer[arm],s*length)+signs[arm]*.34,s*length)
             for arm in sides for s in (.15,.20,.285,.385,.54)]
    return paths, geometry, painted, cones
