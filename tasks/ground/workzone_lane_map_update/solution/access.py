"""Directed transfers between a main road and a parallel frontage road during works."""

from math import ceil, pi, sin
from random import Random

from generate import hard_survey, lane
from geometry import cross_section


ROADS = ("main", "frontage")
MOVEMENTS = [(a, b) for a in ROADS for b in ROADS]


def road_strip(center, start, end, half_width):
    """Use the exact bends and interpolated end faces of the surveyed separator."""
    stations = [start, *(p[1] for p in center if start < p[1] < end), end]
    path = [(cross_section(center, y), y) for y in stations]
    return [(x-half_width, y) for x, y in path] + [(x+half_width, y) for x, y in reversed(path)]


def movement_line(origin, destination, roads, opening=None):
    if origin == destination:
        return [list(p) for p in roads[origin]]
    path = []
    for (xa, y), (xb, _) in zip(roads[origin], roads[destination]):
        t = min(1, max(0, .5 + (y-opening["station_m"]) / opening["transition_m"]))
        blend = t*t*(3-2*t)
        path.append([xa+(xb-xa)*blend, y])
    return path


def sample_station(rng, length, existing):
    """Choose a continuous station with space for the neighbouring divider gaps."""
    start, end = .15*length, .85*length
    intervals = []
    for station in sorted(existing):
        if station-22 > start:
            intervals.append((start, station-22))
        start = max(start, station+22)
    if start < end:
        intervals.append((start, end))
    offset = rng.random()*sum(b-a for a,b in intervals)
    for a,b in intervals:
        if offset <= b-a:
            return a+offset
        offset -= b-a
    raise ValueError("No feasible crossover station")


def opening_spec(rng, identifier, origin, station):
    return {"id": identifier, "from_road": origin,
            "to_road": next(r for r in ROADS if r != origin), "station_m": station,
            "half_gap_m": rng.uniform(7.0, 9.0), "transition_m": rng.uniform(21.0, 25.0)}


def lateral_alignment(rng, length, mode, amplitude):
    """Smooth lateral shifts around a bridge deck or alternating excavation fronts."""
    first, second = rng.uniform(.25,.40), rng.uniform(.60,.75)
    sign = rng.choice((-1,1))
    heights = {"bypass": (1, 1), "staggered": (1, -1), "return": (1, .12)}[mode]
    anchors = [(0,0),(.075,0),(first,sign*amplitude*heights[0]),
               (second,sign*amplitude*heights[1]),(.925,0),(1,0)]
    def offset(station):
        t = station/length
        for (a,xa),(b,xb) in zip(anchors,anchors[1:]):
            if t <= b:
                f = max(0,(t-a)/(b-a))
                return xa+(xb-xa)*f*f*(3-2*f)
        return 0
    return offset


def access_site(rng, variant, blocked_origin, *, works_rng=None, phased=False, paired_inventory=False):
    works_seed = rng.getrandbits(128)
    if works_rng is None:
        works_rng = Random(works_seed)
    length = rng.uniform(174, 204) if paired_inventory else rng.uniform(142, 171)
    side = rng.choice((-1, 1))
    separation = rng.uniform(4.8, 5.6)
    lane_width = rng.uniform(3.6, 3.95)
    bend = rng.uniform(-1.30, 1.30)
    wave = rng.uniform(-.8, .8)
    modes = ("bypass","staggered","return")
    if phased:
        original_bend = lateral_alignment(rng,length,rng.choice(modes),rng.uniform(1.8,3.1))
    count = ceil(length/2)
    center = []
    for i in range(count+1):
        t = i/count
        x = bend*sin(pi*t)**2 + wave*sin(2*pi*t)
        center.append((x + (original_bend(t*length) if phased else 0), t*length))
    surveyed_center = center
    road_offset = {"main": -side*separation/2, "frontage": side*separation/2}
    roads = {r: [(x+road_offset[r], y) for x, y in center] for r in ROADS}
    nodes = {}
    old_lanes = []
    for road in ROADS:
        first, last = roads[road][0], roads[road][-1]
        nodes.update({road+"_approach": [first[0], -12], road+"_in": list(first),
                      road+"_out": list(last), road+"_destination": [last[0], length+12]})
        for start, end in (("approach","in"), ("out","destination")):
            a, b = road+"_"+start, road+"_"+end
            old_lanes.append(lane(road+"_"+start+"_"+end, a, b, [nodes[a], nodes[b]], lane_width))

    historical = []
    for i, origin in enumerate(rng.sample(ROADS, k=2)):
        if paired_inventory:
            y = length*rng.uniform(*((.18,.34) if i==0 else (.66,.82)))
        else:
            y = sample_station(rng, length, [o["station_m"] for o in historical])
        historical.append(opening_spec(rng, f"prework_{i+1}", origin, y))
    normal = {o["from_road"]: o for o in historical}
    openings = historical.copy()
    active = dict(normal)
    if variant == "relocated_access":
        y = sample_station(works_rng, length, [o["station_m"] for o in historical])
        moved = works_rng.choice(ROADS)
        temporary = opening_spec(works_rng, "temporary", moved, y)
        openings.append(temporary)
        blocked_origin = next(r for r in ROADS if r != moved)
        active[moved] = temporary
    if variant != "shoulder_work":
        del active[blocked_origin]
    open_ids = {o["id"] for o in active.values()}
    shifted = phased and variant != "shoulder_work"
    current_roads = roads
    mode = None
    if phased:
        mode = works_rng.choice(modes)
        amount = works_rng.uniform(2.45,3.85) if shifted else works_rng.uniform(.08,.32)
        offset = lateral_alignment(works_rng,length,mode,amount)
        center = [(x+offset(y),y) for x,y in surveyed_center]
        current_roads = {r:[(x+road_offset[r],y) for x,y in center] for r in ROADS}

    legal = list(old_lanes)
    patch = {"close": [], "add": []}
    query_list = []
    for origin, destination in MOVEMENTS:
        old_opening = normal.get(origin) if origin != destination else None
        current = active.get(origin) if origin != destination else None
        edge_id = origin+"_to_"+destination
        old = lane(edge_id, origin+"_in", destination+"_out",
                   movement_line(origin, destination, roads, old_opening), lane_width)
        old_lanes.append(old)
        allowed = origin == destination or current is not None
        query_list.append({"name": edge_id, "from": origin+"_approach", "to": destination+"_destination",
                           "expect": "drivable" if allowed else "unreachable"})
        if current is old_opening and not shifted:
            legal.append(old)
        else:
            patch["close"].append(edge_id)
            if allowed:
                new = lane(edge_id+"_temporary", old["from"], old["to"],
                           movement_line(origin, destination, current_roads, current), lane_width)
                legal.append(new)
                patch["add"].append({key: val for key, val in new.items() if key != "width"})
        query_list.append({"name":"reverse_"+edge_id, "from":destination+"_destination",
                           "to":origin+"_approach", "expect":"unreachable"})

    # The raised divider and the blocks that fill closed gaps are actual solid footprints.
    fixtures = []
    display = []
    divider_width = .32
    def divider(start, end, kind):
        fixtures.append({"kind": kind, "poly": road_strip(center, start, end, divider_width)})
        pieces = max(1, ceil((end-start)/3.5))
        for i in range(pieces):
            a = start+(end-start)*i/pieces
            b = start+(end-start)*(i+1)/pieces
            display.append({"kind":kind, "poly":road_strip(center,a,b,divider_width)})

    previous = 0
    gates = []
    arrows = []
    cones = []
    for opening in sorted(openings, key=lambda o: o["station_m"]):
        y, h = opening["station_m"], opening["half_gap_m"]
        divider(previous, y-h, "separator")
        if opening["id"] not in open_ids:
            divider(y-h, y+h, "closure")
        else:
            # Stage the same temporary concrete units on the shoulder when this gap is in use.
            offset = side*(separation/2 + lane_width/2 + .60)
            shoulder = [(x+offset, station) for x, station in center]
            for i in range(4):
                poly = road_strip(shoulder, y-h+i*h/2, y-h+(i+1)*h/2, .26)
                display.append({"kind":"staged_barrier", "poly":poly})
                fixtures.append(display[-1])
        previous = y+h
        gates.append({**opening, "open": opening["id"] in open_ids})
        transfer = movement_line(opening["from_road"], opening["to_road"], current_roads, opening)
        for station in (y-4, y+4):
            x = cross_section(transfer, station)
            slope = (cross_section(transfer,station+1)-cross_section(transfer,station-1))/2
            arrows.append((x, station, slope))
        cones.extend((cross_section(center,station)+side*(separation/2+lane_width/2+.25),station)
                     for station in (y-h-2,y+h+2))
    divider(previous, length, "separator")
    outer = {key: [(x+sign*(separation/2+lane_width/2), y) for x, y in center]
             for key, sign in (("left",-1), ("right",1))}
    layout = {"length":length,"lane_width":lane_width,"road_half_width":9.2,"access_edges":outer,
              "access_arrows":arrows,
              "survey_stations":[.01+.10*i for i in range(9)],
              "oblique_stations":[-.045+.10*i for i in range(9)],
              "oblique_targets":[(cross_section(surveyed_center,t*length),t*length) for t in (.12+.10*i for i in range(9))]}
    if phased:
        layout.update(survey_alignment=surveyed_center,road_half_width=13.5)
    prior = {"units":"meters","nodes":nodes,"bounds":[[-13,-16],[13,length+16]],"lanes":old_lanes,
             "access_inventory":[{k:o[k] for k in ("id","from_road","to_road","station_m")} for o in historical]}
    survey = hard_survey(prior,layout,side,rng)
    # Fixed survey monuments and the camera itinerary predate construction.
    for fraction in (.15,.32,.49,.66,.83):
        for dx,dy in ((-6.5,0),(6.5,4)):
            prior["survey_marks"].append([dx,fraction*length+dy])
    layout["survey_marks"] = prior["survey_marks"]
    layout["roadview"] = True
    layout["visibility_targets"] = [{"name":o["id"],"center":cross_section(center,o["station_m"]),
                                    "station":o["station_m"],"half_gap":o["half_gap_m"],
                                    "open":o["open"],"arrows":arrows[2*i:2*i+2]}
                                   for i,o in enumerate(gates)]
    truth = {"variant":variant,"vehicle_width":1.8,"legal_lanes":legal,"queries":query_list,
             "access":{"outer":outer,"fixtures":fixtures,"divider":center,"gates":gates,"frontage_side":side}}
    if phased:
        truth["work_alignment"] = mode
    signals = [(road_offset[r], length*.96, True) for r in ROADS]
    return {"prior":prior, "truth":truth, "reference":patch,
            "render":{"layout":layout,"paths":(),"cones":cones,"side":side,"camera_jitter":rng.uniform(-.15,.15),
                      "signals":signals,"survey":survey,"fixtures":display}}
