"""drive_variants - graded stagings of the scenario kinds that practice never shows.

Grading stage only (the practice copy of drive_bridge has no such module). The grader gives
every hidden episode a `variant` and a number of `distractors`; drive_bridge's
ScenarioBuilder hands staging to `stage()` here.

Variants, by kind (`standard` is the practice staging):

    broken_down_vehicle   debris                   street furniture and rubbish in the lane
                          shoulder_pass            a crash closes the car's lane and those to its
                                                   left; the way past is the shoulder on the right
                          spinning_car             a car overtaking in the next lane spins out and
                                                   stops side-on in the car's lane
                          reversing_car            a car ahead stops and reverses toward the car
                                                   until it hits it, unless the car moves over
                          dropped_cargo            a truck ahead sheds its load onto the lane
    occluded_pedestrian   walking_in_lane          walks ahead in the car's lane, same direction
    red_light_runner      crowd_against_light      a group crosses against the light while the car
                                                   has green; some turn back halfway
    oncoming_left_turn    overtaking_oncoming      an oncoming car overtakes a cyclist along the
                                                   centre line, a metre into the car's lane

Distractors need no reaction: a car parked on the shoulder, people standing on the
pavement, street furniture at the kerb, cones in the oncoming lane well before the hazard.
Hitting one counts like any other collision; braking or swerving for one only costs pace.

The variant table and `graded_specs` are plain Python (the grader imports them); everything that
touches CARLA runs inside the bridge.
"""

VARIANTS = {
    "broken_down_vehicle": ("debris", "reversing_car", "dropped_cargo", "shoulder_pass",
                            "spinning_car"),
    "occluded_pedestrian": ("walking_in_lane",),
    "red_light_runner": ("crowd_against_light",),
    "oncoming_left_turn": ("overtaking_oncoming",),
}
#: The graded episodes, by variant: only the shifted stagings, three of each.
GRADED_MIX = tuple((variant, 3) for variant in (
    "debris", "walking_in_lane", "shoulder_pass", "spinning_car", "overtaking_oncoming",
    "reversing_car", "dropped_cargo", "crowd_against_light"))
MAX_DISTRACTORS = 2

STATIC_KINDS = ("construction_zone", "broken_down_vehicle", "occluded_pedestrian")
PROPS_DEBRIS = ("static.prop.trashcan01", "static.prop.trashcan03", "static.prop.box01",
                "static.prop.box03", "static.prop.shoppingcart", "static.prop.garbage01",
                "static.prop.plasticchair", "static.prop.haybale", "static.prop.clothcontainer")
PROPS_KERB = ("static.prop.mailbox", "static.prop.trashcan02", "static.prop.bench01",
              "static.prop.vendingmachine", "static.prop.plantpot01")
PROPS_CARGO = ("static.prop.box01", "static.prop.box02", "static.prop.box03")  # barrels roll on
BIKES = ("vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets")
TRUCKS = ("vehicle.carlamotors.carlacola", "vehicle.carlamotors.european_hgv",
          "vehicle.mercedes.sprinter")


#: Variants that need a road some towns lack, and the towns that have it (Town10HD's
#: shoulders are 2 m wide, too narrow to pass a crash on; Town04's run along highway rails
#: that leave less room than the lane says).
TOWNS_FOR = {"shoulder_pass": ("Town05",)}


def graded_specs(rng, towns):
    """The hidden episode specs: GRADED_MIX over `towns` (cycled; a variant listed in
    TOWNS_FOR cycles over the ones it allows), each with a fresh 62-bit seed and a distractor
    count from `rng`, grouped by town so each town loads once."""
    kind_of = {v: kind for kind, variants in VARIANTS.items() for v in variants}
    specs, index = [], 0
    for variant, count in GRADED_MIX:
        allowed = [t for t in towns if t in TOWNS_FOR.get(variant, towns)] or towns
        for k in range(count):
            seed = int(rng.random() * (1 << 31)) * (1 << 31) + int(rng.random() * (1 << 31))
            town = towns[index % len(towns)] if variant not in TOWNS_FOR else allowed[k % len(allowed)]
            specs.append({"town": town, "kind": kind_of[variant],
                          "variant": variant, "seed": seed,
                          "distractors": int(rng.random() * (MAX_DISTRACTORS + 1))})
            index += 1
    specs.sort(key=lambda s: towns.index(s["town"]))
    return specs


def junction_wanted(variant, default):
    """The junction the route has to take: only crowd_against_light (straight across) needs
    one."""
    return default if variant == "crowd_against_light" else None


def route(b, want):
    """A route laid out around a spot the variant needs, or None for the builder's own.
    overtaking_oncoming needs an oncoming lane right beside the car's (which most of Town04,
    divided highways, lacks) and shoulder_pass a shoulder or parking lane on the right: pick
    such a spot, walk back to the start and drive on past it to the route's length."""
    fits = SPOT_TESTS.get(b.variant)
    if fits is None:
        return None
    rng, RoadOption = b.rng, b.RoadOption
    cache = getattr(b.b, "_variant_spots", None)        # per loaded map and variant
    if cache is None or cache[0] is not b.map:
        cache = (b.map, {})
        b.b._variant_spots = cache
    if b.variant not in cache[1]:
        cache[1][b.variant] = [wp for wp in b.map.generate_waypoints(4.0) if fits(b, wp)]
    spots = cache[1][b.variant]
    if not spots:
        raise RuntimeError("no road for {} in this town".format(b.variant))
    hazard = rng.choice(spots)
    back, wp, run = [], hazard, 0.0
    target = rng.uniform(*b.HAZARD_M)
    while run < target:
        prev = wp.previous(2.0)
        if not prev:
            raise RuntimeError("the road ends behind the closure")
        nxt = rng.choice(prev)
        run += nxt.transform.location.distance(wp.transform.location)
        back.append(nxt)
        wp = nxt
    wps = list(reversed(back)) + [hazard]
    run_in = 0.0
    for prev, cur in zip(wps, wps[1:]):
        if cur.is_junction:
            if run_in < 25.0:
                raise RuntimeError("the route would start at a junction")
            break
        run_in += cur.transform.location.distance(prev.transform.location)
    total = run
    wp = hazard
    while total < b.ROUTE_M:
        nexts = wp.next(2.0)
        if not nexts:
            raise RuntimeError("the road ends ahead of the closure")
        if len(nexts) > 1:
            turns = [(b._turn(n), n) for n in nexts]
            nexts = [n for a, n in turns if abs(a) < 30] or [n for _, n in turns]
        nxt = rng.choice(nexts)
        total += nxt.transform.location.distance(wp.transform.location)
        wps.append(nxt)
        wp = nxt
    opts = [RoadOption.LANEFOLLOW]
    for prev, cur in zip(wps, wps[1:]):
        if cur.is_junction and not prev.is_junction:
            angle = b._turn(cur)
            opts.append(RoadOption.LEFT if angle < -45 else
                        RoadOption.RIGHT if angle > 45 else RoadOption.STRAIGHT)
        elif cur.is_junction:
            opts.append(opts[-1])
        else:
            opts.append(RoadOption.LANEFOLLOW)
    b.planned_index = len(back)
    return list(zip(wps, opts)), None


def _two_lane_straight(b, wp):
    """The car's lane is the one next to the centre line (the oncoming lane right beside it),
    with one or two lanes each way, unchanged and fairly straight from 30 m before to 40 m
    after, with no junction on that stretch."""
    same, opposite = _carriageway(b, wp)
    if opposite is None or same[-1].lane_id != wp.lane_id or len(same) > 2 or wp.is_junction:
        return False
    yaw = wp.transform.rotation.yaw
    probes = [w for d in (10.0, 20.0, 30.0) for w in wp.previous(d)[:1]] + \
        [w for d in (10.0, 20.0, 30.0, 40.0) for w in wp.next(d)[:1]]
    if len(probes) != 7:
        return False
    for probe in probes:
        lanes, opp = _carriageway(b, probe)
        turn = abs((probe.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0)
        if opp is None or len(lanes) != len(same) or probe.is_junction or turn > 20.0:
            return False
    return True


def _shoulder_fits(b, wp):
    """The car's lane is the rightmost driving lane, with a shoulder or parking lane at least
    2.4 m wide to its right from 30 m before to 25 m after, and no junction within 12 m."""
    kinds = (b.carla.LaneType.Shoulder, b.carla.LaneType.Parking)
    if wp.is_junction or any(w.is_junction for w in wp.previous(12.0) + wp.next(12.0)):
        return False
    probes = [wp] + wp.previous(30.0)[:1] + wp.previous(15.0)[:1] + wp.next(12.0)[:1] + \
        wp.next(25.0)[:1]
    if len(probes) != 5:
        return False
    for w in probes:
        right = w.get_right_lane()
        if w.is_junction or right is None or right.lane_type not in kinds or right.lane_width < 2.4:
            return False
    return True


SPOT_TESTS = {"overtaking_oncoming": _two_lane_straight, "shoulder_pass": _shoulder_fits}


# --------------------------------------------------------------------------- staging
def stage(b, route, junction):
    """Stage `b.kind` as `b.variant` on `route`, then the distractors before the hazard.
    Raises the bridge's redraw (via b._at_distance or RuntimeError) when a draw does not fit."""
    variant = b.variant
    if variant in SPOT_TESTS:
        index = b.planned_index
        globals()["_" + variant](b, route, index, route[index][0])
    elif b.kind in STATIC_KINDS:
        index, hwp = _hazard_waypoint(b, route, passable=_needs_pass(b.kind, variant))
        if variant == "standard":
            getattr(b, "_stage_" + b.kind)(hwp)
        else:
            globals()["_" + variant](b, route, index, hwp)
    else:
        index, entry = junction
        if variant == "standard":
            getattr(b, "_stage_" + b.kind)(route, index, entry)
        else:
            globals()["_" + variant](b, route, index, entry)
    # a spin-out is staged on a clear stretch: nothing else on or beside the road
    b.quiet = variant == "spinning_car"
    if not b.quiet:
        _distractors(b, route, index)
    return index


def _needs_pass(kind, variant):
    return variant in ("debris", "dropped_cargo", "walking_in_lane", "reversing_car",
                       "spinning_car")


def _hazard_waypoint(b, route, passable):
    index, hwp = b._at_distance(route, b.rng.uniform(*b.HAZARD_M))
    if hwp.is_junction or any(w.is_junction for w in hwp.previous(12.0) + hwp.next(12.0)):
        raise RuntimeError("hazard too close to a junction")
    if passable and not b._passable(hwp):
        raise RuntimeError("no passing lane")
    return index, hwp


def _ground(b, loc):
    """The road surface's height under a point (scripted actors sit on it, not above it)."""
    here = b.map.get_waypoint(loc, project_to_road=True, lane_type=b.carla.LaneType.Any)
    return here.transform.location.z if here is not None else loc.z


def _available(b, names):
    lib = b.world.get_blueprint_library()
    found = [n for n in names if len(lib.filter(n)) > 0]
    if not found:
        raise RuntimeError("none of {} exists in this CARLA".format(names[:3]))
    return found


def _yawed(b, tf, degrees):
    return b.carla.Transform(tf.location, b.carla.Rotation(pitch=tf.rotation.pitch,
                                                           yaw=tf.rotation.yaw + degrees,
                                                           roll=tf.rotation.roll))


def _car_blueprints(b):
    lib = b.world.get_blueprint_library().filter("vehicle.*")
    return [bp.id for bp in lib if int(bp.get_attribute("number_of_wheels")) == 4]


def _small_cars(b):
    """Four-wheelers no longer than an ordinary car (no vans, buses or trucks)."""
    lib = b.world.get_blueprint_library().filter("vehicle.*")
    big = ("sprinter", "fusorosa", "carlacola", "european_hgv", "firetruck", "ambulance", "t2",
           "cybertruck", "patrol", "charger_police")
    return [bp.id for bp in lib if int(bp.get_attribute("number_of_wheels")) == 4
            and not any(k in bp.id for k in big)]


def _walker(b, transform):
    walkers = b.world.get_blueprint_library().filter("walker.pedestrian.*")
    bp = b.rng.choice(walkers)
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    walker = b.world.spawn_actor(bp, transform)
    b.b.actors.append(walker)
    return walker


# ---- construction_zone, broken_down_vehicle --------------------------------------
def _debris(b, route, index, wp):
    rng = b.rng
    props = _available(b, PROPS_DEBRIS)
    for _ in range(rng.randint(2, 4)):
        b.hazards.append(b.b._spawn(rng.choice(props),
                                    _yawed(b, b._offset(wp, rng.uniform(0.0, 4.0),
                                                        rng.uniform(-1.2, 1.2), 0.1),
                                           rng.uniform(0, 360))))
    b.hazards.append(b.b._spawn(rng.choice(props), b._offset(wp, 2.0, 0.0, 0.1)))


def _carriageway(b, wp):
    """The lanes of the car's direction at `wp` (driving lanes with its lane_id sign, from the
    rightmost) and the first oncoming driving lane directly left of them, or None."""
    Driving = b.carla.LaneType.Driving
    same, w = [wp], wp
    while True:
        right = w.get_right_lane()
        if right is None or right.lane_type != Driving or (right.lane_id > 0) != (wp.lane_id > 0):
            break
        same.insert(0, right)
        w = right
    w = wp
    while True:
        left = w.get_left_lane()
        if left is None or left.lane_type != Driving:
            return same, None
        if (left.lane_id > 0) != (wp.lane_id > 0):
            return same, left
        same.append(left)
        w = left


# ---- occluded_pedestrian ------------------------------------------------------------
def _walking_in_lane(b, route, index, wp):
    """A pedestrian walking ahead in the car's lane, in the car's direction."""
    carla, rng = b.carla, b.rng
    walker = _walker(b, b._offset(wp, 0.0, rng.uniform(-0.3, 0.8), 1.0))
    b.hazards.append(walker)
    speed = rng.uniform(0.9, 1.6)
    plan = [w for w, _ in route[index:index + 40]]

    def hook():
        here = walker.get_location()
        nearest = min(plan, key=lambda w: w.transform.location.distance(here)) if plan else wp
        f = nearest.transform.get_forward_vector()
        walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(f.x, f.y, 0.0),
                                                 speed=speed))
    b.b.scripted.append(hook)


def _shoulder_pass(b, route, index, wp):
    """A crash: two cars stopped askew in the car's lane, every lane to its left closed at
    the same place (barriers), a warning sign before it. The shoulder (or parking lane) on
    the right is the only way past."""
    carla, rng = b.carla, b.rng
    Driving = carla.LaneType.Driving
    cars = _small_cars(b)
    # askew, but kept to the lane's left side so the shoulder stays wide enough to pass
    wrecks = [
        b.b._spawn(rng.choice(cars), _yawed(
            b, b._offset(wp, 0.0, rng.uniform(-0.9, -0.7), 0.3), rng.uniform(5.0, 15.0))),
        b.b._spawn(rng.choice(cars), _yawed(
            b, b._offset(wp, rng.uniform(5.0, 6.5), rng.uniform(-0.9, -0.7), 0.3), rng.uniform(-15.0, -5.0)))]
    for wreck in wrecks:        # on their wheels (physics on), handbrake set
        wreck.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
    b.hazards.extend(wrecks)
    w = wp
    for _ in range(4):
        # walk outward from the car's side, across the centre line if need be
        nxt = w.get_left_lane() if (w.lane_id > 0) == (wp.lane_id > 0) else w.get_right_lane()
        if nxt is None or nxt.lane_type != Driving:
            break
        for lateral in (-nxt.lane_width / 4.0, nxt.lane_width / 4.0):
            b.hazards.append(b.b._spawn("static.prop.streetbarrier",
                                        b._offset(nxt, 0.0, lateral, 0.1)))
        w = nxt
    sign = _available(b, ("static.prop.warningaccident", "static.prop.trafficwarning"))
    b.hazards.append(b.b._spawn(rng.choice(sign), b._offset(wp, -8.0, 0.0, 0.1)))


def _lead(b, route, wp, blueprints, speed_kmh):
    """A vehicle 25 m ahead in the car's lane, held until the car comes within 18 m, then
    driven along the route (and on past its end) by a BasicAgent."""
    from agents.navigation.basic_agent import BasicAgent
    carla, rng = b.carla, b.rng
    start_index, start_wp = b._at_distance(route, 25.0)
    tf = start_wp.transform
    car = b.b._spawn(rng.choice(blueprints),
                     carla.Transform(tf.location + carla.Location(z=0.5), tf.rotation))
    b.hazards.append(car)
    agent = BasicAgent(car, target_speed=speed_kmh,
                       opt_dict={"ignore_traffic_lights": True, "ignore_vehicles": True})
    beyond, last = [], route[-1][0]
    for _ in range(40):
        nxt = last.next(2.0)
        if not nxt:
            break
        last = nxt[0]
        beyond.append((last, b.RoadOption.LANEFOLLOW))
    agent.set_global_plan(route[start_index:] + beyond, stop_waypoint_creation=True,
                          clean_queue=True)
    return car, agent


def _reversing_car(b, route, index, wp):
    """A car ahead in the car's lane stops at the hazard point and, once the car has stopped
    behind it, reverses toward it slowly (reversing lights on) for up to 35 m without looking:
    a driver backing up to a space they passed. Waiting behind it ends in a collision; the way
    out is the lane to the left."""
    carla, rng = b.carla, b.rng
    car, agent = _lead(b, route, wp, _small_cars(b), rng.uniform(22.0, 30.0))
    stop_at = wp.transform.location
    back_m = 35.0
    speed_back = rng.uniform(0.8, 1.2)
    dt = b.world.get_settings().fixed_delta_seconds or 0.05
    lights = carla.VehicleLightState
    state = {"go": False, "phase": "drive", "since": 0.0, "from": None}

    def left_free():
        ego = b.b.ego.get_location()
        left = b.map.get_waypoint(ego).get_left_lane()
        if left is None:
            return True
        for other in b.world.get_actors().filter("vehicle.*"):
            if other.id in (car.id, b.b.ego.id):
                continue
            loc = other.get_location()
            if loc.distance(ego) > 60.0:
                continue
            f = b.b.ego.get_transform().get_forward_vector()
            if (loc.x - ego.x) * f.x + (loc.y - ego.y) * f.y > 30.0:
                continue
            here = b.map.get_waypoint(loc)
            if here is not None and here.road_id == left.road_id and here.lane_id == left.lane_id:
                return False
        return True

    def hook():
        now = b.b.tracker.time
        if not state["go"]:
            if car.get_location().distance(b.b.ego.get_location()) < 18.0:
                state["go"] = True
            else:
                car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
                return
        phase = state["phase"]
        if phase == "drive" and car.get_location().distance(stop_at) < 2.5:
            state.update(phase="stop", since=now)
        if phase == "stop":
            car.apply_control(carla.VehicleControl(brake=1.0))
            # it backs up only once the car has stopped behind it (for a second) and has a
            # way round (the lane to its left free of traffic from 60 m behind the car to
            # 30 m ahead); until then it is just a stopped car
            if b.b._speed() < 0.2:
                state["ego_still"] = state.get("ego_still", 0.0) + dt
            else:
                state["ego_still"] = 0.0
            ready = state["ego_still"] >= 1.0 and left_free()
            if now - state["since"] > 1.0 and ready:
                # backed kinematically along its lane: a reverse gear under physics is not
                # dependable in CARLA
                tf = car.get_transform()
                state.update(phase="reverse", since=now, moved=0.0, tf=tf)
                car.set_simulate_physics(False)
                car.set_light_state(lights(lights.Reverse))
            return
        if phase == "reverse":
            state["moved"] = min(back_m, state["moved"] + speed_back * dt)
            tf = state["tf"]
            f = tf.get_forward_vector()
            car.set_transform(carla.Transform(tf.location - carla.Location(
                f.x * state["moved"], f.y * state["moved"], 0.0), tf.rotation))
            if state["moved"] >= back_m:
                state.update(phase="pause", since=now)
                car.set_simulate_physics(True)
                car.set_light_state(lights(lights.NONE))
            return
        if phase == "drive":
            car.apply_control(agent.run_step())
        else:
            car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))   # parked
    b.b.scripted.append(hook)


def _dropped_cargo(b, route, index, wp):
    """A truck ahead in the car's lane sheds two to four pieces of its load as it passes the
    hazard point; they fall from its tail, tumble and come to rest in the lane."""
    carla, rng = b.carla, b.rng
    truck, agent = _lead(b, route, wp, _available(b, TRUCKS), rng.uniform(25.0, 32.0))
    cargo = _available(b, PROPS_CARGO)
    drop_at = wp.transform.location
    pieces = rng.randint(2, 4)
    state = {"go": False, "dropped": False}

    def drop():
        tf = truck.get_transform()
        f, r = tf.get_forward_vector(), tf.get_right_vector()
        back = truck.bounding_box.extent.x + 0.9
        v = truck.get_velocity()
        lib = b.world.get_blueprint_library()
        for k in range(pieces):
            lateral = rng.uniform(-0.2, 0.7)       # off the tail's kerb side: the next lane stays open
            loc = tf.location + carla.Location(-f.x * back + r.x * lateral, -f.y * back + r.y * lateral,
                                               1.4 + 0.5 * k)
            prop = b.world.try_spawn_actor(lib.find(rng.choice(cargo)),
                                           carla.Transform(loc, carla.Rotation(yaw=rng.uniform(0, 360))))
            if prop is None:
                continue
            prop.set_simulate_physics(True)
            keep, side = rng.uniform(0.1, 0.25), rng.uniform(0.0, 0.6)
            prop.set_target_velocity(carla.Vector3D(v.x * keep + r.x * side, v.y * keep + r.y * side, 0.0))
            prop.set_target_angular_velocity(carla.Vector3D(rng.uniform(-90, 90), rng.uniform(-90, 90),
                                                            rng.uniform(-90, 90)))
            b.b.actors.append(prop)
            b.b.tracker.hazard_ids.add(prop.id)     # hitting the load counts like the hazard

    def hook():
        if not state["go"]:
            if truck.get_location().distance(b.b.ego.get_location()) < 30.0:
                state["go"] = True
            else:
                truck.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
                return
        if not state["dropped"] and truck.get_location().distance(drop_at) < 3.0:
            state["dropped"] = True
            drop()
        truck.apply_control(agent.run_step())
    b.b.scripted.append(hook)


def _crowd_against_light(b, route, index, entry):
    """The car has a green light straight across a junction; four to eight people step off
    the kerb on the car's right into the crossing just past the stop line, against their
    light, a little apart. One or two of them stop in the car's lane and turn back."""
    carla, rng = b.carla, b.rng
    lights = b.world.get_traffic_lights_from_waypoint(entry, 40.0)
    if not lights:
        raise RuntimeError("no traffic light at this junction")
    for light in lights:
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(120.0)
    kerb = entry.lane_width / 2.0
    right = entry.transform.get_right_vector()
    toward = carla.Vector3D(-right.x, -right.y, 0.0)
    back = carla.Vector3D(right.x, right.y, 0.0)
    crossing = 4.0 * entry.lane_width + 4.0
    people = []
    for k in range(rng.randint(4, 8)):
        spot = b._offset(entry, rng.uniform(1.5, 4.5), kerb + rng.uniform(1.8, 3.5), 1.0)
        try:
            people.append({"walker": _walker(b, spot), "delay": rng.uniform(0.0, 2.5),
                           "speed": rng.uniform(1.1, 1.7), "turn": False, "start": spot.location})
        except RuntimeError:
            continue
    if len(people) < 3:
        raise RuntimeError("no room on the pavement for the group")
    for p in rng.sample(people, rng.randint(1, 2)):
        p["turn"] = True
        p["stop_at"] = kerb + rng.uniform(0.5, 2.0)     # metres in from the kerb: the car's lane
    b.hazards.extend(p["walker"] for p in people)
    state = {"t0": None}

    def hook():
        ego = b.b.ego.get_location()
        if state["t0"] is None:
            if ego.distance(entry.transform.location) < b.b._speed() * 3.0 + 12.0:
                state["t0"] = b.b.tracker.time
            return
        elapsed = b.b.tracker.time - state["t0"]
        for p in people:
            w = p["walker"]
            if elapsed < p["delay"]:
                continue
            moved = w.get_location().distance(p["start"])
            if p["turn"]:
                phase = p.setdefault("phase", "out")
                if phase == "out" and moved >= p["stop_at"]:
                    p.update(phase="wait", since=elapsed)
                if phase == "wait" and elapsed - p["since"] > 1.5:
                    p["phase"] = "back"
                if p["phase"] == "back" and moved < 0.5:
                    p["phase"] = "done"
                direction, speed = {"out": (toward, p["speed"]), "wait": (toward, 0.0),
                                    "back": (back, p["speed"]), "done": (back, 0.0)}[p["phase"]]
            else:
                direction, speed = toward, (p["speed"] if moved < crossing else 0.0)
            w.apply_control(carla.WalkerControl(direction=direction, speed=speed))
    b.b.scripted.append(hook)


def _overtaking_oncoming(b, route, index, wp):
    """An oncoming car pulls out to overtake a cyclist riding near its own kerb and comes along
    the centre line, about a metre into the car's lane, until it is clear of the cyclist.
    Both are moved kinematically along the oncoming lane. Stopping does not clear it: the
    car has to move over to its kerb side until the overtaker is back in its own lane."""
    carla, rng = b.carla, b.rng
    same, opposite = _carriageway(b, wp)
    dt = b.world.get_settings().fixed_delta_seconds or 0.05

    def ahead_of_car(lane_wp, metres):
        # the oncoming lane runs toward the car, so what lies ahead of the car is its `previous`
        out = lane_wp.previous(metres)
        if not out:
            raise RuntimeError("the oncoming lane ends")
        return out[0]

    def path(start, length):
        """Transforms every metre along the oncoming lane, in its direction of travel."""
        points, w = [start.transform], start
        for _ in range(int(length)):
            nxt = w.next(1.0)
            if not nxt:
                break
            w = nxt[0]
            points.append(w.transform)
        return points

    bike_wp = ahead_of_car(opposite, rng.uniform(18.0, 26.0))
    car_wp = ahead_of_car(bike_wp, rng.uniform(14.0, 18.0))
    bike = b.b._spawn(rng.choice(_available(b, BIKES)), carla.Transform(
        bike_wp.transform.location, bike_wp.transform.rotation), physics=False)
    car = b.b._spawn(rng.choice(_car_blueprints(b)), carla.Transform(
        car_wp.transform.location, car_wp.transform.rotation), physics=False)
    b.hazards.extend([bike, car])
    bike_path, car_path = path(bike_wp, 160), path(car_wp, 160)
    # where the bike starts, in the overtaker's path metres
    lead = bike_wp.transform.location.distance(car_wp.transform.location)
    clearance = car.bounding_box.extent.x + bike.bounding_box.extent.x
    intrusion = rng.uniform(0.8, 1.1)
    shift = opposite.lane_width / 2.0 + intrusion - car.bounding_box.extent.y
    v_bike, v_car = rng.uniform(3.0, 4.5), rng.uniform(9.0, 11.0)
    kerb_side = -(opposite.lane_width / 2.0 - 0.8)     # the cyclist keeps to its kerb
    release = rng.uniform(55.0, 70.0)
    state = {"go": False, "u": 0.0, "w": 0.0, "side": 0.0, "passed": False}

    def place(actor, points, u, side):
        i = min(int(u), len(points) - 2)
        a, c = points[i], points[i + 1]
        k = u - i
        loc = carla.Location(a.location.x + (c.location.x - a.location.x) * k,
                             a.location.y + (c.location.y - a.location.y) * k,
                             a.location.z)
        r = a.get_right_vector()
        loc += carla.Location(-r.x * side, -r.y * side, 0.0)     # toward the car's lane
        loc.z = _ground(b, loc)
        actor.set_transform(carla.Transform(loc, a.rotation))

    def hook():
        if not state["go"]:
            if b.b.ego.get_location().distance(car.get_location()) < release:
                state["go"] = True
            else:
                return
        state["u"] = min(state["u"] + v_car * dt, len(car_path) - 2)
        state["w"] = min(state["w"] + v_bike * dt, len(bike_path) - 2)
        # out once close behind the bike, back in only when clear past its front, never
        # sideways faster than a lane change: the two never overlap
        behind = lead + state["w"] - state["u"] - clearance
        if not state["passed"] and state["u"] - (lead + state["w"]) > clearance + 6.0:
            state["passed"] = True
        want = shift if (behind < 12.0 and not state["passed"]) else 0.0
        step = 1.2 * dt
        state["side"] += max(-step, min(step, want - state["side"]))
        place(bike, bike_path, state["w"], kerb_side)
        place(car, car_path, state["u"], state["side"])
    place(bike, bike_path, 0.0, kerb_side)
    b.b.scripted.append(hook)


def _spinning_car(b, route, index, wp):
    """A car in the lane to the left, overtaking the car, loses control at the hazard point:
    it spins through 180-270 degrees while sliding across into the car's lane and stops
    there, side-on. Moved kinematically along the left lane, then across."""
    carla, rng = b.carla, b.rng
    left = wp.get_left_lane()
    if left is None or left.lane_type != carla.LaneType.Driving or \
            (left.lane_id > 0) != (wp.lane_id > 0):
        raise RuntimeError("no lane to the left going the same way")
    if any(w.is_junction for d in range(5, 40, 5) for w in wp.next(float(d))[:1]):
        raise RuntimeError("a junction where the car would have to get round the wreck")
    dt = b.world.get_settings().fixed_delta_seconds or 0.05
    back = left.previous(30.0)
    if not back:
        raise RuntimeError("the left lane ends behind the hazard")
    points, w = [back[0].transform], back[0]
    for _ in range(70):
        nxt = w.next(1.0)
        if not nxt:
            break
        w = nxt[0]
        points.append(w.transform)
    if len(points) < 50:
        raise RuntimeError("the left lane is too short")
    car = b.b._spawn(rng.choice(_small_cars(b)), carla.Transform(
        points[0].location, points[0].rotation), physics=False)
    b.hazards.append(car)
    # from the left lane's centre to a metre right of the car's lane centre: side-on there it
    # closes the car's lane and leaves the left lane open to go round
    across = left.transform.location.distance(wp.transform.location) + 1.0
    spin = rng.choice((1.0, -1.0)) * rng.uniform(180.0, 270.0)
    v_car = rng.uniform(9.0, 11.0)
    slide_m, slide_s = rng.uniform(9.0, 13.0), rng.uniform(1.6, 2.2)
    state = {"go": False, "u": 0.0, "t": None}

    def pose(u, side, yaw_extra):
        i = min(int(u), len(points) - 2)
        a, c = points[i], points[i + 1]
        k = u - i
        loc = carla.Location(a.location.x + (c.location.x - a.location.x) * k,
                             a.location.y + (c.location.y - a.location.y) * k, a.location.z)
        r = a.get_right_vector()
        loc += carla.Location(r.x * side, r.y * side, 0.0)         # toward the car's lane
        loc.z = _ground(b, loc)
        rot = carla.Rotation(pitch=a.rotation.pitch, yaw=a.rotation.yaw + yaw_extra, roll=0.0)
        return carla.Transform(loc, rot)

    def hook():
        ego = b.b.ego.get_location()
        if not state["go"]:
            if car.get_location().distance(ego) < 20.0 or b.b.tracker.time > 6.0:
                state["go"] = True
            else:
                return
        if state["t"] is None:
            state["u"] = min(state["u"] + v_car * dt, len(points) - 2)
            if state["u"] >= 30.0:                      # the hazard point
                state["t"] = 0.0
            car.set_transform(pose(state["u"], 0.0, 0.0))
            return
        if state["t"] >= slide_s:
            return                                      # at rest, under physics
        state["t"] = min(slide_s, state["t"] + dt)
        f = state["t"] / slide_s
        ease = 1.0 - (1.0 - f) ** 2                     # decelerating slide
        car.set_transform(pose(30.0 + slide_m * ease, across * ease, spin * ease))
        if state["t"] >= slide_s:
            car.set_simulate_physics(True)              # let it settle on its wheels
            car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
    b.b.scripted.append(hook)


def _straight(route, i, reach=8, max_turn_deg=5.0):
    """Whether the route runs straight around waypoint i: a parked car or a prop set beside
    a bend would stick out into the lane, and a distractor must need no reaction."""
    lo, hi = max(0, i - reach), min(len(route) - 1, i + reach)
    yaws = [route[k][0].transform.rotation.yaw for k in (lo, i, hi)]
    turn = max(abs((a - yaws[1] + 180.0) % 360.0 - 180.0) for a in (yaws[0], yaws[2]))
    return turn <= max_turn_deg


def _distractors(b, route, hazard_index):
    """Things beside the route before the hazard that need no reaction."""
    rng = b.rng
    run, spots = 0.0, []
    for i in range(1, hazard_index):
        run += route[i][0].transform.location.distance(route[i - 1][0].transform.location)
        if 20.0 <= run and i < hazard_index - 12 and not route[i][0].is_junction and \
                _straight(route, i):
            spots.append(route[i][0])
    if not spots:
        return
    kinds = ["parked_car", "pavement_people", "kerb_props", "oncoming_cones"]
    for _ in range(b.distractors):
        wp = rng.choice(spots)
        kind = rng.choice(kinds)
        kerb = wp.lane_width / 2.0
        try:
            if kind == "parked_car":
                right = wp.get_right_lane()
                if right is not None and right.lane_type == b.carla.LaneType.Driving or \
                        b.variant == "shoulder_pass":
                    continue               # would block a lane (or the way round), not sit beside one
                car = b.b._spawn(rng.choice(_car_blueprints(b)),
                                 b._offset(wp, 0.0, kerb + 1.7, 0.3), physics=False)
                car.set_simulate_physics(False)
            elif kind == "pavement_people":
                for k in range(rng.randint(2, 4)):
                    _walker(b, b._offset(wp, rng.uniform(-3.0, 3.0), kerb + rng.uniform(2.0, 3.0), 1.0))
            elif kind == "kerb_props":
                props = _available(b, PROPS_KERB)
                b.b._spawn(rng.choice(props), b._offset(wp, 0.0, kerb + 1.2, 0.1))
            else:
                left = wp.get_left_lane()
                if left is None or left.lane_type != b.carla.LaneType.Driving or \
                        b.variant in SPOT_TESTS:     # that lane is the way round, or the overtaker's
                    continue
                for k in range(3):
                    b.b._spawn("static.prop.constructioncone", b._offset(left, 2.0 * k, 0.0, 0.1))
        except RuntimeError:
            continue                       # a spawn collided: skip this distractor
