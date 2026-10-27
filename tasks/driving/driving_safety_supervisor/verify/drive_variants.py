"""drive_variants - graded stagings of the scenario kinds that practice never shows.

Grading stage only (the practice copy of drive_bridge has no such module). The grader gives
every hidden episode a `variant` and a number of `distractors`; drive_bridge's
ScenarioBuilder hands staging to `stage()` here.

Variants, by kind (`standard` is the practice staging):

    construction_zone     construction_props       other props, other layout, sometimes no sign
                          construction_pedestrian  a pedestrian crosses into the passing lane
    broken_down_vehicle   bad_parking              a car parked at an angle, half in the lane
                          debris                   street furniture and rubbish in the lane
                          fully_blocked            both lanes closed; wait until it is cleared
    occluded_pedestrian   occluder_swap            steps out from behind a truck, kiosk, bus stop
                          walking_in_lane          walks ahead in the car's lane, same direction
    red_light_runner      fast_runner              the runner is faster and later
                          brake_no_lights          a lead car brakes hard with no brake lights
    oncoming_left_turn    two_oncoming             two oncoming cars, a short gap between them

Distractors need no reaction: a car parked on the shoulder, people standing on the
pavement, street furniture at the kerb, cones in the oncoming lane well before the hazard.
Hitting one counts like any other collision; braking or swerving for one only costs pace.

The variant table and `graded_specs` are plain Python (the grader imports them); everything that
touches CARLA runs inside the bridge.
"""

VARIANTS = {
    "construction_zone": ("construction_props", "construction_pedestrian"),
    "broken_down_vehicle": ("bad_parking", "debris", "fully_blocked"),
    "occluded_pedestrian": ("occluder_swap", "walking_in_lane"),
    "red_light_runner": ("fast_runner", "brake_no_lights"),
    "oncoming_left_turn": ("two_oncoming",),
}
#: The graded episodes, by variant: only the shifted stagings, weighted to the ones a
#: front-camera supervisor finds hardest (unfamiliar obstacles, a closed road, slow road users).
GRADED_MIX = (
    ("fully_blocked", 4), ("debris", 4), ("construction_props", 4), ("bad_parking", 3),
    ("walking_in_lane", 3), ("brake_no_lights", 3), ("construction_pedestrian", 2),
    ("occluder_swap", 2),
)
MAX_DISTRACTORS = 2

STATIC_KINDS = ("construction_zone", "broken_down_vehicle", "occluded_pedestrian")
PROPS_ROADWORK = ("static.prop.constructioncone", "static.prop.trafficcone01",
                  "static.prop.trafficcone02", "static.prop.barrel", "static.prop.ironplank",
                  "static.prop.streetbarrier", "static.prop.chainbarrier")
PROPS_DEBRIS = ("static.prop.trashcan01", "static.prop.trashcan03", "static.prop.box01",
                "static.prop.box03", "static.prop.shoppingcart", "static.prop.garbage01",
                "static.prop.plasticchair", "static.prop.haybale", "static.prop.clothcontainer")
PROPS_KERB = ("static.prop.mailbox", "static.prop.trashcan02", "static.prop.bench01",
              "static.prop.vendingmachine", "static.prop.plantpot01")
OCCLUDERS = (("vehicle.carlamotors.carlacola", True), ("vehicle.mercedes.sprinter", True),
             ("vehicle.ford.ambulance", True), ("static.prop.busstop", False),
             ("static.prop.kiosk_01", False), ("static.prop.container", False))


def graded_specs(rng, towns):
    """The hidden episode specs: GRADED_MIX over `towns` (cycled), each with a fresh 62-bit
    seed and a distractor count from `rng`, grouped by town so each town loads once."""
    kind_of = {v: kind for kind, variants in VARIANTS.items() for v in variants}
    specs, index = [], 0
    for variant, count in GRADED_MIX:
        for _ in range(count):
            seed = int(rng.random() * (1 << 31)) * (1 << 31) + int(rng.random() * (1 << 31))
            specs.append({"town": towns[index % len(towns)], "kind": kind_of[variant],
                          "variant": variant, "seed": seed,
                          "distractors": int(rng.random() * (MAX_DISTRACTORS + 1))})
            index += 1
    specs.sort(key=lambda s: towns.index(s["town"]))
    return specs


def junction_wanted(variant, default):
    """The junction the route has to take: brake_no_lights needs none."""
    return None if variant == "brake_no_lights" else default


# --------------------------------------------------------------------------- staging
def stage(b, route, junction):
    """Stage `b.kind` as `b.variant` on `route`, then the distractors before the hazard.
    Raises the bridge's redraw (via b._at_distance or RuntimeError) when a draw does not fit."""
    variant = b.variant
    if b.kind in STATIC_KINDS or variant == "brake_no_lights":
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
    _distractors(b, route, index)


def _needs_pass(kind, variant):
    if variant in ("fully_blocked", "brake_no_lights", "occluder_swap"):
        return False
    return kind != "occluded_pedestrian" or variant == "walking_in_lane"


def _hazard_waypoint(b, route, passable):
    index, hwp = b._at_distance(route, b.rng.uniform(*b.HAZARD_M))
    if hwp.is_junction or any(w.is_junction for w in hwp.previous(12.0) + hwp.next(12.0)):
        raise RuntimeError("hazard too close to a junction")
    if passable and not b._passable(hwp):
        raise RuntimeError("no passing lane")
    return index, hwp


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


def _walker(b, transform):
    walkers = b.world.get_blueprint_library().filter("walker.pedestrian.*")
    bp = b.rng.choice(walkers)
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    walker = b.world.spawn_actor(bp, transform)
    b.b.actors.append(walker)
    return walker


# ---- construction_zone --------------------------------------------------------------
def _construction_props(b, route, index, wp):
    rng = b.rng
    props = _available(b, PROPS_ROADWORK)
    depth = rng.uniform(3.0, 9.0)
    for _ in range(rng.randint(3, 6)):
        forward = rng.uniform(0.0, depth)
        lateral = rng.uniform(-1.3, 1.3)
        b.hazards.append(b.b._spawn(rng.choice(props),
                                    _yawed(b, b._offset(wp, forward, lateral, 0.1),
                                           rng.uniform(-90, 90))))
    # always something across the middle of the lane, so the lane is really closed
    b.hazards.append(b.b._spawn(rng.choice(props), b._offset(wp, depth / 2.0, 0.0, 0.1)))
    if rng.random() < 0.5:
        b.hazards.append(b.b._spawn("static.prop.trafficwarning",
                                    b._offset(wp, depth + 3.0, 0.0, 0.1)))


def _construction_pedestrian(b, route, index, wp):
    """The standard closure, and a pedestrian who crosses from the far side into the lane
    the car needs for passing, waits there, and goes back."""
    carla, rng = b.carla, b.rng
    b._stage_construction_zone(wp)
    left = wp.get_left_lane()
    # in the car's own frame: the far kerb of the passing lane is wp.lane_width / 2 +
    # left.lane_width to the left of the car's lane centre
    far = wp.lane_width / 2.0 + left.lane_width + 1.0
    walker = _walker(b, b._offset(wp, rng.uniform(-2.0, 4.0), -far, 1.0))
    b.hazards.append(walker)
    right = wp.transform.get_right_vector()
    toward = carla.Vector3D(right.x, right.y, 0.0)
    far = left.lane_width / 2.0 + 1.0           # walking distance to the passing lane's centre
    speed = rng.uniform(1.2, 1.8)
    to_lane = far / speed
    hold = rng.uniform(3.0, 6.0)
    state = {"t0": None}
    hazard = wp.transform.location

    def hook():
        ego = b.b.ego.get_location()
        if state["t0"] is None and ego.distance(hazard) < b.b._speed() * 2.0 + 12.0:
            state["t0"] = b.b.tracker.time
        if state["t0"] is None:
            return
        elapsed = b.b.tracker.time - state["t0"]
        if elapsed < to_lane:
            walker.apply_control(carla.WalkerControl(direction=toward, speed=speed))
        elif elapsed < to_lane + hold:
            walker.apply_control(carla.WalkerControl(direction=toward, speed=0.0))
        elif elapsed < 2 * to_lane + hold + 1.0:
            back = carla.Vector3D(-toward.x, -toward.y, 0.0)
            walker.apply_control(carla.WalkerControl(direction=back, speed=speed))
        else:
            walker.apply_control(carla.WalkerControl(direction=toward, speed=0.0))
    b.b.scripted.append(hook)


# ---- broken_down_vehicle ------------------------------------------------------------
def _bad_parking(b, route, index, wp):
    rng = b.rng
    lateral = wp.lane_width / 2.0 - rng.uniform(0.6, 1.1)       # half in the lane
    tf = _yawed(b, b._offset(wp, 0.0, lateral, 0.3), rng.choice((-1, 1)) * rng.uniform(15, 40))
    car = b.b._spawn(rng.choice(_car_blueprints(b)), tf)
    car.apply_control(b.carla.VehicleControl(hand_brake=True, brake=1.0))
    b.hazards.append(car)


def _debris(b, route, index, wp):
    rng = b.rng
    props = _available(b, PROPS_DEBRIS)
    for _ in range(rng.randint(2, 4)):
        b.hazards.append(b.b._spawn(rng.choice(props),
                                    _yawed(b, b._offset(wp, rng.uniform(0.0, 4.0),
                                                        rng.uniform(-1.2, 1.2), 0.1),
                                           rng.uniform(0, 360))))
    b.hazards.append(b.b._spawn(rng.choice(props), b._offset(wp, 2.0, 0.0, 0.1)))


def _fully_blocked(b, route, index, wp):
    """Barriers across the car's lane and every lane beside it; they are cleared a while
    after the car has stopped in front of them. Waiting there is excused."""
    rng = b.rng
    step = wp.lane_width / 2.0
    blockers = []
    for k in range(-8, 5):                      # from 4 half-lanes left to 2 right
        lateral = k * step + step / 2.0
        tf = b._offset(wp, 0.0, lateral, 0.1)
        here = b.map.get_waypoint(tf.location, project_to_road=False,
                                  lane_type=b.carla.LaneType.Driving)
        if here is None:
            continue
        blockers.append(b.b._spawn("static.prop.streetbarrier",
                                   b.carla.Transform(tf.location, wp.transform.rotation)))
    blockers.append(b.b._spawn("static.prop.trafficwarning", b._offset(wp, -6.0, 0.0, 0.1)))
    b.hazards.extend(blockers)
    hold = rng.uniform(12.0, 30.0)
    barrier = wp.transform.location
    state = {"since": None, "cleared": False}

    def waiting():
        if state["cleared"]:
            return False
        return b.b.ego.get_location().distance(barrier) < 30.0

    def hook():
        if state["cleared"]:
            return
        if state["since"] is None and waiting() and b.b._speed() < 0.1:
            state["since"] = b.b.tracker.time
        if state["since"] is not None and b.b.tracker.time - state["since"] >= hold:
            state["cleared"] = True
            for actor in blockers:
                if actor in b.b.actors:
                    b.b.actors.remove(actor)
                actor.destroy()
    b.b.excused = waiting
    b.b.scripted.append(hook)


# ---- occluded_pedestrian ------------------------------------------------------------
def _occluder_swap(b, route, index, wp):
    """The standard step-out, from behind something else than a parked van."""
    carla, rng = b.carla, b.rng
    kerb = wp.lane_width / 2.0
    present = _available(b, [n for n, _ in OCCLUDERS])
    name, is_vehicle = rng.choice([o for o in OCCLUDERS if o[0] in present])
    occluder = b.b._spawn(name, b._offset(wp, 0.0, kerb + (1.3 if is_vehicle else 1.8), 0.3),
                          physics=False)
    occluder.set_simulate_physics(False)
    walker = _walker(b, b._offset(wp, rng.uniform(2.5, 4.5), kerb + 1.6, 1.0))
    b.hazards.append(walker)
    speed = rng.uniform(2.0, 3.5)
    right = wp.transform.get_right_vector()
    direction = carla.Vector3D(-right.x, -right.y, 0.0)
    to_centre = (kerb + 1.6) / speed
    pause = rng.uniform(1.5, 4.0)
    lead = rng.uniform(1.4, 2.2)
    state = {"started": False, "t0": 0.0}

    def hook():
        ahead = walker.get_location().distance(b.b.ego.get_location())
        if not state["started"] and ahead < b.b._speed() * lead + 5.0:
            state["started"], state["t0"] = True, b.b.tracker.time
        if state["started"]:
            elapsed = b.b.tracker.time - state["t0"]
            moving = elapsed < to_centre or to_centre + pause <= elapsed < to_centre + pause + 4.0
            walker.apply_control(carla.WalkerControl(direction=direction,
                                                     speed=speed if moving else 0.0))
    b.b.scripted.append(hook)


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


# ---- red_light_runner -----------------------------------------------------------------
def _fast_runner(b, route, index, entry):
    from agents.navigation.basic_agent import BasicAgent
    carla = b.carla
    car, start, goal, junction = b._conflict_vehicle(entry, (60, 120), True)
    for light in b.world.get_traffic_lights_from_waypoint(entry, 40.0):
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(120.0)
    for light in b.world.get_traffic_lights_from_waypoint(start, 40.0):
        light.set_state(carla.TrafficLightState.Red)
        light.set_red_time(120.0)
    b._drive_when(car, goal, entry, lead_s=b.rng.uniform(1.8, 2.3),
                  speed_kmh=b.rng.uniform(50.0, 60.0),
                  opts={"ignore_traffic_lights": True, "ignore_vehicles": True},
                  agent_cls=BasicAgent)


def _brake_no_lights(b, route, index, wp):
    """A lead car in the car's lane, released when the car comes close; at the hazard point
    it brakes to a stop with its brake lights off, waits, and drives on."""
    from agents.navigation.basic_agent import BasicAgent
    carla, rng = b.carla, b.rng
    start_index, start_wp = b._at_distance(route, 25.0)
    tf = start_wp.transform
    car = b.b._spawn(rng.choice(_car_blueprints(b)),
                     carla.Transform(tf.location + carla.Location(z=0.5), tf.rotation))
    b.hazards.append(car)
    agent = BasicAgent(car, target_speed=rng.uniform(28.0, 38.0),
                       opt_dict={"ignore_traffic_lights": True, "ignore_vehicles": True})
    agent.set_global_plan(route[start_index:], stop_waypoint_creation=True, clean_queue=True)
    stop_at = wp.transform.location
    hold = rng.uniform(2.0, 5.0)
    state = {"go": False, "braked": None}
    off = carla.VehicleLightState(carla.VehicleLightState.NONE)

    def hook():
        car.set_light_state(off)
        if not state["go"]:
            if car.get_location().distance(b.b.ego.get_location()) < 18.0:
                state["go"] = True
            else:
                car.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
                return
        if state["braked"] is None and car.get_location().distance(stop_at) < 3.0:
            state["braked"] = b.b.tracker.time
        if state["braked"] is not None and b.b.tracker.time - state["braked"] < hold + 2.0:
            car.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            return
        car.apply_control(agent.run_step())
    b.b.scripted.append(hook)


# ---- oncoming_left_turn ---------------------------------------------------------------
def _two_oncoming(b, route, index, entry):
    from agents.navigation.basic_agent import BasicAgent
    carla, rng = b.carla, b.rng
    car, start, goal, junction = b._conflict_vehicle(entry, (150, 180), True)
    for light in (b.world.get_traffic_lights_from_waypoint(entry, 40.0) +
                  b.world.get_traffic_lights_from_waypoint(start, 40.0)):
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(120.0)
    opts = {"ignore_traffic_lights": True, "ignore_vehicles": True}
    speed = rng.uniform(32.0, 45.0)
    b._drive_when(car, goal, entry, lead_s=3.0, speed_kmh=speed, opts=opts, agent_cls=BasicAgent)
    behind = start.previous(25.0 + speed / 3.6 * rng.uniform(1.8, 2.8))
    if not behind:
        raise RuntimeError("no room for a second oncoming car")
    tf = behind[0].transform
    second = b.b._spawn(rng.choice(_car_blueprints(b)),
                        carla.Transform(tf.location + carla.Location(z=0.5), tf.rotation))
    b.hazards.append(second)
    b._drive_when(second, goal, entry, lead_s=3.0, speed_kmh=speed, opts=opts, agent_cls=BasicAgent)


# --------------------------------------------------------------------------- distractors
def _distractors(b, route, hazard_index):
    """Things beside the route before the hazard that need no reaction."""
    rng = b.rng
    run, spots = 0.0, []
    for i in range(1, hazard_index):
        run += route[i][0].transform.location.distance(route[i - 1][0].transform.location)
        if 20.0 <= run and i < hazard_index - 12 and not route[i][0].is_junction:
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
                if right is not None and right.lane_type == b.carla.LaneType.Driving:
                    continue               # would block a lane, not sit beside one
                car = b.b._spawn(rng.choice(_car_blueprints(b)),
                                 b._offset(wp, 0.0, kerb + 1.1, 0.3), physics=False)
                car.set_simulate_physics(False)
            elif kind == "pavement_people":
                for k in range(rng.randint(2, 4)):
                    _walker(b, b._offset(wp, rng.uniform(-3.0, 3.0), kerb + rng.uniform(2.0, 3.0), 1.0))
            elif kind == "kerb_props":
                props = _available(b, PROPS_KERB)
                b.b._spawn(rng.choice(props), b._offset(wp, 0.0, kerb + 0.9, 0.1))
            else:
                left = wp.get_left_lane()
                if left is None or left.lane_type != b.carla.LaneType.Driving:
                    continue
                for k in range(3):
                    b.b._spawn("static.prop.constructioncone", b._offset(left, 2.0 * k, 0.0, 0.1))
        except RuntimeError:
            continue                       # a spawn collided: skip this distractor
