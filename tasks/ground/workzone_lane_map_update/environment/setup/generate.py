"""Build an agent packet and its separate ground truth."""

import argparse
import json
import random
from dataclasses import asdict
from math import atan2, degrees, hypot
from pathlib import Path

from expert import expert_site
from geometry import Camera, RoadSurface, cross_section
from render import SIZE, scene


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def lane(lane_id, start, end, geometry, width):
    return {"id": lane_id, "from": start, "to": end, "centerline": geometry, "width": width}


def site_map(rng, expert=False):
    distance = rng.uniform(77, 91) if expert else rng.uniform(56, 68)
    width = rng.uniform(3.45, 3.85)
    side = rng.choice([-1, 1])
    shift = side * rng.uniform(3.65, 4.35)
    alternate = shift + side * rng.uniform(.6, 1.45)
    protected = -side * rng.uniform(3.6, 4.35)
    nodes = {
        "approach": [0, -10], "entry": [0, 0], "through_merge": [0, distance],
        "through_exit": [0, distance + 12], "alternate_merge": [alternate, distance],
        "alternate_exit": [alternate, distance + 12], "parallel_entry": [protected, 0],
        "parallel_exit": [protected, distance + 12],
    }
    connections = [
        ("approach_lane", "approach", "entry"),
        ("through_lane", "entry", "through_merge"),
        ("through_outbound", "through_merge", "through_exit"),
        ("alternate_outbound", "alternate_merge", "alternate_exit"),
        ("parallel_lane", "parallel_entry", "parallel_exit"),
    ]
    prior = {
        "units": "meters", "nodes": nodes,
        "bounds": [[-11, -15], [11, distance + 20]] if expert else [[-9, -15], [9, distance + 20]],
        "lanes": [lane(i, a, b, [nodes[a], nodes[b]], width) for i, a, b in connections],
    }
    layout = {"length": distance, "lane_width": width, "road_half_width": 9.15 if expert else 7.85}
    if expert:
        layout["survey_stations"] = (.07, .17, .27, .36, .45, .54, .63, .72, .81)
    guide = [(shift * x, distance * y) for x, y in [(0, .15), (.64, .26), (1, .36), (1, .46), (1, .60)]]
    cones = [(x + sign * width / 2, y) for x, y in guide for sign in (-1, 1)]
    cones.extend((x, distance * .315) for x in (-1.35, -.45, .45, 1.35))
    shoulder_cones = [(shift + offset, y) for _, y in guide for offset in (-.52, .52)]
    return prior, layout, guide, cones, shoulder_cones, side


def detour_paths(prior, guide, hard_rng=None):
    nodes = prior["nodes"]
    if hard_rng is None:
        return {name: [nodes["entry"], *guide, nodes[name]] for name in ("through_merge", "alternate_merge")}

    shift, _ = guide[-1]
    distance = nodes["through_merge"][1]
    direction = 1 if shift > 0 else -1
    bend = hard_rng.uniform(2.55, 2.95) * direction
    bend_station = distance * hard_rng.uniform(.68, .71)
    fork = (shift - bend, bend_station)
    mid_bend = (shift - .44 * bend, (guide[-1][1] + bend_station) / 2)
    split_end = distance * hard_rng.uniform(.87, .91)
    return {
        name: [nodes["entry"], *guide, mid_bend, fork, (nodes[name][0], split_end), nodes[name]]
        for name in ("through_merge", "alternate_merge")
    }


def hard_survey(prior, layout, side, rng):
    distance = layout["length"]
    marks = [[-side * 6.4, .76 * distance], [side * 7.05, .94 * distance],
             [-side * 6.4, 1.01 * distance]]
    if "survey_stations" in layout:
        marks.extend([[-side * 6.4, .49 * distance], [side * 7.85, .66 * distance],
                      [-side * 6.7, .62 * distance]])
    prior["survey_marks"] = marks
    layout["survey_marks"] = marks
    # An independent survey pass has a different true pose and a constant error in its log.
    sign = rng.choice((-1, 1))
    return {
        "x": -side * rng.uniform(1.9, 4.3), "yaw": side * rng.uniform(2, 10),
        "dx": sign * rng.uniform(1.35, 2.35), "dy": rng.uniform(-1.6, 1.6),
        "dyaw": sign * rng.uniform(2.8, 5.8),
    }


def case_truth(prior, layout, paths, variant, corridors=None):
    changed = variant != "shoulder_work"
    to = "alternate_merge" if variant == "changed_connection" else "through_merge"
    target = "alternate_exit" if to == "alternate_merge" else "through_exit"
    other = "through_exit" if target == "alternate_exit" else "alternate_exit"
    legal = list(prior["lanes"])
    detour = None
    patch = {"close": [], "add": []}
    if changed:
        detour = lane("temporary_connection", "entry", to, paths[to], layout["lane_width"])
        patch = {"close": ["through_lane"], "add": [{k: v for k, v in detour.items() if k != "width"}]}
        if corridors is not None:
            detour["corridor"] = corridors[to]["edges"]
        legal = [item for item in legal if item["id"] != "through_lane"] + [detour]

    truth = {
        "variant": variant, "legal_lanes": legal, "vehicle_width": 1.8,
        "queries": [
            {"name": "required_trip", "from": "approach", "to": target, "expect": "drivable"},
            {"name": "prohibited_connection", "from": "approach", "to": other, "expect": "unreachable"},
            {"name": "against_traffic", "from": target, "to": "approach", "expect": "unreachable"},
            {"name": "parallel_lane", "from": "parallel_entry", "to": "parallel_exit", "expect": "drivable"},
            {"name": "through_outbound", "from": "through_merge", "to": "through_exit", "expect": "drivable"},
            {"name": "alternate_outbound", "from": "alternate_merge", "to": "alternate_exit", "expect": "drivable"},
        ],
    }
    return truth, patch, to if changed else None


def survey_cameras(layout, side, camera_jitter, survey, fixtures):
    surface = RoadSurface(layout["road_surface"]) if "road_surface" in layout else None
    downstream_stations = layout.get("survey_stations", (.32, .48, .61) if survey else (.32, .42, .53))
    clips_at = [("approach", (-10, -5, 0)), ("downstream", downstream_stations)]
    if fixtures:
        clips_at.append(("oblique", layout.get("oblique_stations", (.13, .205, .275))))
    if "survey_alignment" in layout:
        clips_at.append(("opposite_oblique",layout["oblique_stations"]))
    if surface:
        clips_at.append(("elevated",layout.get("elevated_stations",[-.03+.12*i for i in range(9)])))
    if "channel_targets" in layout:
        clips_at.append(("return_inspection",(.56,.59,.63,.535,.465)))
    for name, stations in clips_at:
        for i, station in enumerate(stations):
            y = station if name == "approach" else station * layout["length"]
            camera = Camera([-side * 2.8, y, 2.10 + camera_jitter], side * 5.5, 10.0, 480.0,
                            SIZE[0] / 2, SIZE[1] / 2)
            if survey and name == "downstream":
                phase = 2 * i / (len(stations) - 1) - 1
                camera = Camera([survey["x"] + phase * .24, y, camera.position[2]],
                                survey["yaw"] + phase * 1.3, camera.pitch_down_deg,
                                420.0 if "survey_stations" in layout else camera.focal, camera.cx, camera.cy)
            if name in ("oblique","opposite_oblique"):
                bank = -side if name == "opposite_oblique" else side
                original_x = cross_section(layout["survey_alignment"],y) if "survey_alignment" in layout else 0
                x = original_x + bank * (8.4 + .18*i)
                tx, ty = (layout["oblique_targets"][i] if "oblique_targets" in layout
                          else (side*2.7, layout["length"]*.46))
                yaw = degrees(atan2(tx-x, ty-y))
                camera = Camera([x,y,2.75+camera_jitter],yaw,9.5,440.0,camera.cx,camera.cy)
            if name=="elevated":
                alignment=layout["survey_alignment"]
                x=cross_section(alignment,y)+side*.7
                tx=cross_section(alignment,y+16)
                yaw=degrees(atan2(tx-x,16))
                camera=Camera([x,y,8.3+camera_jitter],yaw,29,440.0,camera.cx,camera.cy)
            if name=="return_inspection":
                alignment=layout["survey_alignment"]
                target_y=layout["length"]*.5
                target_x=cross_section(alignment,target_y)
                if i<3:
                    x=cross_section(alignment,y)+(i-1)*6
                    yaw=degrees(atan2(target_x-x,target_y-y))
                    camera=Camera([x,y,9.4+camera_jitter],yaw,degrees(atan2(9.4,y-target_y)),425.,camera.cx,camera.cy)
                else:
                    # The same two cross-bank photographs cover every work state.
                    x=cross_section(alignment,y)+(-5.5 if i==3 else 5.5)
                    yaw=degrees(atan2(target_x-x,target_y-y))
                    pitch=degrees(atan2(15.0,hypot(target_x-x,target_y-y)))
                    camera=Camera([x,y,15.0+camera_jitter],yaw,pitch,365.,camera.cx,camera.cy)
            if surface:
                camera.surface=surface
                camera.position[2]+=surface.height(*camera.position[:2])
            yield name, i, camera


def supplemental_cameras(layout, camera_jitter, start_index):
    """Inspect both banks on a fixed grid along the pre-construction alignment."""
    surface = RoadSurface(layout["road_surface"])
    alignment = layout["survey_alignment"]
    index = start_index
    for fraction in (.20, .32, .42, .48, .50, .52, .58, .68, .80):
        target_y = fraction * layout["length"]
        target_x = cross_section(alignment, target_y)
        for side in (-1, 1):
            for offset_y in (-5.0, 5.0):
                y = target_y + offset_y
                x = cross_section(alignment, y) + side * 7.0
                z = surface.height(x, y) + 13.0 + camera_jitter
                yaw = degrees(atan2(target_x - x, target_y - y))
                pitch = degrees(atan2(z - surface.height(target_x, target_y), hypot(target_x - x, offset_y)))
                camera = Camera([x, y, z], yaw, pitch, 360.0, SIZE[0] / 2, SIZE[1] / 2)
                camera.surface = surface
                yield "return_inspection", index, camera
                index += 1


def write_case(directory, prior, layout, paths, cones, side, camera_jitter, signals=(), survey=None,
               painted=(), barriers=None, fixtures=(), audit=None, supplemental=False):
    directory.mkdir(exist_ok=supplemental)
    write_json(directory / "old_map.json", prior)
    cameras = list(survey_cameras(layout, side, camera_jitter, survey, fixtures))
    if supplemental:
        start = sum(clip == "return_inspection" for clip, _, _ in cameras)
        cameras.extend(supplemental_cameras(layout, camera_jitter, start))
    if layout.get("roadview"):
        from roadview import write_photographs
        write_photographs(directory,cameras,layout,paths,cones,signals,painted,barriers,fixtures,audit)
    clips = {}
    for name,i,camera in cameras:
        if layout.get("roadview"):
            filename = f"{name}_{i:02}.jpg"
        else:
            filename = f"{name}_{i:02}.png"
            scene(camera, layout, paths, cones, signals, painted, barriers, fixtures).save(directory / filename)
        reported = asdict(camera)
        if survey and name in ("downstream", "oblique", "opposite_oblique", "elevated", "return_inspection"):
            reported["position"][0] += survey["dx"]
            reported["position"][1] += survey["dy"]
            reported["yaw_deg"] += survey["dyaw"]
        clips.setdefault(name,[]).append({"image": filename, "camera": reported})
    write_json(directory / "observations.json", {"resolution": list(SIZE), "clips": clips})


TASK = """# Update lane connectivity from work-zone survey images

For each case listed in `manifest.json`, update the stale directed lane map in
`<case>/old_map.json` based on both image sequences in `<case>/observations.json`.
The survey camera is on a nearby pass and can see the work area. Some cases do
not require any map edit. Use the temporary lane markings and traffic controls.
Preserve the parts of the road network that are still usable.

Submit one `submission.json` in this format:

```json
{
  "patches": {
    "<case ID>": {
      "close": ["<old lane ID>"],
      "add": [
        {"id": "new_lane", "from": "<existing node ID>", "to": "<existing node ID>",
         "centerline": [[0.0, 0.0], [3.0, 30.0], [0.0, 60.0]]}
      ]
    }
  }
}
```

The coordinates in this example illustrate the file format. Use the coordinates
from your case. Close only old lanes by ID. Each new lane ID must be unique in
its case. Centerlines have 2 to 128 points with no repeated adjacent points,
ordered in the direction of travel. Use meters in the map frame, within the
map bounds, and put each end within 0.25 m of its declared node. For an unchanged
case submit `{"close": [], "add": []}` explicitly.

Map axes are x = right and y = forward at yaw zero. World height is z, and the
ground is z = 0. Camera `position` is [x, y, z] in meters; positive yaw turns
toward +x and positive pitch points down. `focal` is the focal length in pixels
for both axes; `cx` and `cy` give the principal point. Image u goes right and v
goes down. Frames in each clip are listed in capture order.

A fixed route planner must be able to use the updated graph to reach legally
connected destinations and must find no route for prohibited connections. On
drivable routes, a 1.8 m wide vehicle must fit inside the actual lane corridor.
A case passes when all its checks pass. The score is the fraction of work-zone
updates that pass, with credit at each site conditional on also passing its
unchanged control case.
"""


SURVEY_GUIDE = """
Where overhead lane-use signals appear, a green downward arrow indicates an
open connection from the temporary corridor and a red X indicates a closed
one. Match each signal to the branch beneath it. Paint can mark either branch.
The signal applies to entering the merge from the work zone. The outbound
lane beyond that merge continues to serve other traffic.

The downstream clip is from a separate survey pass. Its camera log has an
unknown but constant horizontal position offset and yaw offset. Camera height,
pitch and intrinsics are calibrated. The centers of the cyan survey marks on
the ground are stationary and listed in `old_map.json` under `survey_marks`.
Use those observations to align the downstream clip to the map. The approach
clip uses the map's coordinate frame already.
"""

CORRIDOR_GUIDE = """
The work-zone routes have reverse curves and varying width. Both candidate
branches have amber edges and some of their markings cross near the fork.
Use the signal to determine the open branch. Where a concrete barrier
extends inside the paint, its road-level inside face defines the usable
edge of the lane. The painted edges define the remaining boundaries. Leave
clearance for a 1.8 m wide vehicle along the full centerline. The verifier
accepts any forward route between the correct nodes that fits the corridor.
"""


def generate(output, seed, sites, difficulty="standard"):
    output.mkdir(parents=True, exist_ok=False)
    agent = output / "agent"
    private = output / "private"
    agent.mkdir()
    private.mkdir()
    expert = difficulty == "expert"
    hard = difficulty in ("hard", "expert")
    guidance = (SURVEY_GUIDE if hard else "") + (CORRIDOR_GUIDE if expert else "")
    task = TASK.replace("\nSubmit one", guidance + "\nSubmit one")
    (agent / "TASK.md").write_text(task, encoding="utf-8")
    rng = random.Random(seed)
    hard_rng = random.Random(seed ^ 0x7182) if hard else None
    cases = []
    truth = {}
    patches = {}
    site_maps = {}
    for site in range(sites):
        prior, layout, guide, cones, shoulder, side = site_map(rng, expert)
        site_maps[str(site)] = prior
        jitter = rng.uniform(-.15, .15)
        corridors = None
        if expert:
            corridors, cones = expert_site(prior, layout, side, hard_rng)
            paths = {name: corridor["centerline"] for name, corridor in corridors.items()}
        else:
            paths = detour_paths(prior, guide, hard_rng)
        survey = hard_survey(prior, layout, side, hard_rng) if hard else None
        for variant in ("lane_shift", "changed_connection", "shoulder_work"):
            case_id = f"case_{rng.getrandbits(64):016x}"
            truth[case_id], patches[case_id], open_merge = case_truth(prior, layout, paths, variant, corridors)
            truth[case_id]["site"] = site
            markings, painted = (), ()
            barriers = None
            signals = ()
            if open_merge is not None:
                if expert:
                    painted = tuple(corridor["paint"] for corridor in corridors.values())
                    barriers = corridors[open_merge]["barriers"]
                else:
                    markings = tuple(paths.values()) if hard else (paths[open_merge],)
                if hard:
                    signals = [(prior["nodes"][name][0], .95 * layout["length"], name == open_merge)
                               for name in paths]
            write_case(agent / case_id, prior, layout, markings, cones if open_merge else shoulder,
                       side, jitter, signals, survey, painted, barriers)
            cases.append(case_id)
    rng.shuffle(cases)
    write_json(agent / "manifest.json", {"cases": cases})
    write_json(private / "truth.json", {"cases": truth, "site_maps": site_maps})
    write_json(private / "reference.json", {"patches": patches})
    print(f"Generated {len(cases)} cases in {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=109)
    parser.add_argument("--sites", type=int, default=1, help="Generate one triplet per site")
    parser.add_argument("--difficulty", choices=("standard", "hard", "expert"), default="standard")
    args = parser.parse_args()
    if args.sites < 1:
        parser.error("--sites must be positive")
    if args.output.exists():
        parser.error("output directory already exists; choose a new path")
    generate(args.output, args.seed, args.sites, args.difficulty)
