"""Apply map patches and score routes against the evaluator's work-zone geometry."""

import argparse
import heapq
import json
from collections import defaultdict
from math import dist
from pathlib import Path

from geometry import cross_section, length, nearest_station, samples


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def apply_patch(prior, patch):
    if not isinstance(patch, dict) or set(patch) != {"close", "add"}:
        raise ValueError("A patch must have close and add lists")
    closed, added = patch["close"], patch["add"]
    if not isinstance(closed, list) or not isinstance(added, list):
        raise ValueError("close and add must be lists")
    old = {lane["id"]: lane for lane in prior["lanes"]}
    if any(not isinstance(key, str) or key not in old for key in closed):
        raise ValueError("close contains an unknown lane ID")
    if len(closed) != len(set(closed)):
        raise ValueError("close contains a duplicate lane ID")
    nodes = prior["nodes"]
    lower, upper = prior["bounds"]
    if len(added) > len(nodes) * (len(nodes) - 1):
        raise ValueError("Too many new lanes")
    used = set(old)
    new = []
    for lane in added:
        if not isinstance(lane, dict) or set(lane) != {"id", "from", "to", "centerline"}:
            raise ValueError("Added lanes need id, from, to and centerline")
        lane_id, start, end, centerline = (lane[k] for k in ("id", "from", "to", "centerline"))
        if not isinstance(lane_id, str) or not lane_id or lane_id in used:
            raise ValueError("New lane IDs must be nonempty and unique")
        if not isinstance(start, str) or not isinstance(end, str) or start not in nodes or end not in nodes or start == end:
            raise ValueError("New lanes must connect two different existing nodes")
        if not isinstance(centerline, list) or not 2 <= len(centerline) <= 128:
            raise ValueError("Each centerline needs 2 to 128 points")
        for point in centerline:
            if not isinstance(point, list) or len(point) != 2 or any(
                type(x) not in (int, float) or not lo <= x <= hi for x, lo, hi in zip(point, lower, upper)
            ):
                raise ValueError("Points must be numeric [x, y] pairs within the map bounds")
        if dist(centerline[0], nodes[start]) > .25 or dist(centerline[-1], nodes[end]) > .25:
            raise ValueError("Centerline ends must meet the declared nodes within 0.25 m")
        centerline = [nodes[start], *centerline[1:-1], nodes[end]]
        if any(dist(a, b) < 1e-6 for a, b in zip(centerline, centerline[1:])):
            raise ValueError("Adjacent centerline points must be different")
        new.append({**lane, "centerline": centerline})
        used.add(lane_id)

    closed_ids = set(closed)
    return [lane for key, lane in old.items() if key not in closed_ids] + new, new


def route(graph, origin, destination):
    queue = [(0.0, origin)]
    costs = {origin: 0.0}
    previous = {}
    while queue:
        cost, here = heapq.heappop(queue)
        if cost != costs[here]:
            continue
        if here == destination:
            path = []
            while here != origin:
                lane = previous[here]
                path.append(lane)
                here = lane["from"]
            return list(reversed(path))
        for lane, lane_cost in graph[here]:
            there = lane["to"]
            new_cost = cost + lane_cost
            if new_cost < costs.get(there, float("inf")):
                costs[there] = new_cost
                previous[there] = lane
                heapq.heappush(queue, (new_cost, there))
    return None


def corridor_fit(candidate, edges, vehicle_width):
    """Measure the clearance of the vehicle envelope to the actual two boundaries."""
    left, right = edges["left"], edges["right"]
    furthest = left[0][1]
    minimum = (float("inf"), furthest)
    backward = False
    for x, station in samples(candidate["centerline"], spacing=.15):
        inside = (left[0][1] <= station <= left[-1][1]
                  and cross_section(left, station) <= x <= cross_section(right, station))
        distance = min(nearest_station((x, station), edge)[0] for edge in (left, right))
        clearance = (distance if inside else -distance) - vehicle_width / 2
        minimum = min(minimum, (clearance, station))
        backward |= station < furthest - .20
        furthest = max(furthest, station)
    return {"passed": minimum[0] >= -1e-6 and not backward,
            "min_margin_m": round(minimum[0], 4), "station_m": round(minimum[1], 2),
            "reversed": backward}


def fits(candidate, legal, vehicle_width):
    clearance = (legal["width"] - vehicle_width) / 2
    furthest = 0.0
    for center in samples(candidate["centerline"]):
        lateral, station = nearest_station(center, legal["centerline"])
        if lateral > clearance + 1e-6 or station < furthest - .20:
            return False
        furthest = max(furthest, station)
    return True


def footprint_fit(candidate, site, vehicle_width):
    """Measure signed clearance against the painted outside edges and all solid footprints."""
    import numpy as np

    points = np.asarray(list(samples(candidate["centerline"], spacing=.15)), dtype=float)
    x, y = points.T

    def distances(line, selected):
        vertices = np.asarray(line, dtype=float)
        starts, delta = vertices[:-1], np.diff(vertices, axis=0)
        squared = (delta * delta).sum(axis=1)
        results = []
        for start in range(0, len(selected), 512):
            relative = selected[start:start+512, None, :] - starts
            t = np.clip((relative * delta).sum(axis=2) / np.maximum(squared, 1e-20), 0, 1)
            results.append(np.sqrt(((relative-t[:,:,None]*delta)**2).sum(axis=2).min(axis=1)))
        return np.concatenate(results)

    left, right = (np.asarray(site["outer"][key]) for key in ("left", "right"))
    outside = ((x < np.interp(y,left[:,1],left[:,0])) | (x > np.interp(y,right[:,1],right[:,0]))
               | (y < left[0,1]) | (y > left[-1,1]))
    margin = np.minimum(distances(left,points), distances(right,points)) * np.where(outside,-1,1)
    limit = np.full(len(y), "paint", dtype=object)
    for fixture in site["fixtures"]:
        poly = np.asarray(fixture["poly"] + [fixture["poly"][0]])
        inside = np.zeros(len(y),dtype=bool)
        for (xa,ya),(xb,yb) in zip(poly,poly[1:]):
            if ya != yb:
                inside ^= ((ya > y) != (yb > y)) & (x < xa + (y-ya)*(xb-xa)/(yb-ya))
        signed = distances(poly,points) * np.where(inside,-1,1)
        limit[signed < margin] = fixture["kind"]
        margin = np.minimum(margin,signed)
    margin -= vehicle_width/2
    minimum = float(np.min(margin))
    # Equivalent minima on a straight section can differ by a few floating-point ulps.
    worst = int(np.flatnonzero(margin <= minimum + 1e-10)[0])
    reversed_path = bool(np.any(y < np.maximum.accumulate(y)-.20))
    result={"passed": bool(minimum >= -1e-6 and not reversed_path),
            "min_margin_m": round(minimum,4), "station_m": round(float(y[worst]),2),
            "limiting_boundary": limit[worst], "reversed": reversed_path}
    if "narrowing_zones" in site:
        divider=np.asarray(site["divider"])
        bank="left" if x[worst]<np.interp(y[worst],divider[:,1],divider[:,0]) else "right"
        result["at_narrowing"]=any(abs(y[worst]-peak)<=span for peak,span in site["narrowing_zones"][bank])
        result["bank_at_minimum"]=bank
    return result,points


def island_fit(candidate, site, vehicle_width):
    result, points = footprint_fit(candidate, site, vehicle_width)
    gate = site["gate_y"]
    # A path can cross the transverse section more than once if it reverses.
    crossings = [xa + (gate-ya)*(xb-xa)/(yb-ya) for (xa,ya),(xb,yb) in zip(points,points[1:])
                 if min(ya,yb) <= gate < max(ya,yb)]
    lo,hi = site["island_span"]
    arms = {"left" if u < lo else "right" if u > hi else "through_island" for u in crossings}
    arm = next(iter(arms)) if len(arms)==1 else ("multiple" if arms else "unobserved")
    return {**result, "side":arm}


def access_fit(candidate, site, vehicle_width):
    import numpy as np

    result, points = footprint_fit(candidate, site, vehicle_width)
    divider = np.asarray(site["divider"])
    offset = points[:,0] - np.interp(points[:,1], divider[:,1], divider[:,0])
    # Ignore a run along the exact centre of an opening when locating a side change.
    indices = np.flatnonzero(abs(offset) > 1e-7)
    crossings = []
    stations = []
    events = []
    for a,b in zip(indices[:-1],indices[1:]):
        if offset[a]*offset[b] < 0:
            t = offset[a]/(offset[a]-offset[b])
            y = float(points[a,1]+t*(points[b,1]-points[a,1]))
            gate = next((g for g in site["gates"] if abs(g["station_m"]-y) <= g["half_gap_m"]),None)
            crossings.append(gate["id"] if gate else "solid_divider")
            stations.append(round(y, 2))
            if "work_bay" in site:
                turning_from = "frontage" if offset[a]*site["frontage_side"]>0 else "main"
                events.append({"at":crossings[-1],"station_m":stations[-1],"from_road":turning_from,
                               "open":bool(gate and gate["open"]),
                               "work_only":bool(gate and gate.get("work_only")),
                               "direction_ok":bool(gate and gate["from_road"]==turning_from)})
    origin = candidate["from"].removesuffix("_in")
    dest = candidate["to"].removesuffix("_out")
    allowed = {g["id"] for g in site["gates"] if g["open"] and (g["from_road"],g["to_road"]) == (origin,dest)}
    proper_transfer = not crossings if origin == dest else bool(crossings) and set(crossings) <= allowed
    if "work_bay" in site:
        proper_transfer = all(e["open"] and e["direction_ok"] for e in events)
        result["crossings"] = events
    if "channel_islands" in site:
        passages = {}
        for bank,island in site["channel_islands"].items():
            y,sign = island["station"],island["bank_sign"]
            for a,b in zip(points[:-1],points[1:]):
                if not a[1]<=y<b[1]:
                    continue
                x = float(a[0]+(b[0]-a[0])*(y-a[1])/(b[1]-a[1]))
                if sign*(x-np.interp(y,divider[:,1],divider[:,0]))<=0:
                    continue
                offset = sign*(x-island["island_x"])
                side = "through_island" if abs(offset)<=island["curb_half_width"] else "outer" if offset>0 else "inner"
                expected = "outer" if island["open_outer"] else "inner"
                passages[bank] = {"side":side,"open_side":expected,"station_m":round(y,2),
                                   "correct_side":side==expected}
                break
        result["island_passages"] = passages
    return {**result, "passed": result["passed"] and proper_transfer, "crossed_at": crossings,
            "crossing_stations_m": stations, "permitted_crossing":proper_transfer}


def diagnose_new_gap(added, site, geometry, vehicle_width, passed):
    gate = next(g for g in site["gates"] if g["id"] == "temporary")
    expected = (gate["from_road"], gate["to_road"])
    attempts = []
    for lane in added:
        origin = lane["from"].removesuffix("_in")
        dest = lane["to"].removesuffix("_out")
        if {origin, dest} != {"main", "frontage"} or not lane["from"].endswith("_in") or not lane["to"].endswith("_out"):
            continue
        fit = geometry.get(lane["id"]) or access_fit(lane, site, vehicle_width)
        attempts.append({"lane":lane["id"], "direction":[origin,dest], "stations_m":fit["crossing_stations_m"],
                         "at_new_gap":gate["id"] in fit["crossed_at"], "correct_direction":(origin,dest)==expected,
                         "geometry_passed":fit["passed"], "min_margin_m":fit["min_margin_m"]})
    located = [attempt for attempt in attempts if attempt["at_new_gap"]]
    directed = [attempt for attempt in located if attempt["correct_direction"]]
    if not attempts:
        stage = "no_transfer_submitted"
    elif not located:
        stage = "gap_location"
    elif not directed or len(directed) < len(located):
        stage = "direction"
    elif not any(attempt["geometry_passed"] for attempt in directed):
        stage = "transfer_geometry"
    else:
        stage = "passed" if passed else "remaining_map"
    return {"result":stage, "expected_station_m":round(gate["station_m"],2),
            "expected_direction":list(expected), "submitted_transfers":attempts}


def score_case(prior, truth, patch):
    if patch is None:
        return {"passed": False, "error": "missing case patch"}
    try:
        lanes, added = apply_patch(prior, patch)
    except ValueError as exc:
        return {"passed": False, "error": str(exc)}

    graph = defaultdict(list)
    legal = defaultdict(list)
    for lane in lanes:
        graph[lane["from"]].append((lane, length(lane["centerline"])))
    for lane in truth["legal_lanes"]:
        legal[lane["from"], lane["to"]].append(lane)
    drivable = {}
    geometry = {}
    for lane in lanes:
        verdicts = []
        for target in legal[lane["from"], lane["to"]]:
            if "access" in truth and lane["from"].endswith("_in") and lane["to"].endswith("_out"):
                result = access_fit(lane, truth["access"], truth["vehicle_width"])
                geometry[lane["id"]] = result
                verdicts.append(result["passed"])
            elif "island" in truth and lane["from"] == "entry" and lane["to"] == truth["island"]["exit"]:
                result = island_fit(lane, truth["island"], truth["vehicle_width"])
                geometry[lane["id"]] = result
                verdicts.append(result["passed"])
            elif "corridor" in target:
                result = corridor_fit(lane, target["corridor"], truth["vehicle_width"])
                geometry[lane["id"]] = result
                verdicts.append(result["passed"])
            else:
                verdicts.append(fits(lane, target, truth["vehicle_width"]))
        if ("access" in truth and "work_bay" in truth["access"] and not verdicts
                and lane["from"] in ("main_in","frontage_in") and lane["to"] in ("main_out","frontage_out")):
            geometry[lane["id"]] = access_fit(lane,truth["access"],truth["vehicle_width"])
        drivable[lane["id"]] = any(verdicts)
    illegal_additions = [lane["id"] for lane in added if not drivable[lane["id"]]]
    checks = {}
    for query in truth["queries"]:
        path = route(graph, query["from"], query["to"])
        if query["expect"] == "unreachable":
            passed = path is None
        else:
            passed = path is not None and all(drivable[lane["id"]] for lane in path)
        checks[query["name"]] = {"passed": passed, "lanes": [lane["id"] for lane in path] if path is not None else None}
    # A stale lane edge that remains in the graph is still a false promise to a planner,
    # even when a shorter valid route happens to win the queries above.
    stale_open = [lane["id"] for lane in prior["lanes"] if lane["id"] in drivable and not drivable[lane["id"]]]
    report = {"passed": not (illegal_additions or stale_open) and all(c["passed"] for c in checks.values()),
              "checks": checks, "illegal_additions": illegal_additions, "stale_open_lanes": stale_open}
    if geometry:
        report["corridors"] = geometry
    if "access" in truth:
        failures = defaultdict(list)
        for query in truth["queries"]:
            check = checks[query["name"]]
            if check["passed"]:
                continue
            if query["expect"] == "unreachable":
                error = "false_route"
            elif check["lanes"] is None:
                error = "lost_route"
            else:
                wrong_opening = any(not geometry[key]["permitted_crossing"]
                                    for key in check["lanes"] if key in geometry)
                error = "wrong_crossover" if wrong_opening else "undrivable_route"
            failures[error].append(query["name"])
        primary = next((e for e in ("false_route","lost_route","wrong_crossover","undrivable_route")
                        if failures.get(e)), "passed" if report["passed"] else "other_map_edit")
        report["diagnosis"] = {"primary":primary, "movement_errors":dict(failures),
                               "expected_movements":{q["name"]:q["expect"] for q in truth["queries"]
                                                     if not q["name"].startswith("reverse_")}}
        if "work_alignment" in truth:
            straight = {}
            for road in ("main","frontage"):
                check = checks[f"{road}_to_{road}"]
                segments = [geometry[k] for k in (check["lanes"] or []) if k in geometry]
                if segments:
                    worst = min(segments,key=lambda s:s["min_margin_m"])
                    straight[road] = {"passed":check["passed"], "min_margin_m":worst["min_margin_m"],
                                      "station_m":worst["station_m"], "boundary":worst["limiting_boundary"],
                                      "crossed_divider":any(s["crossed_at"] for s in segments)}
                else:
                    straight[road] = {"passed":check["passed"], "missing_route":check["lanes"] is None}
            report["diagnosis"].update(work_alignment=truth["work_alignment"], through_lanes=straight)
        if "work_bay" in truth["access"]:
            movements = {}
            for q in truth["queries"]:
                if q["name"].startswith("reverse_"):
                    continue
                check = checks[q["name"]]
                route_fits = [geometry[k] for k in (check["lanes"] or []) if k in geometry]
                if check["passed"]:
                    stage="passed"
                elif check["lanes"] is None:
                    stage="lost_connection"
                elif any(e["work_only"] for fit in route_fits for e in fit["crossings"]):
                    stage="used_work_only_gate"
                elif any(not e["open"] for fit in route_fits for e in fit["crossings"]):
                    stage="missed_opening"
                elif any(not e["direction_ok"] for fit in route_fits for e in fit["crossings"]):
                    stage="wrong_transfer_direction"
                elif any(fit["min_margin_m"]<0 and fit["limiting_boundary"]=="work_bay" for fit in route_fits):
                    stage="entered_work_bay"
                elif q["expect"]=="unreachable":
                    stage="false_connection"
                elif any(not p["correct_side"] for fit in route_fits for p in fit.get("island_passages",{}).values()):
                    stage="wrong_island_side"
                elif any(fit["min_margin_m"]<0 and fit["limiting_boundary"] in ("channel_island","channel_barrier")
                         for fit in route_fits):
                    stage="island_clearance"
                elif any(fit["min_margin_m"]<0 and fit.get("at_narrowing") for fit in route_fits):
                    stage="independent_lane_clearance"
                else:
                    stage="route_geometry"
                movements[q["name"]] = {"result":stage,"expected":q["expect"],"lanes":check["lanes"]}
            report["diagnosis"]["detour_movements"]=movements
        if truth["variant"] == "relocated_access" and any(g["id"]=="temporary" for g in truth["access"]["gates"]):
            report["diagnosis"]["new_gap"] = diagnose_new_gap(added, truth["access"], geometry,
                                                             truth["vehicle_width"], report["passed"])
    if "island" in truth:
        site = truth["island"]
        required = checks["required_trip"]["lanes"]
        forbidden = checks["prohibited_connection"]["lanes"]
        exits_ok = required is not None and forbidden is None
        traversed = set(required or forbidden or ())
        crossing = next((lane for lane in lanes if lane["id"] in traversed and lane["from"]=="entry"), None)
        detail = None
        if crossing is not None:
            detail = geometry.get(crossing["id"]) or island_fit(crossing,site,truth["vehicle_width"])
        arm = detail["side"] if detail else "unobserved"
        if not exits_ok:
            cause = "exit_connectivity"
        elif arm != site["open_side"]:
            cause = "island_side" if detail else "missing_traversal"
        elif detail["reversed"]:
            cause = "reversed_traversal"
        elif not detail["passed"]:
            cause = "clearance"
        else:
            cause = "passed" if report["passed"] else "other_map_edit"
        report["diagnosis"] = {"primary":cause, "correct_exit_only":exits_ok, "expected_exit":site["exit"],
                               "expected_side":site["open_side"], "submitted_side":arm,
                               "route_geometry":detail}
    return report


def evaluate(suite, submission):
    truth = read_json(suite / "private" / "truth.json")
    cases = truth["cases"]
    document_error = None
    patches = {}
    try:
        document = read_json(submission)
        if not isinstance(document, dict) or set(document) != {"patches"} or not isinstance(document["patches"], dict):
            raise ValueError("Expected a JSON object with a patches object")
        patches = document["patches"]
        if set(patches) - set(cases):
            raise ValueError("Submission includes unknown case IDs")
    except (OSError, ValueError) as exc:
        document_error = str(exc)
        patches = {}

    results = {}
    sites = defaultdict(dict)
    for case_id, expected in cases.items():
        site, variant = expected["site"], expected["variant"]
        prior = truth["site_maps"][str(site)]
        result = score_case(prior, expected, patches.get(case_id))
        results[case_id] = {"site": site, "variant": variant, **result}
        sites[site][variant] = result["passed"]
    credited = sum(
        (site["lane_shift"] + site["changed_connection"]) * site["shoulder_work"] for site in sites.values()
    )
    total = 2 * len(sites)
    report = {"score": credited / total, "credited_updates": credited, "update_cases": total,
              "passed_cases": sum(r["passed"] for r in results.values()), "total_cases": len(results), "cases": results}
    if document_error:
        report["submission_error"] = document_error
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    parser.add_argument("submission", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = evaluate(args.suite, args.submission)
    output = json.dumps(report, indent=2)
    if args.report:
        args.report.write_text(output + "\n", encoding="utf-8")
        print(f"Score: {report['score']:.4f} ({report['credited_updates']}/{report['update_cases']} credited updates)")
    else:
        print(output)
