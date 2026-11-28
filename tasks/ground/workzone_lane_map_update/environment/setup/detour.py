"""Survey a diversion to a parallel road with a protected work bay between access points."""

from math import ceil, cos, pi
from random import Random

from access import MOVEMENTS, ROADS, access_site, movement_line, opening_spec, road_strip
from generate import lane
from geometry import cross_section


def other(road):
    return "frontage" if road=="main" else "main"


def temporary_station(rng, lower, upper, old):
    intervals = [(a,b) for a,b in ((lower,min(upper,old-20)),(max(lower,old+20),upper)) if b>a]
    pick = rng.random()*sum(b-a for a,b in intervals)
    for a,b in intervals:
        if pick<=b-a:
            return a+pick
        pick -= b-a
    raise ValueError("No separated temporary crossover location")


def feasible_sequences(origin, gates, bay):
    candidates = [(origin,())]
    events = [(g["station_m"],g) for g in gates if g["open"]]
    if bay["road"] is not None:
        events.append((bay["station"],None))
    for _,gate in sorted(events,key=lambda event:event[0]):
        if gate is None:
            candidates = [(r,seq) for r,seq in candidates if r!=bay["road"]]
        else:
            candidates += [(gate["to_road"],seq+(gate,)) for r,seq in candidates if r==gate["from_road"]]
    return candidates


def route_line(origin, roads, gates):
    path = []
    for i,(_,y) in enumerate(roads[origin]):
        x = roads[origin][i][0]
        for gate in gates:
            f = min(1,max(0,.5+(y-gate["station_m"])/gate["transition_m"]))
            x += (roads[gate["to_road"]][i][0]-roads[gate["from_road"]][i][0])*f*f*(3-2*f)
        path.append([x,y])
    return path


def shoulder_excavations(rng, control, layout, site, roads):
    """Portable rails protect separate local trenches outside the two traffic lanes."""
    length=layout["length"]
    divider=site["divider"]
    rails=[]
    pinches={}
    site["narrowing_zones"]={}
    for bank,sign in (("left",-1),("right",1)):
        road=next(name for name,line in roads.items() if line[0][0]*sign>0)
        locations=[rng.uniform(.19,.36)*length,rng.uniform(.64,.81)*length]
        windows=[(y,rng.uniform(11,16),rng.uniform(2.82,3.07)) for y in locations]
        site["narrowing_zones"][bank]=[(peak,span) for peak,span,_ in windows] if not control else []
        edge,center=[],[]
        original=layout["access_edges"][bank]
        for (divider_x,y),(old_edge,_),(lane_x,_) in zip(divider,original,roads[road]):
            strengths=[max(0,(1+cos(pi*(y-peak)/width))/2) if abs(y-peak)<width else 0
                       for peak,width,_ in windows]
            index=max(range(len(strengths)),key=strengths.__getitem__)
            s=strengths[index]
            current_width=abs(old_edge-divider_x)-.32
            take=min(.15,current_width-windows[index][2]) if control else current_width-windows[index][2]
            outside=old_edge-sign*s*take
            midpoint=divider_x+sign*(.32+windows[index][2]/2)
            edge.append((outside,y))
            center.append((lane_x+(0 if control else s*(midpoint-lane_x)),y))
        roads[road]=center
        layout["access_edges"][bank]=edge
        site["outer"][bank]=edge
        pinches[bank]=[peak/length for peak,_w,_lane in windows]
        for peak,width,_ in windows:
            start,end=peak-.70*width,peak+.70*width
            rail=[(x+sign*.40,y) for x,y in edge]
            rails.append((rail,start,end))
    layout.update(pinch_stations=pinches,current_divider=divider)
    return rails


def channelized_aprons(rng, control, prior, layout, site, roads, rails, required_outer=None):
    """A paved turnout around a raised island can carry traffic on either side."""
    old = Random("prework-aprons/"+layout["length"].hex())
    permanent = layout["survey_alignment"]
    divider = site["divider"]
    site["channel_islands"] = {}
    layout["channel_targets"] = {}
    fixtures = []
    prior["bounds"][0][0],prior["bounds"][1][0] = -22,22
    layout["road_half_width"] = 19
    bank = old.choice(("left","right"))
    sign = -1 if bank=="left" else 1
    i = 0 if bank=="left" else 1
    road = next(r for r in ROADS if sign*prior["nodes"][r+"_in"][0]>0)
    peak = old.uniform(.485,.515)*layout["length"]
    half = old.uniform(3.0,4.9)
    taper = old.uniform(14.0,17.0)
    crown = old.uniform(3.85,4.25)
    skew = old.uniform(-.42,.42)
    curb = old.uniform(.48,.64)
    full = old.uniform(7.65,8.5)
    old_outer = old.choice((False,True))
    open_outer = old_outer if control else rng.choice((False,True))
    if required_outer is not None and not control:
        open_outer = required_outer
    rail_y = peak + rng.uniform(-.40,.40)*half

    def weight(y):
        q = min(1,max(0,(abs(y-peak)-half-1)/taper))
        return 1-q*q*(3-2*q)
    def offset(y):
        return crown+skew*max(-1,min(1,(y-peak)/half))
    def channel(y,outer):
        return (offset(y)+curb+full)/2 if outer else (.32+offset(y)-curb)/2

    edge = layout["access_edges"][bank]
    def widening(y):
        width = sign*(cross_section(edge,y)-cross_section(divider,y))
        return weight(y)*(full-width)
    new_edge = [(x+sign*widening(y),y) for x,y in edge]
    for j,(path,a,b) in enumerate(rails):
        if sign*(path[0][0]-divider[0][0])>0:
            rails[j] = ([(x+sign*widening(y),y) for x,y in path],a,b)
    layout["access_edges"][bank] = site["outer"][bank] = new_edge
    for j,(x,y) in enumerate(roads[road]):
        target = cross_section(divider,y)+sign*channel(y,open_outer)
        roads[road][j] = (x+weight(y)*(target-x),y)
    old_lane_x = abs(prior["nodes"][road+"_in"][0])
    for lane_record in prior["lanes"]:
        if not lane_record["from"].endswith("_in") or not lane_record["to"].endswith("_out"):
            continue
        upstream = lane_record["from"].removesuffix("_in")
        downstream = lane_record["to"].removesuffix("_out")
        if road not in (upstream,downstream):
            continue
        for point in lane_record["centerline"]:
            x,y = point
            # All old crossovers lie outside the central island and its nose.
            if sign*(x-cross_section(permanent,y))>0:
                point[0] += sign*weight(y)*(channel(y,old_outer)-old_lane_x)
    old_edge = layout["previous_edges"][i]
    layout["previous_edges"][i] = [(x+sign*weight(y)*(full-old_lane_x-layout["lane_width"]/2),y)
                                   for x,y in old_edge]

    outline = [(-1,.15),(-.72,.90),(-.30,1),(.36,1),(.76,.83),(1,.16)]
    nose = [(cross_section(divider,peak+t*half)+sign*(offset(peak+t*half)+s*curb*w),peak+t*half)
            for s,profile in ((-1,outline),(1,reversed(outline))) for t,w in profile]
    fixtures.append({"kind":"channel_island","poly":nose})
    # The physical closure reaches from the island into the protected side.
    # A centreline may use any remaining space that fits the vehicle.
    start,end = (.24,offset(rail_y)) if open_outer else (offset(rail_y),full+.15)
    barrier = [(cross_section(divider,rail_y+d)+sign*x,rail_y+d)
               for x,d in ((start,-.26),(end,-.26),(end,.26),(start,.26))]
    fixtures.append({"kind":"channel_barrier","poly":barrier})
    x0 = cross_section(divider,rail_y)
    s=(rail_y-peak)/half
    width=curb*cross_section([(w,t) for t,w in outline],s)
    site["channel_islands"][bank] = {"station":rail_y,"road":road,"bank_sign":sign,
                                     "island_x":x0+sign*offset(rail_y),"curb_half_width":width,
                                     "open_outer":open_outer}
    layout["channel_targets"][bank] = {
        "curb":(cross_section(divider,peak)+sign*offset(peak),peak,.23),
        "closure":(x0+sign*(start+end)/2,rail_y,.68),
        "pavement":(x0+sign*channel(rail_y,open_outer),rail_y,0)}
    # Inspect the physical island on either side of the transverse barrier.
    inner_curb=nose[:len(outline)]
    outer_curb=list(reversed(nose[len(outline):]))
    for name,station in (("curb_upstream",peak-.6*half),("curb_downstream",peak+.6*half)):
        centre=(cross_section(inner_curb,station)+cross_section(outer_curb,station))/2
        layout["channel_targets"][bank][name]=(centre,station,.23)
    return fixtures


def detour_site(rng, variant, *, island_on_closed_road=None, required_outer=None):
    blocked = rng.choice(ROADS)
    control = variant=="shoulder_work"
    case = access_site(rng,"shoulder_work" if control else "closed_access",blocked,
                       phased=True,paired_inventory=True)
    prior,truth,r = case["prior"],case["truth"],case["render"]
    layout,site = r["layout"],truth["access"]
    center,length = site["divider"],layout["length"]
    if island_on_closed_road is not None and not control:
        old_bank = Random("prework-aprons/"+length.hex()).choice(("left","right"))
        road_at_island=next(name for name in ROADS if (prior["nodes"][name+"_in"][0]<0)==(old_bank=="left"))
        blocked=road_at_island if island_on_closed_road else other(road_at_island)
    # Vertical survey from the pre-work road. Its seed and stations precede the work phase.
    topography = Random("preconstruction-elevation/"+length.hex())
    grade = topography.choice((-1,1))*topography.uniform(.013,.029)
    relief = topography.choice((-1,1))*topography.uniform(.75,1.25)
    phase = topography.uniform(-.65,.65)
    stations = [length*k/18 for k in range(19)]
    profile = {"dz_dx":round(topography.choice((-1,1))*topography.uniform(.015,.029),5),
               "station_elevation_m":[[round(y,4),round(grade*y+relief*(cos(phase)-cos(2*pi*y/length+phase)),4)]
                                      for y in stations]}
    prior["road_surface"] = profile
    layout["road_surface"] = profile
    offsets=[prior["nodes"][r+"_in"][0] for r in ROADS]
    layout["previous_edges"]=[[(x+dx,y) for x,y in layout["survey_alignment"]]
                              for dx in (min(offsets)-layout["lane_width"]/2,max(offsets)+layout["lane_width"]/2)]
    # Continue the survey to the end of the site so far-end signs have close views.
    layout["survey_stations"] = [.01+.083*i for i in range(12)]
    layout["oblique_stations"] = [-.045+.083*i for i in range(13)]
    layout["elevated_stations"] = [-.03+.096*i for i in range(11)]
    layout["oblique_targets"] = [(cross_section(layout["survey_alignment"],t*length),t*length)
                                  for t in (.103+.083*i for i in range(13))]
    width,side = layout["lane_width"],site["frontage_side"]
    roads = {road:[(x+prior["nodes"][road+"_in"][0],y) for x,y in center] for road in ROADS}
    historical = sorted(site["gates"],key=lambda g:g["station_m"])
    near,far = historical
    temps = [opening_spec(rng,"temporary_up",rng.choice(ROADS),
                           temporary_station(rng,.075*length,.37*length,near["station_m"])),
             opening_spec(rng,"temporary_down",rng.choice(ROADS),
                           temporary_station(rng,.63*length,.925*length,far["station_m"]))]
    active = historical if control else [rng.choice(pair) for pair in ((near,temps[0]),(far,temps[1]))]
    if not control:
        # Keeping old turn arrows at closed gaps makes the concrete break essential.
        directions = (blocked,other(blocked)) if variant=="bridge_diversion" else rng.choice(
            ((blocked,blocked),(other(blocked),other(blocked))))
        for gate,origin in zip(active,directions):
            gate.update(from_road=origin,to_road=other(origin))
    gates = sorted(historical+temps,key=lambda g:g["station_m"])
    contractor_gate = rng.choice([g for g in gates if g not in active])
    for gate in gates:
        gate["open"] = gate in active
        gate["work_only"] = gate is contractor_gate
    lower,upper = (g["station_m"] for g in active)
    bay_y = rng.uniform(.45,.55)*(upper-lower)+lower
    half = min(rng.uniform(3.5,7.5), max(1.7, min(bay_y-lower,upper-bay_y)-12))
    bay = {"road":None if control else blocked,"station":bay_y,"half_length":half}
    shoulder_rails=shoulder_excavations(rng,control,layout,site,roads)
    islands=channelized_aprons(rng,control,prior,layout,site,roads,shoulder_rails,required_outer)

    physical,display,arrows,cones,targets,signs = [],[],[],[],[],[]
    def place(poly,kind):
        physical.append({"kind":kind,"poly":poly})
        display.append(physical[-1])
    def median(a,b,kind):
        physical.append({"kind":kind,"poly":road_strip(center,a,b,.32)})
        count=max(1,ceil((b-a)/3.5))
        for i in range(count):
            display.append({"kind":kind,"poly":road_strip(center,a+(b-a)*i/count,a+(b-a)*(i+1)/count,.32)})
    for fixture in islands:
        place(fixture["poly"],fixture["kind"])
    previous=0
    for gate in gates:
        y,h = gate["station_m"],gate["half_gap_m"]
        median(previous,y-h,"separator")
        gap = gate["open"] or gate["work_only"]
        if not gap:
            median(y-h,y+h,"closure")
        previous=y+h
        shoulder = layout["access_edges"]["right" if side>0 else "left"]
        cones.extend((cross_section(shoulder,t)+side*.38,t) for t in (y-h-1.4,y+h+1.4))
        transfer = movement_line(gate["from_road"],gate["to_road"],roads,gate)
        local = [(cross_section(transfer,t),t,(cross_section(transfer,t+1)-cross_section(transfer,t-1))/2)
                 for t in (y-3.9,y+3.9)]
        arrows.extend(local)
        targets.append({"name":gate["id"],"center":cross_section(center,y),"station":y,"half_gap":h,
                        "open":gap,"check_direction":gate["open"],"arrows":local})
        if gap:
            sign_y = y+h+.9
            signs.append({"gate":gate["id"],"x":cross_section(center,sign_y),"y":sign_y,
                          "public":gate["open"]})
    median(previous,length,"separator")
    for rail,start,end in shoulder_rails:
        physical.append({"kind":"shoulder_shoring","poly":road_strip(rail,start,end,.20)})
        count=ceil((end-start)/3.5)
        for i in range(count):
            a=start+(end-start)*i/count
            b=start+(end-start)*(i+1)/count
            display.append({"kind":"shoulder_shoring","poly":road_strip(rail,a,b,.20)})
    # A work bay occupies either the full closed traffic lane or adjacent shoulder storage.
    sign = 1 if prior["nodes"][blocked+"_in"][0]>0 else -1
    bank = layout["access_edges"]["right" if sign>0 else "left"]
    if control:
        bay_center=[(x+sign*1.45,y) for x,y in bank]
        bay_widths=[(.90,y) for x,y in bank]
    else:
        # Fence the occupied lane, leaving the adjacent divider break available
        # for its signed use even where the lanes narrow independently.
        inner=[(x+sign*.62,y) for x,y in center]
        outer=[(x+sign*.10,y) for x,y in bank]
        bay_center=[((a+b)/2,y) for (a,y),(b,_) in zip(inner,outer)]
        bay_widths=[(abs(a-b)/2,y) for (a,y),(b,_) in zip(inner,outer)]
    start,end = bay_y-half,bay_y+half
    def bay_strip(a,b):
        ys=[a,*[y for _,y in bay_center if a<y<b],b]
        return ([(cross_section(bay_center,y)-cross_section(bay_widths,y),y) for y in ys]
                +[(cross_section(bay_center,y)+cross_section(bay_widths,y),y) for y in reversed(ys)])
    footprint=bay_strip(start,end)
    physical.append({"kind":"work_bay","poly":footprint})
    display.append({"kind":"milled_surface","poly":footprint})
    for a,b in ((start,start+.58),(end-.58,end)):
        display.append({"kind":"work_rail","poly":bay_strip(a,b)})
    for direction in (-1,1):
        edge=[(x+direction*(w-.15),y) for (x,y),(w,_) in zip(bay_center,bay_widths)]
        for i in range(max(1,ceil(2*half/3.5))):
            a=start+i*3.5
            display.append({"kind":"work_rail","poly":road_strip(edge,a,min(a+3.5,end),.15)})
    opposite_bank=layout["access_edges"]["left" if sign>0 else "right"]
    staging=[(x-sign*1.4,y) for x,y in opposite_bank]
    place(road_strip(staging,bay_y-half-3,bay_y+half+2,.65),"staged_barrier")
    layout.update(access_arrows=arrows,visibility_targets=targets,access_signs=signs,
                  bay_target=(cross_section(bay_center,start+.29),start+.29))
    if not control:
        layout["occupied_bank"] = ("right" if sign>0 else "left",start,end)
        for bank,target in list(layout["channel_targets"].items()):
            if bank==layout["occupied_bank"][0] and start-1 < target["closure"][1] < end+1:
                del layout["channel_targets"][bank]
    site.update(gates=gates,fixtures=physical,work_bay=bay)
    r.update(cones=cones,fixtures=display)
    truth.update(variant=variant)
    legal=[l for l in prior["lanes"] if not l["id"] in {a+"_to_"+b for a,b in MOVEMENTS}]
    patch={"close":[],"add":[]}
    for origin,dest in MOVEMENTS:
        key=origin+"_to_"+dest
        old=next(l for l in prior["lanes"] if l["id"]==key)
        sequences=[seq for end,seq in feasible_sequences(origin,gates,bay) if end==dest]
        query=next(q for q in truth["queries"] if q["name"]==key)
        query["expect"]="drivable" if sequences else "unreachable"
        if control:
            legal.append(old)
            continue
        patch["close"].append(key)
        if sequences:
            best = min(sequences,key=len)
            link=lane(key+"_temporary",old["from"],old["to"],route_line(origin,roads,best),width)
            legal.append(link)
            patch["add"].append({k:v for k,v in link.items() if k!="width"})
    truth["legal_lanes"]=legal
    case["reference"]=patch
    return case
