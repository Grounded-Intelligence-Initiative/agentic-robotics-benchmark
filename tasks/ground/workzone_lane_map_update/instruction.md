# Update lane connectivity from work-zone survey images

ALE collects your saved file as soon as the main agent returns its final response.
Finish a decision for every site before that response. If you use parallel
workers, wait for all of them within the main session, merge their results,
and check that each empty patch is your assessment of that site. A progress
update that ends the main session also triggers final collection.

Update the supplied directed lane maps for 15 independently surveyed sites.
Each site shows one construction state. The approach clip, downstream clip,
two roadside oblique clips and the outward and return low-altitude inspection views show that same
state from different positions.
The JPEG surveys show worn pavement markings, shadows and localized occlusions
from roadside equipment. Use overlapping views to check divider ends and
direction arrows where one view is obstructed. Parked vans and freestanding
warning boards are on the shoulder. Access signs stand on the median noses.

Read `/home/user/survey/manifest.json` for the case IDs. Within each case
directory, `old_map.json` contains the prior map and `observations.json`
indexes the available images and their camera poses. Some cases show shoulder
work and require no change to the map.

Both the main road and its parallel frontage road carry forward traffic.
Their approach and downstream links remain in service. Update the four direct
movements from main_in or frontage_in to main_out or frontage_out. A move is
available if a public vehicle can travel forward through the site to its
destination. Any number of legal transfers is allowed. A through movement may
have to cross to the other road upstream of a bridge or utility work bay
and return through a different opening downstream.

Use the prior map to identify the pre-construction lanes and crossover
directions. The cameras show the current state. Check three pieces of evidence
at each crossover. A continuous concrete median closes it. At each physical
gap, a rectangular sign on the downstream median nose gives its access status.
PUBLIC ACCESS admits ordinary traffic. WORK ONLY reserves a physically open
gap for construction vehicles. Both signs use the same paint colours.
At a public gap the diagonal pair of pavement arrows indicates the permitted
one-way transfer, from the upstream arrow's road to the downstream arrow's road.
Paint can remain at a closed opening or accompany a work-only gate.

A concrete enclosure protects a milled work bay. Check which side of the
outside edge marking contains the enclosure. It may occupy a traffic lane
or sit off-road on the shoulder. Crossovers must occur in the right order
to avoid an occupied lane. Both downstream gantry arrows refer only to the
local outlet segment. A particular approach may lose access to one or both
destinations even when those outlet segments remain open.

Both lanes can also move sideways during the work. Temporary shoring for
separate shoulder excavations can narrow each lane at different stations.
Partly removed stripes can remain on the old alignment. Follow the current
edge through the taper and past the protective rail. Permanent bends in the
prior map and temporary shifts can resemble one another. Rebuild any obsolete
through or transfer centerline between its original nodes. A 1.8 m vehicle
must fit inside the current marked outside edges throughout the route and
stay clear of the concrete divider and work enclosures. Equivalent safe
paths receive full credit. Keep the approach and downstream links.

Near the middle of the site one roadway widens around a low raised island.
A transverse barrier can close the passage on the median side or the shoulder
side. Inspect the barrier ends, curb and current outside marking in overlapping
views. The pre-construction centerline records the earlier passage. Check whether
the public route still reaches the island during this work phase. A centerline
that clips the island or closure cannot provide clearance for the vehicle.

Map coordinates are in meters. At yaw zero, +x is right and +y is forward.
The old map's `road_surface` gives a surveyed elevation profile and drainage
slope. At a map point (x,y), linearly interpolate the centre elevation at y
from `station_elevation_m`, then add `dz_dx * x`. Beyond the first or last
station use that endpoint's centre elevation. The survey marks, road paint,
barriers and wheels sit on this surface. Camera z is an elevation in the same
datum. Its pitch is measured relative to the horizontal. A pixel ray may
cross more than one profile segment at a crest; use the nearest forward
intersection with the surveyed surface. Lane patches still contain x/y
coordinates and their plan-view clearance is evaluated.
Each camera has a world position `[x,y,z]`, positive
yaw toward +x, and positive pitch downward. Image u goes right and v goes
down. `focal` is the focal length in pixels for both axes and `cx,cy` are
the principal point. The approach clip is aligned with the prior map. The
downstream, both oblique clips and the outward and return elevated views share one constant unknown x/y and yaw offset.
Height, pitch and intrinsics are calibrated. The prior map's `survey_marks`
list the true ground centers of the stationary cyan survey marks.

Submit a single JSON file at `/home/user/submission/submission.json` using
this schema. Coordinates here only illustrate the format.

```json
{
  "patches": {
    "<case ID>": {
      "close": ["<old lane ID>"],
      "add": [
        {
          "id": "temporary_lane",
          "from": "<existing node ID>",
          "to": "<existing node ID>",
          "centerline": [[0.0, 0.0], [3.0, 30.0], [0.0, 60.0]]
        }
      ]
    }
  }
}
```

Use the manifest's case IDs. Close only existing lane IDs. Each added lane
ID must be unique and must differ from all old IDs. Use 2 to 128 ordered
`[x,y]` points with no repeated adjacent points; put each endpoint within
0.25 m of its declared node. Keep the centerline inside the map bounds and
oriented forward along the lane. Keep usable lanes and connections. Submit
an explicit `{"close": [], "add": []}` for an unchanged site.

A fixed planner queries all four entrance-to-outlet combinations and their
reverse trips. Each available trip must be reachable and drivable; closed
movements and reverse trips must be unreachable. Leaving an obsolete lane
in the map is an error even if another route is shorter. An unchanged site
passes with a valid empty patch. Missing or invalid cases fail. The metric
is the changed-site pass fraction multiplied by the unchanged-site pass fraction.
