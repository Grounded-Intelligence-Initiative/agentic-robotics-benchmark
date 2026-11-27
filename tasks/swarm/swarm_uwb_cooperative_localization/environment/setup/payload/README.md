# Swarm localization: integration notes

## The site (`site.json` in every flight folder)

A warehouse hall, 40 m x 24 m, 9 m high. Frame: x along the long wall, y across, z up from the
floor, metres. Five racks of steel shelving stand on the floor (`racks`: x, y and z extents of
each, as a box). The drones fly along the aisles below rack height and cross over the racks
above them; they never fly through a rack.

Six UWB beacons are fixed high in the corners of the hall and lower at the middle of each long
wall (`beacons`, ids 8 to 13, positions surveyed to a few millimetres). The eight drones (ids 0
to 7) start every flight sitting on their charging pads (`drones[].pad`, surveyed), each placed
there by hand with whatever heading it happened to get.

## The drones' sensors

Every log is stamped on one common clock (seconds from the start of the flight). None of the
sensors has been calibrated in this installation.

**UWB ranging** (`uwb.csv`: `t_s, initiator, responder, range_m`). Two-way ranging between
pairs of nodes (drone-drone and drone-beacon), one pair per 10 ms slot, round-robin in a
shuffled order; a slot whose exchange failed is missing. `range_m` is the radios' own estimate
of the distance between the two nodes. The beacons carry the same radio as the drones.

**Optical flow and ToF** (`drone_<id>.csv`, 50 Hz: `t_s, flow_vx_mps, flow_vy_mps, tof_m,
gyro_z_rps`). A downward camera measures the drone's horizontal velocity in its body frame
(x forward, y left); when it cannot track the surface below it reports nothing (empty fields).
The downward ToF rangefinder beside it reports the distance to whatever is directly below; it
reports nothing beyond 8 m.

**Gyro** (same file): yaw rate about the body z axis, rad/s. Its bias is re-estimated by the
autopilot at every power-up and is therefore different in every flight (typically under 0.01
rad/s, rarely up to 0.02 rad/s). There is no usable magnetometer in a steel warehouse. A
drone's yaw is measured counter-clockwise from the warehouse x axis.

## The flights (`~/flights/`)

`flight_01/`, `flight_02/`, `flight_03/`: three ordinary flights, in exactly the format of the
flights your program is graded on. All flights are flown by the same eight drones and the same
six beacons; nothing was re-mounted between them. They were flown at different times of the
day: some drones take off cold, others still warm from the charger, and what the racks hold
changes from one flight to the next as stock comes and goes. Nothing in this warehouse has ever
measured where the drones actually were, so there is no ground truth for any flight.

Each flight folder also has `flight.json` (`duration_s`, the drone ids, the output rate).

## Tools

- `python3 ~/tools/check_output.py <flight_dir> <out_dir>` checks that `<out_dir>/trajectories.csv`
  has a well-formed row for every drone and output instant (it knows nothing about accuracy).
