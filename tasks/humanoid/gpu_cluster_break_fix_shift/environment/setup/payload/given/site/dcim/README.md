# DCIM exports: the AI pod's drawing set

Exported from DC1's DCIM for the pods of this design (every pod is built to the same drawing). These files hold what
is the same in every pod: the floor plan, the rack elevations, the cable plant and the PDU outlet maps. **Serial
numbers, asset states and faults are not in them**: they differ from pod to pod and change with every repair; query
them at the time from the DCIM service (`dcim.assets`, `dcim.elevation`, `dcim.cables`) and the equipment (BMC, fabric
manager). All dimensions are the drawing's nominal values.

| file | content |
|---|---|
| `floor_plan.png` | the pod and its service area from above, in the pod frame |
| `rack_elevations.png` | a GPU rack front and rear, a network rack rear, the leaf switch port face |
| `rack_elevations.csv` | every rack's equipment: `rack`, `units`, `device`, `model`, `mounted`, `rail` (leaf switches) |
| `cable_table.csv` | the 64 node-to-leaf trunks: `label`, `rail`, `node`, `node_port`, `hca`, `node_end`, `leaf`, `leaf_port`, `switch_end`, `length_m`, `type` |
| `pdu_outlet_map.csv` | every rack's 12 switched outlets: `rack`, `role`, `outlet`, `feed`, `load` (the device and power input it feeds, or spare) |

## Frames

* **Pod frame**: origin on the floor at the west end of the cold aisle's centreline (the west face of racks A01 and
  B01, half-way between the rows); **x east, y north, z up**; metres.
* **Rack frame** (one per rack): origin at the middle of the rack's front edge on the floor; x to the right of a
  person facing the rack's front; y into the rack (front to rear); z up. Row A's racks face south: their frame is the
  pod frame moved to the rack's front edge (yaw 0). Row B's racks face north: their frame is turned 180 deg (x points
  west, y south).

## Racks

| rack | role | x [m] | y [m] | front faces | rack frame origin, yaw | holds |
|---|---|---|---|---|---|---|
| A01 | network | 0.00 .. 0.60 | 0.60 .. 1.80 | south | (0.30, 0.60), 0 | 4 leaf switches |
| A02 | gpu | 0.60 .. 1.20 | 0.60 .. 1.80 | south | (0.90, 0.60), 0 | gpn-a02 |
| A03 | gpu | 1.20 .. 1.80 | 0.60 .. 1.80 | south | (1.50, 0.60), 0 | gpn-a03 |
| A04 | gpu | 1.80 .. 2.40 | 0.60 .. 1.80 | south | (2.10, 0.60), 0 | gpn-a04 |
| A05 | gpu | 2.40 .. 3.00 | 0.60 .. 1.80 | south | (2.70, 0.60), 0 | gpn-a05 |
| B01 | network | 0.00 .. 0.60 | -1.80 .. -0.60 | north | (0.30, -0.60), 180 deg | 4 leaf switches |
| B02 | gpu | 0.60 .. 1.20 | -1.80 .. -0.60 | north | (0.90, -0.60), 180 deg | gpn-b02 |
| B03 | gpu | 1.20 .. 1.80 | -1.80 .. -0.60 | north | (1.50, -0.60), 180 deg | gpn-b03 |
| B04 | gpu | 1.80 .. 2.40 | -1.80 .. -0.60 | north | (2.10, -0.60), 180 deg | gpn-b04 |
| B05 | gpu | 2.40 .. 3.00 | -1.80 .. -0.60 | north | (2.70, -0.60), 180 deg | gpn-b05 |

Racks: 600 mm wide, 1200 mm deep, 2265 mm high, 48U. Rack unit U1 starts 0.100 m above the floor; one unit is
44.45 mm, so unit n spans 0.100 + (n - 1) x 0.04445 to 0.100 + n x 0.04445 m. The front mounting rails are 0.100 m
behind the rack's front edge, the rear rails 1.090 m; 19-inch equipment (482.6 mm wide with its ears, 450 mm between
the rails).

| equipment | units | height above the floor [m] | mounting |
|---|---|---|---|
| KX-8H GPU node (GPU racks) | U8-U15 | 0.411 - 0.767 | front, flush with the front rails |
|   its CPU tray / GPU tray | U8-U9 / U10-U15 | 0.411 - 0.500 / 0.500 - 0.767 |  |
| storage node stor-xNN-1 (GPU racks) | U28-U29 | 1.300 - 1.389 | front; not serviced |
| storage node stor-xNN-2 (GPU racks) | U32-U33 | 1.478 - 1.567 | front; not serviced |
| management switch mgmt-xNN (GPU racks) | U40 | 1.834 - 1.878 | front; fed from A6 |
| leaf switch x01-1 (network racks) | U16 | 0.767 - 0.811 | rear rails, ports to the hot aisle, 0.62 m deep |
| leaf switch x01-2 (network racks) | U18 | 0.856 - 0.900 | rear rails, ports to the hot aisle, 0.62 m deep |
| leaf switch x01-3 (network racks) | U20 | 0.945 - 0.989 | rear rails, ports to the hot aisle, 0.62 m deep |
| leaf switch x01-4 (network racks) | U22 | 1.033 - 1.078 | rear rails, ports to the hot aisle, 0.62 m deep |
| fibre patch panel (network racks) | U40 | 1.834 - 1.878 | front; not serviced |
| fibre patch panel (network racks) | U42 | 1.922 - 1.967 | front; not serviced |

* **Rear doors**: one per rack, perforated, hinged on the rack's rear post at rack-frame x = -0.29 (on the right as
  seen from the hot aisle), opening outwards into the hot aisle; the handle is a vertical bar on the free edge at about
  x = +0.27, 1.03-1.17 m above the floor; the door latches when closed. No front doors.
* **PDUs**: two vertical switched PDUs per rack at the rear corners, 0.30-2.00 m above the floor: PDU A (feed A,
  outlets A1-A6) at rack-frame x = +0.26 (the left rear corner as seen from the hot aisle), PDU B (feed B, outlets
  B1-B6) at x = -0.26 (the right rear corner).

## Aisles, containment, overhead

| item | position (pod frame) |
|---|---|
| cold aisle (contained) | -0.6 <= y <= 0.6, 0 <= x <= 3.0; translucent roof at 2.275 m |
| cold-aisle doors | at x = -0.03 (west) and x = 3.03 (east); each two glass leaves (0.58 m wide) that slide apart along y, up to 0.56 m each; handles on the leaves' inner edges, 0.97-1.13 m above the floor |
| hot aisle A / B | 1.8 <= y <= 3.0 / -3.0 <= y <= -1.8 |
| other pods' rows | rear faces at y = 3.0 and y = -3.0 (not this pod's) |
| fibre raceways (yellow) | over each row, centred at y = +1.42 and y = -1.42, 0.30 m wide, at 2.55 m |
| power busways A / B | over each row's front half, at 2.78 m, a tap-off box above every rack |

## Service area

| item | position (pod frame) |
|---|---|
| parts cart | centred at (-0.95, -1.55): 0.60 m (x) by 0.90 m (y), top at 0.80 m; spares lie on its top |
| RMA tote (red) | on its steel stand, centred at (-0.95, -2.26) just south of the cart: 0.56 m (x) by 0.40 m (y), 0.30 m deep, its bottom at 0.45 m and its rim at 0.75 m, open at the top |
| ESD bench | -1.85 <= x <= -0.95, 2.00 <= y <= 2.75, top at 0.90 m |
| spares cage | -4.2 <= x <= -2.2, -2.2 <= y <= 2.2 (the store's; closed during the shift) |
| robot dock | about (-1.2, -0.9) |

## Cable plant

Node port n of node m (node numbers: a02 = 1 .. a05 = 4, b02 = 5 .. b05 = 8) goes to port m of the rail n-1 leaf:
rails 0-3 are `leaf-a01-1` .. `leaf-a01-4` (A01 U16, U18, U20, U22), rails 4-7 are `leaf-b01-1` .. `leaf-b01-4` (B01
U16 .. U22). Each trunk is an MPO-12 OM4 multimode cable with Type-B polarity, labelled at both ends with
`<node rack>-08:<node port>/<leaf rack>-<leaf U>:<leaf port>`; 7 m to the same row's network rack, 10 m to the other
row's. A cable's serial is in `dcim.cables` and on its ticket.

## Outlet map

The same in every rack of a kind: in a GPU rack the node's six supplies are on A1, A2, A3 (PSU1, PSU3, PSU5) and B1,
B2, B3 (PSU2, PSU4, PSU6); A4/B4 and A5/B5 feed the two storage nodes; A6 feeds the rack's management switch (the
BMCs' network) alone; B6 is spare. In a network rack leaf k is on Ak and Bk (k = 1..4); 5 and 6 are spare.
