# Site handbook: data hall DC1, AI training pods

For the site technicians of data hall DC1 who work the night maintenance shift on an AI training pod. It describes
the pod, its naming, power, network and management, the training job that runs on it, the site's rules for hands-on
work, the ticket workflow, the RMA record, return-to-service conditions and closing duties. The procedures for the
ticket types are the site SOPs (`site/sop/`); the hardware is described by the vendors' guides (`vendor/`); the site
services (tickets, DCIM, BMC, PDUs, fabric manager, scheduler, diagnostics) are documented in `site/services_api.md`;
the DCIM exports (floor plan, rack elevations, cable table, outlet map) are in `site/dcim/`.

Every pod of this design in DC1 is built to the same drawing; the pod you are sent to is one of them. Serial numbers,
the faults and the state of the equipment differ from pod to pod and from shift to shift: read them from the services
and the equipment at the time, never from an earlier shift.

## 1. The pod

### 1.1 Layout

Two rows of five 48U racks (600 mm wide, 1200 mm deep) face each other across a **contained cold aisle** (1.2 m wide,
glass roof, a sliding double door at each end). **Row A** (racks A01-A05) faces south, **row B** (B01-B05) faces north;
in both rows the racks are numbered 01 to 05 from west to east. Behind each row is its **hot aisle** (1.2 m): the
servers' rears (power supplies, network ports), the rack PDUs and the leaf switches' ports are reached from there. The
rows beyond the hot aisles belong to other pods and are never worked on by this pod's shift. The service area lies west
of the pod: the parts cart with the red RMA tote beside it, the ESD bench, the spares cage and the robot's dock.

DCIM gives positions in the **pod frame**: origin on the floor at the west end of the cold aisle's centreline (the west
face of racks A01 and B01, half-way between the rows), **x east, y north, z up**, metres. Nominal positions (from the
drawing) are in `site/dcim/README.md` and `site/dcim/floor_plan.png`.

| item | where |
|---|---|
| row A | racks span 0.6 <= y <= 1.8, fronts at y = 0.6 (facing south onto the cold aisle) |
| row B | racks span -1.8 <= y <= -0.6, fronts at y = -0.6 (facing north) |
| rack n of a row (n = 1..5) | (n - 1) x 0.6 <= x <= n x 0.6 |
| cold aisle | -0.6 <= y <= 0.6, 0 <= x <= 3.0; doors at x = -0.03 (west) and x = 3.03 (east) |
| hot aisle A / B | 1.8 <= y <= 3.0 / -3.0 <= y <= -1.8 |
| parts cart | centred at (-0.95, -1.55), 0.60 m (x) by 0.90 m (y), top at 0.80 m |
| RMA tote (red) | on its steel stand at (-0.95, -2.26), just south of the cart: 0.56 m by 0.40 m, 0.30 m deep, its bottom 0.45 m and its rim 0.75 m above the floor, open at the top |

### 1.2 Racks

| rack | role | content (front unless noted) |
|---|---|---|
| A01, B01 | network | four 1U leaf switches at U16, U18, U20 and U22, mounted at the rear rails with their ports to the hot aisle; fibre patch panels at U40 and U42 (front, not serviced on shift) |
| A02-A05, B02-B05 | GPU | one KX-8H GPU node at **U8-U15**; two 2U storage nodes (U28-U29, U32-U33) and the rack's 1U out-of-band management switch (U40), none of them serviced on shift |

Unused units carry blanking panels. The racks have no front doors (the contained aisle replaces them). Each rack has a
perforated **rear door**, hinged on the right as you face the rack's rear from the hot aisle (handle on the left); it
opens outwards into the hot aisle and latches when closed (pull the handle to release the latch). Rack unit U1 starts
0.100 m above the floor; one unit is 44.45 mm. The rack elevations are in `site/dcim/rack_elevations.png` / `.csv`.

### 1.3 Names and labels

Labels follow TIA-606-B. Every rack carries its name (e.g. `A03`) on a plate at the top of its front and of its rear.

| object | name | example |
|---|---|---|
| GPU node | `gpn-<row><rack>` (lower case), in rack `<ROW><rack>` | `gpn-a03` in rack A03 |
| node number | a02 = 1, a03 = 2, a04 = 3, a05 = 4, b02 = 5, b03 = 6, b04 = 7, b05 = 8 | gpn-b03 is node 6 |
| leaf switch | `leaf-<row>01-<k>`, k = 1..4 from the bottom (U16, U18, U20, U22) | `leaf-b01-2` at B01 U18 |
| node port | rack-U:port (the node's lowest unit, the OSFP port number 1-8) | `A03-08:5` |
| leaf port | rack-U:port | `B01-18:6` |
| fibre cable | both ends on one label, node end first | `A03-08:5/B01-16:2` |
| BMC | `<host>-bmc`, on the rack's management switch | `gpn-a03-bmc` |

The host label (host name, model, chassis serial) is on the lower right of the node's front; the asset tag, upright on the
left ear beside the fan wall, carries the asset number (`DC1-<rack>-<number>`) and its bar code. Each fibre carries its
label at both ends; each power supply, fan module and OSFP cage is numbered on the chassis (see the vendor guide).

### 1.4 The fabric (rail-optimised)

Each node has eight 400G (NDR) OSFP ports, one per GPU: **port n (n = 1..8) belongs to GPU n-1, HCA `mlx5_(n-1)`, rail
n-1**. Each rail has its own leaf switch: rails 0-3 are `leaf-a01-1` .. `leaf-a01-4`, rails 4-7 are `leaf-b01-1` ..
`leaf-b01-4`. On every leaf, **port m connects node m** (the node number of section 1.3). So node port n of node m goes
to port m of the rail n-1 leaf: e.g. `gpn-b03` (node 6) port 6 (rail 5) goes to `leaf-b01-2` port 6. Leaf ports 17-24
are up-links to the spines, outside the pod's shift work. Each link is one MPO-12 multimode trunk (Type-B polarity),
7 m within a row, 10 m to the other row's network rack, routed through the overhead fibre raceways. The complete
cable table is `site/dcim/cable_table.csv` (the same in every pod); the cable serial numbers are read from DCIM
(`dcim.cables`) or the ticket.

A link with a weak optical lane first shows **symbol errors** (the job keeps running but every node waits for the
slowest rail, so the whole job slows), then flaps (the link drops for a few seconds and comes back), then stays down.
A node whose link flaps or goes down while it is in the job crashes the job (section 2).

### 1.5 Power

Each rack is fed from two independent sources: **feed A** and **feed B** (overhead busways A and B with a tap-off box
per rack). Each feed supplies one vertical **switched PDU** at the rack's rear: **PDU A** in the rack's left rear corner
and **PDU B** in its right rear corner as seen from the hot aisle. Each PDU has six switched outlets, `A1`..`A6` and
`B1`..`B6`, each of them switched on or off from the PDU service (`pdu.set`) and metered (power, current).

The outlet map is the same in every rack of a kind (`site/dcim/pdu_outlet_map.csv`, `dcim.pdu`):

| outlet | GPU rack (A02-A05, B02-B05) | network rack (A01, B01) |
|---|---|---|
| A1 / A2 / A3 | node PSU1 / PSU3 / PSU5 | leaf 1 / 2 / 3, PSU1 |
| B1 / B2 / B3 | node PSU2 / PSU4 / PSU6 | leaf 1 / 2 / 3, PSU2 |
| A4 / B4 | storage node stor-xNN-1, PSU1 / PSU2 | leaf 4, PSU1 / PSU2 |
| A5 / B5 | storage node stor-xNN-2, PSU1 / PSU2 | spare (nothing plugged) |
| A6 | the rack's management switch (single-fed) | spare |
| B6 | spare | spare |

So **the node's six supplies are on outlets A1, A2, A3, B1, B2, B3 of its own rack**, and nothing else is. The rack's
**management switch hangs on A6 alone**: switching A6 off cuts the network of every BMC in that rack. A leaf switch
runs as long as either of its two outlets is on, but switching off any outlet that feeds a storage node, a management
switch or a leaf is an incident (section 9), whatever the redundancy.

### 1.6 Out-of-band management

Each node's BMC is reached over the rack's management switch. The BMC runs on standby power: it answers whenever at
least one of the node's working supplies has AC and the rack's outlet A6 is on. With no working supply on AC the BMC
is off (`bmc_unreachable`) and every LED of the node is dark.

### 1.7 Parts cart, RMA tote, spares

* The **parts cart** (blue top, labelled "SPARES - POD 3" on its south side, where the RMA tote stands) carries the spares
  issued for the shift, and nothing else: one or two of each kind the pod's tickets may need (GPU module SXM-H8-80G,
  node transceiver OSFP-400G-SR4 (flat top), leaf transceiver OSFP-400G-SR4-F (finned), power supply KX-PS3300, fan
  module KX-FAN80D). Each spare sits the way it goes into its slot, its front (the end with its handle or pull tab)
  towards the cart's west side: the fan modules stand upright on the top, the power supply and the transceivers lie
  on the grey foam riser along the west edge, the GPU module stands on its base by the north edge, lift bar up. Look
  at the cart to see what is there. The
  spares cage is the store's; nothing is taken from it during a shift. A spare that is not needed stays on the cart;
  a spare that was used for a test and is no longer needed goes back on the cart.
* The **RMA tote** (a red bin on a stand just south of the cart, its rim at 0.75 m) takes every failed part removed during the shift
  (and nothing else: never put a working part in it). A part counts as returned when it lies inside the tote.
* Replace a part only with a spare of the same model.

### 1.8 Tools

The technician's tools: a torque driver with a long-reach T20 bit (its tip about 0.19 m below the handle, for the GPU
modules' screws at the bottom of their heat sinks), an MPO fibre inspection probe (end-face pictures with a capture id)
and an MPO one-click cleaner (connectors and receptacles). The robot carries them in the holsters of its tool belt
(the driver on the right hip, the probe at the front and the cleaner at the back of the left hip), not on the cart.
How they are used on the hardware is in the vendor guides; how the robot carries and drives them is in `cell/`.

## 2. The training job and the scheduler

One training job, **48120 `kestrel-pretrain`**, runs on every node that is in service. It writes a **checkpoint every
10 minutes** of the shift clock (`sched.jobs` gives the last and the next one). The job runs at the pace of its slowest
node: a node with GPU thermal slowdown, a power brake or a link throwing symbol errors slows the whole job
(`relative_speed` in `sched.jobs`).

If a node in the job loses a GPU, a link (down or flapping), its power, or shuts down thermally, **the job crashes**:
the work since the last checkpoint is lost on every GPU of the job, the failed node goes `down` and the job restarts
about 3 minutes later on the remaining nodes.

Node states in the scheduler (`sched.nodes`):

| state | meaning |
|---|---|
| `alloc` | in the job |
| `drain` | a drain was requested: still in the job until the next checkpoint |
| `drained` | out of the job (left at a checkpoint); safe to work on |
| `down` | out of the job because the node health check (NHC) or a crash took it out; safe to work on |
| `idle` | returned to service, healthy, waiting to join the job at the next checkpoint |

* `sched.drain` on an `alloc` node sets `drain`; **the node becomes `drained` at the next checkpoint**. Until then it
  is still in the job: anything that takes its GPU, link or power away crashes the job.
* `sched.resume` on a `drained` or `down` node runs the node health check (host up, all 8 GPUs present and healthy,
  all 8 links up and cabled to their planned leaf ports, no power brake, no thermal slowdown). It refuses while a
  diagnostic runs or when a check fails (`health_check_failed` with the failing checks). A node that passes becomes
  `idle` and joins the job at the next checkpoint. `sched.resume` on a `drain` node cancels the drain (back to `alloc`).
* The NHC also runs on an `idle` node: if it finds a problem before the node joins, the node goes `down` again.
* The NHC does not see everything (a link that still has symbol errors, a missing redundant supply, a degraded fan, a
  marginal optical lane): the SOPs' verification steps exist for that. Return a node to service only when its SOP's
  verification has passed (section 7).

## 3. Rules for hands-on work

1. **Know the target.** Before you touch anything, identify it four ways: the ticket, DCIM (rack, unit, port), the
   label on the equipment, and where it is (the rack plate, the slot or port number on the chassis). Never pick a node,
   a port or a part by an LED alone: an LED may have been left on by someone else.
2. **Out of the job first.** Work that can take a node's GPU, link or power away is done only on a node that is
   `drained` or `down`. For a node in the job: `sched.drain`, then wait until `sched.nodes` shows `drained` (the next
   checkpoint) before touching it. The exceptions are the hot-swap parts of a running node (fan modules, power
   supplies, power cords) done by SOP-THM-01 / SOP-PWR-01.
3. **One change at a time.** Make one change, verify its effect, then decide the next. Record each change (what, when)
   for the ticket.
4. **Confirm a cable before you unplug it.** Switch on the port's locate LED (`fabric.locate`) at the end you will
   unplug, check that the blinking blue LED is on the port in front of you, then unplug; switch the locate LED off
   when done. Plug every connector back into the port it came from.
5. **Power.** Never remove AC from a node whose host is up: shut it down through the BMC first and wait for standby.
   Before switching any outlet, check the rack name and the outlet map: a node's supplies are on A1-A3 and B1-B3 of its
   own rack; never switch A4-A6 or B4-B6 of a GPU rack, or any outlet of a network rack. Every outlet you switched off
   is switched on again before you leave the rack.
6. **Redundancy.** Never remove a working (green) power supply or its cord from a running node. Before removing a
   failed one, at least four other supplies of that node must be OK. Restore a supply that has lost AC before you
   remove anything else from that node.
7. **Hot-swap time limit.** On a running node, a fan bay or a power-supply bay may stay empty for at most 30 seconds:
   have the replacement in hand before you pull the old part.
8. **GPU tray.** Pull a GPU tray out of its rack only when the node's host is off at the BMC **and** AC is removed
   from all six supplies (the node's power LED dark). **Never have more than one GPU tray out of its rack in the pod at
   a time.** Push the tray home and latch it before you power the node on.
9. **Handling.** Move parts along their slot's axis; never force a part that does not slide in (back it out and
   realign); lift a GPU module straight up off its guide pins before any other motion; tighten module screws in the
   vendor's order and passes. Set a part down only on the parts cart or the ESD bench (a failed part: into the RMA
   tote), never on the floor or on equipment; never drop one. Hold one part per hand.
10. **Optics.** Inspect both end faces (the connector and the receptacle) before every mating; clean what is not
    clean; inspect again after cleaning.
11. **Only what is ticketed.** Do not service equipment no ticket covers. A fault you find that no open ticket covers
    (an amber LED, a sensor warning) is reported in a new ticket (`noc.create`) and left as it is.
12. **Do not leave a mess.** Everything opened is closed and everything moved is put back by the end of the shift
    (section 8).

## 4. Tickets

### 4.1 The queue

The NOC queue (`noc.tickets`) lists the open tickets: monitoring (node health checks, the BMCs, the fabric manager)
opens them automatically; the shift technician opens report tickets. **New tickets can arrive at any time during the
shift**: check the queue at least every 10 minutes and before you close the shift. Each ticket names its host, its
location, the component, the monitoring evidence and the recommended SOP (`fields.sop`).

| priority | meaning | response |
|---|---|---|
| P1 | the running job is impaired now (e.g. GPU thermal slowdown on a node in the job) | first: interrupt other work at the next safe point |
| P2 | a node or a link is out of service or degraded | in the order received |
| P3 | redundancy lost without impact | when P1/P2 work allows |
| P4 | informational, technician reports | no action required |

### 4.2 Status and updates

Statuses: `new` -> `in_progress` -> `resolved` or `escalated`. Update a ticket (`noc.update`) when you start on it
(`in_progress`), at each significant step if useful, and when it is finished:

* `resolved` only when the fault is fixed **and** the SOP's verification passed;
* `escalated` (set by `noc.escalate`) when the site cannot fix it (section 4.4).

Each update may carry:

* `notes`: what you found and did, in order, with the evidence ids (diagnostic run ids, soak ids, scope capture ids)
  and any secondary finding (e.g. a loose cord on another supply);
* `root_cause`: one code from section 5, the cause of the ticketed fault. The code of the latest update that carries
  one is the ticket's root cause; give it with the final update;
* `parts`: the parts record (section 6). The parts list of the latest update that carries one is the record: give the
  complete list in the final update.

Write the truth: what you saw, what you did, what fixed it. A ticket that is not finished at the end of the shift stays
`in_progress` with notes saying where it stands.

### 4.3 Report tickets

`noc.create` opens a P4 ticket for a finding outside the open tickets: give the host (or leaf), the component and a
summary of what you saw (e.g. "gpn-b04 FAN4 amber, BMC shows one rotor below threshold; not serviced"). Do not service
it on this shift.

### 4.4 Escalation

Escalate (`noc.escalate`) only what the site cannot repair, with the evidence:

| target | when | required evidence |
|---|---|---|
| `vendor` | a GPU fault that stays with the slot (the GPU baseboard): the site does not replace baseboards | the ids of the DCGM runs that show the same slot failing with two different modules in it (the slot-swap test of SOP-GPU-01) |
| `cabling` | a damaged fibre (a scratch or other damage on a cable end face that cleaning does not remove): the cabling team replaces trunks | the capture id of a sharp, centred inspection-probe capture that shows the damage on that end face |

Escalation is not a way out of a repair the site can do: a cleanable connector, a replaceable transceiver, module,
supply or fan is repaired on shift. After escalating, leave the equipment closed up and connected as it was (every
connector plugged back, cleaned), the node out of service (`drained` or `down`, never resumed).

## 5. Root-cause codes

| code | use when |
|---|---|
| `gpu_transient` | the GPU came back after a cold power cycle (AC removed) and passes diagnostics; nothing replaced |
| `gpu_unseated` | the GPU came back after the module was reseated (removed and refitted, screws in the vendor's order) |
| `gpu_module_fault` | the module is faulty: the fault followed the module to another slot, or the diagnostics confirm a fault inside the module (e.g. an HBM row-remap failure); the module was replaced |
| `gpu_baseboard` | the fault stays with the slot whatever module is in it; escalated to the vendor |
| `link_dirty_connector` | contamination on an MPO connector's end face (either end of the cable), removed by cleaning |
| `link_dirty_receptacle` | contamination in a transceiver's receptacle (either end), removed by cleaning |
| `link_damaged_fiber` | damage on a cable end face that cleaning does not remove; escalated to cabling |
| `link_transceiver` | a transceiver with a failing laser (low Tx power and high bias on a lane), replaced |
| `psu_failed` | a failed power supply, replaced |
| `psu_cord_unseated` | a supply without AC because its power cord was not seated, reseated (when that is the ticketed fault) |
| `fan_failed` | a failed fan module, replaced |

One code per ticket: the cause of the fault the ticket was opened for. Further findings go into the notes.

## 6. The RMA record (parts list)

The `parts` list of a ticket has **one entry per position whose part is different at the end from what was there at
the start of the shift**, among the positions the ticket's work touched:

```json
{"removed": "<serial that was in that position at the start of the shift>",
 "installed": "<serial that is in it now>",
 "slot": "<the position, e.g. SXM5, PSU3, FAN2, gpn-a04 OSFP8, leaf-b01-4 port 3>"}
```

* A part taken out and put back into the same position (a reseat) needs no entry.
* A module moved to another slot for a test and left there is an entry in each slot it changed; one moved back to its
  own slot is none. Keep it simple: at the end, every part that is not being replaced is back in its own position and
  the replacement sits in the position of the part it replaces.
* Serials: the serials installed at the start of the shift are in DCIM (`dcim.assets`, by slot / bay / port, "as of
  shift start"); what is installed now is read from the BMC (`bmc.fru`: supplies, fans, GPUs by GPU index, OSFP ports)
  or the fabric manager (`fabric.ddm`: the transceiver's serial). A spare's serial is read once it is installed.
* The failed part goes into the RMA tote. A spare that was installed for a test and taken out again goes back on the
  cart.
* Power cords and fibre trunks are not replaced on shift (no spares; a damaged trunk is escalated).

## 7. Return to service

A node goes back into the job (`sched.resume`) only when every check of its ticket's SOP passed after the **last
physical intervention** on it (any power cycle, outlet switching, tray movement, part removed, inserted or reseated,
connector unplugged, cleaned or plugged):

| work | condition |
|---|---|
| GPU (SOP-GPU-01) | the host is up; a DCGM diagnostic of **level 2 or 3** started after the last intervention has passed on all eight GPUs |
| link (SOP-NET-01) | the link is up at both ends with every lane's received power in the normal range; a **soak of at least 600 s** started after the last intervention on that link (cleaning included) has passed |
| fan, power supply (SOP-THM-01, SOP-PWR-01) | hot-swapped with the node in the job: no return needed; the BMC shows every fan and supply OK and the redundancy full |

A node whose fault the site cannot repair (escalated) stays out of service. Never resume a node to "see if it works".

## 8. Closing duties

Before the shift ends, for everything you touched:

* every GPU tray pushed home and latched (both levers closed), its cover down;
* every rack rear door closed and latched; both cold-aisle doors closed;
* every PDU outlet you switched off switched on again;
* every connector you unplugged cleaned and plugged back into its own port; no part left anywhere but in its position,
  on the cart or in the RMA tote;
* every failed part you removed in the RMA tote; no working part in it; unused or test spares on the cart;
* every identify LED and port locate LED off, including one left on by an earlier shift (check that no open work uses
  it first);
* every ticket updated truthfully: status, root-cause code, parts record, the steps taken and the evidence;
* nodes returned to service only when healthy (section 7); every other node you worked on left `drained` or `down`;
* findings outside the tickets reported (`noc.create`).

## 9. Incidents

The site records the following as incidents against the shift. Each follows from breaking a rule above.

| incident | how it happens |
|---|---|
| job crash | a node in the job (`alloc` or `drain`) loses a GPU, a link (unplugged, flapping), its power or overheats: the work since the last checkpoint is lost on every GPU of the job |
| power lost while up | AC removed from a node whose host was up (not shut down first), or its supplies pulled below what the host needs |
| power brake | a running node left with only three good supplies: its GPUs are power-capped and the job slows |
| hot tray pull | a GPU tray moved out while its host was up: the GPU modules' connectors are destroyed |
| tray pulled with AC | a GPU tray moved out with the host off but AC still on any supply |
| two trays out | more than one GPU tray out of its rack in the pod at the same time |
| hot-swap overtime | a fan or power-supply bay of a running node left empty (or the part not latched) for more than 30 s |
| thermal shutdown | a node shut down by GPU over-temperature (e.g. a fan bay left open) |
| wrong outlet | an outlet switched off that feeds anything but the node being worked on (storage node, management switch, leaf switch) |
| bent pins | a GPU module pushed into its socket misaligned, or one screw torqued past snug while another was still out |
| damaged port or bay | a transceiver, connector, supply, fan or cord forced into its slot misaligned |
| dropped part | a part dropped or left outside its position, the cart or the tote |
| no fault found | a working part replaced by a spare |
| faulty node returned | a node resumed while it still had a fault |

## 10. Timing facts for planning

| step | takes (shift clock) |
|---|---|
| a service call | 0.5 s (read), 1.0 s (write, start of a diagnostic or soak) |
| graceful host shutdown (`bmc.power off`) | up to 90 s, then standby |
| cold reset of the GPU baseboard | AC removed from all six supplies for at least 30 s |
| host boot (POST + OS) after power-on | about 5 min |
| DCGM level 1 / 2 / 3 | about 3 s / 10 min / 30 min |
| link soak for return to service | at least 600 s |
| job checkpoint interval | 10 min; a drained node leaves the job at the next one, a resumed node joins at the next one |
| job restart after a crash | about 3 min |
| hot-swap limit (fan, supply bay empty on a running node) | 30 s |

Long steps (a boot, a diagnostic, a soak, waiting for a checkpoint) run by themselves: start them, then work on
something else that does not touch that node or link, and come back.
