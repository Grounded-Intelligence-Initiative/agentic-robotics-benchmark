# SOP-PWR-01: Power supply hot swap (KX-8H)

| | |
|---|---|
| applies to | BMC alerts on a KX-8H's power supplies: a supply failure, a supply without AC, power redundancy degraded |
| method | hot swap: the node stays in the job; do not drain it |
| references | vendor/kx8h_service_guide.md (power, supplies, cords, LEDs), site_handbook.md (power, outlet map, rules 5-7) |
| spares | one power supply KX-PS3300 on the parts cart |

The KX-8H has six 3.3 kW supplies, 4+2 redundant: PSU1, 3, 5 on feed A (outlets A1, A2, A3 of the node's rack), PSU2,
4, 6 on feed B (B1, B2, B3). The node runs at full power on any four good supplies. With only three the BMC asserts the
**power brake** (the GPUs are power-capped and the whole job slows); with two or fewer the host loses power (a job
crash). So on a running node: **never remove a green supply or its cord; never let a supply bay stay empty for more
than 30 s.**

## 1. Assess

1. `bmc.sensors`: the status of each supply (`ok`, `failure`, `ac_lost`, `absent`), `psu_redundancy`, `power_brake`.
   `bmc.sel`: the supply events and their times. Count the supplies that are `ok`.
2. At the node's rear (open the rack's rear door): the supply LEDs. Green = OK, amber = failed (it has AC but no
   output), dark = no AC at that supply (or the bay is empty). The supplies sit in two rows of three in the middle of
   the chassis rear: PSU1-3 on top and PSU4-6 below, each row from the left as seen from the hot aisle; each face is
   labelled (e.g. `PSU3 A`).
3. `dcim.pdu` and `pdu.outlets` for the rack: the outlet of each supply, its state (on/off) and its power. A good
   supply draws its share of the node's load; a failed supply with AC draws a few watts; a supply without AC draws 0 W.

## 2. Restore every supply that has lost AC (before anything else)

For each supply that shows `ac_lost` (LED dark):

1. Its outlet switched off? Switch it on (`pdu.set`) if it is this node's outlet, and find out why it was off.
2. Its outlet on but delivering nothing (0 W) to the supply: the cord is not making contact. **Reseat the cord at the
   supply's inlet:** release and pull the locking plug straight out of the inlet, then push it straight back in until
   it latches. The supply stays in its bay (no time limit). Within seconds its LED turns green and the BMC shows it
   `ok`.
3. Never touch the cord of a supply that is `ok`.

## 3. Hot-swap the failed supply

1. Check: besides the failed supply, at least **four** supplies are `ok`. If not, do not remove anything: restore AC
   first (step 2), or report in the ticket.
2. Bring the spare (KX-PS3300) to the node's rear, in hand.
3. Unplug the failed supply's cord at its inlet (a supply cannot be pulled while its cord is plugged in).
4. Grip the failed supply's black pull handle (the grip presses the green release tab) and pull it straight out of its
   bay. **From now the bay is empty: 30 s.**
5. Insert the spare square into the same bay and push it home until it latches.
6. Plug the cord back into the new supply's inlet until it latches.
7. The failed supply goes into the RMA tote.

## 4. Verify

1. `bmc.sensors`: all six supplies `ok`, `psu_redundancy` `full`, `power_brake` false. All six supply LEDs green.
2. `pdu.outlets`: the node's load shared over its six outlets.

## 5. Close

1. Close the rack's rear door.
2. `noc.update`: status `resolved`; root cause: the cause of the ticketed fault (`psu_failed` for a failed supply,
   `psu_cord_unseated` when the ticketed fault was a supply without AC because of its cord); parts record:
   `{"removed": <failed supply's serial>, "installed": <spare's serial>, "slot": "PSU<n>"}` (serials: `dcim.assets` for
   the shift start, `bmc.fru` for what is in the bay now); notes: what you found and did, including any cord you
   reseated on another supply (which supply, which outlet).
