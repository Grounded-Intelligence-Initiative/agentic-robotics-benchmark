# SOP-GPU-01: GPU fault triage (KX-8H)

| | |
|---|---|
| applies to | tickets for a GPU of a KX-8H node that fell off the bus (Xid 79) or reports an HBM row-remap failure (Xid 64, Xid 48); the node health check (NHC) has set the node `down` |
| order | NVIDIA's GPU node triage order: logs, reset, diagnostics, reseat, isolate (slot swap), replace or escalate. One change at a time; diagnose again after each change |
| references | vendor/kx8h_service_guide.md (slot map, tray service, screws, DCGM), site_handbook.md (rules, records, return to service) |
| spares | one GPU module SXM-H8-80G on the parts cart |

A step that needs the node powered down costs a shutdown (up to 90 s), the AC wait (30 s), the work and a boot (about
5 min). Plan the steps, and do other work (that does not touch this node) while the node boots or a diagnostic runs.

## 0. Before you start

1. `noc.ticket`: host, GPU index, PCI address, Xid(s), related tickets. Set the ticket `in_progress`.
2. `sched.nodes`: the node must be `down` or `drained`. If it is in the job (`alloc` / `drain`), drain it and wait for
   `drained` before anything below.
3. Map the GPU index to the physical slot with the vendor's table (GPU index is **not** the slot number), e.g. GPU1 is
   SXM7. Note the slot's position in the tray (front or back row, column).
4. `bmc.sel`: the Xid entries of this fault; earlier Xid events on **other** GPU indices of the same node (several GPU
   indices dropping over days points at the baseboard: the slot-swap test of step 3 decides); power and thermal events.
5. `dcim.assets` for the host: the module serial in each slot as of the shift start (the RMA record needs it).
6. Check the cart for the spare module. Check that no other GPU tray in the pod is out (handbook rule 8).

## 1. Cold power cycle (AC removed)

A GPU that dropped off the bus for a transient reason comes back after the GPU baseboard has been without power. A
warm reboot or a BMC `cycle` does **not** do this: the baseboard's standby domain stays powered as long as any supply
has AC. Only removing AC from all six supplies for at least 30 s resets it.

1. `bmc.power` `off` (graceful). Wait until `bmc.power_state` is `standby` (up to 90 s; the power LED blinks slowly).
2. `dcim.pdu` for the node's rack: confirm that the node's supplies are on outlets A1, A2, A3, B1, B2, B3.
3. `pdu.set` those six outlets `off`. Check: `pdu.outlets` shows them off at 0 W; the BMC no longer answers; the
   node's power LED is dark.
4. Wait at least 30 s.
5. `pdu.set` the six outlets `on`. Wait until `bmc.power_state` answers `standby` (the host stays off when AC returns).
6. `bmc.power` `on`. The host boots: power LED blinking fast during POST, steady when the host is up (about 5 min).
   `bmc.power_state` `on`.
7. Diagnose:
   * Xid 79 ticket: DCGM level 1 (3 s) shows whether all eight GPUs are on the bus. If they are, run level 2: a pass on
     all eight is the fix (root cause `gpu_transient`): go to step 6. If the GPU is still missing, go to step 2.
   * Xid 64 / 48 ticket: run DCGM level 2 (level 1 does not test the memory). "HBM row remap failure" on the GPU
     confirms a memory fault inside the module: go to step 4 (replace the module; no swap test needed). A pass on all
     eight: go to step 6 (root cause `gpu_transient`).

## 2. Reseat the module

1. Power down and remove AC exactly as in step 1.1-1.3, and keep AC off until the tray is home and latched again.
2. At the node's front (cold aisle), by the vendor's tray procedure: open both release levers; pull
   the tray out to its service stop (it holds there); fold both levers back; lift the hinged cover to its stay.
3. Find the slot by its silk-screened name in front of the socket and the slot map. Run its four screws out (T20,
   loosen until free; they are captive and stay in the module).
4. Take the blue lift bar and lift deliberately, straight up: the module rides about 15 mm up its guide pins, then
   it is free. Make no other motion before it is clear of the pins; then lift it out.
5. Refit it: hold it level and square over the socket, lower it onto the guide pins (aligned within about 1.5 mm and
   2 deg) and set it down: it seats by its own weight. Never press down a module that does not drop onto its pins:
   lift, realign.
6. Fasten in two passes, each in the order **1-4-2-3**: pass 1 snug (0.3 N m), pass 2 final (0.9 N m). Never take a
   screw past snug while another screw of the module is not yet run down.
7. Close up: lower the cover; push the tray home (hold a lever's release tab to free the service stop) until both
   levers close and the tray latches. `bmc.sensors` will show `gpu_tray` `seated` once AC is back.
8. Restore AC (six outlets on), `bmc.power` `on`, wait for the boot, DCGM level 1:
   * the GPU is back: run level 2; a pass on all eight is the fix (root cause `gpu_unseated`): step 6;
   * still missing: step 3.

## 3. Slot-swap test: does the fault follow the module or stay with the slot?

Put a known-good module into the suspect slot and the suspect module elsewhere, then diagnose again. The known-good
module is the cart's spare (usually the shorter way: if the module is faulty, the spare simply stays) or a module from
a slot whose GPU passed (then the two modules exchange slots).

1. Power down, AC off, tray out, cover up (as in step 2).
2. Run out the screws of the suspect module and lift it out. With the spare: put the suspect module on the parts cart
   and fit the spare in the suspect slot. With a neighbour: run out the neighbour's screws too and exchange the two
   modules. Fasten every module you fitted (two passes, 1-4-2-3).
3. Close up, restore AC, power on, wait for the boot, run DCGM (level 1 for the presence check). Note the run ids.
4. Read the result by **GPU index** (convert with the slot map):
   * the suspect slot's GPU now passes (with the spare in it), or the failure moved to the GPU index of the slot the
     suspect module went to: **the fault follows the module**: step 4;
   * the suspect slot's GPU fails again with the other module in it: **the fault stays with the slot** (the baseboard):
     step 5. The DCGM run with the original module failing in the slot and the run with the other module failing in
     it are the evidence.

## 4. Replace the module

1. Power down, AC off, tray out, cover up.
2. The faulty module goes into the RMA tote. The spare SXM-H8-80G goes into the slot the faulty module came from.
   Every module that was moved for the test goes back into its own slot. Fasten every module you fitted (two passes,
   1-4-2-3). (If the spare already sits in the suspect slot after the swap test, it stays there and only the faulty
   module goes to the tote: no further tray work.)
3. Close up, restore AC, power on, wait for the boot.
4. Go to step 6 (verification). Root cause `gpu_module_fault`. Parts record: the slot, the faulty module's serial
   (removed), the spare's serial (installed, read from `bmc.fru` by GPU index once it is up).

## 5. Fault stays with the slot: escalate to the vendor

The site does not replace baseboards.

1. Power down, AC off, tray out: put every module back into the slot it had at the start of the shift; the spare goes
   back on the parts cart. Fasten (two passes, 1-4-2-3), close up, restore AC. The host may stay in standby or be
   booted; it is not returned to service.
2. `noc.escalate` to `vendor` with the evidence: the ids of the DCGM runs that show this slot's GPU failing with two
   different modules in it, and notes (Xid history, the steps taken).
3. `noc.update`: status `escalated`, root cause `gpu_baseboard`, notes. No parts record when every module is back in
   its own slot.
4. Leave the node `down` or `drained`. Never resume it.

## 6. Verify and return to service

1. The host is up and nothing has been touched since the last power-on: DCGM **level 2** (about 10 min) or level 3
   (about 30 min), started after the last physical intervention. All eight GPUs must pass.
2. A failure: read its message (below) and continue at the matching step.
3. `sched.resume` the node: it becomes `idle` and joins the job at the next checkpoint. If the resume is refused
   (`health_check_failed`), read the failing checks and fix them first.
4. `noc.update`: status `resolved`, the root-cause code, the parts record (if a module was replaced), notes with the
   steps in order and the DCGM run ids.

## DCGM results and what they mean

| message (per GPU index) | meaning | next |
|---|---|---|
| GPU not found on PCI bus `<address>` | the GPU is not on the bus (as in Xid 79) | the triage steps above, in order |
| HBM row remap failure (...) (level 2, 3) | the module's memory has an uncorrectable error it cannot remap | replace the module (step 4) |
| Targeted power: GPU power capped below target (system power brake) (level 3) | the node had only three good supplies during the run; not a GPU fault | SOP-PWR-01, then run the diagnostic again |
| Targeted stress: clocks throttled (thermal slowdown) (level 3) | the GPUs reached their slowdown temperature during the run; not a GPU fault | check the fans (SOP-THM-01), then run again |

A diagnostic needs the host up and the node out of the job (`drained` or `down`); only one runs at a time per node; it
aborts if the host shuts down or loses power. Level 1 checks presence only; use level 2 or 3 for the return to service.
