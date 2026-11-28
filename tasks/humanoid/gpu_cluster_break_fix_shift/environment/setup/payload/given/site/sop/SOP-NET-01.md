# SOP-NET-01: Bad link (node to leaf optical link)

| | |
|---|---|
| applies to | fabric manager events on a node-to-leaf link: 916 "High Symbol BER", link flapping, link down (node OSFP port n, rail n-1, to its leaf's port m = the node number) |
| order | NVIDIA's bad-link order, one change at a time: isolate, clear counters, pull out / inspect / clean / reinsert, then transceiver, then fibre (cabling team); validate and soak |
| references | vendor/osfp_mpo_guide.md (lanes, polarity, DDM, inspection, cleaning, handling), site_handbook.md |
| spares | two node-side transceivers OSFP-400G-SR4 (flat top) and two leaf-side transceivers OSFP-400G-SR4-F (finned) on the parts cart |

## 0. Identify the link and take it out of the job

1. `noc.ticket`: host, node port, rail, leaf, leaf port, cable label and serial. Confirm the cable in `dcim.cables` (or
   `site/dcim/cable_table.csv`): node end `<rack>-08:<n>` at the node's rear, OSFP cage n; switch end `<rack>-<U>:<m>`
   at the network rack's rear, leaf port m. Set the ticket `in_progress`.
2. `sched.nodes`: the node is usually still in the job (`alloc`). `sched.drain` it (reason: the ticket id) right away,
   then wait until it shows `drained` (it leaves the job at the next checkpoint; `sched.jobs` shows when). **Until
   then do not unplug, clean, swap or reseat anything on this link**: a link that drops while the node is in the job
   crashes the job. Reading DDM and counters is fine meanwhile.

## 1. Read the evidence (both ends powered)

Both ends must be powered to read them: the node's host up (its transceivers are read through the host) and the leaf
switch powered.

1. `fabric.port` at both ends: status, peer, symbol errors, link-down count, BER.
2. `fabric.ddm` at both ends: per lane Tx power, Rx power, laser bias; module temperature; alarms.
3. Find the weak lane(s): every receive lane should be close to the others and in the normal range (about -2 to
   -2.5 dBm on a healthy link; error-free operation needs at least -5.0 dBm). Look at all eight receive lanes (four at
   each end).
4. Trace each weak lane to its source with the lane map (osfp_mpo_guide.md section 2): receive lane **r** at one end
   is fed by transmit lane **r** of the far end (lane n reaches lane n); at the near end it is fibre position
   **12-r**, at the far end position **r+1**.
5. Decide:

| far end's Tx lane r | bias of that lane | meaning | go to |
|---|---|---|---|
| clearly lower than its other lanes (a dB or more) | high (`tx_bias_high`, above 10 mA) | the far end's laser is failing | step 3 (replace that transceiver) |
| normal (like its other lanes) | normal | light is lost between the lasers and the receiver: contamination or damage on one of the four end faces on that fibre (the near connector and receptacle at position 12-r, the far connector and receptacle at position r+1) | step 2 (inspect and clean) |

Replace a transceiver only on laser evidence: a transceiver whose laser readings are normal is not faulty because its
link is dirty (replacing it is a "no fault found").

6. `fabric.clear` at both ends, so the counters show only what happens from now on.

## 2. Inspect and clean (connector and receptacle, both ends)

Work one end at a time; the link is down while a connector is out (that is why the node must be `drained`).

1. `fabric.locate` `on` for the port you will open; check that the blinking blue LED is on the port in front of you.
2. Unplug the MPO connector: pull its housing straight out along the port's axis (push-pull latch).
3. Inspect the connector's end face with the probe: a sharp, centred capture (note its capture id). Inspect the
   transceiver's receptacle: the probe into the empty port.
4. Clean every face that does not pass (one push of the one-click cleaner = one cleaning), inspect again; repeat until
   it passes, at most five cleanings per face. Also clean a face with loose particles anywhere on its fibres: loose
   particles move onto the other face when the two are mated.
5. A defect that does not change at all over two cleanings is not dirt. A **scratch** (a thin straight dark line)
   across a fibre's core on the cable's connector means the trunk end is damaged: keep the id of a sharp, centred
   capture that shows it and go to step 4 once both ends are inspected. Damage on a receptacle means the transceiver
   is damaged: replace it (step 3).
6. Plug the connector back into the same port: square and straight in until it latches. `fabric.locate` `off`.
7. Do the same at the other end.
8. `fabric.ddm` at both ends: every lane back in the normal range? Then step 5. A lane that improved but is still low:
   inspect again (a thin residue film is faint in the picture and fades with each cleaning). A lane that did
   not change: look again at which faces are on its fibre (the lane map) and at the far end's laser readings.

## 3. Replace a transceiver (laser evidence)

1. Take the spare of the right kind: node side OSFP-400G-SR4 (flat top), leaf side OSFP-400G-SR4-F (finned).
2. `fabric.locate` `on` at that port, confirm the port, unplug its MPO connector (a transceiver cannot be pulled with
   its fibre plugged).
3. Pull the transceiver out by its beige pull tab, straight out of the cage. It goes into the RMA tote.
4. Insert the spare: square to the cage, slide it in, push until it latches. Never force it.
5. Inspect the spare's receptacle and the connector; clean if needed; plug the connector in. `fabric.locate` `off`.
6. `fabric.ddm` at both ends: the new transceiver's lanes and the far end's receive lanes in the normal range.
7. Parts record: the position (e.g. `gpn-a04 OSFP8` or `leaf-b01-4 port 3`), the removed transceiver's serial, the
   installed one's serial (`fabric.ddm` shows the module serial; DCIM has the shift-start serials).

## 4. Damaged trunk: escalate to cabling

The site does not replace fibre trunks.

1. Plug both ends back (cleaned), locate LEDs off.
2. `noc.escalate` to `cabling` with the capture id of a sharp, centred capture of the damaged end face, and notes
   (which end, which fibre position, the DDM readings).
3. `noc.update`: status `escalated`, root cause `link_damaged_fiber`, notes.
4. Leave the node `drained`; never resume it while its link is degraded.

## 5. Verify and return to service

1. `fabric.ddm` at both ends: every receive lane in the normal range, no alarms.
2. `fabric.clear` at both ends.
3. `fabric.soak` on the link (either end), **at least 600 s**, started after the last change on the link (plugging,
   cleaning, swapping). Do not touch the link while it runs.
4. `fabric.soak_result`: `passed` (no symbol error, at most one link-down, the link up at the end). A failed soak: back
   to step 1.
5. `sched.resume` the node (it joins the job at the next checkpoint).
6. `noc.update`: status `resolved`, the root cause, the parts record if a transceiver was replaced, notes with the DDM
   readings before and after, the capture ids, the soak id. The root cause is where the loss on the weak lane was:
   contamination over that fibre's core on a cable connector (`link_dirty_connector`) or in a transceiver receptacle
   (`link_dirty_receptacle`), or a failing laser (`link_transceiver`).
