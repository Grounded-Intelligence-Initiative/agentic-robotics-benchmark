# SOP-THM-01: Fan module hot swap (KX-8H)

| | |
|---|---|
| applies to | BMC alerts on a KX-8H's fan modules: a fan module failed (0 RPM), GPU thermal slowdown caused by it |
| method | hot swap with the node in the job: do not drain it |
| priority | P1 when the node's GPUs are in thermal slowdown while it is in the job (the whole job runs at that node's pace) |
| references | vendor/kx8h_service_guide.md (front, fans, LEDs, thermal limits), site_handbook.md |
| spares | two fan modules KX-FAN80D on the parts cart |

The GPU tray is cooled by five dual-rotor fan modules (FAN1 to FAN5 from the left as seen from the cold aisle) in the
GPU tray's fan wall at the node's front. One failed module under full load drives the GPUs to their slowdown temperature (87 C): they throttle
and the job slows. **An empty fan bay is worse than a failed module** (air short-circuits through the opening): under
load the node reaches its GPU shutdown temperature (95 C) within about a minute, which crashes the job. Hence the
vendor's limit: **a fan bay of a running node may stay empty for at most 30 s.**

## 1. Assess

1. `bmc.sensors`: `fan_rpm` (two rotors per module, `FAN<n>_F` and `FAN<n>_R`, about 12,000 RPM each when healthy),
   `fan_status` (`ok`, `degraded` = a rotor below its threshold, `failed` = both rotors stopped, `absent`),
   `gpu_temp_c`, `gpu_thermal_slowdown`. `bmc.sel`: the fan and thermal events.
2. The module to replace is the one the ticket names, `failed` in the BMC; its LED is amber.
3. A module that is only `degraded` (one slow rotor) and that no ticket covers is not replaced on this shift: report it
   (`noc.create`) and leave it.

## 2. Swap

1. Take the spare (KX-FAN80D) from the cart and bring it to the node's front, in hand.
2. Count the bays from the left (FAN1 is the leftmost) and check the module's label and LED.
3. With the spare in one hand, grip the failed module's orange handle with the other (the grip releases its latch) and
   pull it straight out. **From now the bay is empty: 30 s.**
4. Insert the spare square into the same bay and push it home until it latches.
5. The failed module goes into the RMA tote.

## 3. Verify

1. `bmc.sensors`: all five modules `ok`, both rotors of the new module at their normal speed.
2. Within a few minutes the hottest GPU falls below 85 C and `gpu_thermal_slowdown` clears; `sched.jobs` shows the
   job's `relative_speed` back to normal. The node's status light bar turns cyan when no other fault is asserted.

## 4. Close

`noc.update`: status `resolved`, root cause `fan_failed`, parts record `{"removed": <failed module's serial>,
"installed": <spare's serial>, "slot": "FAN<n>"}` (serials: `dcim.assets` for the shift start, `bmc.fru` for what is
in the bay now), notes.

If the spare will not latch, pull it and insert it again at once; if the time is running out, put a module back in the
bay (even the failed one) rather than leave it empty.
