# Site services: programming reference

The site's operations services, as the shift technician's tools reach them: the NOC ticket queue, DCIM, the switched
rack PDUs, the nodes' BMCs, the fabric manager, the scheduler and the DCGM diagnostics. A program calls them through
the cell's `noc` request (see `cell/cell_api.md`): a method name `"<service>.<call>"` and a JSON object of parameters;
the reply is a JSON object.

## Conventions

* **Time.** Each call takes time on the shift clock: **0.5 s for a read, 1.0 s for a write or for starting a
  diagnostic or a soak**; the robot holds its last commands meanwhile. Fields that carry a shift-clock time in seconds
  (`time` in the SEL, `ends_at`, `elapsed_s`, `remaining_s`, the `since` parameter of `bmc.sel`) count from the shift's
  start (0 = 2026-11-27T01:30:00Z; negative = before the shift). Other times are UTC strings
  (`"2026-11-27T01:40:00Z"`).
* **Readings are live**: sensor values (temperatures, power, optical power, fan speed) carry measurement noise and
  differ slightly from one read to the next.
* **Errors.** A failed call returns `{"error": <code>, "detail": ...}` and changes nothing:

| code | when |
|---|---|
| `unknown_method` | no such method |
| `bad_params` | a parameter is missing, of the wrong type or not one of the allowed values; `detail` says which (also for an unknown ticket, run or soak id) |
| `bmc_unreachable` | the node's BMC does not answer: no working supply of the node has AC, or the rack's management switch (outlet A6) is off |
| `busy` | `bmc.power` `on` while a graceful shutdown runs |
| `device_unreachable` | `fabric.ddm` on a node whose host is not up, or on a leaf without power |
| `port_down` | `fabric.soak` on a link that is down |
| `node_off`, `node_busy` | `dcgm.run` on a node whose host is not up / that is in the job or already running a diagnostic |
| `diag_running`, `health_check_failed` | `sched.resume` refused: a diagnostic runs / the node health check failed (`detail` lists the failed checks) |

* **Names.** Hosts `gpn-a02` .. `gpn-a05`, `gpn-b02` .. `gpn-b05`; leaves `leaf-a01-1` .. `leaf-a01-4`,
  `leaf-b01-1` .. `leaf-b01-4`; racks `A01` .. `A05`, `B01` .. `B05`; fabric ports 1-8 (a node's OSFP port number, a
  leaf's down-link port number = the node number).
* **Examples** below are replies captured on a training copy of the pod: hosts, serial numbers, readings, times and
  faults differ in every pod and shift. Long lists are shortened (`"... 6 more"`).

## 1. NOC tickets (`noc.*`)

| method | parameters | returns | cost |
|---|---|---|---|
| `noc.tickets` | `all` (bool, optional: include resolved tickets) | `tickets`: [{`id`, `status`, `priority`, `opened`, `host`, `summary`}], `time` (now, UTC) | 0.5 s |
| `noc.ticket` | `id` | the ticket: the fields above plus `description` (the monitoring text, lines separated by `\n`), `fields` (structured: see below), `updates`, `escalations` | 0.5 s |
| `noc.update` | `id`, `status` (`in_progress`, `resolved`, `escalated`), `notes` (string, optional), `root_cause` (a code of site_handbook.md section 5, optional), `parts` (list of {`removed`, `installed`, `slot`}, optional) | `ok`, `id`, `status` | 1.0 s |
| `noc.escalate` | `id`, `to` (`vendor` or `cabling`), `evidence` (list of strings: diagnostic run ids, capture ids, soak ids), `notes` | `ok`, `id`, `status` (`escalated`), `to` | 1.0 s |
| `noc.create` | `summary`, `host` (a host or leaf name, optional), `component` (optional), `notes` (optional) | `ok`, `id` (a new P4 ticket) | 1.0 s |

* `noc.tickets` lists the tickets visible now: new tickets appear when monitoring opens them, during the shift too.
* Statuses: `new`, `in_progress`, `resolved`, `escalated`. Every `noc.update` is appended to `updates` (with its time);
  the ticket's status is the latest one set. The record that counts is the latest `root_cause` given and the latest
  non-empty `parts` list given (site_handbook.md sections 4-6).
* `fields` by ticket type: GPU: `host`, `location`, `component`, `gpu_index`, `pci`, `xid`, `node_state`, `sop`; link:
  `host`, `rail`, `node_port`, `leaf`, `leaf_port`, `cable`, `cable_serial`, `symbol_errors`, `sop`; power supply:
  `host`, `location`, `component` (`PSU<n>`), `psu_ok`, `sop`; fan: `host`, `location`, `component` (`FAN<n>`), `sop`
  (and `gpu_temp_c` when it reports a temperature).

The queue at the start of a shift:

```text
shift clock 0 s  ->  noc.tickets {}
```
```json
{
  "tickets": [
    {
      "id": "INC-48440",
      "status": "new",
      "priority": "P2",
      "opened": "2026-11-27T00:56:16Z",
      "host": "gpn-a03",
      "summary": "gpn-a03: GPU 1 (PCI 0000:2a:00.0) fell off the bus (Xid 79) - node DOWN"
    },
    {
      "id": "INC-48447",
      "status": "new",
      "priority": "P2",
      "opened": "2026-11-27T01:15:21Z",
      "host": "gpn-a02",
      "summary": "gpn-a02: PSU4 failure - power redundancy degraded (4 of 6 OK)"
    },
    {
      "id": "INC-48455",
      "status": "new",
      "priority": "P2",
      "opened": "2026-11-27T01:19:54Z",
      "host": "gpn-a04",
      "summary": "Link errors: gpn-a04 port 8 (rail 7) <-> leaf-b01-4 port 3: symbol BER above threshold"
    }
  ],
  "time": "2026-11-27T01:30:00Z"
}
```

A GPU ticket, a power-supply ticket and a link ticket:

```text
shift clock 1 s  ->  noc.ticket {"id": "INC-48440"}
```
```json
{
  "id": "INC-48440",
  "status": "new",
  "priority": "P2",
  "opened": "2026-11-27T00:56:16Z",
  "host": "gpn-a03",
  "summary": "gpn-a03: GPU 1 (PCI 0000:2a:00.0) fell off the bus (Xid 79) - node DOWN",
  "description": "Alert source: node health check (NHC) on gpn-a03 at 2026-11-27T00:55:46Z\ncheck_hw_gpu: expected 8 GPUs, found 7 (missing PCI 0000:2a:00.0)\nkernel: NVRM: Xid (PCI:0000:2a:00): 79, pid='<unknown>', name=<unknown>, GPU has fallen off the bus.\nScheduler: gpn-a03 set DOWN, reason \"NHC: GPU 0000:2a:00.0 missing\"\nLocation: row A, rack A03, U8-U15 (KX-8H, SN KX8-24-0121)\nComponent: GPU index 1, PCI 0000:2a:00.0\nRecommended procedure: SOP-GPU-01 (GPU fault triage)",
  "fields": {
    "host": "gpn-a03",
    "location": "row A, rack A03, U8-U15",
    "component": "GPU",
    "gpu_index": 1,
    "pci": "0000:2a:00.0",
    "xid": [79],
    "node_state": "down",
    "sop": "SOP-GPU-01"
  },
  "updates": [],
  "escalations": []
}
```

```text
shift clock 2 s  ->  noc.ticket {"id": "INC-48447"}
```
```json
{
  "id": "INC-48447",
  "status": "new",
  "priority": "P2",
  "opened": "2026-11-27T01:15:21Z",
  "host": "gpn-a02",
  "summary": "gpn-a02: PSU4 failure - power redundancy degraded (4 of 6 OK)",
  "description": "Alert source: BMC gpn-a02 at 2026-11-27T01:15:21Z\nPSU4 Status: Failure detected (feed B)\nPSU3 Status: AC input lost (since 2026-11-26T23:42:18Z)\nPower supplies OK: 4 of 6 (4+2 redundant)\nLocation: row A, rack A02, U8-U15, rear, power supply bays\nRecommended procedure: SOP-PWR-01 (power supply hot swap)",
  "fields": {
    "host": "gpn-a02",
    "location": "row A, rack A02, U8-U15",
    "component": "PSU4",
    "psu_ok": 4,
    "sop": "SOP-PWR-01"
  },
  "updates": [],
  "escalations": []
}
```

```text
shift clock 1.5 s  ->  noc.ticket {"id": "INC-48455"}
```
```json
{
  "id": "INC-48455",
  "status": "new",
  "priority": "P2",
  "opened": "2026-11-27T01:19:54Z",
  "host": "gpn-a04",
  "summary": "Link errors: gpn-a04 port 8 (rail 7) <-> leaf-b01-4 port 3: symbol BER above threshold",
  "description": "Alert source: fabric manager, event 916 \"High Symbol BER\" (WARNING) at 2026-11-27T01:19:54Z\nReporting port: gpn-a04 mlx5_7 port 1 (peer leaf-b01-4 port 3)\nSymbol errors in the last 15 min: 32; effective BER 8.9e-14 (threshold 1e-16); link down count: 0\nNode gpn-a04: ALLOC in job 48120 (kestrel-pretrain)\nCable: A04-08:8/B01-22:3 (MPO12-APC-OM4, CBL-67390, 10 m)\nNode end: row A, rack A04, U8-U15, rear, OSFP port 8 (HCA mlx5_7)\nSwitch end: row B, rack B01, U22 (leaf-b01-4), rear, port 3\nRecommended procedure: SOP-NET-01 (bad link)",
  "fields": {
    "host": "gpn-a04",
    "rail": 7,
    "node_port": 8,
    "leaf": "leaf-b01-4",
    "leaf_port": 3,
    "cable": "A04-08:8/B01-22:3",
    "cable_serial": "CBL-67390",
    "symbol_errors": 32,
    "sop": "SOP-NET-01"
  },
  "updates": [],
  "escalations": []
}
```

Updating, escalating, reporting:

```text
shift clock 1500 s  ->  noc.update {"id": "INC-48447", "status": "in_progress", "notes": "On site. PSU4 amber, PSU3 dark (AC lost); 4 of 6 OK."}
```
```json
{"ok": true, "id": "INC-48447", "status": "in_progress"}
```

```text
shift clock 1501 s  ->  noc.update {"id": "INC-48447", "status": "resolved", "root_cause": "psu_failed", "parts": [{"removed": "KX8-PSU-000000", "installed": "KX8-PSU-111111", "slot": "PSU4"}], "notes": "example only"}
```
```json
{"ok": true, "id": "INC-48447", "status": "resolved"}
```

```text
shift clock 1502 s  ->  noc.escalate {"id": "INC-48455", "to": "cabling", "evidence": ["SCOPE-0003"], "notes": "example only"}
```
```json
{"ok": true, "id": "INC-48455", "status": "escalated", "to": "cabling"}
```

```text
shift clock 1503 s  ->  noc.create {"host": "gpn-a05", "component": "FAN3", "summary": "example: FAN3 amber, one rotor slow; not serviced", "notes": "example only"}
```
```json
{"ok": true, "id": "INC-48464"}
```

```text
shift clock 1505 s  ->  noc.ticket {"id": "INC-48447"}
```
```json
{
  "id": "INC-48447",
  "status": "resolved",
  "priority": "P2",
  "opened": "2026-11-27T01:15:21Z",
  "host": "gpn-a02",
  "summary": "gpn-a02: PSU4 failure - power redundancy degraded (4 of 6 OK)",
  "description": "Alert source: BMC gpn-a02 at 2026-11-27T01:15:21Z\nPSU4 Status: Failure detected (feed B)\nPSU3 Status: AC input lost (since 2026-11-26T23:42:18Z)\nPower supplies OK: 4 of 6 (4+2 redundant)\nLocation: row A, rack A02, U8-U15, rear, power supply bays\nRecommended procedure: SOP-PWR-01 (power supply hot swap)",
  "fields": {
    "host": "gpn-a02",
    "location": "row A, rack A02, U8-U15",
    "component": "PSU4",
    "psu_ok": 4,
    "sop": "SOP-PWR-01"
  },
  "updates": [
    {
      "time": 1500.0,
      "timestamp": "2026-11-27T01:55:00Z",
      "status": "in_progress",
      "notes": "On site. PSU4 amber, PSU3 dark (AC lost); 4 of 6 OK.",
      "root_cause": null,
      "parts": []
    },
    {
      "time": 1501.0,
      "timestamp": "2026-11-27T01:55:01Z",
      "status": "resolved",
      "notes": "example only",
      "root_cause": "psu_failed",
      "parts": [
        {"removed": "KX8-PSU-000000", "installed": "KX8-PSU-111111", "slot": "PSU4"}
      ]
    }
  ],
  "escalations": []
}
```

```text
shift clock 1504 s  ->  noc.tickets {"all": true}
```
```json
{
  "tickets": [
    {
      "id": "INC-48440",
      "status": "new",
      "priority": "P2",
      "opened": "2026-11-27T00:56:16Z",
      "host": "gpn-a03",
      "summary": "gpn-a03: GPU 1 (PCI 0000:2a:00.0) fell off the bus (Xid 79) - node DOWN"
    },
    {
      "id": "INC-48447",
      "status": "resolved",
      "priority": "P2",
      "opened": "2026-11-27T01:15:21Z",
      "host": "gpn-a02",
      "summary": "gpn-a02: PSU4 failure - power redundancy degraded (4 of 6 OK)"
    },
    {
      "id": "INC-48455",
      "status": "escalated",
      "priority": "P2",
      "opened": "2026-11-27T01:19:54Z",
      "host": "gpn-a04",
      "summary": "Link errors: gpn-a04 port 8 (rail 7) <-> leaf-b01-4 port 3: symbol BER above threshold"
    },
    {
      "id": "INC-48464",
      "status": "new",
      "priority": "P4",
      "opened": "2026-11-27T01:55:03Z",
      "host": "gpn-a05",
      "summary": "example: FAN3 amber, one rotor slow; not serviced"
    }
  ],
  "time": "2026-11-27T01:55:04Z"
}
```

## 2. DCIM (`dcim.*`)

The pod's records: racks, rack elevations, the cable plant, the asset inventory and the PDU outlet maps. All reads,
0.5 s each. The static exports in `site/dcim/` hold the same racks, elevations, cables and outlet maps (without serial
numbers).

| method | parameters | returns |
|---|---|---|
| `dcim.racks` | - | `racks`: [{`rack`, `row`, `position` (1-5 from the west), `front_faces`, `size`, `role` (`gpu` / `network`), `devices`}] |
| `dcim.elevation` | `rack` | `rack`, `units` (48), `devices`: [{`u`, `device`, `model`, `serial`, `asset` (GPU nodes), `mounted`, `rail` (leaves)}], `blanks` |
| `dcim.cables` | - | `cables`: [{`label`, `serial`, `type`, `length_m`, `rail`, `a` (node end: `device`, `port`, `hca`, `location`), `b` (leaf end: `device`, `port`, `location`)}] (64 trunks), `polarity` |
| `dcim.assets` | `host` (a host or a leaf) | host: `chassis`, `psu` [{`slot`, `serial`, `model`, `feed`}], `fan` [{`slot`, `serial`, `model`}], `gpu` [{`slot` (`SXM1`..`SXM8`), `serial`, `model`}], `osfp` [{`port`, `serial`, `model`}], `power_cords` [{`psu`, `serial`}]; leaf: `osfp` [{`port`, `serial`, `model`}]; `as_of` |
| `dcim.pdu` | `rack` | `rack`, `pdus` (where PDU A and B are mounted), `outlets`: [{`outlet`, `feed`, `feeds` (the device and input it powers, or spare)}] |

* `dcim.assets` is the inventory **as installed at the start of the shift** (`as_of`): it does not follow parts moved
  during the shift. GPUs are listed by physical slot (not by GPU index). What is installed now: `bmc.fru`,
  `fabric.ddm`.

```text
shift clock 4 s  ->  dcim.racks {}
```
```json
{
  "racks": [
    {
      "rack": "A01",
      "row": "A",
      "position": 1,
      "front_faces": "south",
      "size": "48U 600x1200 mm",
      "role": "network",
      "devices": ["leaf-a01-1", "leaf-a01-2", "... 2 more"]
    },
    {
      "rack": "A02",
      "row": "A",
      "position": 2,
      "front_faces": "south",
      "size": "48U 600x1200 mm",
      "role": "gpu",
      "devices": ["gpn-a02"]
    },
    "... 8 more"
  ]
}
```

```text
shift clock 4.5 s  ->  dcim.elevation {"rack": "A03"}
```
```json
{
  "rack": "A03",
  "units": 48,
  "devices": [
    {
      "u": "U8-U15",
      "device": "gpn-a03",
      "model": "KX-8H (8x SXM-H8-80G)",
      "serial": "KX8-24-0121",
      "asset": "DC1-A03-4403",
      "mounted": "front"
    },
    {"u": "U28-U29", "device": "stor-a03-1", "model": "KX-2S storage node", "mounted": "front"},
    {"u": "U32-U33", "device": "stor-a03-2", "model": "KX-2S storage node", "mounted": "front"},
    {"u": "U40", "device": "mgmt-a03", "model": "KX-M48 1G management switch", "mounted": "front"}
  ],
  "blanks": "all other units"
}
```

```text
shift clock 5 s  ->  dcim.elevation {"rack": "A01"}
```
```json
{
  "rack": "A01",
  "units": 48,
  "devices": [
    {
      "u": "U16",
      "device": "leaf-a01-1",
      "model": "KX-L32R (32-port 400G leaf)",
      "serial": "KXL-A01-3100",
      "mounted": "rear (ports to hot aisle)",
      "rail": 0
    },
    {
      "u": "U18",
      "device": "leaf-a01-2",
      "model": "KX-L32R (32-port 400G leaf)",
      "serial": "KXL-A01-3117",
      "mounted": "rear (ports to hot aisle)",
      "rail": 1
    },
    {
      "u": "U20",
      "device": "leaf-a01-3",
      "model": "KX-L32R (32-port 400G leaf)",
      "serial": "KXL-A01-3134",
      "mounted": "rear (ports to hot aisle)",
      "rail": 2
    },
    {
      "u": "U22",
      "device": "leaf-a01-4",
      "model": "KX-L32R (32-port 400G leaf)",
      "serial": "KXL-A01-3151",
      "mounted": "rear (ports to hot aisle)",
      "rail": 3
    },
    {"u": "U40", "device": "patch-a01-40", "model": "1U MPO patch panel", "mounted": "front"},
    {"u": "U42", "device": "patch-a01-42", "model": "1U MPO patch panel", "mounted": "front"}
  ],
  "blanks": "all other units"
}
```

```text
shift clock 5.5 s  ->  dcim.cables {}
```
```json
{
  "cables": [
    {
      "label": "A02-08:1/A01-16:1",
      "serial": "CBL-58516",
      "type": "MPO12-APC-OM4",
      "length_m": 7,
      "rail": 0,
      "a": {"device": "gpn-a02", "port": 1, "hca": "mlx5_0", "location": "A02 rear, OSFP port 1"},
      "b": {"device": "leaf-a01-1", "port": 1, "location": "A01 U16 rear, port 1"}
    },
    {
      "label": "A02-08:2/A01-18:1",
      "serial": "CBL-04177",
      "type": "MPO12-APC-OM4",
      "length_m": 7,
      "rail": 1,
      "a": {"device": "gpn-a02", "port": 2, "hca": "mlx5_1", "location": "A02 rear, OSFP port 2"},
      "b": {"device": "leaf-a01-2", "port": 1, "location": "A01 U18 rear, port 1"}
    },
    "... 62 more"
  ],
  "polarity": "Type-B (key-up to key-up): fibre f at one end is fibre 13-f at the other"
}
```

```text
shift clock 6 s  ->  dcim.assets {"host": "gpn-a03"}
```
```json
{
  "host": "gpn-a03",
  "chassis": {"model": "KX-8H", "serial": "KX8-24-0121"},
  "as_of": "shift start",
  "psu": [
    {"slot": "PSU1", "serial": "KX8-PSU-688981", "model": "KX-PS3300", "feed": "A"},
    {"slot": "PSU2", "serial": "KX8-PSU-698022", "model": "KX-PS3300", "feed": "B"},
    {"slot": "PSU3", "serial": "KX8-PSU-922336", "model": "KX-PS3300", "feed": "A"},
    "... 3 more"
  ],
  "fan": [
    {"slot": "FAN1", "serial": "FAN-076316", "model": "KX-FAN80D"},
    {"slot": "FAN2", "serial": "FAN-610804", "model": "KX-FAN80D"},
    {"slot": "FAN3", "serial": "FAN-168960", "model": "KX-FAN80D"},
    "... 2 more"
  ],
  "gpu": [
    {"slot": "SXM1", "serial": "SXM-69280095", "model": "SXM-H8-80G"},
    {"slot": "SXM2", "serial": "SXM-65874031", "model": "SXM-H8-80G"},
    {"slot": "SXM3", "serial": "SXM-12565870", "model": "SXM-H8-80G"},
    "... 5 more"
  ],
  "osfp": [
    {"port": 1, "serial": "OSFP-PWL0W6K7", "model": "OSFP-400G-SR4"},
    {"port": 2, "serial": "OSFP-VTW4BJKS", "model": "OSFP-400G-SR4"},
    {"port": 3, "serial": "OSFP-ZZ2V6JE6", "model": "OSFP-400G-SR4"},
    "... 5 more"
  ],
  "power_cords": [
    {"psu": "PSU1", "serial": "CORD-79116"},
    {"psu": "PSU2", "serial": "CORD-90583"},
    {"psu": "PSU3", "serial": "CORD-62274"},
    "... 3 more"
  ]
}
```

```text
shift clock 6.5 s  ->  dcim.assets {"host": "leaf-a01-2"}
```
```json
{
  "device": "leaf-a01-2",
  "osfp": [
    {"port": 1, "serial": "OSFP-14YRH3JY", "model": "OSFP-400G-SR4-F"},
    {"port": 2, "serial": "OSFP-DN7XF0W9", "model": "OSFP-400G-SR4-F"},
    "... 6 more"
  ],
  "as_of": "shift start"
}
```

```text
shift clock 7 s  ->  dcim.pdu {"rack": "A03"}
```
```json
{
  "rack": "A03",
  "pdus": {"A": "rear, left side seen from the hot aisle", "B": "rear, right side"},
  "outlets": [
    {"outlet": "A1", "feed": "A", "feeds": "gpn-a03 PSU1"},
    {"outlet": "A2", "feed": "A", "feeds": "gpn-a03 PSU3"},
    {"outlet": "A3", "feed": "A", "feeds": "gpn-a03 PSU5"},
    {"outlet": "A4", "feed": "A", "feeds": "stor-a03-1 PSU1"},
    {"outlet": "A5", "feed": "A", "feeds": "stor-a03-2 PSU1"},
    {"outlet": "A6", "feed": "A", "feeds": "mgmt-a03 PSU"},
    {"outlet": "B1", "feed": "B", "feeds": "gpn-a03 PSU2"},
    {"outlet": "B2", "feed": "B", "feeds": "gpn-a03 PSU4"},
    {"outlet": "B3", "feed": "B", "feeds": "gpn-a03 PSU6"},
    {"outlet": "B4", "feed": "B", "feeds": "stor-a03-1 PSU2"},
    {"outlet": "B5", "feed": "B", "feeds": "stor-a03-2 PSU2"},
    {"outlet": "B6", "feed": "B", "feeds": "spare (nothing plugged)"}
  ]
}
```

## 3. Rack PDUs (`pdu.*`)

| method | parameters | returns | cost |
|---|---|---|---|
| `pdu.outlets` | `rack` | `rack`, `feeds`: {`A`: [{`outlet`, `state` (`on` / `off`), `power_w`, `current_a`}], `B`: [...]} | 0.5 s |
| `pdu.set` | `rack`, `outlet` (`A1`..`A6`, `B1`..`B6`), `state` (`on` / `off`) | `ok`, `rack`, `outlet`, `state` | 1.0 s |

* Switching takes effect at once. An outlet's power is what its load draws now: a node's good supplies share the
  node's load; a failed supply with AC draws a few watts; a supply without contact (or an empty bay) 0 W.
* Check the outlet map before switching: switching off an outlet that feeds anything but the node you work on is an
  incident (site_handbook.md sections 1.5, 9).

```text
shift clock 7.5 s  ->  pdu.outlets {"rack": "A05"}
```
```json
{
  "rack": "A05",
  "feeds": {
    "A": [
      {"outlet": "A1", "state": "on", "power_w": 1511, "current_a": 6.71},
      {"outlet": "A2", "state": "on", "power_w": 1511, "current_a": 6.7},
      {"outlet": "A3", "state": "on", "power_w": 1540, "current_a": 6.83},
      {"outlet": "A4", "state": "on", "power_w": 412, "current_a": 1.83},
      {"outlet": "A5", "state": "on", "power_w": 411, "current_a": 1.82},
      {"outlet": "A6", "state": "on", "power_w": 153, "current_a": 0.68}
    ],
    "B": [
      {"outlet": "B1", "state": "on", "power_w": 1522, "current_a": 6.75},
      {"outlet": "B2", "state": "on", "power_w": 1548, "current_a": 6.87},
      {"outlet": "B3", "state": "on", "power_w": 1536, "current_a": 6.81},
      {"outlet": "B4", "state": "on", "power_w": 414, "current_a": 1.84},
      {"outlet": "B5", "state": "on", "power_w": 415, "current_a": 1.84},
      {"outlet": "B6", "state": "on", "power_w": 0, "current_a": 0.0}
    ]
  }
}
```

```text
shift clock 1401 s  ->  pdu.set {"rack": "A03", "outlet": "A1", "state": "off"}
```
```json
{"ok": true, "rack": "A03", "outlet": "A1", "state": "off"}
```

The six outlets of a node switched off:

```text
shift clock 1408 s  ->  pdu.outlets {"rack": "A03"}
```
```json
{
  "rack": "A03",
  "feeds": {
    "A": [
      {"outlet": "A1", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "A2", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "A3", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "A4", "state": "on", "power_w": 407, "current_a": 1.81},
      {"outlet": "A5", "state": "on", "power_w": 399, "current_a": 1.77},
      {"outlet": "A6", "state": "on", "power_w": 152, "current_a": 0.67}
    ],
    "B": [
      {"outlet": "B1", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "B2", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "B3", "state": "off", "power_w": 0, "current_a": 0.0},
      {"outlet": "B4", "state": "on", "power_w": 405, "current_a": 1.8},
      {"outlet": "B5", "state": "on", "power_w": 413, "current_a": 1.83},
      {"outlet": "B6", "state": "on", "power_w": 0, "current_a": 0.0}
    ]
  }
}
```

## 4. BMC (`bmc.*`)

| method | parameters | returns | cost |
|---|---|---|---|
| `bmc.power_state` | `host` | `host`, `state` (`standby`, `booting`, `on`, `shutting_down`) | 0.5 s |
| `bmc.power` | `host`, `action` (`on`, `off`, `cycle`, `forceoff`) | `ok`, `host`, `state` (right after the request) | 1.0 s |
| `bmc.sel` | `host`, `since` (shift-clock seconds, optional) | `host`, `entries`: [{`id`, `time`, `timestamp`, `sensor`, `event`, `severity`}] (entries at or after `since`) | 0.5 s |
| `bmc.sensors` | `host` | `power_state`, `inlet_temp_c`, `power_w`, `gpu_temp_c` {`GPU0`..`GPU7`: C or `na`}, `fan_rpm` {`FAN<n>_F`, `FAN<n>_R`: RPM or `absent`}, `fan_status` {`FAN<n>`: `ok` / `degraded` / `failed` / `absent` / `standby`}, `psu` {`PSU<n>`: `ok` / `failure` / `ac_lost` / `absent`}, `psu_redundancy` (`full` / `degraded` / `lost`), `power_brake`, `gpu_thermal_slowdown`, `gpu_tray` (`seated` / `not seated`) | 0.5 s |
| `bmc.fru` | `host` | `chassis`, `psu` {`PSU<n>`: serial / `absent`}, `fan` {`FAN<n>`: serial / `absent`}, `gpu` {`GPU<i>`: serial / `not present` / `unavailable (host off)`}, `osfp` {`OSFP<n>`: serial / `absent` / `unavailable (host off)`} | 0.5 s |
| `bmc.identify` | `host`, `state` (`on`, `off`, `blink`) | `ok`, `host`, `identify` | 1.0 s |

* Every BMC call fails with `bmc_unreachable` while the BMC has no power (no working supply with AC) or the rack's
  management switch is off.
* `bmc.power`: `on` from `standby` starts the boot (`booting`, about 5 min, then `on`); it is refused (`busy`) during a
  graceful shutdown and does nothing in other states. `off` starts a graceful shutdown (`shutting_down`, then
  `standby` after up to 90 s). `cycle` = graceful off, 10 s, on (AC stays on). `forceoff` stops the host at once.
  Powering on fails (the host stays in `standby`, SEL "POST error: GPU tray not seated") if the GPU tray is not
  latched home, and is denied with fewer than three good supplies.
* `bmc.fru` lists GPUs by GPU index and only those on the bus with the host up; map an index to its slot with the
  vendor's table.

```text
shift clock 10 s  ->  bmc.power_state {"host": "gpn-a03"}
```
```json
{"host": "gpn-a03", "state": "on"}
```

```text
shift clock 8 s  ->  bmc.sel {"host": "gpn-a03"}
```
```json
{
  "host": "gpn-a03",
  "entries": [
    {
      "id": 1,
      "time": -2161030.33,
      "timestamp": "2026-11-02T01:12:49Z",
      "sensor": "System ACPI Power State",
      "event": "S0/G0: working (host power on)",
      "severity": "info"
    },
    {
      "id": 2,
      "time": -2160734.33,
      "timestamp": "2026-11-02T01:17:45Z",
      "sensor": "System Event",
      "event": "OS boot completed",
      "severity": "info"
    },
    {
      "id": 3,
      "time": -2058.83,
      "timestamp": "2026-11-27T00:55:41Z",
      "sensor": "GPU1 (0000:2a:00.0)",
      "event": "Xid 79: GPU has fallen off the bus",
      "severity": "critical"
    }
  ]
}
```

```text
shift clock 8.5 s  ->  bmc.sel {"host": "gpn-a03", "since": 0}
```
```json
{"host": "gpn-a03", "entries": []}
```

```text
shift clock 9 s  ->  bmc.sensors {"host": "gpn-a05"}
```
```json
{
  "host": "gpn-a05",
  "power_state": "on",
  "inlet_temp_c": 23.1,
  "power_w": 9266,
  "gpu_temp_c": {
    "GPU0": 75.1,
    "GPU1": 75.0,
    "GPU2": 75.8,
    "GPU3": 76.8,
    "GPU4": 80.1,
    "GPU5": 79.2,
    "GPU6": 79.7,
    "GPU7": 79.3
  },
  "fan_rpm": {
    "FAN1_F": 12109,
    "FAN1_R": 11872,
    "FAN2_F": 12108,
    "FAN2_R": 11823,
    "FAN3_F": 11958,
    "FAN3_R": 11994,
    "FAN4_F": 11801,
    "FAN4_R": 12286,
    "FAN5_F": 11980,
    "FAN5_R": 11867
  },
  "fan_status": {"FAN1": "ok", "FAN2": "ok", "FAN3": "ok", "FAN4": "ok", "FAN5": "ok"},
  "psu": {"PSU1": "ok", "PSU2": "ok", "PSU3": "ok", "PSU4": "ok", "PSU5": "ok", "PSU6": "ok"},
  "psu_redundancy": "full",
  "power_brake": false,
  "gpu_thermal_slowdown": false,
  "gpu_tray": "seated"
}
```

```text
shift clock 9.5 s  ->  bmc.fru {"host": "gpn-a03"}
```
```json
{
  "host": "gpn-a03",
  "chassis": "KX8-24-0121",
  "psu": {
    "PSU1": "KX8-PSU-688981",
    "PSU2": "KX8-PSU-698022",
    "PSU3": "KX8-PSU-922336",
    "PSU4": "KX8-PSU-367288",
    "PSU5": "KX8-PSU-026722",
    "PSU6": "KX8-PSU-301537"
  },
  "fan": {
    "FAN1": "FAN-076316",
    "FAN2": "FAN-610804",
    "FAN3": "FAN-168960",
    "FAN4": "FAN-802740",
    "FAN5": "FAN-305870"
  },
  "gpu": {
    "GPU0": "SXM-42764947",
    "GPU1": "not present",
    "GPU2": "SXM-27742521",
    "GPU3": "SXM-63504046",
    "GPU4": "SXM-69280095",
    "GPU5": "SXM-12565870",
    "GPU6": "SXM-88470396",
    "GPU7": "SXM-65874031"
  },
  "osfp": {
    "OSFP1": "OSFP-PWL0W6K7",
    "OSFP2": "OSFP-VTW4BJKS",
    "OSFP3": "OSFP-ZZ2V6JE6",
    "OSFP4": "OSFP-NDY24P82",
    "OSFP5": "OSFP-S3E5YA3W",
    "OSFP6": "OSFP-C4TMR8H8",
    "OSFP7": "OSFP-6R6YTUP6",
    "OSFP8": "OSFP-S98FRHDT"
  }
}
```

```text
shift clock 15 s  ->  bmc.identify {"host": "gpn-a05", "state": "blink"}
```
```json
{"ok": true, "host": "gpn-a05", "identify": "blink"}
```

```text
shift clock 16 s  ->  bmc.identify {"host": "gpn-a05", "state": "off"}
```
```json
{"ok": true, "host": "gpn-a05", "identify": "off"}
```

A cold power cycle, step by step (graceful off, the node's six outlets off, BMC gone, outlets on, power on):

```text
shift clock 1300 s  ->  bmc.power {"host": "gpn-a03", "action": "off"}
```
```json
{"ok": true, "host": "gpn-a03", "state": "shutting_down"}
```

```text
shift clock 1310 s  ->  bmc.power {"host": "gpn-a03", "action": "on"}
```
```json
{"error": "busy", "detail": "a graceful shutdown is in progress"}
```

```text
shift clock 1400 s  ->  bmc.power_state {"host": "gpn-a03"}
```
```json
{"host": "gpn-a03", "state": "standby"}
```

```text
shift clock 1409 s  ->  bmc.power_state {"host": "gpn-a03"}
```
```json
{"error": "bmc_unreachable", "detail": "no response from gpn-a03-bmc"}
```

```text
shift clock 1460 s  ->  bmc.power_state {"host": "gpn-a03"}
```
```json
{"host": "gpn-a03", "state": "standby"}
```

```text
shift clock 1461 s  ->  bmc.power {"host": "gpn-a03", "action": "on"}
```
```json
{"ok": true, "host": "gpn-a03", "state": "booting"}
```

```text
shift clock 1470 s  ->  bmc.sel {"host": "gpn-a03", "since": 1290}
```
```json
{
  "host": "gpn-a03",
  "entries": [
    {
      "id": 4,
      "time": 1300.0,
      "timestamp": "2026-11-27T01:51:40Z",
      "sensor": "System Event",
      "event": "Graceful shutdown requested (BMC)",
      "severity": "info"
    },
    {
      "id": 5,
      "time": 1390.0,
      "timestamp": "2026-11-27T01:53:10Z",
      "sensor": "System ACPI Power State",
      "event": "S5/G2: host soft-off",
      "severity": "info"
    },
    {
      "id": 6,
      "time": 1401.0,
      "timestamp": "2026-11-27T01:53:21Z",
      "sensor": "PSU1 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 7,
      "time": 1401.0,
      "timestamp": "2026-11-27T01:53:21Z",
      "sensor": "Power Unit Redundancy",
      "event": "Redundancy degraded (5 of 6 PSUs OK)",
      "severity": "warning"
    },
    {
      "id": 8,
      "time": 1402.0,
      "timestamp": "2026-11-27T01:53:22Z",
      "sensor": "PSU3 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 9,
      "time": 1403.0,
      "timestamp": "2026-11-27T01:53:23Z",
      "sensor": "PSU5 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 10,
      "time": 1403.0,
      "timestamp": "2026-11-27T01:53:23Z",
      "sensor": "Power Unit Redundancy",
      "event": "Redundancy lost (3 of 6 PSUs OK)",
      "severity": "warning"
    },
    {
      "id": 11,
      "time": 1404.0,
      "timestamp": "2026-11-27T01:53:24Z",
      "sensor": "PSU2 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 12,
      "time": 1405.0,
      "timestamp": "2026-11-27T01:53:25Z",
      "sensor": "PSU4 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 13,
      "time": 1450.0,
      "timestamp": "2026-11-27T01:54:10Z",
      "sensor": "System Event",
      "event": "AC power restored: BMC started",
      "severity": "info"
    },
    {
      "id": 14,
      "time": 1450.0,
      "timestamp": "2026-11-27T01:54:10Z",
      "sensor": "PSU1 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 15,
      "time": 1450.0,
      "timestamp": "2026-11-27T01:54:10Z",
      "sensor": "PSU6 Status",
      "event": "AC input lost",
      "severity": "warning"
    },
    {
      "id": 16,
      "time": 1451.0,
      "timestamp": "2026-11-27T01:54:11Z",
      "sensor": "PSU3 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 17,
      "time": 1452.0,
      "timestamp": "2026-11-27T01:54:12Z",
      "sensor": "PSU5 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 18,
      "time": 1453.0,
      "timestamp": "2026-11-27T01:54:13Z",
      "sensor": "PSU2 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 19,
      "time": 1453.0,
      "timestamp": "2026-11-27T01:54:13Z",
      "sensor": "Power Unit Redundancy",
      "event": "Redundancy degraded (4 of 6 PSUs OK)",
      "severity": "warning"
    },
    {
      "id": 20,
      "time": 1454.0,
      "timestamp": "2026-11-27T01:54:14Z",
      "sensor": "PSU4 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 21,
      "time": 1455.0,
      "timestamp": "2026-11-27T01:54:15Z",
      "sensor": "PSU6 Status",
      "event": "AC input restored",
      "severity": "info"
    },
    {
      "id": 22,
      "time": 1455.0,
      "timestamp": "2026-11-27T01:54:15Z",
      "sensor": "Power Unit Redundancy",
      "event": "Fully redundant (6 of 6 PSUs OK)",
      "severity": "info"
    },
    {
      "id": 23,
      "time": 1461.0,
      "timestamp": "2026-11-27T01:54:21Z",
      "sensor": "System Event",
      "event": "Power on requested (BMC)",
      "severity": "info"
    },
    {
      "id": 24,
      "time": 1461.0,
      "timestamp": "2026-11-27T01:54:21Z",
      "sensor": "System ACPI Power State",
      "event": "Host power on: POST started",
      "severity": "info"
    }
  ]
}
```

## 5. Fabric manager (`fabric.*`)

`device` is a host or a leaf; `port` is 1-8 (a node's OSFP port; a leaf's down-link port = the node number).

| method | parameters | returns | cost |
|---|---|---|---|
| `fabric.port` | `device`, `port` | `status` (`up`, `flapping`, `down`), `speed`, `width`, `peer` {`device`, `port`} or null, `counters` {`symbol_errors`, `link_down_count`, `effective_ber`, `raw_ber` (while the link is up)}, `locate`, `hca` (nodes) | 0.5 s |
| `fabric.ddm` | `device`, `port` | `module` {`present`, `model`, `serial`, `temp_c`, `vcc_v`}, `lanes` [{`lane`, `tx_dbm`, `rx_dbm`, `bias_ma`}] (4), `alarms` [{`lane`, `flag`, `level`}], `thresholds` {`rx_power_low_dbm`, `tx_bias_high_ma`} | 0.5 s |
| `fabric.clear` | `device` and `port`, or nothing (= every port) | `ok` | 1.0 s |
| `fabric.locate` | `device`, `port`, `state` (`on` / `off`) | `ok`, `device`, `port`, `locate` | 1.0 s |
| `fabric.soak` | `device`, `port`, `seconds` (10-7200) | `ok`, `soak_id`, `link` {`host`, `port`, `leaf`, `leaf_port`}, `ends_at` (shift clock, s) | 1.0 s |
| `fabric.soak_result` | `id` | running: `status`, `elapsed_s`, `remaining_s`; done: `status`, `passed`, `seconds`, `symbol_errors`, `link_down_count`, `link_up_at_end`, `criteria` | 0.5 s |

* `fabric.ddm` reads the module in that cage; with no module: `{"present": false}`. A node's modules are read through
  its host (`device_unreachable` while the host is not up). `rx_dbm` shows no light (-40 dBm, alarm `rx_los`) when the
  far end is not transmitting. Alarm flags: `tx_bias_high` (warning), `rx_power_low` (warning), `rx_los` (alarm).
* A node's port counters restart from zero when its host boots; `fabric.clear` zeroes the counters shown. A soak
  counts on its own (clearing during a soak does not change its result); it covers both ends of the link and passes
  only with no symbol error, at most one link-down and the link up at the end. A soak can only start on a link that is
  up.
* The locate LED of a port blinks blue until it is switched off.

```text
shift clock 10.5 s  ->  fabric.port {"device": "gpn-a05", "port": 1}
```
```json
{
  "device": "gpn-a05",
  "port": 1,
  "status": "up",
  "speed": "400G",
  "width": "4x",
  "peer": {"device": "leaf-a01-1", "port": 4},
  "counters": {"symbol_errors": 0, "link_down_count": 0, "effective_ber": 0.0, "raw_ber": 1e-15},
  "locate": false,
  "hca": "mlx5_0"
}
```

```text
shift clock 11 s  ->  fabric.port {"device": "leaf-a01-1", "port": 4}
```
```json
{
  "device": "leaf-a01-1",
  "port": 4,
  "status": "up",
  "speed": "400G",
  "width": "4x",
  "peer": {"device": "gpn-a05", "port": 1},
  "counters": {"symbol_errors": 0, "link_down_count": 0, "effective_ber": 0.0, "raw_ber": 1e-15},
  "locate": false
}
```

```text
shift clock 11.5 s  ->  fabric.ddm {"device": "gpn-a05", "port": 1}
```
```json
{
  "device": "gpn-a05",
  "port": 1,
  "module": {
    "present": true,
    "model": "OSFP-400G-SR4",
    "serial": "OSFP-7CLWEBL3",
    "temp_c": 52.3,
    "vcc_v": 3.314
  },
  "lanes": [
    {"lane": 0, "tx_dbm": -0.06, "rx_dbm": -1.78, "bias_ma": 7.01},
    {"lane": 1, "tx_dbm": -0.0, "rx_dbm": -2.02, "bias_ma": 7.24},
    {"lane": 2, "tx_dbm": 0.13, "rx_dbm": -1.74, "bias_ma": 7.28},
    {"lane": 3, "tx_dbm": -0.01, "rx_dbm": -2.14, "bias_ma": 7.22}
  ],
  "alarms": [],
  "thresholds": {"rx_power_low_dbm": -6.0, "tx_bias_high_ma": 10.0}
}
```

```text
shift clock 14 s  ->  fabric.clear {"device": "gpn-a05", "port": 1}
```
```json
{"ok": true, "device": "gpn-a05", "port": 1}
```

```text
shift clock 12 s  ->  fabric.locate {"device": "leaf-a01-1", "port": 4, "state": "on"}
```
```json
{"ok": true, "device": "leaf-a01-1", "port": 4, "locate": "on"}
```

```text
shift clock 13 s  ->  fabric.locate {"device": "leaf-a01-1", "port": 4, "state": "off"}
```
```json
{"ok": true, "device": "leaf-a01-1", "port": 4, "locate": "off"}
```

```text
shift clock 650 s  ->  fabric.soak {"device": "gpn-a05", "port": 1, "seconds": 600}
```
```json
{
  "ok": true,
  "soak_id": "SOAK-0001",
  "link": {"host": "gpn-a05", "port": 1, "leaf": "leaf-a01-1", "leaf_port": 4},
  "ends_at": 1250.0
}
```

```text
shift clock 700 s  ->  fabric.soak_result {"id": "SOAK-0001"}
```
```json
{"id": "SOAK-0001", "status": "running", "elapsed_s": 50.0, "remaining_s": 550.0}
```

```text
shift clock 1260 s  ->  fabric.soak_result {"id": "SOAK-0001"}
```
```json
{
  "id": "SOAK-0001",
  "status": "done",
  "passed": true,
  "seconds": 600,
  "symbol_errors": 0,
  "link_down_count": 0,
  "link_up_at_end": true,
  "criteria": "fail if any symbol error, more than 1 link down, or the link not up at the end"
}
```

With the node's host off (the leaf end sees no light; the node end cannot be read):

```text
shift clock 1410 s  ->  fabric.ddm {"device": "gpn-a03", "port": 1}
```
```json
{"error": "device_unreachable", "detail": "gpn-a03 is not running (the HCA is read through the host)"}
```

```text
shift clock 1411 s  ->  fabric.ddm {"device": "leaf-a01-1", "port": 2}
```
```json
{
  "device": "leaf-a01-1",
  "port": 2,
  "module": {
    "present": true,
    "model": "OSFP-400G-SR4-F",
    "serial": "OSFP-6LGDPUM9",
    "temp_c": 40.8,
    "vcc_v": 3.303
  },
  "lanes": [
    {"lane": 0, "tx_dbm": 0.09, "rx_dbm": -40.0, "bias_ma": 7.15},
    {"lane": 1, "tx_dbm": -0.16, "rx_dbm": -40.0, "bias_ma": 7.18},
    {"lane": 2, "tx_dbm": 0.04, "rx_dbm": -40.0, "bias_ma": 7.27},
    {"lane": 3, "tx_dbm": -0.02, "rx_dbm": -40.0, "bias_ma": 6.71}
  ],
  "alarms": [
    {"lane": 0, "flag": "rx_los", "level": "alarm"},
    {"lane": 1, "flag": "rx_los", "level": "alarm"},
    {"lane": 2, "flag": "rx_los", "level": "alarm"},
    {"lane": 3, "flag": "rx_los", "level": "alarm"}
  ],
  "thresholds": {"rx_power_low_dbm": -6.0, "tx_bias_high_ma": 10.0}
}
```

```text
shift clock 1412 s  ->  fabric.port {"device": "leaf-a01-1", "port": 2}
```
```json
{
  "device": "leaf-a01-1",
  "port": 2,
  "status": "down",
  "speed": null,
  "width": "4x",
  "peer": null,
  "counters": {"symbol_errors": 0, "link_down_count": 1, "effective_ber": 0.0},
  "locate": false
}
```

## 6. Scheduler (`sched.*`)

| method | parameters | returns | cost |
|---|---|---|---|
| `sched.nodes` | - | `nodes`: [{`node`, `state` (`alloc`, `drain`, `drained`, `down`, `idle`), `reason`, `since`}] | 0.5 s |
| `sched.jobs` | - | `jobs`: [{`id`, `name`, `state` (`running`, `restarting`, `stalled`), `nodes`, `gpus`, `last_checkpoint`, `next_checkpoint`, `checkpoint_interval_s`, `restarts`, `relative_speed` (while running), `restart_at` (while restarting)}] | 0.5 s |
| `sched.drain` | `node`, `reason` (optional) | `ok`, `node`, `state` | 1.0 s |
| `sched.resume` | `node` | `ok`, `node`, `state` (`idle`), `joins_at`; or an error | 1.0 s |

* `sched.drain`: `alloc` becomes `drain` and then `drained` at the next checkpoint; an `idle` node becomes `drained` at
  once; for a node already out of the job it only records the reason.
* `sched.resume`: `drain` goes back to `alloc` (the drain is cancelled); `drained` / `down` becomes `idle` if the node
  health check passes (host up, 8 GPUs present and healthy, 8 links up to their planned leaf ports, no power brake, no
  thermal slowdown) and no diagnostic runs; it joins the job at the next checkpoint (`joins_at`). Otherwise an error
  with the reason.

```text
shift clock 3 s  ->  sched.nodes {}
```
```json
{
  "nodes": [
    {"node": "gpn-a02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {
      "node": "gpn-a03",
      "state": "down",
      "reason": "NHC: GPU 0000:2a:00.0 missing",
      "since": "2026-11-27T00:56:16Z"
    },
    {"node": "gpn-a04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-a05", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b03", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b05", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"}
  ]
}
```

```text
shift clock 3.5 s  ->  sched.jobs {}
```
```json
{
  "jobs": [
    {
      "id": 48120,
      "name": "kestrel-pretrain",
      "state": "running",
      "nodes": ["gpn-a02", "gpn-a04", "gpn-a05", "gpn-b02", "gpn-b03", "gpn-b04", "gpn-b05"],
      "gpus": 56,
      "last_checkpoint": "2026-11-27T01:30:00Z",
      "next_checkpoint": "2026-11-27T01:40:00Z",
      "checkpoint_interval_s": 600.0,
      "restarts": 0,
      "relative_speed": 0.9
    }
  ]
}
```

```text
shift clock 120 s  ->  sched.drain {"node": "gpn-a05", "reason": "INC-00000 example: link work"}
```
```json
{"ok": true, "node": "gpn-a05", "state": "drain"}
```

```text
shift clock 121 s  ->  sched.nodes {}
```
```json
{
  "nodes": [
    {"node": "gpn-a02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {
      "node": "gpn-a03",
      "state": "down",
      "reason": "NHC: GPU 0000:2a:00.0 missing",
      "since": "2026-11-27T00:56:16Z"
    },
    {"node": "gpn-a04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {
      "node": "gpn-a05",
      "state": "drain",
      "reason": "INC-00000 example: link work",
      "since": "2026-11-27T01:32:00Z"
    },
    {"node": "gpn-b02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b03", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b05", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"}
  ]
}
```

At the next checkpoint:

```text
shift clock 602 s  ->  sched.nodes {}
```
```json
{
  "nodes": [
    {"node": "gpn-a02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {
      "node": "gpn-a03",
      "state": "down",
      "reason": "NHC: GPU 0000:2a:00.0 missing",
      "since": "2026-11-27T00:56:16Z"
    },
    {"node": "gpn-a04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {
      "node": "gpn-a05",
      "state": "drained",
      "reason": "INC-00000 example: link work",
      "since": "2026-11-27T01:40:00Z"
    },
    {"node": "gpn-b02", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b03", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b04", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"},
    {"node": "gpn-b05", "state": "alloc", "reason": "", "since": "2026-11-27T01:30:00Z"}
  ]
}
```

```text
shift clock 100 s  ->  sched.resume {"node": "gpn-a03"}
```
```json
{"error": "diag_running", "detail": "DCGM run DIAG-gpn-a03-002 is in progress"}
```

```text
shift clock 1265 s  ->  sched.resume {"node": "gpn-a05"}
```
```json
{"ok": true, "node": "gpn-a05", "state": "idle", "joins_at": "2026-11-27T02:00:00Z"}
```

## 7. DCGM diagnostics (`dcgm.*`)

| method | parameters | returns | cost |
|---|---|---|---|
| `dcgm.run` | `host`, `level` (1, 2 or 3) | `ok`, `run_id`, `level`, `expected_s` | 1.0 s |
| `dcgm.result` | `run_id` | `run_id`, `host`, `level`, `status` (`running`, `done`, `aborted`), `started`; running: `elapsed_s`, `remaining_s`; aborted: `reason`, `finished`; done: `result` (`pass` / `fail`), `gpus` [{`index`, `pci`, `result`, `message`}], `finished` | 0.5 s |

* A run needs the host up (`node_off` otherwise) and the node `drained` or `down` with no other run in progress
  (`node_busy` otherwise). It takes about 3 s (level 1), 10 min (level 2) or 30 min (level 3) and aborts if the host
  shuts down, power-cycles or loses power. What each level tests: vendor/kx8h_service_guide.md section 7.3.
* GPUs are reported by GPU index.

```text
shift clock 20 s  ->  dcgm.run {"host": "gpn-a03", "level": 1}
```
```json
{"ok": true, "run_id": "DIAG-gpn-a03-001", "level": 1, "expected_s": 3.0}
```

```text
shift clock 24 s  ->  dcgm.result {"run_id": "DIAG-gpn-a03-001"}
```
```json
{
  "run_id": "DIAG-gpn-a03-001",
  "host": "gpn-a03",
  "level": 1,
  "status": "done",
  "started": "2026-11-27T01:30:20Z",
  "result": "fail",
  "gpus": [
    {"index": 0, "pci": "0000:18:00.0", "result": "pass", "message": ""},
    {"index": 1, "pci": "0000:2a:00.0", "result": "fail", "message": "GPU not found on PCI bus 0000:2a:00.0"},
    {"index": 2, "pci": "0000:3a:00.0", "result": "pass", "message": ""},
    {"index": 3, "pci": "0000:5d:00.0", "result": "pass", "message": ""},
    {"index": 4, "pci": "0000:9a:00.0", "result": "pass", "message": ""},
    {"index": 5, "pci": "0000:ab:00.0", "result": "pass", "message": ""},
    {"index": 6, "pci": "0000:ba:00.0", "result": "pass", "message": ""},
    {"index": 7, "pci": "0000:db:00.0", "result": "pass", "message": ""}
  ],
  "finished": "2026-11-27T01:30:23Z"
}
```

```text
shift clock 30 s  ->  dcgm.run {"host": "gpn-a03", "level": 2}
```
```json
{"ok": true, "run_id": "DIAG-gpn-a03-002", "level": 2, "expected_s": 600.0}
```

```text
shift clock 90 s  ->  dcgm.result {"run_id": "DIAG-gpn-a03-002"}
```
```json
{
  "run_id": "DIAG-gpn-a03-002",
  "host": "gpn-a03",
  "level": 2,
  "status": "running",
  "started": "2026-11-27T01:30:30Z",
  "elapsed_s": 60.0,
  "remaining_s": 540.0
}
```

```text
shift clock 640 s  ->  dcgm.result {"run_id": "DIAG-gpn-a03-002"}
```
```json
{
  "run_id": "DIAG-gpn-a03-002",
  "host": "gpn-a03",
  "level": 2,
  "status": "done",
  "started": "2026-11-27T01:30:30Z",
  "result": "fail",
  "gpus": [
    {"index": 0, "pci": "0000:18:00.0", "result": "pass", "message": ""},
    {"index": 1, "pci": "0000:2a:00.0", "result": "fail", "message": "GPU not found on PCI bus 0000:2a:00.0"},
    {"index": 2, "pci": "0000:3a:00.0", "result": "pass", "message": ""},
    {"index": 3, "pci": "0000:5d:00.0", "result": "pass", "message": ""},
    {"index": 4, "pci": "0000:9a:00.0", "result": "pass", "message": ""},
    {"index": 5, "pci": "0000:ab:00.0", "result": "pass", "message": ""},
    {"index": 6, "pci": "0000:ba:00.0", "result": "pass", "message": ""},
    {"index": 7, "pci": "0000:db:00.0", "result": "pass", "message": ""}
  ],
  "finished": "2026-11-27T01:40:30Z"
}
```

```text
shift clock 95 s  ->  dcgm.run {"host": "gpn-a05", "level": 1}
```
```json
{"error": "node_busy", "detail": "gpn-a05 is alloc"}
```

## 8. More errors

```text
shift clock 1510 s  ->  bmc.sensors {"host": "gpn-a09"}
```
```json
{
  "error": "bad_params",
  "detail": "'host' must be one of ['gpn-a02', 'gpn-a03', 'gpn-a04', 'gpn-a05', 'gpn-b02', 'gpn-b03', 'gpn-b04', 'gpn-b05']"
}
```

```text
shift clock 1511 s  ->  pdu.set {"rack": "A03", "outlet": "A1"}
```
```json
{"error": "bad_params", "detail": "missing 'state'"}
```

```text
shift clock 1512 s  ->  bmc.reboot {"host": "gpn-a05"}
```
```json
{"error": "unknown_method"}
```

```text
shift clock 1513 s  ->  noc.ticket {"id": "INC-00001"}
```
```json
{"error": "bad_params", "detail": "no ticket INC-00001"}
```

```text
shift clock 1514 s  ->  fabric.soak {"device": "gpn-a03", "port": 1, "seconds": 600}
```
```json
{"error": "port_down", "detail": "gpn-a03 port 1 is down: nothing to soak"}
```

```text
shift clock 1515 s  ->  dcgm.run {"host": "gpn-a03", "level": 1}
```
```json
{"error": "node_off", "detail": "gpn-a03 is booting"}
```
