# The fixed hidden shift set (tests/hidden/seeds.json)

The grader runs the three shifts of `tests/hidden/seeds.json`, for agents and the oracle alike (verify.py: every
episode builds a fresh cell on each seed; the oracle's privileged cells get the same pods and nights). A shift is the
cell's draw from its seed (dc_cell/cluster.py and cell.py: the night's tickets and their root causes, the nodes, ports,
slots and faces, the dirt, the distractors, the pod's installation tolerances, the spares' places on the cart, the
robot's start pose, the sensors' noise and the cameras' white balance), by the rules cell_manual.md section 2 states.

## Selection rule (written before any candidate was drawn or ran)

1. The oracle is frozen first: the commit and the sha256 of every file of `tests/dc_cell/`, `solution/` and
   `tests/grader_config.json` are recorded below.
2. 16 candidate seeds are drawn from `os.urandom` (63 bits each, `tools/draw_candidates.py`), in one call; they are
   listed below in drawn order, with each one's scenario summary, before any of them runs.
3. Each candidate runs once: the privileged reference exactly as the oracle's episode does it (`tools/run_ref.py`: a
   fresh privileged `CellServer` on the seed with the grader's configuration, the oracle's three camera frames,
   `reference_shift`, `finalize`, scored by `dc_cell/scoring.py`), in the task image, one process per candidate, with
   tests/test.sh's numeric environment.
4. A candidate passes when its shift has no safety violation, every ticket earns its full credit (the GPU ticket
   3.5, the link ticket 3.5, the hot swap 1.0, the mid-shift ticket 1.0) and its points after the charges are at least
   9.5 of 10 (the closing's details and one small charge are the margin above the anchor).
5. The hidden set is the first three passing candidates in drawn order, with one coverage rule: the three may not all
   share the GPU ticket's root cause, nor all share the link ticket's root cause, nor all have the same hot-swap part
   (supply or fan module). When the first three passing candidates break it, the third is replaced by the next passing
   candidate in drawn order that satisfies it (then the second, if needed). If fewer than three of the 16 pass, or no
   combination satisfies the coverage rule, 16 more are drawn the same way and the rule goes on in their order.
6. Every candidate's result is recorded below, kept or not.

The anchor (`tests/anchor.json`) is then one full verify of the oracle submission on the kept set (`ale validate`'s
oracle pass), and `full_at` is sized from the difference between that verify and the selection runs.

## The frozen oracle

Frozen 2026-11-27T18:25Z (2026-11-28 02:25 HKT): the task folder as it stood uncommitted on branch
`task/gpu_cluster_break_fix_shift`, based on upstream `abb2c8e8`. sha256 of every file the oracle's episodes run (the
cell and its reference program, the oracle submission, the grader's configuration):

```
ee60e837d3188c6c61175c7527494675b0bfab0ff3791dcc1483cb6bbf15f3dd  solution/reference/oracle_token
975fd31fc4d41f4396e55703755db9804de88c8137153d462c3cf261967101dd  solution/reference/shift.py
d59e915b03c8a474ab30b6e4bbe3e8594786183ea6bdc0db771023c66a2ad549  solution/solve.sh
2955e36a2591919b58e803579e4e5eab0929eeb194378828f1bb860711f9abff  tests/dc_cell/__init__.py
e8820420d4d8aed333efaa3f84cc31b519af461a648d723c7eb924f22dc79b7e  tests/dc_cell/cell.py
a1324c7bab29ca2aebeb799d3d31364ec094d268597923215b516d9008f80f0f  tests/dc_cell/cluster.py
c5041581eb235f9fbb93a70695e011136479d93bd7a9b4a1457e8617a72d0537  tests/dc_cell/control.py
eb1159201f8a0ede736bb7597bf4c9955eac4596f38c24bddd23eaca6f67c761  tests/dc_cell/equipment.py
42c2a3649e5d125d68be40645ac21916aac9af9c5cf169eceaf74267e3251573  tests/dc_cell/grips.py
481e69d0dac48f1a3b3d5506e1ff3a62234102519ea2f353aebc407f777e8f08  tests/dc_cell/hall.py
3b34a59cf3177644958fda39cc1d9a0ebe307714dbf605491ad54fcc75888664  tests/dc_cell/ibl.py
c713c5fab44451825566124534344e0edad72ad19ee72d0d2a4360abfc319d1f  tests/dc_cell/look.py
fc266e6086fc73fe3ea9b95e1bf98e7eded0cb46390994bff8c93e7f48c5ef73  tests/dc_cell/mech.py
fb7972a8794fa9782ca8c57e8271666dfd4cff53d51e8b0cd7a6f20ca5cf7d30  tests/dc_cell/meshes.py
afb51466dd4d48c34615d8f7872b2058880a88bc39aee898a16da12b86d76bda  tests/dc_cell/nav.py
bd6dfcda2391f3badca9bb33d4a3f05edcbeaf250d532bcdf5c91ca711f86980  tests/dc_cell/parts.py
014590563932b305e1f32776696d14daf3fe30a283d34ebecb1f977140957049  tests/dc_cell/ref_data/cart.json
a0365232db063fd536736773fb05463269cdda1529bb61f56f8000aff0c99b9f  tests/dc_cell/ref_data/gpu_driver.json
38da79e6c9ec1bdba3d88a52b06afb85f2aee5729bb1fa6b0e11a00348cb8337  tests/dc_cell/ref_data/gpu_lift_left.json
7be363e615be79a903ae6a33c15d34a34e6e96d0817c8733c05d229e4cf3ea6e  tests/dc_cell/ref_data/gpu_lift_right.json
d6ed776ac14060f0a5f233290d3ed1ab5445f6007fdc160aa83514542fd994b0  tests/dc_cell/ref_data/rear_part.json
3047f53b039c637afde32c2baf27ebb7b396ab3fda06eef00fea75eedc36a2f9  tests/dc_cell/ref_data/rear_tool.json
8fe5194e80ebc6ab28e357f4b00fb857482a8514bf6a4107ed10161d8cc0062a  tests/dc_cell/reference.py
172f2f29650bd3fa025677b057a84445d787944ea9a640967667b8c78f5e56db  tests/dc_cell/refgpu.py
b2bad5de7c8bf3103458e870e7339c10897de7c96b17e326a22b3c79bfe7c379  tests/dc_cell/refnode.py
02db9788f92b4664c9d511b889a597784753e3455bf24a7e9f202367d71d73ac  tests/dc_cell/refparts.py
44bdf77e2a8dcdd622ed06189800eafab85297f532877c74ee552269886299a1  tests/dc_cell/refrear.py
6713dd4a33fc85fc20fe09d87f912b6eb4111e4e076958e59042227403a34e3e  tests/dc_cell/refskills.py
cdf7cdc1fd0678cb02619981a9ec84736b0eda0c248bafe0dd95b37584133d1f  tests/dc_cell/robot.py
d9e417a9daf77c0781b2a8c9519ee0eae8560d6e23e8425848e6a4336fd836ad  tests/dc_cell/scene.py
3b887ca0005bc680308b1a7419f27c64bf706fd2315f4b1d0b163a7c005d4eba  tests/dc_cell/scoring.py
f27fcc2238dac85e476b337f97779e55e5ef4e259a81a62bdcd9ff903757ee2f  tests/dc_cell/server.py
763cbe9b2522893c8cbb0a2650b8f6c603ec1b37188b74272a36ea8c56de021e  tests/dc_cell/textures.py
dbac8d1a656f5d93bc0354d8748091aab64b7167ed4d85d08b6f4968d5c1f729  tests/dc_cell/walk_core.py
5083a279f3a8f7167c32a9077bbdc0740cecf80a0b85f253ec45fe954547c899  tests/grader_config.json
```

## The candidates

Drawn 2026-11-27T18:26Z with `tools/draw_candidates.py 16` (one `os.urandom` call, 63-bit seeds), in drawn order,
before any of them ran (the summary is each seed's scenario as the cell draws it: host, slot or port, root cause):

| # | seed | GPU ticket | link ticket | hot swap | mid-shift |
|---|---|---|---|---|---|
| 1 | 7127052870086496390 | gpn-b05 sxm2 transient | gpn-b04 k4 dirty_leaf_receptacle | gpn-b02 fan1 | fan_failed gpn-a05 |
| 2 | 4310799903370640525 | gpn-b03 sxm6 module_dead | gpn-a02 k5 dirty_node_connector | gpn-a05 psu1 | link gpn-b04 |
| 3 | 3563498726050192049 | gpn-b04 sxm1 hbm_bad | gpn-a03 k2 dirty_leaf_connector | gpn-a04 fan4 | link gpn-a02 |
| 4 | 8743177256946226189 | gpn-a04 sxm2 transient | gpn-a05 k5 aged_tx_node | gpn-b03 fan5 | link gpn-a02 |
| 5 | 6962499043991829572 | gpn-b05 sxm7 unseated | gpn-a02 k7 scratched_cable_end | gpn-a04 fan2 | fan_failed gpn-a03 |
| 6 | 168276881647560003 | gpn-b02 sxm6 hbm_bad | gpn-b04 k0 dirty_leaf_receptacle | gpn-a04 psu4 loose6 | fan_failed gpn-a05 |
| 7 | 4974410402135564909 | gpn-a03 sxm3 hbm_bad | gpn-a05 k2 scratched_cable_end | gpn-a04 fan3 | fan_failed gpn-b03 |
| 8 | 4329195368052289139 | gpn-b05 sxm5 transient | gpn-b02 k7 scratched_cable_end | gpn-a03 psu2 | link gpn-a04 |
| 9 | 2928995892624956408 | gpn-b05 sxm7 module_dead | gpn-a03 k0 dirty_leaf_receptacle | gpn-b02 fan4 | link gpn-b04 |
| 10 | 4983101627133604866 | gpn-b02 sxm7 unseated | gpn-a03 k1 dirty_node_connector | gpn-b05 fan3 | fan_failed gpn-b03 |
| 11 | 4808019844849934337 | gpn-b05 sxm6 transient | gpn-a05 k5 aged_tx_node | gpn-a03 psu1 loose2 | link gpn-b04 |
| 12 | 2388517499808385285 | gpn-b03 sxm5 transient | gpn-a03 k4 scratched_cable_end | gpn-b02 psu1 loose2 | link gpn-a02 |
| 13 | 1167301163179408160 | gpn-b02 sxm1 baseboard | gpn-a03 k0 dirty_node_receptacle | gpn-b04 fan4 | link gpn-b03 |
| 14 | 5869227872611576301 | gpn-b02 sxm3 unseated | gpn-b04 k2 dirty_node_connector | gpn-a05 fan1 | fan_failed gpn-b03 |
| 15 | 2605887746357119166 | gpn-b03 sxm4 hbm_bad | gpn-a05 k2 dirty_leaf_connector | gpn-b02 psu2 loose6 | link gpn-b04 |
| 16 | 9023841040613621371 | gpn-b02 sxm7 module_dead | gpn-b05 k2 dirty_node_connector | gpn-a04 fan3 | link gpn-b03 |

## Results

Run 2026-11-27 18:25-19:08Z, one process per candidate (`tools/launch_ref.sh SEED a`, 5 CPUs each, all 16 at once),
scored by `dc_cell/scoring.py`; `tools/select_set.py a` applies the rule:

| # | seed | stage score | points | charges | result |
|---|---|---|---|---|---|
| 1 | 7127052870086496390 | 1.000 | 10.00 | none | pass |
| 2 | 4310799903370640525 | 0.000 | 5.10 | none | safety: robot_contact:g1/left_wrist_pitch_link/rack_B01 |
| 3 | 3563498726050192049 | 0.860 | 8.60 | none | short: mid_shift 0.00 |
| 4 | 8743177256946226189 | 1.000 | 10.00 | none | pass |
| 5 | 6962499043991829572 | 1.000 | 10.00 | none | pass |
| 6 | 168276881647560003 | 1.000 | 10.00 | none | pass |
| 7 | 4974410402135564909 | 0.925 | 9.25 | part_on_floor 0.5 | points 9.25 |
| 8 | 4329195368052289139 | 0.860 | 8.60 | none | short: hot_swap 0.00 |
| 9 | 2928995892624956408 | 0.635 | 6.35 | none | short: gpu 0.00 |
| 10 | 4983101627133604866 | 1.000 | 10.00 | none | pass |
| 11 | 4808019844849934337 | 1.000 | 10.00 | none | pass |
| 12 | 2388517499808385285 | 1.000 | 10.00 | none | pass |
| 13 | 1167301163179408160 | 0.925 | 9.25 | part_on_floor 0.5 | points 9.25 |
| 14 | 5869227872611576301 | 0.000 | 8.85 | none | safety: robot_contact:g1/left_hip_yaw_link/cart |
| 15 | 2605887746357119166 | 0.875 | 8.75 | part_on_floor 0.5, part_on_floor 0.5 | points 8.75 |
| 16 | 9023841040613621371 | 0.635 | 6.35 | none | short: gpu 0.00 |

Passing, in drawn order: 1, 4, 5, 6, 10, 11, 12. The first three (1, 4, 5) all have a fan module as their hot swap,
which the coverage rule forbids; the third is replaced by the next passing candidate that satisfies it, candidate 6
(a supply, with a loose cord on another). **The hidden set is candidates 1, 4 and 6** (`tests/hidden/seeds.json`):

* #1: a transient GPU fault (gpn-b05 SXM2), a dirty leaf receptacle (gpn-b04 port 4), a failed fan module (gpn-b02
  FAN1), a fan failing mid-shift (gpn-a05);
* #4: a transient GPU fault (gpn-a04 SXM2), an aged node transceiver (gpn-a05 port 5), a failed fan module (gpn-b03
  FAN5), a second link mid-shift (gpn-a02);
* #6: failing HBM (gpn-b02 SXM6), a dirty leaf receptacle (gpn-b04 port 0), a failed supply with a loose cord on
  another (gpn-a04 PSU4, PSU6's cord), a fan failing mid-shift (gpn-a05).

Each scored 1.0 (10 points, no charge, no violation; #1: 1169 s wall, 2867 s simulated; #4: 930 s wall, 2739 s simulated; #6: 1360 s wall, 2935 s simulated). The candidates that did not pass show where the reference
itself falls short (a GPU module back into SXM7 after the cart; a leaf MPO plug at B01; a hip against the cart while
taking a fan module; a part left on the floor): the agent's program may do better there, but they are not graded.

## After the selection

* 2026-11-28: `tests/hidden/oracle_token.sha256` corrected. The first `ale validate` (validate-ce326372) gave the oracle
  0.0 with `oracle_token_mismatch`: the file held the sha256 of `solution/reference/oracle_token` as a file, with its
  trailing newline, while `verify.py` hashes the stripped token. It now holds the sha256 of the stripped token. The
  file is not among the frozen files above, and none of them changed.
* The anchor (`tests/anchor.json`): the oracle's `shift_score` in the oracle pass of `ale validate` validate-87236f3e
  (2026-11-28), 1.0, the same as each kept shift's selection run; `full_at` 0.98.
