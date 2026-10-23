# object_finding_and_organizing_on_messy_desk — scoring and world state machine

Design notes for maintainers and reviewers. The agent-facing rules are in `instruction.md`;
the geometry in `setup/payload/task_card.md`. The code is `setup/payload/messy_desk_env.py`
(the simulator and its state machine) and `setup/payload/messy_desk_score.py` (the score),
both byte-identical copies of the files under `verify/` (this folder).

## Entities

| kind | what | scored |
|---|---|---|
| robot | Franka Panda arm and two-finger gripper | no |
| table | the desk top, the plane z = 0 | no (support surface) |
| container | four KLT containers: gray (class A), light-blue (class B), blue (class C), red (deliveries) | no (they define "in container") |
| target object | the object named by the open request | yes |
| non-target object | every other YCB object in the scene | yes |

## Detectors (every control step, from the simulator's privileged state)

| detector | definition |
|---|---|
| `contact(o)` | PhysX contact report between object `o` and either finger link (summed contact force > 1e-4 N); if the contact view is unavailable, the fallback is centre within 6 cm of the tool centre point |
| `in_container(o)` | the object's centre lies inside a container's interior footprint in that container's own frame (\|x\| ≤ 0.140, \|y\| ≤ 0.109 m) and its height is between the floor − 0.02 and the rim + 0.12 m |
| `on_table(o)` | not inside any container, centre below 0.16 m, placed |
| `resting(o)` | speed < 0.08 m/s |
| `placed(o)` | `resting(o)` and not `contact(o)`: released by the gripper and still |
| `lost(o)` | centre below z = −0.2 m: it left the table |

## Object automaton

Every object runs this automaton; its state and the detector values are exported each step
(`state`, `contact`, `in_container`, `on_table`, `resting`, `role`).

```mermaid
stateDiagram-v2
    [*] --> absent
    absent --> transit : arrival, or the world drops it into a wrong container
    transit --> on_table : placed, outside every container
    transit --> in_container : placed inside a container
    on_table --> held : finger contact
    in_container --> held : finger contact
    held --> transit : released while moving
    held --> on_table : released, still, on the table
    held --> in_container : released, still, inside a container
    on_table --> transit : nudged or pushed
    in_container --> transit : nudged
    on_table --> lost : pushed off the table
    transit --> lost : falls off the table
    in_container --> removed : container service (7 placed objects), or delivered and marked
    removed --> absent : the instance is reused for a later arrival
    lost --> [*]
```

Value of an object in each state (`v_o(t)`): `in_container` +1 when the container is right
for it (its class container; for the requested object the red container as well), −1 when it
is a wrong one; `on_table`, `held`, `transit` 0; `lost` −1 for the rest of the episode;
`removed` keeps the value it had when it was taken away.

## Request automaton

```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> open : the world names an object (present and unique, or absent with p = 0.2)
    open --> delivered : mark 1 while the target is placed in the red container
    open --> claim_wrong : mark 1 otherwise
    open --> absent_correct : mark 2 and no such object exists
    open --> absent_wrong : mark 2 but the object exists
    open --> expired : 1200 steps without a mark
    delivered --> idle
    claim_wrong --> idle
    absent_correct --> idle
    absent_wrong --> idle
    expired --> idle
```

`delivered` and `absent_correct` are successes, the other three outcomes are failures. A
delivered object is taken away by the worker (state `removed`, value frozen at +1).

## Score

$$
S \;=\; \tfrac{1}{2}\,O \;+\; \tfrac{1}{2}\,R,
\qquad
O \;=\; \operatorname{clip}\!\left(
\frac{\sum_{t=1}^{T}\sum_{o}\bigl(v_o(t) - b_o\bigr)}
     {\sum_{t=1}^{T}\sum_{o}\max\bigl(0,\;1 - b_o\bigr)},\;0,\;1\right),
\qquad
R \;=\; \frac{N_{\mathrm{success}}}{N_{\mathrm{closed}}}
$$

and when the episode was cut after $T < H$ steps, $S \leftarrow S \cdot T / H$.

| term | meaning |
|---|---|
| $S$ | the episode score in $[0, 1]$; the task metric `mean_episode_score` is its mean over the hidden seeds |
| $O$ | organization: how much of the sorting work that was available over the episode was done, every step counted (anytime) |
| $R$ | requests: the fraction of closed requests that ended in a success |
| $v_o(t)$ | value of object $o$ at step $t$: +1 placed in a right container, −1 placed in a wrong one, 0 on the table / held / in flight, −1 for ever once lost, last value once removed |
| $b_o$ | baseline of object $o$: its value when it entered the scene (0 for an arrival, +1 for an object that starts in its pile) or when the world last moved it (−1 after a misplacement; re-read once a nudged object rests again) |
| $v_o(t) - b_o$ | the agent's net effect on object $o$: untouched objects contribute 0 |
| $\max(0, 1 - b_o)$ | the gain that was available for $o$: 1 per step for a loose object, 2 for one the world misfiled, 0 for one already in place |
| $T$, $H$ | steps actually run and the horizon (20000); an episode the grader cuts for exhausting its wall-clock budget keeps what it earned, scaled by $T/H$ |
| $N_{\mathrm{success}}$, $N_{\mathrm{closed}}$ | requests closed as `delivered` or `absent_correct`, and all closed requests; $R = 0$ when none closed |

Properties: an untouched scene scores exactly 0 (every $v_o = b_o$, no successful request);
holding an object inside a container is not "placed", so it earns nothing; knocking an object
off the table is as bad as misfiling it and cannot be undone; an aborted episode (protocol
error, step deadline, malformed action) scores 0 and stays in the mean. The grader scales the
measured mean by the oracle's anchor and `full_at` (see `verify/anchor.json`); nothing the
controller prints is a scoring input.
