"""Sealed split and independently generated reconstruction stimuli."""
from itertools import product

import numpy as np

from arm_dynamics import pose_row
from world import ARM_CARTRIDGES, ARM_PAYLOADS, LIMITS, World, arm_rng, drive_cassette


class InvalidStimulus(RuntimeError):
    """The sampled input failed to excite the intended physical regime."""


def transfer_plan(stream, exploration_seed):
    """One independent layout per ballast, pressure class and bolt side."""
    families = list(product(ARM_PAYLOADS, ARM_CARTRIDGES, (-1, 1)))
    for i in range(len(families)-1, 0, -1):
        j = stream.choice(range(i+1))
        families[i], families[j] = families[j], families[i]
    mounts = list(product((-1, 1), repeat=3))
    cassettes = list(product((-1, 1), (.8, 1.2), (.8, 1.2)))
    for group in (mounts, cassettes):
        for i in range(len(group)-1, 0, -1):
            j = stream.choice(range(i+1))
            group[i], group[j] = group[j], group[i]
    excluded = {exploration_seed}
    plan = []
    for family, (back, turn, tare), cassette in zip(families, mounts, cassettes):
        seed = stream.seed(64)
        while (seed in excluded or drive_cassette(seed) != cassette
               or arm_rng(seed, b"arm-sleeve-dir").choice((-1, 1)) != back
               or arm_rng(seed, b"arm-key-turn").choice((-1, 1)) != turn
               or arm_rng(seed, b"arm-cradle-tare").choice((-1, 1)) != tare):
            seed = stream.seed(64)
        excluded.add(seed)
        plan.append((seed, family))
    return plan


def trace(seed, stimulus_seed, case):
    """Run the actual plant from a fresh copy of the explored scene."""
    rng = arm_rng(stimulus_seed, f"heldout:{case}".encode("ascii"))
    # Isolate the explored cartridge on the bare bench, at its reset XY.
    # Live episodes keep the source sleeve and start with the bolt engaged.
    world = World("arm", seed, render=False, stop_parked=True, shipping=False)
    actions, poses = [], []
    bx, by = world.block
    tx, ty = world.tray
    sag = world.arm_mass*9.81/world.arm_gains[2]
    gx = bx + (
        rng.uniform(-.065, .065) if case == 0
        else world.arm_com+rng.uniform(-.018, .018) if case in (1, 3)
        else .56*world.arm_com+rng.uniform(-.035, .035)
    )
    hold = float(rng.uniform(.620, .644) if world.arm_cartridge == "light"
                 else rng.uniform(.952, .989))

    def apply(action):
        if not world.done:
            world.act(action)
            actions.append(action)
            poses.append(pose_row(world.data))

    def go(goal, pressure, speed, dwell):
        tick = 0
        while not world.done:
            error = np.array(goal)-world.arm.command
            limit = (np.minimum(min(speed, .013*(tick+1)), np.sqrt(.026*np.abs(error)))
                     if pressure else speed)
            delta = np.clip(error, -limit, limit)
            motor = world.arm.to_motor @ delta[:2]
            motor /= max(1., np.max(abs(motor))/.14)
            apply(np.r_[motor/.14, delta[2]/.14, pressure].tolist())
            tick += 1
            if np.linalg.norm(delta) < .0001:
                dwell -= 1
                if not dwell:
                    break

    for xy in rng.uniform(-.72, .72, (int(rng.integers(6, 12)), 2)):
        apply([*xy, float(rng.uniform(-.48, .48)), 0.])
    go([gx, by, .79], 0., .132, 6)
    seat = float(rng.uniform(.202, .213))
    go([gx, by, seat], 0., .086, 7)
    if case == 3:
        hold = float(rng.uniform(.620, .644))
    go([gx, by, seat], hold, .05, 3)
    if case == 0:
        # A tilted lift at a previously unseen pickup point; one end can remain supported.
        go([gx, by, float(rng.uniform(.385, .415))+sag], hold, .023, 12)
        go([gx, by, .204], hold, .030, 9)
        go([gx, by, .204], 0., .05, 7)
    elif case == 1:
        # Balanced clearances in a real contact fixture, rather than a free-air replay.
        z = float(rng.uniform(1.04, 1.10))
        dx = gx-bx
        stage = tx-1.61+dx
        go([gx, by, z], hold, .057, 7)
        go([stage, ty, z], hold, .079, 6)
        entry = float(rng.uniform(.692, .704))+sag
        go([stage, ty, entry], hold, .044, 7)
        go([tx+dx, ty, entry], hold, .061, 7)
        dock = .632+sag
        go([tx+dx, ty, dock], hold, .024, 7)
        go([tx+dx, ty, dock], 0., .04, 8)
    elif case == 2:
        # Off-center pendular response under simultaneous horizontal/vertical loading.
        z = float(rng.uniform(.56, .63))
        go([gx, by, z], hold, .033, 8)
        px, py = gx+float(rng.uniform(-.21, -.10)), by+float(rng.uniform(.10, .19))
        go([px, py, z+.035], hold, .052, 7)
        go([px, py, .215], hold, .026, 11)
        go([px, py, .215], 0., .04, 8)
    elif case == 3:
        # Same low setting in both classes, safely approached above the ballast.
        go([gx, by, float(rng.uniform(.398, .413))], hold, .020, 9)
        go([gx, by, .204], hold, .024, 9)
        go([gx, by, .204], 0., .04, 6)
    else:
        raise ValueError("unknown held-out case")
    result = np.asarray(poses)
    motion = np.max(np.linalg.norm(result[:, 3:6]-result[0, 3:6], axis=1))
    if (world.broken or not 10 < len(result) < LIMITS["arm"]
            or (case != 3 and motion < .11)
            or (case in (0, 2) and np.max(np.abs(result[:, 6])) < .05)
            or (case == 3 and (world.damage or (motion > .035) != (world.arm_cartridge == "light")))
            or (case == 1 and not world.success)):
        raise InvalidStimulus(f"invalid held-out stimulus {case}, {world.metrics()}")
    world.close()
    return actions, result


def reconstruction_cases(scene_seed, stimulus_seed):
    cases = []
    for case in range(4):
        for attempt in range(8):
            try:
                cases.append(trace(scene_seed, stimulus_seed+attempt, case))
                break
            except InvalidStimulus:
                if attempt == 7:
                    raise
    return cases
