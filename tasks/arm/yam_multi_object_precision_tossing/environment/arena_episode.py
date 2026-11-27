"""Run a scored arena alongside a second physical layout for same-step observation probes."""
import json
from pathlib import Path

import numpy as np

from workcell import ArenaRound, DURATION, MAX_ACTIONS


def action_vector(action, arena):
    # Compare the actual actuator trajectories on a common clock, regardless of knot count.
    if action == {"finish": True}:
        return np.zeros(2 + 7 * 31)
    item, _, rows = arena.validate_action(action)
    times = np.linspace(0, DURATION, 31)
    joints = [np.interp(times, rows[:, 0], column) for column in rows[:, 1:7].T]
    jaw_indices = np.searchsorted(rows[:, 0], times, side="right") - 1
    return np.r_[1, list(arena.items).index(item.id), np.ravel(joints), rows[jaw_indices, 7]]


def run_episode(seed, other_seed, solver, lock, record_path=None, *,
                max_actions=MAX_ACTIONS, observe=None):
    if not isinstance(max_actions, int) or not 1 <= max_actions <= MAX_ACTIONS:
        raise ValueError("action cap must be within the declared round budget")
    if observe is None:
        observe = lambda world: world.observation()
    arena = ArenaRound(seed, record=record_path is not None)
    other = ArenaRound(other_seed) if other_seed is not None else None
    try:
        lock.canonical = lambda action: action_vector(action, arena)
        for turn in range(max_actions):
            observation = observe(arena)
            if lock.due():
                if other is None:
                    raise ValueError("responsiveness probes require a second layout")
                action = lock.probe(solver, turn, observation, observe(other))
            else:
                action = solver.act(turn, observation)
            if action == {"finish": True}:
                break
            arena.step(action)
            if other is not None:
                other.step(action)
        return arena.result()
    finally:
        try:
            if record_path is not None:
                folder = Path(record_path)
                folder.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(folder / "frames.npz",
                                    qpos=np.stack([frame[2] for frame in arena.frames]),
                                    time=np.array([frame[0] for frame in arena.frames], dtype=np.float64),
                                    action=np.array([frame[1] for frame in arena.frames], dtype=np.int32))
                (folder / "result.json").write_text(json.dumps({"seed": seed, "result": arena.result()}))
        finally:
            try:
                arena.close()
            finally:
                if other is not None:
                    other.close()
