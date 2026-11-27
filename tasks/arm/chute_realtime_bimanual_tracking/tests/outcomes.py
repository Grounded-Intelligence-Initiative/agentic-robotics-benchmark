"""The scene's random layout draws and the scoring predicates of cell_env.py, as pure functions of the state."""
import numpy as np


def layout_offsets(seed, salt, low_end, tote_at):
    """Keep the scene's original random draws and return its remaining RNG."""
    g = np.random.default_rng([21, seed] + ([int(salt)] if salt else []))
    low = np.asarray(low_end) + [g.uniform(-0.05, 0.05), g.uniform(-0.08, 0.08), g.uniform(-0.03, 0.03)]
    yaw = float(g.uniform(-0.15, 0.15))
    tote = np.asarray(tote_at) + [g.uniform(-0.05, 0.05), g.uniform(-0.05, 0.05), 0.0]
    return g, low, yaw, tote


def inside_tote(position, tote_position, tote_rotation, inner):
    local = np.asarray(tote_rotation).T @ (np.asarray(position) - np.asarray(tote_position))
    a, b, h = inner
    return bool(abs(local[0]) < a / 2 and abs(local[1]) < b / 2 and -0.01 < local[2] < h + 0.30)


def released_and_resting(in_tote, finger_contact, speed, rest_speed):
    return bool(in_tote and not finger_contact and speed < rest_speed)


def summarize_credits(credit, bystanders_moved):
    n = len(credit)
    if n < 1:
        raise ValueError("a shift must contain at least one part")
    third = max(1, n // 3)
    return {
        "score": max(0.0, (sum(credit) - 0.5 * bystanders_moved) / n),
        "placed": sum(c >= 1.0 for c in credit),
        "held_only": sum(c == 0.5 for c in credit),
        "bystanders_moved": int(bystanders_moved),
        "score_first_third": sum(credit[:third]) / third,
        "score_last_third": sum(credit[-third:]) / third,
    }
