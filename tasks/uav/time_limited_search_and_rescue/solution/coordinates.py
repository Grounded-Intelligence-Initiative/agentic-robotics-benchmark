"""Calibration from the task instruction; contains no world construction code."""
SIZE = 256


def pixel_to_world(u, v):
    return ((u+.5)/32-4, 4-(v+.5)/32)
