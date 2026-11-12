"""Camera calibration from the task instruction: 256 x 256 px, 32 px per metre, centred on the bay."""
SIZE = 256


def pixel_to_world(u, v):
    return ((u+.5)/32-4, 4-(v+.5)/32)


def world_to_pixel(x, y):
    return ((x+4)*32-.5, (4-y)*32-.5)
