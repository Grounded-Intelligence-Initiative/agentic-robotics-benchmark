"""Fixed calibrated world-frame RGB views of the physical scene."""


CAMERAS = ("overview", "hand", "side")


def default_camera_rig():
    """Return an independent copy of the released camera poses in meters and degrees."""
    return {
        "version": 1,
        "cameras": {
            "overview": dict(lookat=[0.13, -0.1, 0.71], distance=2.45, azimuth=117.0, elevation=-16.0),
            "hand": dict(lookat=[0.29, -0.21, 0.81], distance=0.93, azimuth=108.0, elevation=-25.0),
            "side": dict(lookat=[0.29, -0.21, 0.81], distance=1.1, azimuth=175.0, elevation=-28.0),
        },
    }


# The render API's RGB8 raster uses the center of pixel (0, 0) as its origin.
def camera_calibration(renderer, width, height):
    left = renderer.scene.camera[0]
    near = left.frustum_near
    f_px = height * near / (left.frustum_top - left.frustum_bottom)
    cx = (width - 1) / 2 - f_px * left.frustum_center / near
    cy = height * left.frustum_top / (left.frustum_top - left.frustum_bottom) - 0.5
    return dict(focal_px=float(f_px), center_px=[float(cx), float(cy)], width=width, height=height)
