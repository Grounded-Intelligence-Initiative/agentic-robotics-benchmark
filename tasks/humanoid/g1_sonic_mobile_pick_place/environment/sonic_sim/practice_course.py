"""Optional practice layouts for the free-base mobile tabletop task."""

import mujoco

from . import MobileSession
from .mobile import HORIZON, MobileTask


class FiveMetreTask(MobileTask):
    start_x_bounds = (-5.005, -4.965)

    def __init__(self, task, seed, robot_root):
        self.dock_displacement = None
        super().__init__(task, seed, robot_root)

    def _record(self, dt, contacts, velocity):
        super()._record(dt, contacts, velocity)
        if self.dock_tick is not None and self.dock_displacement is None:
            progress = self.data.xpos[self.ankles, 0] - self.initial_ankles[:, 0]
            forward = float(self.data.xpos[self.pelvis, 0] - self.initial_pelvis[0])
            self.dock_displacement = (forward, float(progress[0]), float(progress[1]))
            if forward < 4.75 or min(progress) < 4.60:
                self.violation = "insufficient_five_metre_travel"

    def metrics(self):
        out = super().metrics()
        dock = self.dock_displacement
        out.update(
            course="five-metre",
            five_metre_docked=dock is not None and self.violation is None,
            dock_forward_m=dock[0] if dock else None,
            dock_left_ankle_forward_m=dock[1] if dock else None,
            dock_right_ankle_forward_m=dock[2] if dock else None,
        )
        return out

    def course_view(self):
        """Render a whole-course practice figure without modifying any solver view."""
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [-2.4, -0.06, 0.63]
        camera.distance, camera.azimuth, camera.elevation = 6.15, 98.0, -12.0
        # The collision plane is infinite. MuJoCo bakes its finite drawing mesh
        # when this separate review renderer is created. Restore the normal extent
        # before returning to the caller or stepping physics.
        extent = self.model.geom_size[self.floor, :2]
        saved = extent.copy()
        try:
            extent[:] = [8.0, 6.0]
            with mujoco.Renderer(self.model, width=960, height=480) as renderer:
                renderer.update_scene(self.data, camera=camera)
                return renderer.render().copy()
        finally:
            extent[:] = saved


class FiveMetreSession(MobileSession):
    scene_type = FiveMetreTask

    def __init__(self, task, seed, assets_root, horizon=1800):
        super().__init__(task, seed, assets_root, horizon)

    def course_view(self):
        return self._scene.course_view()


COURSES = {"one-metre": (MobileSession, HORIZON), "five-metre": (FiveMetreSession, 1800)}
