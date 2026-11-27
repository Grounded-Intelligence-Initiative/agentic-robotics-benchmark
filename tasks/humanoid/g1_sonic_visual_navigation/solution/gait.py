"""Drive a periodic G1 gait with measured goal, velocity, and heading errors."""

import numpy as np

from sonic_sim.decoder import DEFAULT_Q


class GaitPolicy:
    def __init__(self, encoder):
        self.encoder = encoder

    def codes(
        self,
        tick,
        motion,
        forward,
        left,
        heading_error,
        speed,
        yaw_speed,
        lateral_speed=0.0,
    ):
        """Track a sensor-derived target in horizontal pelvis coordinates at 50 Hz."""
        # The encoder takes ten poses spaced five 50 Hz control ticks apart.
        # Sample each pose on both sides as well, for its central-difference velocity.
        future_ticks = np.arange(10) * 5
        samples = future_ticks[:, None] + np.array([-0.25, 0.0, 0.25])[None, :]
        phase = 2 * np.pi * (tick + samples) / 45
        warmup = np.clip((tick + samples - 25) / 15, 0, 1)

        if motion == "walk":
            # Express remaining distance and lateral displacement in the route frame.
            cross_track = (
                -np.sin(heading_error) * forward + np.cos(heading_error) * left
            )
            along_track = np.cos(heading_error) * forward + np.sin(heading_error) * left
            steering = heading_error + np.arctan(1.5 * cross_track)
            remaining = along_track - 0.075 - np.clip(speed, 0.0, 0.48) * samples / 50
            gain = np.clip(remaining / 0.40, 0.0, 1.0)
            stride = 0.25
            sideways = 0.0
            yaw_bias = np.clip(0.18 * steering, -0.19, 0.19)
            yaw_rate = np.clip(1.8 * steering - 0.08 * yaw_speed, -0.75, 0.75)
        elif motion == "stage":
            # Two-axis braking and sideways translation in the aisle frame.
            stride = np.clip(0.75 * forward - 0.16 * speed + 0.025, -0.12, 0.25)
            sideways = np.clip(0.78 * left - 0.14 * lateral_speed, -0.25, 0.25)
            gain = np.clip(max(abs(forward) / 0.14, abs(left) / 0.14), 0.0, 1.0)
            yaw_bias = np.clip(0.24 * heading_error, -0.16, 0.16)
            yaw_rate = np.clip(2.0 * heading_error - 0.12 * yaw_speed, -0.65, 0.65)
        elif motion == "dock":
            # The final stopping point lies on the aisle axis. Translating and
            # turning independently avoids a late heading correction as the
            # native scorer starts to see a quiet body.
            along = np.cos(heading_error) * forward + np.sin(heading_error) * left
            cross = -np.sin(heading_error) * forward + np.cos(heading_error) * left
            forecast = along - np.clip(speed, 0.0, 0.5) * samples / 50
            gain = np.clip((forecast - 0.015) / 0.36, 0.0, 1.0)
            gain = np.maximum(gain, np.clip((abs(cross) - 0.035) / 0.32, 0, 0.85))
            stride = 0.25 * np.clip(along / 0.22, -1.0, 1.0)
            sideways = np.clip(0.72 * cross - 0.12 * lateral_speed, -0.20, 0.20)
            yaw_bias = np.clip(0.24 * heading_error, -0.16, 0.16)
            yaw_rate = np.clip(2.1 * heading_error - 0.12 * yaw_speed, -0.65, 0.65)
        elif motion == "gate":
            # Regulate lateral position separately from heading so the next
            # cabinet stays in view. Slow forward progress while crossing the
            # aisle; restore a full forward step when centered in the opening.
            speed_fraction = (forward - 0.20) / max(0.25, abs(left) * 2.8) + np.clip(
                1.0 - 4.0 * abs(left), 0.0, 1.0
            )
            stride = 0.25 * np.clip(speed_fraction, 0.42, 1.0)
            sideways = np.clip(0.6 * left - 0.095 * lateral_speed, -0.25, 0.25)
            gain = np.ones_like(samples)
            yaw_bias = np.clip(0.22 * heading_error, -0.18, 0.18)
            yaw_rate = np.clip(1.8 * heading_error - 0.08 * yaw_speed, -0.75, 0.75)
        elif motion == "pivot":
            sign = np.sign(heading_error)
            turning_speed = np.maximum(0.0, sign * yaw_speed)
            remaining = abs(heading_error) - 0.055 - turning_speed * samples / 50
            gain = np.clip(remaining / 0.065, 0.0, 1.0)
            stride = np.clip(0.40 * forward, -0.085, 0.085)
            sideways = np.clip(0.35 * left, -0.08, 0.08)
            yaw_bias = sign * np.clip(0.25 * abs(heading_error), 0.050, 0.16)
            yaw_rate = (
                sign * np.clip(1.65 * abs(heading_error), 0.5, 0.8) - 0.10 * yaw_speed
            )
        elif motion == "stand":
            gain = np.zeros_like(samples)
            stride = sideways = yaw_bias = yaw_rate = 0.0
        else:
            raise ValueError("unknown locomotion motion")

        amplitude = gain * warmup
        q = np.broadcast_to(DEFAULT_Q, (*phase.shape, 29)).copy()
        # Hardware order: hip pitch/roll and knee/ankle pitch/roll on each leg;
        # contralateral shoulder swing follows the feet.
        for side, hip, shoulder in ((1, 0, 15), (-1, 6, 22)):
            foot = -side * stride * np.cos(phase)
            knee = 0.40 * np.maximum(side * np.sin(phase), 0.0)
            roll = 0.065 * np.sin(phase) - side * sideways * np.cos(phase)
            q[:, :, hip] += (-foot - knee / 2) * amplitude
            q[:, :, hip + 3] += knee * amplitude
            q[:, :, hip + 4] += (foot - knee / 2) * amplitude
            q[:, :, hip + 1] += roll * amplitude
            q[:, :, hip + 5] -= roll * amplitude
            q[:, :, shoulder] += 0.55 * foot * amplitude

        yaw_amplitude = amplitude[:, 1]
        if motion == "dock":
            # A small translation error scales the stepping cycle toward zero.
            # Keep the encoder's heading reference active until optical heading
            # is also settled; the stopping disk does not grant a yaw allowance.
            turn_gain = np.clip((abs(heading_error) - 0.065) / 0.13, 0.0, 0.85)
            yaw_amplitude = np.maximum(yaw_amplitude, turn_gain * warmup[:, 1])
        yaw = (yaw_bias + yaw_rate * future_ticks / 50) * yaw_amplitude
        cs, sn = np.cos(yaw), np.sin(yaw)
        zeros = np.zeros_like(cs)
        anchor = np.stack((cs, -sn, sn, cs, zeros, zeros), axis=1)
        velocity_reference = (q[:, 2] - q[:, 0]) * 100  # poses are 0.01 s apart
        return self.encoder.codes_g1(q[:, 1], velocity_reference, anchor)
