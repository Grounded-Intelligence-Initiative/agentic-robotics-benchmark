"""Drive a periodic G1 gait with measured goal, velocity, and heading errors."""
import numpy as np

from sonic_sim.decoder import DEFAULT_Q


class GaitPolicy:
    def __init__(self, encoder):
        self.encoder = encoder

    def codes(self, tick, command, observation):
        # Goal coordinates are in the current pelvis frame. Positive yaw turns left.
        forward = command["target_forward_m"]
        left = command["target_left_m"]
        heading_error = command["heading_error_rad"]
        heading = observation["heading_rad"]
        velocity = observation["base_velocity_world"]
        speed = np.cos(heading) * velocity[0] + np.sin(heading) * velocity[1]
        yaw_speed = observation["yaw_rate_rad_s"]

        # The encoder takes ten poses spaced five 50 Hz control ticks apart.
        # Sample each pose on both sides as well, for its central-difference velocity.
        future_ticks = np.arange(10) * 5
        samples = future_ticks[:, None] + np.array([-0.25, 0.0, 0.25])[None, :]
        phase = 2 * np.pi * (tick + samples) / 45
        warmup = np.clip((tick + samples - 25) / 15, 0, 1)

        if command["route"] == "straight_stop":
            # Express remaining distance and lateral displacement in the route frame.
            cross_track = -np.sin(heading_error) * forward + np.cos(heading_error) * left
            along_track = np.cos(heading_error) * forward + np.sin(heading_error) * left
            steering = heading_error + np.arctan(1.5 * cross_track)
            remaining = along_track - 0.075 - np.clip(speed, 0.0, 0.48) * samples / 50
            gain = np.clip(remaining / 0.24, 0.0, 1.0)
            stride = 0.25
            sideways = 0.0
            yaw_bias = np.clip(0.18 * steering, -0.19, 0.19)
            yaw_rate = np.clip(1.8 * steering - 0.08 * yaw_speed, -0.75, 0.75)
        elif command["route"] == "turn_stop":
            sign = np.sign(heading_error)
            turning_speed = np.maximum(0.0, sign * yaw_speed)
            remaining = abs(heading_error) - 0.055 - turning_speed * samples / 50
            gain = np.clip(remaining / 0.065, 0.0, 1.0)
            stride = np.clip(0.40 * forward, -0.085, 0.085)
            sideways = np.clip(0.35 * left, -0.08, 0.08)
            yaw_bias = sign * np.clip(0.25 * abs(heading_error), 0.050, 0.16)
            yaw_rate = sign * np.clip(1.65 * abs(heading_error), 0.5, 0.8) - 0.10 * yaw_speed
        else:
            raise ValueError("unknown locomotion route")

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

        yaw = (yaw_bias + yaw_rate * future_ticks / 50) * amplitude[:, 1]
        cs, sn = np.cos(yaw), np.sin(yaw)
        zeros = np.zeros_like(cs)
        anchor = np.stack((cs, -sn, sn, cs, zeros, zeros), axis=1)
        velocity_reference = (q[:, 2] - q[:, 0]) * 100  # poses are 0.01 s apart
        return self.encoder.codes_g1(q[:, 1], velocity_reference, anchor)
