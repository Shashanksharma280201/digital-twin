"""IMU simulation from the ground-truth trajectory.

The IMU frame trajectory is T_world_base * T_base_imu, so the lever arm is exact. Specific
force f = R_wi^T (a_w - g_w), gyro = body angular rate; white noise (density * sqrt(rate))
plus bias random walk. Output is in g (as the Livox and Airy drivers report) in the raw
sensor axes: a_raw = R_raw^T a_imu, with R_raw = I (Mid-360) or the Airy bridge rotation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .trajectory import Trajectory

GRAVITY = 9.80665


@dataclass
class ImuNoise:
    acc_density: float = 2.0e-3    # m/s^2/sqrt(Hz)
    gyro_density: float = 2.0e-4   # rad/s/sqrt(Hz)
    acc_bias_rw: float = 1.0e-4    # m/s^3/sqrt(Hz)
    gyro_bias_rw: float = 1.0e-5   # rad/s^2/sqrt(Hz)
    acc_bias0: float = 0.02        # m/s^2, initial bias sigma
    gyro_bias0: float = 0.002      # rad/s

    @classmethod
    def ideal(cls) -> "ImuNoise":
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class ImuSamples:
    t: np.ndarray       # (N,) run seconds
    acc_g: np.ndarray   # (N, 3) in g, raw axes
    gyro: np.ndarray    # (N, 3) rad/s, raw axes


def simulate_imu(traj_imu: Trajectory, rate: float = 200.0, R_raw: np.ndarray | None = None,
                 noise: ImuNoise | None = None, seed: int = 0, t_start: float | None = None,
                 t_end: float | None = None) -> ImuSamples:
    noise = noise or ImuNoise()
    rng = np.random.default_rng(seed)
    t0 = traj_imu.start if t_start is None else t_start
    t1 = traj_imu.end if t_end is None else t_end
    t = np.arange(t0, t1, 1.0 / rate)

    rot = traj_imu.rotation(t)
    acc_world = traj_imu.acceleration(t) - np.array([0.0, 0.0, -GRAVITY])
    f = rot.inv().apply(acc_world)
    w = traj_imu.angular_velocity_body(t)

    n = len(t)
    dt = 1.0 / rate
    acc_bias = rng.normal(0, noise.acc_bias0, 3) + np.cumsum(
        rng.normal(0, noise.acc_bias_rw * np.sqrt(dt), (n, 3)), axis=0)
    gyro_bias = rng.normal(0, noise.gyro_bias0, 3) + np.cumsum(
        rng.normal(0, noise.gyro_bias_rw * np.sqrt(dt), (n, 3)), axis=0)
    f = f + acc_bias + rng.normal(0, noise.acc_density * np.sqrt(rate), (n, 3))
    w = w + gyro_bias + rng.normal(0, noise.gyro_density * np.sqrt(rate), (n, 3))

    if R_raw is not None:
        f = f @ R_raw  # row-wise R_raw.T @ v
        w = w @ R_raw
    return ImuSamples(t, f / GRAVITY, w)
