from dataclasses import dataclass
import numpy as np


@dataclass
class SimConfig:
    dt: float = 0.01
    t_final: float = 20.0

    # Initial truth state
    q0: np.ndarray = None
    w0: np.ndarray = None

    # Spacecraft inertia matrix
    inertia: np.ndarray = None

    # Gyro parameters
    gyro_bias0: np.ndarray = None
    gyro_noise_std: float = 0.002  # rad/s
    gyro_bias_walk_std: float = 1e-5  # rad/s/sqrt(s)

    def __post_init__(self):
        if self.q0 is None:
            self.q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        if self.w0 is None:
            self.w0 = np.array([0.05, -0.03, 0.02], dtype=float)
        if self.inertia is None:
            self.inertia = np.diag([10.0, 12.0, 8.0])
        if self.gyro_bias0 is None:
            self.gyro_bias0 = np.array([0.001, -0.0015, 0.0008], dtype=float)
