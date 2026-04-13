import numpy as np


class GyroSensor:
    def __init__(self, bias0: np.ndarray, noise_std: float, bias_walk_std: float):
        self.bias = bias0.astype(float).copy()
        self.noise_std = noise_std
        self.bias_walk_std = bias_walk_std

    def measure(self, true_w: np.ndarray, dt: float) -> np.ndarray:
        bias_walk = np.random.randn(3) * self.bias_walk_std * np.sqrt(dt)
        self.bias += bias_walk
        noise = np.random.randn(3) * self.noise_std
        return true_w + self.bias + noise


class StarTracker:
    def __init__(self, noise_std_rad: float):
        self.noise_std = noise_std_rad

    def measure(self, true_q: np.ndarray) -> np.ndarray:
        noise = np.random.randn(3) * self.noise_std
        angle = np.linalg.norm(noise)

        if angle < 1e-12:
            dq = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            axis = noise / angle
            dq = np.hstack((
                np.cos(angle / 2.0),
                axis * np.sin(angle / 2.0)
            ))

        from utils import quat_multiply, normalize_quaternion
        q_meas = quat_multiply(dq, true_q)
        return normalize_quaternion(q_meas)
