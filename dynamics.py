import numpy as np
from utils import quat_derivative, normalize_quaternion


def angular_acceleration(w: np.ndarray, inertia: np.ndarray, torque: np.ndarray) -> np.ndarray:
    iw = inertia @ w
    coriolis = np.cross(w, iw)
    return np.linalg.inv(inertia) @ (torque - coriolis)


def propagate_truth(
    q: np.ndarray,
    w: np.ndarray,
    inertia: np.ndarray,
    torque: np.ndarray,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    q_dot = quat_derivative(q, w)
    w_dot = angular_acceleration(w, inertia, torque)

    q_next = normalize_quaternion(q + q_dot * dt)
    w_next = w + w_dot * dt
    return q_next, w_next
