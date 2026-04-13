import numpy as np
from utils import quat_derivative, normalize_quaternion, quat_multiply, skew


class AttitudeEKF:
    def __init__(self, q0: np.ndarray, bias0: np.ndarray):
        self.q = normalize_quaternion(q0.astype(float).copy())
        self.bias = bias0.astype(float).copy()

        # State covariance for [attitude_error(3), gyro_bias(3)]
        self.P = np.eye(6) * 0.01

        # Process noise
        self.Q = np.diag([
            1e-6, 1e-6, 1e-6,
            1e-4, 1e-4, 1e-4
        ])

        # Measurement noise
        self.R = np.eye(3) * 5e-4

        # Residual-based monitoring
        self.last_residual = np.zeros(3, dtype=float)
        self.last_residual_norm = 0.0
        self.fault_detected = False
        self.residual_threshold = 0.2

        # Recovery / FDIR status
        self.recovery_active = False
        self.recovery_scale_factor = 2.0

    def predict(self, gyro_meas: np.ndarray, dt: float) -> None:
        w = gyro_meas - self.bias

        q_dot = quat_derivative(self.q, w)
        self.q = normalize_quaternion(self.q + q_dot * dt)

        F = np.zeros((6, 6), dtype=float)
        F[0:3, 0:3] = -skew(w)
        F[0:3, 3:6] = -np.eye(3)

        Phi = np.eye(6) + F * dt
        self.P = Phi @ self.P @ Phi.T + self.Q * dt

    def update_star_tracker(self, q_meas: np.ndarray) -> None:
        q_conj = np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]], dtype=float)
        q_err = quat_multiply(q_meas, q_conj)
        q_err = normalize_quaternion(q_err)

        delta_theta = 2.0 * q_err[1:]
        self.last_residual = delta_theta.copy()
        self.last_residual_norm = float(np.linalg.norm(delta_theta))
        self.fault_detected = self.last_residual_norm > self.residual_threshold

        H = np.zeros((3, 6), dtype=float)
        H[:, 0:3] = np.eye(3)

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ delta_theta
        dtheta = dx[0:3]
        dbias = dx[3:6]

        angle = np.linalg.norm(dtheta)
        if angle < 1e-12:
            dq = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            axis = dtheta / angle
            dq = np.hstack((
                np.cos(angle / 2.0),
                axis * np.sin(angle / 2.0)
            ))

        self.q = normalize_quaternion(quat_multiply(dq, self.q))

        # Adaptive recovery: keep bias updates, but slower in recovery mode
        if self.recovery_active:
            self.bias += 0.2 * dbias
        else:
            self.bias += dbias

        I = np.eye(6)
        self.P = (I - K @ H) @ self.P