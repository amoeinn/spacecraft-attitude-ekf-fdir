import numpy as np

from config import SimConfig
from dynamics import propagate_truth
from sensors import GyroSensor, StarTracker
from ekf import AttitudeEKF
from fdir import FDIR
from plots import (
    plot_angular_rates,
    plot_attitude_error,
    plot_bias_detection,
    plot_recovery,
)


def quat_error_angle_deg(q_true: np.ndarray, q_est: np.ndarray) -> float:
    dot = np.clip(np.dot(q_true, q_est), -1.0, 1.0)
    return 2.0 * np.arccos(abs(dot)) * 180.0 / np.pi


def main(return_error: bool = False):
    cfg = SimConfig()

    n_steps = int(cfg.t_final / cfg.dt)
    time = np.arange(n_steps) * cfg.dt

    q = cfg.q0.copy()
    w = cfg.w0.copy()

    gyro = GyroSensor(
        bias0=cfg.gyro_bias0,
        noise_std=cfg.gyro_noise_std,
        bias_walk_std=cfg.gyro_bias_walk_std,
    )

    star_tracker = StarTracker(noise_std_rad=0.005)

    ekf = AttitudeEKF(
        q0=cfg.q0.copy(),
        bias0=np.zeros(3, dtype=float),
    )

    fdir = FDIR(
        residual_threshold=0.2,
        bias_threshold=0.04,
    )

    q_hist = np.zeros((n_steps, 4))
    w_hist = np.zeros((n_steps, 3))
    gyro_hist = np.zeros((n_steps, 3))
    bias_hist = np.zeros((n_steps, 3))
    q_est_hist = np.zeros((n_steps, 4))
    attitude_error_deg = np.zeros(n_steps)

    residual_hist = np.full(n_steps, np.nan)
    residual_fault_hist = np.zeros(n_steps, dtype=int)

    bias_norm_hist = np.zeros(n_steps)
    bias_fault_hist = np.zeros(n_steps, dtype=int)

    recovery_hist = np.zeros(n_steps, dtype=int)

    torque = np.zeros(3)

    fault_injected = False
    fault_time = 10.0
    fault_step = int(fault_time / cfg.dt)

    startup_guard_time = 2.0
    detection_time = None
    recovery_triggered_once = False

    for k in range(n_steps):
        t = k * cfg.dt

        q_hist[k] = q
        w_hist[k] = w

        if (not fault_injected) and (k == fault_step):
            print("Injecting gyro fault at t = {:.2f} s".format(t))
            gyro.bias += np.array([0.05, -0.03, 0.02], dtype=float)
            fault_injected = True

        gyro_meas = gyro.measure(w, cfg.dt)
        gyro_hist[k] = gyro_meas
        bias_hist[k] = gyro.bias

        ekf.predict(gyro_meas, cfg.dt)

        if k % 5 == 0:
            q_meas = star_tracker.measure(q)
            ekf.update_star_tracker(q_meas)

            residual_hist[k] = ekf.last_residual_norm
            res_fault = fdir.check_residual(ekf.last_residual_norm)
            residual_fault_hist[k] = int(res_fault)

            if res_fault:
                print(
                    "Residual fault detected at t = {:.2f} s, residual norm = {:.4f}".format(
                        t, ekf.last_residual_norm
                    )
                )

        q_est_hist[k] = ekf.q
        attitude_error_deg[k] = quat_error_angle_deg(q, ekf.q)

        bias_norm_hist[k] = np.linalg.norm(ekf.bias)

        if t > startup_guard_time:
            bias_fault = fdir.check_bias(ekf.bias)
        else:
            bias_fault = False

        bias_fault_hist[k] = int(bias_fault)

        if fault_injected and bias_fault and detection_time is None:
            detection_time = t
            print(
                "Bias fault detected at t = {:.2f} s, bias norm = {:.4f}".format(
                    t, bias_norm_hist[k]
                )
            )

        if fault_injected and bias_fault:
            fdir.trigger_recovery()

        if fdir.recovery_active and not recovery_triggered_once:
            print("Recovery action activated.")
            ekf.recovery_active = True
            ekf.P *= ekf.recovery_scale_factor
            recovery_triggered_once = True

        recovery_hist[k] = int(fdir.recovery_active)

        q, w = propagate_truth(q, w, cfg.inertia, torque, cfg.dt)

    detection_delay = None
    if detection_time is not None:
        detection_delay = detection_time - fault_time
        print("Detection delay: {:.3f} s".format(detection_delay))
    else:
        print("No bias fault detected during simulation.")

    if return_error:
        return attitude_error_deg

    plot_angular_rates(time, w_hist, gyro_hist, bias_hist)
    plot_attitude_error(time, attitude_error_deg, fault_time, detection_time)
    plot_bias_detection(
        time,
        bias_norm_hist,
        fdir.bias_threshold,
        fault_time,
        detection_time,
    )
    plot_recovery(time, recovery_hist, fault_time, detection_time)

    return None


if __name__ == "__main__":
    main()