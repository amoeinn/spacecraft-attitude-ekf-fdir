import matplotlib.pyplot as plt


def plot_angular_rates(time, w_hist, gyro_hist, bias_hist):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ["x", "y", "z"]

    for i in range(3):
        axs[i].plot(time, w_hist[:, i], label=f"true w_{labels[i]}")
        axs[i].plot(time, gyro_hist[:, i], label=f"gyro meas {labels[i]}", alpha=0.7)
        axs[i].plot(time, bias_hist[:, i], linestyle="--", label=f"bias {labels[i]}")
        axs[i].legend()
        axs[i].grid(True)

    axs[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def plot_attitude_error(time, error, fault_time=None, detection_time=None):
    plt.figure(figsize=(10, 4))
    plt.plot(time, error, label="attitude error")

    if fault_time is not None:
        plt.axvline(fault_time, linestyle="--", label="fault injection")

    if detection_time is not None:
        plt.axvline(detection_time, linestyle=":", label="detection")

    plt.xlabel("Time [s]")
    plt.ylabel("Error [deg]")
    plt.title("Attitude Estimation Error")
    plt.grid(True)
    plt.legend()
    plt.show()


def plot_bias_detection(time, bias_norm, threshold, fault_time, detection_time):
    plt.figure(figsize=(10, 4))
    plt.plot(time, bias_norm, label="bias norm")
    plt.axhline(threshold, linestyle="--", label="threshold")
    plt.axvline(fault_time, linestyle="--", label="fault injection")

    if detection_time is not None:
        plt.axvline(detection_time, linestyle=":", label="detection")

    plt.xlabel("Time [s]")
    plt.ylabel("Bias Norm")
    plt.title("Bias-Based Fault Detection")
    plt.grid(True)
    plt.legend()
    plt.show()


def plot_recovery(time, recovery_hist, fault_time, detection_time):
    plt.figure(figsize=(10, 3))
    plt.plot(time, recovery_hist, label="recovery active")
    plt.axvline(fault_time, linestyle="--", label="fault injection")

    if detection_time is not None:
        plt.axvline(detection_time, linestyle=":", label="detection")

    plt.xlabel("Time [s]")
    plt.ylabel("Recovery Flag")
    plt.title("Recovery Mode")
    plt.grid(True)
    plt.legend()
    plt.show()