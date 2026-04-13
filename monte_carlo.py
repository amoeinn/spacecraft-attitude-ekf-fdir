import numpy as np
import matplotlib.pyplot as plt
from main import main


def run_monte_carlo(n_runs=50):
    final_errors = []

    for i in range(n_runs):
        print(f"Run {i+1}/{n_runs}")

        error = main(return_error=True)
        final_errors.append(error[-1])

    return np.array(final_errors)


if __name__ == "__main__":
    errors = run_monte_carlo(50)

    plt.hist(errors, bins=15)
    plt.xlabel("Final Attitude Error [deg]")
    plt.ylabel("Count")
    plt.title("Monte Carlo Final Error Distribution")
    plt.grid()
    plt.show()

    print("Mean error:", np.mean(errors))
    print("Std error:", np.std(errors))