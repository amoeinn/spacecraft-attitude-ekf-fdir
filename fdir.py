import numpy as np


class FDIR:
    def __init__(self, residual_threshold=0.2, bias_threshold=0.04):
        self.residual_threshold = residual_threshold
        self.bias_threshold = bias_threshold

        self.residual_fault = False
        self.bias_fault = False
        self.recovery_active = False

    def check_residual(self, residual_norm: float) -> bool:
        self.residual_fault = residual_norm > self.residual_threshold
        return self.residual_fault

    def check_bias(self, bias: np.ndarray) -> bool:
        bias_norm = np.linalg.norm(bias)
        self.bias_fault = bias_norm > self.bias_threshold
        return self.bias_fault

    def trigger_recovery(self):
        if self.bias_fault:
            self.recovery_active = True
        return self.recovery_active