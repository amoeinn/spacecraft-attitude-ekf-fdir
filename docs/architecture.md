# Architecture Overview

## System Flow

Truth Dynamics  
→ Sensor Models (Gyro + Star Tracker)  
→ EKF Prediction  
→ EKF Update  
→ Fault Detection  
→ Recovery Mode  

---

## State Representation

The EKF uses an error-state formulation:

- Quaternion for attitude
- 3D vector for gyro bias
- Covariance for attitude error and bias error

---

## Estimation

- Gyro used in prediction step
- Star tracker used in measurement update
- Bias estimated as part of the state

---

## Fault Detection

Two methods are implemented:

### Residual Monitoring
- Based on measurement innovation
- Not effective for gyro faults in this setup

### Bias-Based Detection
- Monitors norm of estimated gyro bias
- Triggers when threshold exceeded
- Includes startup guard to avoid false positives

---

## Recovery Strategy

After detection:

- Covariance is inflated
- Bias update is slowed (not frozen)
- Estimator remains stable while adapting

---

## Key Insight

Gyro faults affect the prediction model, not directly the measurement.

Therefore:
- Residual may remain small
- Bias estimation is a more reliable detection signal

---

## Future Improvements

- Multi-sensor fault isolation
- NEES/NIS consistency checks
- UKF implementation
- Full 6-DOF dynamics
