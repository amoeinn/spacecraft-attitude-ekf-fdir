# Spacecraft Attitude Estimation with Error-State EKF and FDIR

A Python project for spacecraft attitude estimation using quaternion dynamics, IMU and star tracker sensor fusion, an error-state Extended Kalman Filter, Monte Carlo validation, and basic fault detection, isolation, and recovery logic.

## Overview

This project simulates a spacecraft rotational dynamics problem and estimates vehicle attitude using an error-state EKF. The estimator fuses:
- gyroscope measurements with noise, bias, and random walk
- star tracker attitude measurements

The project also includes:
- Monte Carlo robustness analysis
- injected gyro fault scenarios
- bias-based fault detection
- detection gating to avoid startup false positives
- adaptive recovery mode after fault detection

## Features

### Estimation
- Quaternion-based spacecraft attitude propagation
- Error-state EKF for attitude and gyro bias estimation
- IMU plus star tracker sensor fusion

### Validation
- Time-domain estimation error analysis
- Monte Carlo final-error distribution

### FDIR
- Gyro bias fault injection
- Residual monitoring
- Bias-based fault detection
- Startup transient guard window
- Recovery mode with covariance inflation
- Adaptive bias update during recovery

## Repository Structure

- `main.py`  
  Main simulation loop, fault injection, detection timing, and plotting

- `monte_carlo.py`  
  Repeated simulation runs for robustness analysis

- `config.py`  
  Simulation constants and initial conditions

- `dynamics.py`  
  Spacecraft rotational dynamics propagation

- `sensors.py`  
  Gyro and star tracker sensor models

- `ekf.py`  
  Error-state attitude EKF

- `fdir.py`  
  Fault detection and recovery logic

- `plots.py`  
  Plotting utilities

- `utils.py`  
  Quaternion math and helper functions

## Example Workflow

1. Propagate truth attitude dynamics
2. Generate noisy gyro and star tracker measurements
3. Run EKF prediction and update
4. Inject a gyro bias fault at a chosen time
5. Detect the fault using bias monitoring
6. Trigger recovery mode
7. Evaluate estimation performance

## Typical Results

### Nominal case
- attitude estimation error typically remains around 0.1 to 0.2 deg

### Faulted case
- a gyro bias jump causes a transient attitude error spike
- bias-based detection triggers after a short delay
- recovery mode stabilizes the estimator
- attitude error settles back to a bounded level

## How to Run

### Main simulation
```bash
python main.py
