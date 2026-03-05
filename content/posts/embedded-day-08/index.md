---
title: "Day 8 — Kalman Filter: Theory and Implementation"
date: 2026-03-06
description: "Complete Kalman filter derivation — predict and update steps, Q and R matrices, complementary filter comparison, and EKF introduction for IMU sensor fusion"
categories: ["Autonomous Driving"]
tags: ["Kalman Filter", "Sensor Fusion", "EKF", "Complementary Filter", "State Estimation"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 8
draft: false
---

{{< katex >}}

## What You'll Learn

- Why the Kalman Filter is the most important algorithm in autonomous driving
- The Predict-Update cycle with full equations
- What Q and R matrices mean physically
- How to implement a 1D Kalman Filter from scratch in Python
- Complementary Filter as a simpler alternative
- Introduction to the Extended Kalman Filter (EKF) for nonlinear systems

---

## 1. Why Kalman Filter?

Every sensor lies. Accelerometers are noisy and drift. Gyroscopes accumulate error over time. GPS jumps around. No single sensor gives you the truth.

The **Kalman Filter** combines multiple noisy measurements with a mathematical model of how the system evolves. It gives the **optimal estimate** (minimum variance) when:
1. The system is linear
2. Noise is Gaussian (normal distribution)

```
         ┌─────────────────────────────────────┐
         │         Kalman Filter                │
         │                                      │
Model ──►│  Predict: "Where should I be?"       │──► Best
(physics)│                                      │    Estimate
         │  Update: "What do sensors say?"      │
Sensors──►│                                      │
(noisy)  │  Blend based on confidence levels     │
         └─────────────────────────────────────┘
```

---

## 2. The Predict Step

### State Prediction

Given the previous state estimate \(\hat{x}_{k-1|k-1}\), predict the next state:

$$\hat{x}_{k|k-1} = F \hat{x}_{k-1|k-1} + B u_k$$

- \(\hat{x}_{k|k-1}\): Predicted state (before seeing measurement)
- \(F\): State transition matrix ("how the system evolves")
- \(B\): Control input matrix
- \(u_k\): Control input (e.g., motor command)

**Example**: For a 1D position/velocity system:

$$\hat{x}_k = \begin{bmatrix} \text{position} \\ \text{velocity} \end{bmatrix}$$

$$F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$

This says: new position = old position + velocity × dt, velocity stays the same (constant velocity model).

### Covariance Prediction

The uncertainty also propagates:

$$P_{k|k-1} = F P_{k-1|k-1} F^T + Q$$

- \(P_{k|k-1}\): Predicted covariance (uncertainty after prediction)
- \(Q\): **Process noise covariance** — how much we distrust our model

**What Q means physically**: If your model is "constant velocity" but the car can accelerate, \(Q\) captures that unmodeled acceleration. Larger \(Q\) = "I don't trust my model much" = filter responds faster to measurements.

---

## 3. The Update Step

When a new measurement \(z_k\) arrives:

### Kalman Gain

$$K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}$$

- \(K_k\): Kalman gain (0 to 1 for scalar case)
- \(H\): Measurement matrix ("what we can observe")
- \(R\): **Measurement noise covariance** — how much we distrust the sensor

### State Update

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})$$

The term \((z_k - H \hat{x}_{k|k-1})\) is the **innovation** (measurement residual) — the difference between what we measured and what we predicted.

### Covariance Update

$$P_{k|k} = (I - K_k H) P_{k|k-1}$$

After incorporating the measurement, our uncertainty decreases.

### The Kalman Gain Intuition

$$K = \frac{\text{Prediction uncertainty}}{\text{Prediction uncertainty} + \text{Measurement uncertainty}}$$

| Scenario | K value | Behavior |
|----------|---------|----------|
| Sensor very accurate (R small) | K → 1 | Trust measurement |
| Sensor very noisy (R large) | K → 0 | Trust prediction |
| Model very uncertain (P large) | K → 1 | Trust measurement |
| Model very confident (P small) | K → 0 | Trust prediction |

---

## 4. Q and R Tuning — Physical Meaning

This is the **art** of Kalman filtering. Q and R are the knobs you tune.

### Process Noise Q

$$Q \uparrow \implies \text{"I don't trust my model"} \implies \text{Filter follows measurements more closely}$$

```
Q small (trust model):     Q large (trust measurements):
True ──────────────        True ──────────────
Est  ──╱────╲──────        Est  ──╱╲──╱╲──╱╲─ (tracks noise)
Meas · · · · · · ·        Meas · · · · · · ·
       (smooth but slow)          (responsive but noisy)
```

### Measurement Noise R

$$R \uparrow \implies \text{"I don't trust my sensor"} \implies \text{Filter smooths measurements more}$$

```
R small (trust sensor):    R large (trust model):
True ──────────────        True ──────────────
Est  ──╱╲──╱╲──╱╲─        Est  ──────╱────╲── (smooth)
Meas · · · · · · ·        Meas · · · · · · ·
       (follows noise)            (ignores noise)
```

### Practical Guidelines

| Situation | Q | R |
|-----------|---|---|
| Good model, noisy sensor | Small | Large |
| Poor model, accurate sensor | Large | Small |
| IMU gyro integration | Medium | N/A (prediction only) |
| GPS position | N/A | Small (but varies) |
| Wheel odometry | Medium | Medium |

---

## 5. Complementary Filter

Before building a full Kalman filter, let's try a simpler approach that works surprisingly well for IMU fusion.

### The Idea

- Gyroscope: Good for **short-term** (low noise), bad for **long-term** (drift)
- Accelerometer: Good for **long-term** (no drift), bad for **short-term** (noisy, affected by vibration)

The complementary filter blends them with a tunable parameter \(\alpha\):

$$\theta_k = \alpha \cdot (\theta_{k-1} + \dot{\theta}_{gyro} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{accel}$$

- \(\alpha\) close to 1: Trust gyro more (smooth but may drift)
- \(\alpha\) close to 0: Trust accelerometer more (noisy but no drift)
- Typical: \(\alpha = 0.98\) (98% gyro, 2% accelerometer)

This is actually a **high-pass filter** on gyro + **low-pass filter** on accelerometer:

$$\theta = \text{HPF}(\text{gyro}) + \text{LPF}(\text{accel})$$

### Why It Works

The cutoff frequency:

$$f_c = \frac{1 - \alpha}{2\pi \cdot \alpha \cdot \Delta t}$$

With \(\alpha = 0.98\), \(\Delta t = 0.01\)s:

$$f_c = \frac{0.02}{2\pi \times 0.98 \times 0.01} \approx 0.32 \text{ Hz}$$

Gyro drift is below 0.32 Hz → filtered out. Accelerometer noise is above 0.32 Hz → filtered out.

---

## 6. Extended Kalman Filter (EKF) — Preview

When the system is **nonlinear** (which is the case for 3D rotation), the standard Kalman filter doesn't apply directly.

The **EKF** linearizes around the current estimate using Jacobians:

$$F_k = \frac{\partial f}{\partial x}\bigg|_{\hat{x}_{k-1}} \qquad H_k = \frac{\partial h}{\partial x}\bigg|_{\hat{x}_{k|k-1}}$$

Then applies the standard Kalman equations with these linearized matrices.

**For IMU sensor fusion**:
- State: quaternion orientation + gyro biases
- Prediction: integrate gyro (nonlinear quaternion kinematics)
- Update: compare predicted gravity direction with measured acceleration

This is what runs inside RTAB-Map (Day 12) and most autonomous vehicle localization systems.

---

## 7. Hands-On Lab

### Lab 1: 1D Kalman Filter — Position Estimation

```python
#!/usr/bin/env python3
"""1D Kalman Filter: Estimate position from noisy measurements."""

import numpy as np
import matplotlib.pyplot as plt

# --- System setup ---
dt = 0.1  # Time step (100ms)
num_steps = 100

# True trajectory: constant velocity
true_velocity = 2.0  # m/s
true_positions = np.arange(num_steps) * dt * true_velocity

# Noisy measurements (GPS-like)
measurement_noise_std = 5.0  # meters
measurements = true_positions + np.random.randn(num_steps) * measurement_noise_std

# --- Kalman Filter ---
# State: [position, velocity]
x = np.array([0.0, 0.0])  # Initial estimate
P = np.array([[100.0, 0.0],  # Initial uncertainty (large = unknown)
              [0.0, 100.0]])

# State transition
F = np.array([[1.0, dt],
              [0.0, 1.0]])

# Measurement matrix (we only observe position)
H = np.array([[1.0, 0.0]])

# Process noise (how much acceleration can happen)
q = 0.1  # acceleration variance
Q = np.array([[dt**4/4, dt**3/2],
              [dt**3/2, dt**2]]) * q

# Measurement noise
R = np.array([[measurement_noise_std**2]])

# Storage for plotting
est_positions = []
est_velocities = []
kalman_gains = []

for k in range(num_steps):
    # --- PREDICT ---
    x = F @ x
    P = F @ P @ F.T + Q

    # --- UPDATE ---
    z = np.array([measurements[k]])
    y = z - H @ x                          # Innovation
    S = H @ P @ H.T + R                    # Innovation covariance
    K = P @ H.T @ np.linalg.inv(S)         # Kalman gain
    x = x + (K @ y).flatten()
    P = (np.eye(2) - K @ H) @ P

    est_positions.append(x[0])
    est_velocities.append(x[1])
    kalman_gains.append(K[0, 0])

# --- Plot results ---
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
time = np.arange(num_steps) * dt

axes[0].plot(time, true_positions, 'g-', linewidth=2, label='True Position')
axes[0].scatter(time, measurements, c='red', s=10, alpha=0.5, label='Measurements (noisy)')
axes[0].plot(time, est_positions, 'b-', linewidth=2, label='Kalman Estimate')
axes[0].set_ylabel('Position (m)')
axes[0].legend()
axes[0].set_title('1D Kalman Filter — Position Estimation')
axes[0].grid(True, alpha=0.3)

axes[1].plot(time, [true_velocity] * num_steps, 'g-', linewidth=2, label='True Velocity')
axes[1].plot(time, est_velocities, 'b-', linewidth=2, label='Estimated Velocity')
axes[1].set_ylabel('Velocity (m/s)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].plot(time, kalman_gains, 'purple', linewidth=2)
axes[2].set_ylabel('Kalman Gain')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('Kalman Gain (converges as filter becomes confident)')
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('kalman_1d.png', dpi=150)
plt.show()
```

### Lab 2: Complementary Filter for IMU

```python
#!/usr/bin/env python3
"""Complementary filter: Fuse accelerometer + gyroscope for Roll/Pitch."""

import numpy as np
import matplotlib.pyplot as plt

# Simulate IMU data
dt = 0.01  # 100 Hz
t = np.arange(0, 10, dt)

# True angle: sinusoidal motion
true_angle = 30 * np.sin(0.5 * t)  # degrees

# Gyroscope: derivative of true angle + noise + bias drift
true_rate = 30 * 0.5 * np.cos(0.5 * t)  # °/s
gyro_noise = 0.5 * np.random.randn(len(t))
gyro_bias_drift = 0.02 * np.cumsum(np.random.randn(len(t))) * dt
gyro = true_rate + gyro_noise + gyro_bias_drift

# Accelerometer: true angle + high-frequency noise
accel_noise = 3.0 * np.random.randn(len(t))
accel_angle = true_angle + accel_noise

# --- Complementary Filter ---
alpha = 0.98
comp_angle = np.zeros(len(t))
comp_angle[0] = accel_angle[0]

for k in range(1, len(t)):
    # High-pass gyro + low-pass accel
    comp_angle[k] = alpha * (comp_angle[k-1] + gyro[k] * dt) + (1 - alpha) * accel_angle[k]

# --- Gyro-only integration (for comparison) ---
gyro_only = np.cumsum(gyro * dt)

# --- Plot ---
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axes[0].plot(t, true_angle, 'g-', linewidth=2, label='True Angle')
axes[0].plot(t, accel_angle, 'r.', markersize=1, alpha=0.3, label='Accel Only (noisy)')
axes[0].plot(t, gyro_only, 'm-', linewidth=1, alpha=0.7, label='Gyro Only (drifts)')
axes[0].set_ylabel('Angle (deg)')
axes[0].legend()
axes[0].set_title('Raw Sensor Estimates')
axes[0].grid(True, alpha=0.3)

axes[1].plot(t, true_angle, 'g-', linewidth=2, label='True Angle')
axes[1].plot(t, comp_angle, 'b-', linewidth=2, label=f'Complementary (alpha={alpha})')
axes[1].set_ylabel('Angle (deg)')
axes[1].legend()
axes[1].set_title('Complementary Filter Result')
axes[1].grid(True, alpha=0.3)

axes[2].plot(t, true_angle - accel_angle, 'r-', alpha=0.5, label='Accel Error')
axes[2].plot(t, true_angle - gyro_only, 'm-', alpha=0.5, label='Gyro Error (grows!)')
axes[2].plot(t, true_angle - comp_angle, 'b-', linewidth=2, label='Comp. Filter Error')
axes[2].set_ylabel('Error (deg)')
axes[2].set_xlabel('Time (s)')
axes[2].legend()
axes[2].set_title('Error Comparison')
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('complementary_filter.png', dpi=150)
plt.show()
```

### Lab 3: Kalman Filter for IMU Fusion

```python
#!/usr/bin/env python3
"""Kalman filter for IMU sensor fusion — compare with complementary filter."""

import numpy as np
import matplotlib.pyplot as plt

dt = 0.01
t = np.arange(0, 10, dt)
true_angle = 30 * np.sin(0.5 * t)
true_rate = 30 * 0.5 * np.cos(0.5 * t)

# Simulated sensors
gyro = true_rate + 0.5 * np.random.randn(len(t)) + 0.02 * np.cumsum(np.random.randn(len(t))) * dt
accel_angle = true_angle + 3.0 * np.random.randn(len(t))

# --- Kalman Filter ---
# State: [angle, gyro_bias]
x = np.array([0.0, 0.0])
P = np.array([[1.0, 0.0],
              [0.0, 1.0]])

F = np.array([[1.0, -dt],
              [0.0, 1.0]])

B = np.array([[dt],
              [0.0]])

H = np.array([[1.0, 0.0]])

Q = np.array([[0.001, 0.0],    # angle process noise
              [0.0, 0.003]])   # bias process noise

R = np.array([[9.0]])  # accel noise variance (3.0^2)

kf_angle = np.zeros(len(t))
kf_bias = np.zeros(len(t))

for k in range(len(t)):
    # Predict
    u = np.array([[gyro[k]]])
    x = F @ x + (B @ u).flatten()
    P = F @ P @ F.T + Q

    # Update with accelerometer
    z = np.array([accel_angle[k]])
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + (K @ y).flatten()
    P = (np.eye(2) - K @ H) @ P

    kf_angle[k] = x[0]
    kf_bias[k] = x[1]

# Complementary filter for comparison
alpha = 0.98
comp_angle = np.zeros(len(t))
comp_angle[0] = accel_angle[0]
for k in range(1, len(t)):
    comp_angle[k] = alpha * (comp_angle[k-1] + gyro[k] * dt) + (1 - alpha) * accel_angle[k]

# Plot comparison
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t, true_angle, 'g-', linewidth=2, label='True')
axes[0].plot(t, comp_angle, 'orange', linewidth=1.5, alpha=0.7, label='Complementary')
axes[0].plot(t, kf_angle, 'b-', linewidth=2, label='Kalman Filter')
axes[0].set_ylabel('Angle (deg)')
axes[0].legend()
axes[0].set_title('Complementary Filter vs Kalman Filter')
axes[0].grid(True, alpha=0.3)

axes[1].plot(t, true_angle - comp_angle, 'orange', linewidth=1, alpha=0.7, label='Comp. Error')
axes[1].plot(t, true_angle - kf_angle, 'b-', linewidth=1.5, label='KF Error')
axes[1].set_ylabel('Error (deg)')
axes[1].set_xlabel('Time (s)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

comp_rmse = np.sqrt(np.mean((true_angle - comp_angle)**2))
kf_rmse = np.sqrt(np.mean((true_angle - kf_angle)**2))
axes[1].set_title(f'Error Comparison — Comp RMSE: {comp_rmse:.2f}  |  KF RMSE: {kf_rmse:.2f}')

plt.tight_layout()
plt.savefig('kalman_vs_complementary.png', dpi=150)
plt.show()
```

### Lab 4: Q and R Parameter Experiment

```python
#!/usr/bin/env python3
"""Experiment: How Q and R affect Kalman filter behavior."""

import numpy as np
import matplotlib.pyplot as plt

dt = 0.01
t = np.arange(0, 10, dt)
true_angle = 30 * np.sin(0.5 * t)
true_rate = 30 * 0.5 * np.cos(0.5 * t)
gyro = true_rate + 0.5 * np.random.randn(len(t))
accel_angle = true_angle + 3.0 * np.random.randn(len(t))

def run_kalman(q_scale, r_scale, label):
    x = np.array([0.0, 0.0])
    P = np.eye(2)
    F = np.array([[1, -dt], [0, 1]])
    B = np.array([[dt], [0]])
    H = np.array([[1, 0]])
    Q = np.array([[0.001, 0], [0, 0.003]]) * q_scale
    R = np.array([[9.0]]) * r_scale

    angles = np.zeros(len(t))
    for k in range(len(t)):
        x = F @ x + (B @ np.array([[gyro[k]]])).flatten()
        P = F @ P @ F.T + Q
        z = np.array([accel_angle[k]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + (K @ (z - H @ x)).flatten()
        P = (np.eye(2) - K @ H) @ P
        angles[k] = x[0]

    rmse = np.sqrt(np.mean((true_angle - angles)**2))
    return angles, rmse, label

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

configs = [
    (1, 1, "Baseline (Q=1x, R=1x)"),
    (10, 1, "Q x10 (distrust model)"),
    (1, 10, "R x10 (distrust sensor)"),
    (0.1, 0.1, "Q x0.1, R x0.1 (trust both)"),
]

for ax, (q_s, r_s, label) in zip(axes.flatten(), configs):
    angles, rmse, lbl = run_kalman(q_s, r_s, label)
    ax.plot(t, true_angle, 'g-', linewidth=1, alpha=0.5, label='True')
    ax.plot(t, accel_angle, 'r.', markersize=0.5, alpha=0.2)
    ax.plot(t, angles, 'b-', linewidth=2, label=f'KF (RMSE={rmse:.2f})')
    ax.set_title(lbl)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-50, 50)

plt.suptitle('Effect of Q and R on Kalman Filter Behavior', fontsize=14)
plt.tight_layout()
plt.savefig('kalman_qr_experiment.png', dpi=150)
plt.show()
```

---

## 8. Review

### Key Takeaways

1. **Kalman Filter** = Predict (model) + Update (measurement) cycle
2. **Q matrix**: Process noise — larger = less trust in model = more responsive
3. **R matrix**: Measurement noise — larger = less trust in sensor = smoother
4. **Kalman Gain K**: Automatically balances model vs sensor trust
5. **Complementary Filter**: Simple but effective for IMU — 98/2 gyro/accel split
6. **EKF**: Extends to nonlinear systems using Jacobian linearization

### Quiz

**Q**: You increase Q by 10× while keeping R the same. What happens?
**A**: The filter becomes more responsive to measurements (larger K). The estimate tracks measurements more closely but also picks up more noise. The filter "distrusts" the model and "listens" to the sensor more.

**Q**: You increase R by 10× while keeping Q the same. What happens?
**A**: The filter becomes smoother. It "distrusts" the sensor and relies more on the model prediction. Response to sudden changes becomes slower.

### Looking Ahead

Tomorrow (Day 9), we build a complete **PID controller** that uses the Hall sensor RPM from Day 6 as feedback and the motor PWM as output. We'll tune P, I, and D gains and see the effect of integral windup — all building toward autonomous speed control.
