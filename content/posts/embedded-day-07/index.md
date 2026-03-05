---
title: "Day 7 — IMU Sensors and MEMS Principles"
date: 2026-03-05T07:00:00
description: "MEMS accelerometer and gyroscope working principles, sensor noise models, Allan Variance analysis, and Euler angles vs Quaternions"
categories: ["Autonomous Driving"]
tags: ["IMU", "MEMS", "Accelerometer", "Gyroscope", "Allan Variance", "Quaternion"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 7
draft: false
---

{{< katex >}}

## What You'll Learn

- How MEMS accelerometers measure acceleration using capacitance changes
- How MEMS gyroscopes use the Coriolis effect to measure rotation
- Sensor noise models and how to characterize IMU quality with Allan Variance
- Euler angles vs Quaternions and the Gimbal Lock problem
- Reading raw IMU data and computing orientation from accelerometer alone

---

## 1. MEMS Accelerometer

### Working Principle

A MEMS (Micro-Electro-Mechanical System) accelerometer contains a tiny **proof mass** suspended by spring-like structures, etched from silicon:

```
  Fixed plate        Proof mass        Fixed plate
  (electrode)        (movable)         (electrode)
  ┌────────┐    ┌──────────────┐    ┌────────┐
  │        │    │              │    │        │
  │   C1   │←d1→│   ████████   │←d2→│   C2   │
  │        │    │   ████████   │    │        │
  │        │    │   ████████   │    │        │
  └────────┘    └──────┬───────┘    └────────┘
                       │
                   Spring (k)
                       │
                   ┌───┴───┐
                   │ Frame │ (fixed to chip package)
                   └───────┘
```

When the chip accelerates, the proof mass **lags behind** due to inertia (Newton's second law):

$$F = ma \implies x = \frac{ma}{k}$$

This displacement changes the gap between the proof mass and the fixed electrodes. Since capacitance depends on gap distance:

$$C = \frac{\varepsilon A}{d}$$

The differential capacitance change is:

$$\Delta C = C_1 - C_2 = \varepsilon A \left(\frac{1}{d_0 - x} - \frac{1}{d_0 + x}\right) \approx \frac{2\varepsilon A x}{d_0^2}$$

For small displacements (\(x \ll d_0\)):

$$\Delta C \propto x \propto a$$

The capacitance change is **proportional to acceleration**. An ASIC on the chip measures this tiny capacitance change (femtofarads!) and converts it to a digital value.

### Sensitivity and Range

| Parameter | Typical Value |
|-----------|--------------|
| Range | ±2g, ±4g, ±8g, ±16g (selectable) |
| Sensitivity (±2g) | 16384 LSB/g |
| Noise density | 100-300 µg/√Hz |
| Bandwidth | Up to 1 kHz |
| Size | 2mm × 2mm × 1mm |

The **raw register value** relates to acceleration:

$$a_g = \frac{\text{raw value}}{\text{sensitivity}} = \frac{\text{raw}}{16384} \text{ (for ±2g range)}$$

---

## 2. MEMS Gyroscope

### Coriolis Effect

When a mass is moving in a rotating frame, it experiences a force perpendicular to both its velocity and the rotation axis:

$$\vec{F}_{Coriolis} = -2m(\vec{\omega} \times \vec{v})$$

**Intuitive analogy**: Imagine walking outward on a spinning merry-go-round. You feel a sideways push — that's the Coriolis force.

### How a MEMS Gyro Works

The MEMS gyro has a proof mass that **vibrates back and forth** at a known frequency (driven by electrostatic comb drives):

```
  Side View:

     Vibration direction (x)
     ←─────────→
  ┌──────────────────┐
  │  ┌────┐          │
  │  │Proof│ ←→ vibrates at resonant frequency
  │  │Mass │          │
  │  └──┬─┘          │
  │     │             │
  │   Springs         │
  │     │             │
  │  ┌──┴──┐          │
  │  │Sense│← detects Coriolis displacement (y)
  │  │Plate│           │
  └──────────────────┘

  When chip rotates around z-axis (ω_z):
  Vibrating mass (velocity in x) feels Coriolis force in y
  F_y = 2m × ω_z × v_x
```

1. The proof mass vibrates in the **x-direction** at its resonant frequency (~10-30 kHz)
2. When the chip rotates around **z-axis** with angular velocity \(\omega_z\)
3. The Coriolis force deflects the mass in the **y-direction**
4. This y-displacement is measured by capacitive sensing (same as accelerometer)
5. The measured Coriolis force gives us \(\omega_z\)

$$F_y = 2m \cdot \omega_z \cdot v_x \implies \omega_z = \frac{F_y}{2m \cdot v_x}$$

A 3-axis gyroscope has three such structures oriented along different axes.

### Gyroscope Parameters

| Parameter | Typical Value |
|-----------|--------------|
| Range | ±250, ±500, ±1000, ±2000 °/s |
| Sensitivity (±250°/s) | 131 LSB/(°/s) |
| Noise density | 0.005-0.01 °/s/√Hz |
| Bias stability | 1-10 °/hr (consumer), 0.01 °/hr (tactical) |

$$\omega_{deg/s} = \frac{\text{raw value}}{\text{sensitivity}} = \frac{\text{raw}}{131} \text{ (for ±250°/s range)}$$

---

## 3. Sensor Noise Models

Real IMU sensors are far from perfect. Understanding noise is critical for designing filters (Day 8).

### Types of Noise

**White noise** (high-frequency jitter):
- Random, zero-mean fluctuations at each sample
- Characterized by **noise density** (µg/√Hz for accel, °/s/√Hz for gyro)
- Averaging N samples reduces by \(\sqrt{N}\)

**Bias** (constant offset):
- Sensor reads non-zero when stationary
- Can be measured and subtracted (calibration)
- Example: accelerometer reads 0.02g when it should read 0.00g

**Bias instability** (slowly drifting offset):
- The bias changes slowly over time (minutes to hours)
- Caused by temperature changes, mechanical stress
- Cannot be fixed by one-time calibration
- Measured in °/hr for gyro — lower is better

**Random walk** (integrated noise):
- For gyroscopes: integrating angular rate noise gives **angle random walk**
- Orientation error grows as \(\sqrt{t}\)
- ARW units: °/√hr

```
Signal over time:

True value: ────────────────────────────────
                    ╱╲    ╱╲
White noise: ──╱╲──╱──╲──╱──╲──╱╲──────── (fast jitter)
                  ╲╱    ╲╱    ╲╱

Bias:        ─────────────────────────── (constant offset)
             ↑ 0.02g above true value

Bias drift:  ────╱─────────╲────╱────── (slow wandering)
             changes over minutes/hours
```

---

## 4. Allan Variance

### What Is It?

Allan Variance is a method to **characterize different noise types** in a time-domain signal. It was originally developed for atomic clocks and is now the standard tool for IMU characterization.

### How to Compute

1. Collect a long stationary dataset (30-60 minutes at constant rate)
2. Divide into clusters of averaging time \(\tau\)
3. Compute the variance of the averaged clusters

$$\sigma^2(\tau) = \frac{1}{2(N-1)} \sum_{i=1}^{N-1} (\bar{y}_{i+1} - \bar{y}_i)^2$$

Where \(\bar{y}_i\) is the average of cluster \(i\) over time \(\tau\).

### Reading the Log-Log Plot

```
log(σ(τ))
    │
    │╲                          Angle Random Walk
    │  ╲   slope = -1/2         (white noise of gyro)
    │    ╲
    │      ╲
    │        ──────             Bias Instability
    │              ──────       (minimum of the curve)
    │                    ╱
    │                  ╱        Rate Random Walk
    │                ╱          slope = +1/2
    │              ╱
    └───────────────────────── log(τ)
```

| Region | Slope | Noise Type | Read Value At |
|--------|-------|-----------|---------------|
| Left (short τ) | -1/2 | Angle Random Walk | τ = 1 sec |
| Minimum | 0 | Bias Instability | Bottom of curve |
| Right (long τ) | +1/2 | Rate Random Walk | slope region |

**Consumer IMU** (MPU6050): ARW ≈ 0.3°/√hr, Bias instability ≈ 10°/hr
**Tactical IMU** (ADIS16490): ARW ≈ 0.006°/√hr, Bias instability ≈ 0.8°/hr

---

## 5. Euler Angles vs Quaternions

### Euler Angles

Euler angles describe orientation using three successive rotations:

- **Roll** (\(\phi\)): Rotation around x-axis (tilting left/right)
- **Pitch** (\(\theta\)): Rotation around y-axis (tilting forward/backward)
- **Yaw** (\(\psi\)): Rotation around z-axis (turning left/right)

For a car:
- Roll = leaning into a turn
- Pitch = going uphill/downhill
- Yaw = steering direction

### Gimbal Lock Problem

When pitch approaches ±90°, roll and yaw become indistinguishable — you lose one degree of freedom:

```
Normal orientation:          Gimbal Lock (pitch = 90°):
  Yaw (z)                     Yaw and Roll aligned!
    │                           │
    │   Pitch (y)               │   ← both rotate
    │  ╱                        │      around same axis
    │╱                          │
    └──── Roll (x)              └──── (lost DOF)
```

Mathematically, the rotation matrix becomes singular. The Jacobian loses rank.

### Quaternions — The Solution

A quaternion represents rotation as a 4-component number:

$$q = w + xi + yj + zk$$

Where \(i^2 = j^2 = k^2 = ijk = -1\).

A rotation of angle \(\theta\) around unit axis \(\hat{n} = (n_x, n_y, n_z)\):

$$q = \cos\frac{\theta}{2} + \sin\frac{\theta}{2}(n_x i + n_y j + n_z k)$$

**Advantages**:
- No gimbal lock (singularity-free)
- Smooth interpolation (SLERP)
- Compact (4 numbers vs 9 for rotation matrix)
- Composing rotations: \(q_{total} = q_2 \times q_1\) (quaternion multiplication)

**In practice**: Most robotics frameworks (ROS2, Eigen) use quaternions internally but can convert to Euler angles for human readability.

---

## 6. Hands-On Lab

### Lab 1: Read Raw IMU Data

```python
#!/usr/bin/env python3
"""Read raw accelerometer and gyroscope data from MPU6050 via I2C."""

import smbus2
import time
import struct

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B

bus = smbus2.SMBus(1)

# Wake up MPU6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

# Set accel range to ±2g (sensitivity: 16384 LSB/g)
bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)

# Set gyro range to ±250°/s (sensitivity: 131 LSB/°/s)
bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
time.sleep(0.01)

ACCEL_SCALE = 16384.0  # LSB/g
GYRO_SCALE = 131.0     # LSB/(°/s)

def read_imu():
    """Read all 6 axes (accel + gyro) in one burst."""
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)

    ax = struct.unpack('>h', bytes(data[0:2]))[0] / ACCEL_SCALE
    ay = struct.unpack('>h', bytes(data[2:4]))[0] / ACCEL_SCALE
    az = struct.unpack('>h', bytes(data[4:6]))[0] / ACCEL_SCALE
    # data[6:8] is temperature
    gx = struct.unpack('>h', bytes(data[8:10]))[0] / GYRO_SCALE
    gy = struct.unpack('>h', bytes(data[10:12]))[0] / GYRO_SCALE
    gz = struct.unpack('>h', bytes(data[12:14]))[0] / GYRO_SCALE

    return ax, ay, az, gx, gy, gz

print(f"{'ax':>8s} {'ay':>8s} {'az':>8s} {'gx':>8s} {'gy':>8s} {'gz':>8s}")
print("-" * 56)

try:
    while True:
        ax, ay, az, gx, gy, gz = read_imu()
        print(f"{ax:>8.3f} {ay:>8.3f} {az:>8.3f} "
              f"{gx:>8.2f} {gy:>8.2f} {gz:>8.2f}")
        time.sleep(0.02)  # 50 Hz

except KeyboardInterrupt:
    bus.close()
    print("\nDone.")
```

### Lab 2: Stationary Bias Measurement

```python
#!/usr/bin/env python3
"""Measure IMU bias by averaging stationary data."""

import smbus2
import struct
import time
import numpy as np

MPU6050_ADDR = 0x68
bus = smbus2.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
time.sleep(0.1)

ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0
NUM_SAMPLES = 1000

def read_imu():
    data = bus.read_i2c_block_data(MPU6050_ADDR, 0x3B, 14)
    ax = struct.unpack('>h', bytes(data[0:2]))[0] / ACCEL_SCALE
    ay = struct.unpack('>h', bytes(data[2:4]))[0] / ACCEL_SCALE
    az = struct.unpack('>h', bytes(data[4:6]))[0] / ACCEL_SCALE
    gx = struct.unpack('>h', bytes(data[8:10]))[0] / GYRO_SCALE
    gy = struct.unpack('>h', bytes(data[10:12]))[0] / GYRO_SCALE
    gz = struct.unpack('>h', bytes(data[12:14]))[0] / GYRO_SCALE
    return ax, ay, az, gx, gy, gz

print(f"Collecting {NUM_SAMPLES} samples... KEEP IMU STATIONARY!")
samples = []
for i in range(NUM_SAMPLES):
    samples.append(read_imu())
    time.sleep(0.005)  # 200 Hz

samples = np.array(samples)

# Expected stationary values: ax=0, ay=0, az=1g, gx=0, gy=0, gz=0
print("\n--- Bias Measurement ---")
labels = ['ax(g)', 'ay(g)', 'az(g)', 'gx(°/s)', 'gy(°/s)', 'gz(°/s)']
expected = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

for i, (label, exp) in enumerate(zip(labels, expected)):
    mean = samples[:, i].mean()
    std = samples[:, i].std()
    bias = mean - exp
    print(f"  {label:>8s}: mean={mean:+.5f}  std={std:.5f}  bias={bias:+.5f}")

# Save bias for later calibration
bias_accel = samples[:, :3].mean(axis=0) - np.array([0, 0, 1.0])
bias_gyro = samples[:, 3:].mean(axis=0)

print(f"\n--- Calibration Offsets (subtract these) ---")
print(f"  Accel bias: [{bias_accel[0]:+.5f}, {bias_accel[1]:+.5f}, {bias_accel[2]:+.5f}] g")
print(f"  Gyro bias:  [{bias_gyro[0]:+.5f}, {bias_gyro[1]:+.5f}, {bias_gyro[2]:+.5f}] °/s")

bus.close()
```

### Lab 3: Allan Variance Plot

```python
#!/usr/bin/env python3
"""Compute and plot Allan Variance for IMU gyroscope."""

import numpy as np
import matplotlib.pyplot as plt

def allan_variance(data, dt):
    """Compute Allan Variance for a 1D time series."""
    N = len(data)
    max_clusters = N // 2
    taus = []
    avars = []

    for m in np.logspace(0, np.log10(max_clusters), num=50).astype(int):
        m = max(1, m)
        if m > max_clusters:
            break

        tau = m * dt
        n_clusters = N // m
        if n_clusters < 2:
            break

        # Average clusters
        truncated = data[:n_clusters * m]
        clusters = truncated.reshape(n_clusters, m).mean(axis=1)

        # Allan variance
        avar = 0.5 * np.mean(np.diff(clusters) ** 2)

        taus.append(tau)
        avars.append(avar)

    return np.array(taus), np.array(avars)

# Simulate or load gyro data
# For real data: collect 30+ minutes at 200 Hz while stationary
dt = 0.005  # 200 Hz
N = 200 * 60 * 30  # 30 minutes

# Simulated gyro noise (replace with real data)
np.random.seed(42)
white_noise = 0.01 * np.random.randn(N)  # °/s white noise
bias_drift = 0.001 * np.cumsum(np.random.randn(N)) / np.sqrt(N)
gyro_data = white_noise + bias_drift

taus, avars = allan_variance(gyro_data, dt)
adevs = np.sqrt(avars)

# Plot
fig, ax = plt.subplots(figsize=(10, 6))
ax.loglog(taus, adevs, 'b.-', linewidth=1.5)

# Reference slopes
tau_ref = np.array([taus[0], taus[-1]])
ax.loglog(tau_ref, adevs[0] * np.sqrt(taus[0] / tau_ref),
          'r--', alpha=0.5, label='Slope -1/2 (White Noise)')
ax.loglog(tau_ref, adevs[-1] * np.sqrt(tau_ref / taus[-1]),
          'g--', alpha=0.5, label='Slope +1/2 (Random Walk)')

ax.set_xlabel('Averaging Time tau (s)')
ax.set_ylabel('Allan Deviation (deg/s)')
ax.set_title('Allan Deviation Plot — Gyroscope Z-axis')
ax.legend()
ax.grid(True, which='both', alpha=0.3)

# Annotate bias instability (minimum)
min_idx = np.argmin(adevs)
ax.annotate(f'Bias Instability\n{adevs[min_idx]:.4f} deg/s\nat tau={taus[min_idx]:.1f}s',
            xy=(taus[min_idx], adevs[min_idx]),
            xytext=(taus[min_idx] * 5, adevs[min_idx] * 3),
            arrowprops=dict(arrowstyle='->', color='red'),
            fontsize=10, color='red')

plt.tight_layout()
plt.savefig('allan_variance.png', dpi=150)
plt.show()
```

### Lab 4: Accelerometer-Only Orientation

```python
#!/usr/bin/env python3
"""Compute Roll/Pitch from accelerometer only — and see its limitations."""

import numpy as np
import time
import smbus2
import struct

MPU6050_ADDR = 0x68
bus = smbus2.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
time.sleep(0.1)

def read_accel():
    data = bus.read_i2c_block_data(MPU6050_ADDR, 0x3B, 6)
    ax = struct.unpack('>h', bytes(data[0:2]))[0] / 16384.0
    ay = struct.unpack('>h', bytes(data[2:4]))[0] / 16384.0
    az = struct.unpack('>h', bytes(data[4:6]))[0] / 16384.0
    return ax, ay, az

print("Accelerometer-only Roll/Pitch estimation")
print("Move the IMU and observe the noise!")
print(f"{'Roll':>8s} {'Pitch':>8s}")
print("-" * 20)

try:
    while True:
        ax, ay, az = read_accel()

        # Roll and Pitch from gravity vector
        roll = np.degrees(np.arctan2(ay, az))
        pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))

        print(f"{roll:>8.1f} {pitch:>8.1f}")
        time.sleep(0.05)

except KeyboardInterrupt:
    bus.close()
    print("\nDone.")
    print("\nLimitations observed:")
    print("1. Very noisy when stationary (vibration sensitive)")
    print("2. Completely wrong during linear acceleration (car accelerating)")
    print("3. Cannot measure yaw (gravity is along z, not x-y)")
    print("\nSolution: Fuse with gyroscope → Kalman Filter (Day 8)")
```

---

## 7. Review

### Key Takeaways

1. **Accelerometer**: Measures force (including gravity) via differential capacitance
2. **Gyroscope**: Measures angular rate via Coriolis force on vibrating mass
3. **Noise types**: White noise (fast), bias (constant), bias instability (slow drift), random walk (integrated)
4. **Allan Variance**: Log-log plot identifies noise types — minimum = bias instability
5. **Euler angles** suffer from gimbal lock; **quaternions** are singularity-free
6. **Accelerometer alone** can estimate roll/pitch but fails with vibration and linear acceleration

### Looking Ahead

Tomorrow (Day 8), we'll build the **Kalman Filter** — the algorithm that fuses noisy accelerometer and gyroscope data into a clean, accurate orientation estimate. This is the mathematical heart of sensor fusion in every autonomous vehicle.
