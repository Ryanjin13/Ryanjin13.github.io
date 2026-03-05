---
title: "Day 10 — 1D LiDAR and Depth Cameras: ToF and Structured Light"
date: 2026-03-15
description: "Time-of-Flight distance measurement, phase-shift method, triangulation, structured light depth cameras, and comparing depth sensing technologies"
categories: ["Autonomous Driving"]
tags: ["LiDAR", "Depth Camera", "ToF", "Structured Light", "Distance Sensing"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 10
draft: false
---

{{< katex >}}

## What You'll Learn

- Three methods of distance measurement: pulse ToF, phase-shift, and triangulation
- How structured light and ToF depth cameras extend 1D sensing to full depth maps
- Noise characteristics and failure modes of each technology
- Filtering distance data with moving average and Kalman filter (Day 8 callback)
- Practical Python code for reading LiDAR and depth camera data

---

## 1. 1D LiDAR — Pulse Time-of-Flight

### Principle

Send a laser pulse, measure the time until the reflection returns:

$$d = \frac{c \cdot t_{round\text{-}trip}}{2}$$

Where \(c = 3 \times 10^8\) m/s (speed of light) and the factor of 2 accounts for the round trip.

```
  Sensor                                  Target
  ┌──────┐        Laser pulse            ┌──────┐
  │ TX ──├─────────────────────────────►  │      │
  │      │                                │      │
  │ RX ◄─├─────────────────────────────── │      │
  └──────┘        Reflected pulse         └──────┘
           ◄──────── d ────────►

  t_roundtrip measured by high-speed timer
  d = c × t / 2
```

**Example**: Object at 1.5m:

$$t = \frac{2 \times 1.5}{3 \times 10^8} = 10 \text{ ns}$$

Measuring 10 nanoseconds accurately requires a timer resolution of ~1 ns, which is achievable with modern TDC (Time-to-Digital Converter) circuits.

---

## 2. Phase-Shift Method

### Principle

Instead of measuring time directly, modulate the laser at a known frequency and measure the **phase difference** between transmitted and received signals:

$$d = \frac{c \cdot \varphi}{4\pi f_{mod}}$$

Where:
- \(\varphi\) = measured phase shift (radians)
- \(f_{mod}\) = modulation frequency

### Maximum Unambiguous Range

Phase wraps around at \(2\pi\), so:

$$d_{max} = \frac{c}{2 f_{mod}}$$

| Modulation Frequency | Max Range |
|---------------------|-----------|
| 10 MHz | 15.0 m |
| 30 MHz | 5.0 m |
| 100 MHz | 1.5 m |

**Trade-off**: Higher frequency → better precision, lower range. Many sensors use multiple modulation frequencies to extend range while maintaining precision.

---

## 3. Triangulation (IR)

### Principle

An IR LED projects a spot onto the target. A position-sensitive detector (PSD) measures where the reflected spot lands:

```
  IR LED ──────── light ────────► Target
    ╲                              │
     ╲                             │ reflected
      ╲ baseline (b)               │
       ╲                           │
  PSD ◄──────── reflected spot ────┘
   │
   └── spot position on PSD ∝ 1/distance
```

$$d = \frac{f \cdot b}{\Delta x}$$

Where \(f\) = lens focal length, \(b\) = baseline distance, \(\Delta x\) = spot displacement on PSD.

**Examples**: Sharp GP2Y0A21 (10-80 cm), cheap and simple but low accuracy at long range.

---

## 4. Noise and Failure Modes

| Issue | Cause | Affected Sensors |
|-------|-------|-----------------|
| Black surfaces | Low reflectivity → weak return | All |
| Glass/mirrors | Specular reflection (bounces away) | All |
| Minimum distance | Overlap of TX/RX optics | ToF, Phase-shift |
| Temperature drift | Electronics change with temperature | All |
| Multipath | Signal bounces off multiple surfaces | Phase-shift, ToF |
| Ambient light | Sunlight overwhelms return signal | IR triangulation |

---

## 5. Depth Cameras

### Structured Light

A projector sends a known IR pattern (dots, stripes, or grid) onto the scene. An IR camera observes how the pattern deforms:

```
  IR Projector          Scene              IR Camera
  ┌──────────┐                            ┌──────────┐
  │ ● ● ● ● │──── known pattern ────►    │ captures │
  │ ● ● ● ● │                    ╱╲      │ deformed │
  │ ● ● ● ● │              object  ╲     │ pattern  │
  └──────────┘                       ╲    └──────────┘
       ◄────── baseline ──────►

  Pattern deformation → triangulation → depth per pixel
```

- **Near range**: Excellent precision (sub-mm at <1m)
- **Far range**: Pattern becomes too spread out, precision drops
- **Sunlight**: IR pattern washed out outdoors

**Examples**: Intel RealSense SR305 (structured light), RealSense D435 (active IR stereo)

### ToF Depth Camera (Phase-Shift Array)

Extend the 1D phase-shift concept to a **2D pixel array**. Each pixel independently measures the phase shift of modulated IR light:

$$\text{1D LiDAR ToF extended to 2D pixel array} = \text{Depth Camera}$$

- **Frame rate**: 30-60 FPS typically
- **Resolution**: 320×240 to 640×480 (lower than RGB cameras)
- **Range**: 0.2m to 10m
- **Sunlight**: Better than structured light (modulated signal filtering)

**Examples**: PMD/Infineon sensors, Microsoft Azure Kinect (ToF mode)

### Comparison

| Feature | Structured Light | ToF Depth Camera |
|---------|-----------------|-----------------|
| Near-range precision | Excellent | Good |
| Far range | Degrades quickly | Better |
| Outdoor (sunlight) | Poor | Moderate |
| Frame rate | 30 FPS typical | 30-60 FPS |
| Power consumption | Lower | Higher |
| Multipath artifacts | Less susceptible | More susceptible |
| Resolution | Higher (up to 1280×720) | Lower (320×240 typical) |

### Common Weaknesses

Both types struggle with:
- **Direct sunlight**: IR interference
- **Transparent objects**: IR passes through glass
- **Multipath interference**: IR bounces off multiple surfaces
- **Very dark surfaces**: Low reflectivity
- **Edges**: Depth discontinuities cause mixed pixels (flying pixels)

---

## 6. Hands-On Lab

### Lab 1: 1D LiDAR Real-Time Distance

```python
#!/usr/bin/env python3
"""Read distance from 1D LiDAR (TFmini/TF-Luna) via UART."""

import serial
import time
import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

print("1D LiDAR distance reading")
print(f"{'Time':>8s} {'Dist(cm)':>10s} {'Strength':>10s}")

try:
    while True:
        # TF-Luna protocol: 9 bytes per frame
        # Header: 0x59 0x59
        if ser.read(1) == b'\x59':
            if ser.read(1) == b'\x59':
                data = ser.read(7)
                if len(data) == 7:
                    dist_cm = struct.unpack('<H', data[0:2])[0]
                    strength = struct.unpack('<H', data[2:4])[0]
                    print(f"{'':>8s} {dist_cm:>10d} {strength:>10d}")
        time.sleep(0.01)

except KeyboardInterrupt:
    ser.close()
    print("\nDone.")
```

### Lab 2: Moving Average vs Kalman Filter

```python
#!/usr/bin/env python3
"""Compare moving average and Kalman filter for LiDAR smoothing."""

import numpy as np
import matplotlib.pyplot as plt

# Simulated noisy LiDAR data
np.random.seed(42)
t = np.arange(0, 10, 0.02)  # 50 Hz
true_dist = 150 + 30 * np.sin(0.5 * t)  # cm, slow movement
noise = 5 * np.random.randn(len(t))
measured = true_dist + noise

# Moving Average (window = 10)
window = 10
ma_filtered = np.convolve(measured, np.ones(window)/window, mode='same')

# 1D Kalman Filter (from Day 8)
kf_filtered = np.zeros(len(t))
x = measured[0]  # state estimate
P = 100.0        # uncertainty
Q = 0.1          # process noise
R = 25.0         # measurement noise (5^2)

for k in range(len(t)):
    # Predict (constant model)
    P_pred = P + Q

    # Update
    K = P_pred / (P_pred + R)
    x = x + K * (measured[k] - x)
    P = (1 - K) * P_pred

    kf_filtered[k] = x

# Plot comparison
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t, true_dist, 'g-', linewidth=2, label='True Distance')
axes[0].plot(t, measured, 'r.', markersize=1, alpha=0.3, label='Raw LiDAR')
axes[0].plot(t, ma_filtered, 'orange', linewidth=2, label=f'Moving Avg (n={window})')
axes[0].plot(t, kf_filtered, 'b-', linewidth=2, label='Kalman Filter')
axes[0].set_ylabel('Distance (cm)')
axes[0].legend()
axes[0].set_title('LiDAR Filtering Comparison')
axes[0].grid(True, alpha=0.3)

# Error comparison
axes[1].plot(t, np.abs(true_dist - measured), 'r-', alpha=0.3, label='Raw Error')
axes[1].plot(t, np.abs(true_dist - ma_filtered), 'orange', linewidth=1.5, label='MA Error')
axes[1].plot(t, np.abs(true_dist - kf_filtered), 'b-', linewidth=1.5, label='KF Error')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Absolute Error (cm)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

raw_rmse = np.sqrt(np.mean((true_dist - measured)**2))
ma_rmse = np.sqrt(np.mean((true_dist - ma_filtered)**2))
kf_rmse = np.sqrt(np.mean((true_dist - kf_filtered)**2))
axes[1].set_title(f'RMSE — Raw: {raw_rmse:.2f}  MA: {ma_rmse:.2f}  KF: {kf_rmse:.2f}')

plt.tight_layout()
plt.savefig('lidar_filtering.png', dpi=150)
plt.show()
```

### Lab 3: Depth Camera Stream

```python
#!/usr/bin/env python3
"""Read and visualize depth camera stream (Intel RealSense example)."""

import numpy as np
import cv2

# For Intel RealSense:
# pip install pyrealsense2
try:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Colorize depth for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Side by side display
            combined = np.hstack((color_image, depth_colormap))

            # Show distance at center pixel
            center_dist = depth_frame.get_distance(320, 240)
            cv2.putText(combined, f"Center: {center_dist:.2f}m",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('RGB + Depth', combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

except ImportError:
    print("pyrealsense2 not installed. Using OpenCV VideoCapture for generic depth camera.")
    # Generic depth camera via V4L2
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
```

---

## 7. Review

### Key Takeaways

1. **Pulse ToF**: \(d = ct/2\) — simple, accurate, single-point
2. **Phase-shift**: \(d = c\varphi/(4\pi f)\) — better precision, limited range
3. **Triangulation**: Geometric, cheap, but limited to short range
4. **Structured light depth camera**: IR pattern → deformation → depth map (great indoors)
5. **ToF depth camera**: Phase-shift per pixel → depth map (better outdoors)
6. **Kalman filter** smooths LiDAR data better than moving average (Day 8 connection)

### Sensor Selection for Our Car

| Sensor | Use Case | Connection |
|--------|----------|------------|
| 1D LiDAR | Forward obstacle distance | UART |
| Depth Camera | 3D mapping, obstacle avoidance | USB |
| RGB Camera | Lane detection, object recognition | USB/MIPI CSI |

### Looking Ahead

Tomorrow (Day 11), we'll learn **camera geometry and calibration** — the mathematical framework that turns pixel coordinates into real-world measurements. This is essential for combining camera and depth data accurately.
