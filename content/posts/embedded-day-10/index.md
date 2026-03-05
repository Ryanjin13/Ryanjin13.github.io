---
title: "Day 10 — 1D LiDAR and Depth Cameras: ToF and Structured Light"
date: 2026-03-06
description: "Time-of-Flight distance measurement, phase-shift method, triangulation, structured light depth cameras, and comparing depth sensing technologies"
categories: ["Autonomous Driving"]
tags: ["LiDAR", "Depth Camera", "ToF", "Structured Light", "Distance Sensing"]
series: ["Embedded Basics for Autonomous Car"]
series_order: 10
draft: false
---

{{< katex >}}

## What You'll Learn

In Day 9 we built a PID controller that can track a velocity setpoint. But a speed controller alone is useless if the car cannot perceive its environment. Today we add the first perception sensors: **1D LiDAR** for point distance measurement and **depth cameras** for full 2D depth maps.

By the end of this post you will be able to:

1. Explain how Time-of-Flight (ToF) distance measurement works with pulse timing and phase-shift methods.
2. Derive the maximum unambiguous range of a phase-shift sensor.
3. Describe the triangulation principle used by Sharp IR sensors.
4. Understand how **structured light** depth cameras project IR patterns to compute depth.
5. Understand how **ToF depth cameras** extend 1D ToF to a full pixel array.
6. Compare structured light vs ToF depth cameras and know when to use each.
7. Filter noisy LiDAR data using a moving average and a Kalman filter (connecting to Day 8).
8. Visualize depth camera output with Python and OpenCV.

---

## 1. Distance Sensing Overview

An autonomous car needs to know "how far is that object?" There are three fundamental physics principles used in distance sensing:

| Method | Principle | Typical sensor |
|--------|-----------|---------------|
| Pulse ToF | Measure round-trip time of light pulse | TFmini, VL53L0X |
| Phase-shift ToF | Measure phase difference of modulated light | VL53L5CX, ToF cameras |
| Triangulation | Measure angle/displacement of reflected beam | Sharp GP2Y0A21, structured light cameras |

All three are forms of **active sensing**: the sensor emits its own signal (IR laser or LED) rather than relying on ambient light. This is a crucial distinction from passive cameras that depend on scene illumination.

---

## 2. 1D LiDAR: Pulse Time-of-Flight

### 2.1 Basic Principle

A pulse ToF sensor fires a short laser pulse and starts a timer. The pulse reflects off the target and returns to the detector. The distance is:

$$
d = \frac{c \cdot t_{\text{round-trip}}}{2}
$$

where \(c \approx 3 \times 10^8\) m/s is the speed of light and the factor of 2 accounts for the round trip.

```
  Sensor                          Target
  ┌──────┐     Laser pulse       ┌──────┐
  │ TX ──├──────────────────────►│      │
  │      │                       │      │
  │ RX ◄─├──────────────────────│      │
  └──────┘     Reflected pulse   └──────┘
         ◄──────── d ────────►

  t_start ─────────────────── t_return
          t_round_trip = t_return - t_start
```

### 2.2 Practical Numbers

Light travels 30 cm in 1 nanosecond. For a 3-meter distance:

$$
t = \frac{2d}{c} = \frac{2 \times 3}{3 \times 10^8} = 20 \text{ ns}
$$

Measuring 20 ns accurately requires high-speed electronics. This is why consumer 1D LiDAR modules (like TFmini-Plus) use specialized **time-to-digital converters (TDC)** or correlators rather than raw timers. A TDC can achieve sub-nanosecond resolution using delay-locked loops and vernier techniques.

### 2.3 Signal-to-Noise Ratio

The return signal strength follows the **LiDAR equation**:

$$
P_r = P_t \cdot \frac{\rho \cdot A_r}{d^2} \cdot \eta_{\text{atm}} \cdot \eta_{\text{opt}}
$$

where:
- \(P_t\) is the transmitted power,
- \(\rho\) is the target reflectivity,
- \(A_r\) is the receiver aperture area,
- \(d\) is the distance,
- \(\eta_{\text{atm}}\) and \(\eta_{\text{opt}}\) are atmospheric and optical efficiencies.

The key takeaway: return power drops with \(d^2\), so noise increases significantly at longer ranges.

### 2.4 Typical Module Specifications

For a typical module like TFmini-Plus:

| Parameter | Value |
|-----------|-------|
| Range | 0.1 -- 12 m |
| Resolution | 1 cm |
| Accuracy | \(\pm 1\%\) at short range |
| Update rate | 100 -- 1000 Hz |
| Wavelength | 850 nm (near-IR) |
| Interface | UART (115200 baud) |
| Frame format | 9 bytes: 0x59, 0x59, dist_lo, dist_hi, str_lo, str_hi, temp_lo, temp_hi, checksum |

---

## 3. Phase-Shift Time-of-Flight

### 3.1 Principle

Instead of a single pulse, the sensor emits **continuously modulated** light — typically a sinusoidal intensity modulation at frequency \(f_{\text{mod}}\). The reflected signal has the same frequency but is shifted in phase by \(\phi\):

$$
\phi = 2\pi f_{\text{mod}} \cdot t_{\text{round-trip}} = 2\pi f_{\text{mod}} \cdot \frac{2d}{c}
$$

Solving for distance:

$$
\boxed{d = \frac{c \cdot \phi}{4\pi f_{\text{mod}}}}
$$

```
Emitted:   ────/\──/\──/\──/\──/\──>  (frequency f_mod)

Received:  ──────/\──/\──/\──/\──/\─  (same f, phase shifted by phi)

           |<--->|
            phi = phase difference proportional to distance
```

### 3.2 Maximum Unambiguous Range

The phase \(\phi\) can only be measured modulo \(2\pi\). When \(\phi = 2\pi\), the sensor cannot distinguish it from \(\phi = 0\). This gives the maximum unambiguous range:

$$
d_{\max} = \frac{c}{2 f_{\text{mod}}}
$$

| Modulation Frequency | Max Unambiguous Range |
|---------------------|----------------------|
| 10 MHz | 15.0 m |
| 20 MHz | 7.5 m |
| 30 MHz | 5.0 m |
| 100 MHz | 1.5 m |

For \(f_{\text{mod}} = 20\) MHz:

$$
d_{\max} = \frac{3 \times 10^8}{2 \times 20 \times 10^6} = 7.5 \text{ m}
$$

To extend range, lower the modulation frequency — but this reduces depth resolution. Some sensors use **dual-frequency** modulation to get both range and resolution:

$$
d_{\max,\text{effective}} = \frac{c}{2 \cdot \gcd(f_1, f_2)}
$$

### 3.3 Depth Resolution

The depth resolution depends on how precisely we can measure the phase. For a phase noise of \(\sigma_\phi\):

$$
\sigma_d = \frac{c}{4\pi f_{\text{mod}}} \cdot \sigma_\phi
$$

Higher modulation frequency gives better depth resolution but shorter max range. This is the fundamental tradeoff in phase-shift ToF design.

### 3.4 Four-Bucket Sampling

In practice, the phase is measured by sampling the return signal at four equally spaced phase offsets (0, \(\pi/2\), \(\pi\), \(3\pi/2\)):

$$
\phi = \arctan\!\left(\frac{S_3 - S_1}{S_0 - S_2}\right)
$$

where \(S_0, S_1, S_2, S_3\) are the integrated signal at each phase offset. This technique is called **four-bucket demodulation** and is the basis of most ToF sensor pixels.

The amplitude (signal quality indicator):

$$
A = \frac{1}{2}\sqrt{(S_0 - S_2)^2 + (S_1 - S_3)^2}
$$

Low amplitude means unreliable depth — this serves as a confidence metric.

---

## 4. Triangulation (Sharp IR Sensor)

### 4.1 Principle

A triangulation sensor uses geometry rather than time. An IR LED emits a beam at an angle. The beam hits the target and reflects back to a position-sensitive detector (PSD). The position of the reflected spot on the detector changes with distance:

```
        LED
        /│\
       / │ \     beam
      /  │  \────────────> Target at distance d1
     /   │   \
    /    │    \──────────────────> Target at distance d2
   /     │
  PSD    │ baseline b
  ┌──────┤
  │ x1   │     (spot position moves with distance)
  │  x2  │
  └──────┘
```

By similar triangles:

$$
d = \frac{b \cdot f}{x}
$$

where \(b\) is the baseline distance between LED and PSD, \(f\) is the focal length of the receiving lens, and \(x\) is the position of the spot on the PSD.

### 4.2 Nonlinear Output

The relationship \(d \propto 1/x\) means the output voltage vs distance curve is **hyperbolic**, not linear. At long range, small changes in distance produce tiny changes in voltage — resolution degrades rapidly. This is why Sharp sensors are best at close range (10-80 cm).

The voltage-to-distance conversion is typically:

$$
d \approx \frac{a}{V - b}
$$

where \(a\) and \(b\) are calibration constants specific to each sensor model.

### 4.3 Limitations

| Limitation | Cause |
|-----------|-------|
| Short range (< 1 m) | Inverse relationship degrades resolution at distance |
| Blind spot (< 10 cm) | Objects too close: reflected spot falls outside PSD |
| Angular dependency | Specular surfaces reflect beam away from detector |
| Slow response (25-40 ms) | PSD integration time |
| Color dependency | Dark surfaces absorb more IR, reducing return signal |

---

## 5. Noise Sources in Distance Sensors

All distance sensors suffer from noise. Understanding the sources helps you design filters (connecting to Day 8).

### 5.1 Common Noise Sources

| Source | Effect | Mitigation |
|--------|--------|-----------|
| **Black surfaces** | Absorb IR, weak return signal, noisy or no reading | Increase laser power, use averaging |
| **Glass/transparent objects** | Beam passes through, measures wall behind glass | Cannot be fixed optically; use ultrasonic backup |
| **Sunlight** | Saturates IR detector, especially outdoors | Use narrow bandpass filter at sensor wavelength |
| **Multipath** | Signal bounces off multiple surfaces before return | Phase unwrapping algorithms, multi-frequency |
| **Temperature drift** | Electronic component values shift | Onboard calibration, temperature compensation |
| **Motion blur** | Object moves during measurement | Higher sample rate, predictive filtering |
| **Crosstalk** | Multiple sensors interfere with each other | Time-division multiplexing, unique modulation codes |

### 5.2 Noise Model

For most 1D LiDAR sensors, the measurement noise is approximately Gaussian with distance-dependent variance:

$$
z[k] = d_{\text{true}} + v[k], \qquad v[k] \sim \mathcal{N}(0, \sigma^2(d))
$$

where \(\sigma(d)\) increases with distance (weaker return signal means more noise). For the TFmini-Plus at indoor ranges, \(\sigma \approx 1\text{--}3\) cm is typical.

The signal strength reading provides a direct indicator of measurement quality. A common rule:

$$
\text{confidence} = \begin{cases}
\text{high} & \text{if strength} > 100 \\
\text{medium} & \text{if } 20 < \text{strength} \leq 100 \\
\text{low} & \text{if strength} \leq 20
\end{cases}
$$

---

## 6. Structured Light Depth Camera

### 6.1 Concept: From 1D to 2D

A 1D LiDAR gives a single distance value per measurement. A depth camera produces a complete **depth map** — distance for every pixel. There are two main technologies: structured light and ToF arrays.

### 6.2 How Structured Light Works

A structured light camera (e.g., Intel RealSense D435, original Kinect) projects a known **IR pattern** (dots, stripes, or speckle) onto the scene. An IR camera observes the pattern deformation.

```
  IR Projector          Scene            IR Camera
  ┌──────────┐                          ┌──────────┐
  │ . . . .  │──── known pattern ──────>│ captures  │
  │ . . . .  │                   ╱╲     │ deformed  │
  │ . . . .  │            object   ╲    │ pattern   │
  └──────────┘                      ╲   └──────────┘
       ◄────── baseline b ──────►

  Pattern deformation → triangulation → depth per pixel
```

**Step 1**: The projector emits a known dot or speckle pattern using an IR laser or LED.

**Step 2**: The IR camera captures the pattern as it appears on surfaces in the scene.

**Step 3**: For each dot (or correlation window), the system finds the **disparity** — how much the dot has shifted compared to where it would appear on a flat reference surface at a known distance.

**Step 4**: Using triangulation (the same principle as stereo vision), compute depth:

$$
d = \frac{b \cdot f}{\text{disparity}}
$$

where \(b\) is the projector-camera baseline and \(f\) is the focal length in pixels.

### 6.3 Disparity and Depth Resolution

Since \(d = bf / \text{disparity}\), the depth resolution depends on the disparity resolution \(\delta_{\text{disp}}\):

$$
\delta_d = \frac{d^2}{b \cdot f} \cdot \delta_{\text{disp}}
$$

This means depth resolution **degrades quadratically** with distance. At 1 m with sub-pixel disparity, you get millimeter-level depth. At 5 m, the resolution drops to centimeters.

### 6.4 Structured Light Characteristics

| Property | Value (typical D435) |
|----------|---------------------|
| Resolution | 1280 x 720 |
| Depth range | 0.1 -- 10 m |
| Accuracy | < 2% at 2 m |
| Frame rate | 30 -- 90 fps |
| Baseline | ~55 mm |
| Principle | Active IR stereo with structured light assist |

### 6.5 Strengths and Weaknesses

**Strengths**: works well indoors, high resolution, good at texture-less surfaces (the projected pattern provides artificial texture), mature software ecosystem.

**Weaknesses**: sunlight washes out IR pattern (poor outdoor performance), accuracy degrades with \(d^2\) (because disparity resolution is fixed), power-hungry projector, limited by baseline for minimum range.

---

## 7. Phase-Shift ToF Depth Camera

### 7.1 Concept

A ToF depth camera (e.g., Microsoft Azure Kinect, PMD sensors, Intel RealSense L515) applies the phase-shift ToF principle (Section 3) to every pixel simultaneously.

```
  Modulated              Scene            Sensor Array
  IR Flood               ┌──────┐        ┌──────────┐
  ┌───┐                  │      │        │ ░░░░░░░  │
  │~~~│── modulated IR──>│      │<──────│ ░░░░░░░  │  (each pixel
  │~~~│   (entire scene) │      │reflect│ ░░░░░░░  │   measures phase)
  └───┘                  └──────┘        └──────────┘
```

Each pixel in the sensor array independently measures the phase shift \(\phi\) of the returning modulated light. The depth at pixel \((u, v)\) is:

$$
d(u, v) = \frac{c \cdot \phi(u, v)}{4\pi f_{\text{mod}}}
$$

### 7.2 Correlation-Based Phase Measurement

Each pixel uses the four-bucket sampling technique from Section 3.4:

$$
\phi(u,v) = \arctan\!\left(\frac{S_{3}(u,v) - S_{1}(u,v)}{S_0(u,v) - S_{2}(u,v)}\right)
$$

The amplitude at each pixel indicates measurement reliability:

$$
A(u,v) = \frac{1}{2}\sqrt{(S_0 - S_2)^2 + (S_1 - S_3)^2}
$$

Low amplitude means unreliable depth — pixels with \(A < A_{\text{threshold}}\) are typically masked out in the depth map.

### 7.3 ToF Camera Characteristics

| Property | Value (typical) |
|----------|----------------|
| Resolution | 320 x 240 -- 640 x 480 |
| Depth range | 0.1 -- 5 m (indoor) |
| Accuracy | 1 -- 2 cm |
| Frame rate | 30 -- 60 fps |
| Max range | Limited by \(f_{\text{mod}}\) |
| Power consumption | Moderate (flood illumination) |

### 7.4 Multipath Interference

ToF cameras are particularly susceptible to multipath. When light bounces off multiple surfaces before reaching the sensor, the measured phase is a weighted average of multiple distances:

```
  Sensor ──── direct path (d1) ─────────── Wall
    │                                        │
    │                                        │
    └──── indirect path (d1 + d2) ── Floor ──┘
              (multipath)

  Measured phase = weighted mix of phi(d1) and phi(d1+d2)
  → Incorrect depth reading
```

**Mitigation**: multi-frequency modulation, computational multipath correction, or switching to structured light in affected areas.

---

## 8. Key Insight: 1D LiDAR to Depth Camera

The connection between the technologies is beautifully simple. Understand this and the entire landscape of depth sensing clicks into place:

```
1D LiDAR (single point)
    │
    │  Extend to 2D scanning (rotating mirror)
    ▼
2D LiDAR (e.g., RPLiDAR — a ring of distance points)
    │
    │  Replace mechanical scanning with pixel array
    ▼
ToF Depth Camera (every pixel measures ToF independently)
```

And structured light cameras are the "triangulation version" of this extension:

```
Sharp IR Sensor (single point triangulation)
    │
    │  Project a pattern of thousands of points
    ▼
Structured Light Depth Camera (triangulation per dot/correlation window)
```

**The fundamental physics does not change — only the parallelism of measurement.**

A 2D scanning LiDAR like RPLiDAR measures 360 points per revolution by mechanically spinning a 1D ToF sensor. A ToF depth camera achieves the equivalent of thousands of simultaneous 1D measurements by replacing the spinning mechanism with a pixel array, each pixel acting as an independent phase-shift ToF sensor.

---

## 9. Structured Light vs ToF Comparison

| Feature | Structured Light | ToF Camera |
|---------|-----------------|------------|
| **Depth principle** | Triangulation (disparity) | Phase shift |
| **Range** | 0.1 -- 10 m | 0.1 -- 5 m |
| **Resolution** | Higher (up to 1280x720) | Lower (typically 320x240) |
| **Accuracy at close range** | Excellent (sub-mm) | Good (cm-level) |
| **Accuracy vs distance** | Degrades with \(d^2\) | Roughly constant |
| **Outdoor performance** | Poor (sunlight washes pattern) | Better but still affected |
| **Multipath** | Minimal (disparity-based) | Significant (corrupts phase) |
| **Power consumption** | Higher (projector) | Moderate (flood illumination) |
| **Latency** | Higher (correlation computation) | Lower (per-pixel measurement) |
| **Cost** | Lower | Higher |
| **Multi-sensor interference** | Low | Can interfere if same \(f_{\text{mod}}\) |
| **Edge artifacts** | Flying pixels at depth discontinuities | Mixed pixels at boundaries |

**For our autonomous car project**: we use an Intel RealSense D435 (structured light + active IR stereo) because it has good indoor performance, reasonable range, and excellent software support through the `pyrealsense2` library.

---

## 10. Common Weaknesses of All Depth Sensors

### 10.1 Sunlight Interference

Both structured light and ToF sensors use near-IR wavelengths (typically 850 nm or 940 nm). Sunlight contains strong near-IR components that saturate the detector.

**Mitigation strategies**:
- Use narrow bandpass optical filters centered on the emitter wavelength
- Increase emitter power (limited by eye safety regulations: IEC 60825)
- Use 940 nm wavelength (where solar radiation has a dip due to atmospheric water absorption)
- Reduce exposure time and increase modulation frequency

### 10.2 Multipath Interference

In concave geometries (corners, bowls), light bounces between surfaces before returning to the sensor:

```
  Sensor ──── direct path ───────── Wall
    │                                 │
    │                                 │
    └──── indirect path ── Floor ─────┘
              (multipath)

  ToF sensor measures average of both paths → incorrect depth
```

**Mitigation**: multi-frequency modulation allows separation of direct and indirect components, though this reduces frame rate.

### 10.3 Transparent and Specular Objects

Glass windows let IR pass through (ToF measures the wall behind the glass). Mirrors reflect IR at the specular angle (beam never returns to sensor). Shiny metal surfaces create mixed reflections.

```
  Sensor ──── IR beam ──────► Glass Window ──────► Actual wall
                                (passes through)
  Sensor sees the wall, not the glass!

  Sensor ──── IR beam ──────► Mirror
                                └──── reflected away (no return)
  Sensor sees nothing!
```

**Mitigation**: no good optical fix exists for these cases. Use ultrasonic sensors as backup, or fuse multiple sensor modalities.

### 10.4 Flying Pixels

At depth discontinuities (edges of objects), a single sensor pixel may receive light from both the foreground and background, producing a depth value that belongs to neither:

```
  Foreground (1m)    Background (3m)
       │                  │
       │     Pixel sees   │
       │  ◄─ both ──►     │
       │                  │
  Reports ~2m (incorrect!)
```

These "flying pixels" appear as a fringe of incorrect depth values around object edges. They must be filtered out before using the depth data for 3D reconstruction.

---

## 11. Hands-On Lab: 1D LiDAR Real-Time Plotting

### 11.1 Reading TFmini-Plus over UART

```python
"""
tfmini_reader.py
Read distance from TFmini-Plus 1D LiDAR over UART.
Works on Raspberry Pi 5 with TFmini-Plus connected to GPIO UART.
"""

import serial
import struct
import time


class TFminiPlus:
    """Driver for TFmini-Plus 1D LiDAR module."""

    HEADER = 0x59
    FRAME_SIZE = 9

    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 115200):
        """
        Args:
            port: Serial port (RPi 5: /dev/ttyAMA0, USB: /dev/ttyUSB0)
            baudrate: Communication speed (default 115200 for TFmini-Plus)
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.ser.reset_input_buffer()

    def read_distance(self) -> dict:
        """
        Read one distance frame from the sensor.

        TFmini-Plus protocol (9 bytes per frame):
          Byte 0: 0x59 (header)
          Byte 1: 0x59 (header)
          Byte 2: distance low byte
          Byte 3: distance high byte
          Byte 4: signal strength low byte
          Byte 5: signal strength high byte
          Byte 6: temperature low byte
          Byte 7: temperature high byte
          Byte 8: checksum (sum of bytes 0-7, low 8 bits)

        Returns:
            dict with keys: distance_cm, strength, temperature_C
            or None if read failed.
        """
        # Synchronize to frame header (two 0x59 bytes)
        while True:
            byte = self.ser.read(1)
            if len(byte) == 0:
                return None
            if byte[0] == self.HEADER:
                byte2 = self.ser.read(1)
                if len(byte2) == 0:
                    return None
                if byte2[0] == self.HEADER:
                    break

        # Read remaining 7 bytes
        data = self.ser.read(7)
        if len(data) < 7:
            return None

        # Parse frame
        dist_lo, dist_hi = data[0], data[1]
        str_lo, str_hi = data[2], data[3]
        temp_lo, temp_hi = data[4], data[5]
        checksum = data[6]

        # Verify checksum (sum of first 8 bytes, modulo 256)
        frame = bytes([self.HEADER, self.HEADER]) + data[:6]
        calc_checksum = sum(frame) & 0xFF
        if calc_checksum != checksum:
            return None

        distance_cm = dist_lo + (dist_hi << 8)
        strength = str_lo + (str_hi << 8)
        # Temperature: raw value / 8 - 256 gives degrees Celsius
        temperature = ((temp_lo + (temp_hi << 8)) / 8.0) - 256.0

        return {
            'distance_cm': distance_cm,
            'strength': strength,
            'temperature_C': round(temperature, 1)
        }

    def close(self):
        """Release the serial port."""
        self.ser.close()


# --- Quick test ---
if __name__ == "__main__":
    lidar = TFminiPlus('/dev/ttyAMA0')
    try:
        print(f"{'Time':>8s} {'Dist(cm)':>10s} {'Strength':>10s} {'Temp(C)':>8s}")
        t_start = time.monotonic()
        for _ in range(500):
            result = lidar.read_distance()
            if result:
                elapsed = time.monotonic() - t_start
                print(f"{elapsed:8.2f} {result['distance_cm']:10d} "
                      f"{result['strength']:10d} {result['temperature_C']:8.1f}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        lidar.close()
```

### 11.2 Real-Time Distance Plot with Scrolling Window

```python
"""
lidar_realtime_plot.py
Real-time scrolling plot of 1D LiDAR distance measurements.
Uses matplotlib animation for smooth updates.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time


# --- Simulation mode (replace with TFminiPlus for real hardware) ---
class FakeLidar:
    """Simulates a 1D LiDAR with a moving target and realistic noise."""
    def __init__(self):
        self.t = 0

    def read_distance(self):
        self.t += 0.01
        # Simulated target: sinusoidal motion + step change
        if self.t < 5:
            true_dist = 150 + 80 * np.sin(0.5 * self.t)
        elif self.t < 8:
            true_dist = 250  # stationary
        else:
            true_dist = 100 + 30 * np.sin(1.5 * self.t)

        # Distance-dependent noise (worse at long range)
        noise_std = 2 + 0.01 * true_dist
        noise = np.random.normal(0, noise_std)

        # Occasional outlier (1% chance, simulating specular reflection)
        if np.random.random() < 0.01:
            noise += np.random.choice([-50, 50, 100])

        return {'distance_cm': max(0, int(true_dist + noise)),
                'strength': max(10, 1000 - int(true_dist * 2)),
                'temperature_C': 25.0}


# --- Setup ---
WINDOW_SIZE = 500  # number of points in scrolling window
distances = np.zeros(WINDOW_SIZE)
strengths = np.zeros(WINDOW_SIZE)

lidar = FakeLidar()  # Replace with: TFminiPlus('/dev/ttyAMA0')

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

line_dist, = ax1.plot([], [], 'b-', linewidth=1)
ax1.set_xlim(0, WINDOW_SIZE)
ax1.set_ylim(0, 400)
ax1.set_ylabel('Distance [cm]')
ax1.set_title('1D LiDAR Real-Time Distance')
ax1.grid(True, alpha=0.3)

line_str, = ax2.plot([], [], 'g-', linewidth=1)
ax2.set_xlim(0, WINDOW_SIZE)
ax2.set_ylim(0, 1200)
ax2.set_ylabel('Signal Strength')
ax2.set_xlabel('Sample')
ax2.grid(True, alpha=0.3)


def update(frame):
    """Animation callback: read one sample and update plot."""
    global distances, strengths
    result = lidar.read_distance()
    if result:
        distances = np.roll(distances, -1)
        distances[-1] = result['distance_cm']
        strengths = np.roll(strengths, -1)
        strengths[-1] = result['strength']

        line_dist.set_data(range(WINDOW_SIZE), distances)
        line_str.set_data(range(WINDOW_SIZE), strengths)
    return line_dist, line_str


ani = animation.FuncAnimation(fig, update, interval=10, blit=True)
plt.tight_layout()
plt.show()
```

### 11.3 Filtering Comparison: Moving Average vs Kalman Filter

This connects directly to Day 8. We compare three approaches on the same noisy LiDAR data:

```python
"""
lidar_filter_comparison.py
Compare raw, moving average, and Kalman filter on 1D LiDAR data.
Demonstrates why Kalman filtering (Day 8) is superior.
"""

import numpy as np
import matplotlib.pyplot as plt


# --- Generate synthetic LiDAR data ---
np.random.seed(42)
N = 1000
dt = 0.01
t = np.arange(N) * dt

# True distance: segments with different behaviors
true_distance = np.piecewise(
    t,
    [t < 3, (t >= 3) & (t < 6), t >= 6],
    [lambda x: 100 + 5 * x,        # slow approach (100 → 115 cm)
     lambda x: 130.0,               # stationary target
     lambda x: 130 - 20 * (x - 6)] # moving away
)

# Noisy measurement (distance-dependent noise)
noise_std = 4.0
measured = true_distance + np.random.normal(0, noise_std, N)

# Add some outliers (5% of samples)
outlier_mask = np.random.random(N) < 0.05
measured[outlier_mask] += np.random.choice([-30, 30, 50], size=outlier_mask.sum())


# --- Filter 1: Moving Average ---
def moving_average(data, window=10):
    """Simple moving average filter."""
    kernel = np.ones(window) / window
    filtered = np.convolve(data, kernel, mode='same')
    return filtered


ma_filtered = moving_average(measured, window=20)


# --- Filter 2: 1D Kalman Filter (from Day 8) ---
class KalmanFilter1D:
    """Simple 1D Kalman filter with constant-velocity model."""

    def __init__(self, x0=0.0, v0=0.0, p0=100.0,
                 q_pos=0.1, q_vel=0.5, r=16.0, dt=0.01):
        # State: [position, velocity]
        self.x = np.array([x0, v0])
        self.P = np.diag([p0, p0])
        self.dt = dt

        # State transition: x_new = x + v*dt
        self.F = np.array([[1, dt],
                           [0, 1]])

        # Measurement: we observe position only
        self.H = np.array([[1, 0]])

        # Process noise
        self.Q = np.array([[q_pos, 0],
                           [0, q_vel]])

        # Measurement noise
        self.R = np.array([[r]])

    def update(self, z):
        """Predict then update with measurement z."""
        # Predict
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update
        y = z - self.H @ x_pred             # innovation
        S = self.H @ P_pred @ self.H.T + self.R  # innovation covariance
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        self.x = x_pred + K @ np.array([y])
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        return self.x[0]  # return position estimate


kf = KalmanFilter1D(x0=measured[0], v0=0.0, p0=100.0,
                     q_pos=0.5, q_vel=1.0, r=noise_std**2, dt=dt)
kf_filtered = np.zeros(N)
for i in range(N):
    kf_filtered[i] = kf.update(measured[i])


# --- Filter 3: Median filter (robust to outliers) ---
def median_filter(data, window=5):
    """Sliding median filter."""
    filtered = np.zeros_like(data)
    half = window // 2
    for i in range(len(data)):
        start = max(0, i - half)
        end = min(len(data), i + half + 1)
        filtered[i] = np.median(data[start:end])
    return filtered


med_filtered = median_filter(measured, window=7)


# --- Plot comparison ---
fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)

# Raw
axes[0].plot(t, true_distance, 'g-', linewidth=2, label='True')
axes[0].plot(t, measured, 'b.', markersize=1, alpha=0.5, label='Raw LiDAR')
axes[0].set_title('Raw Measurement (with outliers)')
axes[0].legend()
axes[0].set_ylabel('Distance [cm]')
axes[0].grid(True, alpha=0.3)

# Moving Average
axes[1].plot(t, true_distance, 'g-', linewidth=2, label='True')
axes[1].plot(t, ma_filtered, 'r-', linewidth=1.5, label='Moving Avg (w=20)')
axes[1].set_title('Moving Average Filter — introduces lag, fooled by outliers')
axes[1].legend()
axes[1].set_ylabel('Distance [cm]')
axes[1].grid(True, alpha=0.3)

# Median
axes[2].plot(t, true_distance, 'g-', linewidth=2, label='True')
axes[2].plot(t, med_filtered, 'orange', linewidth=1.5, label='Median Filter (w=7)')
axes[2].set_title('Median Filter — robust to outliers but introduces lag')
axes[2].legend()
axes[2].set_ylabel('Distance [cm]')
axes[2].grid(True, alpha=0.3)

# Kalman
axes[3].plot(t, true_distance, 'g-', linewidth=2, label='True')
axes[3].plot(t, kf_filtered, 'm-', linewidth=1.5, label='Kalman Filter (CV model)')
axes[3].set_title('Kalman Filter — tracks transitions, adapts gain automatically')
axes[3].legend()
axes[3].set_ylabel('Distance [cm]')
axes[3].set_xlabel('Time [s]')
axes[3].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('lidar_filter_comparison.png', dpi=150)
plt.show()

# --- Quantitative comparison ---
rmse_raw = np.sqrt(np.mean((measured - true_distance)**2))
rmse_ma = np.sqrt(np.mean((ma_filtered - true_distance)**2))
rmse_med = np.sqrt(np.mean((med_filtered - true_distance)**2))
rmse_kf = np.sqrt(np.mean((kf_filtered - true_distance)**2))

print(f"\n{'Filter':<25s} {'RMSE (cm)':>10s}")
print("-" * 37)
print(f"{'Raw (no filter)':<25s} {rmse_raw:10.2f}")
print(f"{'Moving Average (w=20)':<25s} {rmse_ma:10.2f}")
print(f"{'Median (w=7)':<25s} {rmse_med:10.2f}")
print(f"{'Kalman Filter (CV)':<25s} {rmse_kf:10.2f}")
```

**Expected results**: the Kalman filter achieves the lowest RMSE because:
1. It uses a **constant-velocity model** that predicts motion during transitions (where moving average lags).
2. It automatically adjusts its gain through the Kalman gain \(K\) — high gain when uncertain, low gain when confident.
3. With the constant-velocity model, it tracks linear ramps without the phase lag of a moving average.

The median filter handles outliers better than the moving average but still introduces lag. In practice, a **median pre-filter followed by a Kalman filter** is a robust combination for LiDAR data.

### 11.4 Depth Camera Stream and Visualization

```python
"""
depth_camera_stream.py
Capture and visualize RGB + Depth from Intel RealSense D435.
Includes depth scale handling and basic alignment.
"""

import numpy as np
import cv2

try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    print("pyrealsense2 not installed. Using synthetic data.")


def create_realsense_pipeline():
    """Initialize RealSense D435 pipeline with depth and color streams."""
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Get depth scale (converts raw uint16 to meters)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale: {depth_scale:.6f} m/unit")

    # Enable post-processing filters for cleaner depth
    # (these run on the host CPU)
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 2)

    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)

    temporal = rs.temporal_filter()

    hole_filling = rs.hole_filling_filter()

    # Create alignment object (align depth to color frame)
    align = rs.align(rs.stream.color)

    filters = [decimation, spatial, temporal, hole_filling]

    return pipeline, align, depth_scale, filters


def create_synthetic_frames():
    """Generate synthetic depth and color frames for testing without hardware."""
    # Synthetic color image with simple objects
    color = np.zeros((480, 640, 3), dtype=np.uint8)
    color[:] = [100, 80, 60]  # brown background
    cv2.circle(color, (320, 240), 100, (0, 200, 0), -1)   # green sphere
    cv2.rectangle(color, (100, 100), (250, 350), (200, 0, 0), -1)  # blue box

    # Synthetic depth map (in mm, uint16)
    depth = np.full((480, 640), 2000, dtype=np.uint16)  # 2m background

    # Closer rectangular object at 1m
    depth[100:350, 100:250] = 1000

    # Even closer spherical object at 0.5m
    Y, X = np.ogrid[:480, :640]
    sphere_mask = (X - 320)**2 + (Y - 240)**2 < 100**2
    depth[sphere_mask] = 500

    # Add realistic noise (increases with distance)
    noise_scale = depth.astype(np.float32) * 0.01  # 1% of distance
    noise = (np.random.normal(0, 1, depth.shape) * noise_scale).astype(np.int16)
    depth = np.clip(depth.astype(np.int16) + noise, 0, 10000).astype(np.uint16)

    return color, depth


def visualize_depth(depth_image, depth_scale=0.001, max_range_m=5.0):
    """Convert raw depth image to colorized visualization."""
    # Convert to meters
    depth_m = depth_image.astype(np.float32) * depth_scale

    # Normalize to 0-255 for colormap
    depth_normalized = np.clip(depth_m / max_range_m, 0, 1)
    depth_uint8 = (depth_normalized * 255).astype(np.uint8)

    # Apply JET colormap (blue=close, red=far)
    depth_colormap = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

    # Mark invalid pixels (depth = 0) as black
    depth_colormap[depth_image == 0] = [0, 0, 0]

    return depth_colormap


def compute_histogram(depth_image, depth_scale=0.001, max_range_m=5.0, bins=100):
    """Compute depth histogram for analysis."""
    valid = depth_image[depth_image > 0].astype(np.float32) * depth_scale
    hist, edges = np.histogram(valid, bins=bins, range=(0, max_range_m))
    return hist, edges


def main():
    if HAS_REALSENSE:
        pipeline, align, depth_scale, filters = create_realsense_pipeline()
    else:
        depth_scale = 0.001  # 1 mm per unit

    print("Controls:")
    print("  'q' - quit")
    print("  's' - save snapshot")
    print("  'f' - toggle post-processing filters")

    use_filters = True
    frame_count = 0

    try:
        while True:
            if HAS_REALSENSE:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Apply post-processing filters
                if use_filters:
                    for f in filters:
                        depth_frame = f.process(depth_frame)

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
            else:
                color_image, depth_image = create_synthetic_frames()

            # Visualize depth
            depth_colormap = visualize_depth(depth_image, depth_scale)

            # Show center pixel distance
            cy, cx = depth_image.shape[0] // 2, depth_image.shape[1] // 2
            center_dist = depth_image[cy, cx] * depth_scale
            cv2.putText(color_image,
                        f"Center: {center_dist:.3f} m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2)

            # Show valid pixel count
            valid_pct = np.count_nonzero(depth_image) / depth_image.size * 100
            cv2.putText(depth_colormap,
                        f"Valid: {valid_pct:.1f}%",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)

            # Side by side display
            combined = np.hstack([color_image, depth_colormap])
            cv2.imshow('RGB | Depth', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite(f'color_{frame_count:04d}.png', color_image)
                cv2.imwrite(f'depth_color_{frame_count:04d}.png', depth_colormap)
                np.save(f'depth_raw_{frame_count:04d}.npy', depth_image)
                print(f"Snapshot {frame_count} saved!")
                frame_count += 1
            elif key == ord('f'):
                use_filters = not use_filters
                print(f"Filters: {'ON' if use_filters else 'OFF'}")

            if not HAS_REALSENSE:
                import time
                time.sleep(0.033)  # simulate 30 fps

    finally:
        if HAS_REALSENSE:
            pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

### 11.5 RGB-Depth Alignment Analysis

```python
"""
rgbd_alignment.py
Demonstrate the importance of RGB-Depth alignment.
Shows what happens when depth and color are NOT aligned vs aligned.
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt


def show_alignment_comparison():
    """
    Visualize misaligned vs aligned RGB-Depth overlay.

    In real hardware, misalignment comes from the physical offset
    (baseline ~55mm) between the depth sensor and RGB camera.
    """
    h, w = 480, 640

    # Color image: vertical edge (brown wall | blue wall)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:, :320] = [200, 100, 50]   # left half: brown
    color[:, 320:] = [50, 150, 200]   # right half: blue

    # True depth: left half closer (1m), right half farther (3m)
    depth_aligned = np.zeros((h, w), dtype=np.float32)
    depth_aligned[:, :320] = 1.0
    depth_aligned[:, 320:] = 3.0

    # Misaligned depth: shifted by 20 pixels (simulating baseline offset)
    shift = 20
    depth_misaligned = np.zeros_like(depth_aligned)
    depth_misaligned[:, shift:] = depth_aligned[:, :-shift]

    # Create overlay visualizations
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # Original color
    axes[0].imshow(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    axes[0].axvline(x=320, color='white', linestyle='--', linewidth=2)
    axes[0].set_title('Color Image (edge at x=320)')
    axes[0].axis('off')

    # Misaligned overlay
    overlay_mis = color.copy()
    depth_vis_mis = (depth_misaligned * 80).astype(np.uint8)
    edge_mis = cv2.Canny(depth_vis_mis, 50, 150)
    overlay_mis[edge_mis > 0] = [0, 0, 255]  # red depth edge
    axes[1].imshow(cv2.cvtColor(overlay_mis, cv2.COLOR_BGR2RGB))
    axes[1].axvline(x=320, color='lime', linestyle='--', linewidth=2,
                    label='Color edge')
    axes[1].axvline(x=320 - shift, color='red', linestyle='--', linewidth=2,
                    label='Depth edge')
    axes[1].set_title('MISALIGNED: edges do not match')
    axes[1].legend(loc='upper right')
    axes[1].axis('off')

    # Aligned overlay
    overlay_ali = color.copy()
    depth_vis_ali = (depth_aligned * 80).astype(np.uint8)
    edge_ali = cv2.Canny(depth_vis_ali, 50, 150)
    overlay_ali[edge_ali > 0] = [0, 0, 255]
    axes[2].imshow(cv2.cvtColor(overlay_ali, cv2.COLOR_BGR2RGB))
    axes[2].axvline(x=320, color='lime', linestyle='--', linewidth=2,
                    label='Color edge')
    axes[2].set_title('ALIGNED: edges match perfectly')
    axes[2].legend(loc='upper right')
    axes[2].axis('off')

    plt.suptitle('RGB-Depth Alignment: Why It Matters', fontsize=14)
    plt.tight_layout()
    plt.savefig('alignment_comparison.png', dpi=150)
    plt.show()

    print("""
    Why alignment matters for autonomous driving:
    -----------------------------------------------
    1. The depth sensor and RGB camera are physically separated (~55mm baseline)
    2. Without alignment, pixel (u,v) in color != pixel (u,v) in depth
    3. This means wrong 3D reconstruction: objects appear shifted
    4. RealSense SDK: rs.align(rs.stream.color) reprojects depth onto color frame
    5. After alignment: color pixel (u,v) and depth pixel (u,v) see the same point

    For our car:
    - Lane detection uses color → needs aligned depth to measure distance
    - Obstacle detection uses depth → needs aligned color for classification
    - SLAM (Day 12) requires consistent RGB-D pairs
    """)


def depth_to_pointcloud(depth_image, K, depth_scale=0.001):
    """
    Convert depth image to 3D point cloud using camera intrinsics.

    This is the inverse of the pinhole projection (Day 11 preview):
      X = (u - cx) * Z / fx
      Y = (v - cy) * Z / fy
      Z = depth * depth_scale

    Args:
        depth_image: HxW uint16 depth image
        K: 3x3 intrinsic matrix
        depth_scale: conversion factor to meters

    Returns:
        Nx3 array of 3D points
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    h, w = depth_image.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    z = depth_image.astype(np.float32) * depth_scale
    valid = z > 0

    x = (u[valid] - cx) * z[valid] / fx
    y = (v[valid] - cy) * z[valid] / fy

    points = np.stack([x, y, z[valid]], axis=-1)
    return points


# Run the demonstration
show_alignment_comparison()

# Example point cloud generation (preview of Day 11 concepts)
print("\n--- Point Cloud Generation Preview ---")
K_example = np.array([[615.0, 0, 320.0],
                       [0, 615.0, 240.0],
                       [0, 0, 1.0]])

# Small synthetic depth image
depth_small = np.array([[1000, 1500, 2000],
                         [1200, 0, 1800],
                         [1100, 1300, 1900]], dtype=np.uint16)

points = depth_to_pointcloud(depth_small, K_example, depth_scale=0.001)
print(f"Generated {len(points)} 3D points from {depth_small.size} pixels")
print(f"(Skipped {depth_small.size - len(points)} invalid pixels with depth=0)")
for i, pt in enumerate(points):
    print(f"  Point {i}: ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}) m")
```

---

## Review

Today we covered the physics and practice of distance sensing for autonomous vehicles.

| Topic | Key equation / takeaway |
|-------|------------------------|
| Pulse ToF | \(d = \frac{c \cdot t}{2}\) |
| Phase-shift ToF | \(d = \frac{c \cdot \phi}{4\pi f_{\text{mod}}}\) |
| Max unambiguous range | \(d_{\max} = \frac{c}{2 f_{\text{mod}}}\) |
| Four-bucket demodulation | \(\phi = \arctan\frac{S_3 - S_1}{S_0 - S_2}\) |
| Triangulation | \(d = \frac{b \cdot f}{x}\), nonlinear, short range |
| Structured light camera | Projects IR pattern, measures disparity, depth \(\propto 1/\text{disp}\) |
| ToF depth camera | Per-pixel phase measurement, constant accuracy with distance |
| Key insight | 1D ToF + pixel array = ToF depth camera |
| Noise sources | Black surfaces, glass, sunlight, multipath, flying pixels |
| Filtering | Kalman filter outperforms moving average (Day 8 connection) |

### Sensor Selection for Our Autonomous Car

| Sensor | Use Case | Interface | Why |
|--------|----------|-----------|-----|
| 1D LiDAR (TFmini-Plus) | Forward obstacle distance | UART | Fast, reliable, cheap |
| Depth Camera (RealSense D435) | 3D mapping, obstacle avoidance | USB 3.0 | Rich depth map + RGB |
| RGB Camera (built into D435) | Lane detection, object recognition | USB 3.0 | Color information for classification |

### Connection to Previous Days

- **Day 8** (Kalman Filter): we applied the Kalman filter to LiDAR data, demonstrating the practical benefit of Bayesian filtering with a constant-velocity model.
- **Day 9** (PID Control): the distance measurements from today's sensors can serve as the feedback signal for a distance-keeping PID controller (maintain 50 cm following distance).

### What Comes Next

In **Day 11**, we move from raw sensor hardware to **camera geometry and calibration**. We will derive the pinhole camera model, understand intrinsic and extrinsic parameters, correct lens distortion, and compute a Bird's Eye View transform. This is essential groundwork before we can do any meaningful computer vision on our depth camera images.
