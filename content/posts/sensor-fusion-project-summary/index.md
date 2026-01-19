---
title: "Sensor Fusion Project: LiDAR-Camera Integration"
date: 2024-07-20
description: "Complete summary of TurtleBot3 sensor fusion project with ROS"
categories: ["Autonomous Driving"]
tags: ["Sensor Fusion", "TurtleBot3", "ROS", "LiDAR", "Camera"]
draft: false
---

{{< katex >}}

## Overview

This post summarizes a sensor fusion project integrating 2D LiDAR and camera data on TurtleBot3 using ROS. The key challenge was matching the 1D LiDAR point data with the 2D camera image plane.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    TurtleBot3                           │
│  ┌─────────┐     ┌─────────┐                           │
│  │ 2D LiDAR│     │  Camera │                           │
│  └────┬────┘     └────┬────┘                           │
│       │               │                                 │
│       └───────┬───────┘                                │
│               │                                         │
│       ┌───────┴───────┐                                │
│       │  Raspberry Pi  │                                │
│       └───────┬───────┘                                │
└───────────────┼─────────────────────────────────────────┘
                │ ROS Topics
                │ (Wi-Fi)
┌───────────────┼─────────────────────────────────────────┐
│               │                                         │
│       ┌───────┴───────┐                                │
│       │   PC Master    │                                │
│       │   fusion.py    │                                │
│       └───────┬───────┘                                │
│               │                                         │
│       ┌───────┴───────┐                                │
│       │ Fused Output  │                                │
│       └───────────────┘                                │
└─────────────────────────────────────────────────────────┘
```

## Data Flow

### ROS Topic Organization

| Topic | Data Type | Source |
|-------|-----------|--------|
| /scan | LaserScan | LiDAR |
| /image_raw | Image | Camera |
| /fusion_image | Image | fusion.py |

### Processing Pipeline

```
LiDAR (/scan)     Camera (/image_raw)
     │                    │
     └────────┬───────────┘
              │
       Time Synchronization
              │
       Coordinate Transform
              │
       Point Projection
              │
       Fused Output
```

## Technical Challenge

### The Problem

2D LiDAR sensors capture only 1D point data in a horizontal plane:

```
         LiDAR Scan Plane
              ─────
            /       \
           /    ●    \  ← Single scan line
          /  (robot)  \
         ──────────────
```

This single scan line must be mapped to the 2D camera image.

### Solution: Row Masking

Distribute LiDAR points across a specific image row matching the LiDAR's vertical position:

```
Camera Image:
┌─────────────────────────┐
│                         │
│                         │
│ ● ● ● ● ● ● ● ● ● ● ● │ ← LiDAR points projected here
│                         │
│                         │
└─────────────────────────┘
```

## Field of View Calibration

### Initial Configuration

- LiDAR valid range: ±45° (90° total)
- Camera FOV: Unknown

### Calibration Process

1. **Measure camera FOV experimentally**
   - Place markers at known angles
   - Capture images and measure visible range

2. **Result**: Camera FOV = ~30°

3. **Adjust LiDAR range to match**

$$
\text{Valid LiDAR Range} = \pm 15° = 30° \text{ total}
$$

### Angle Mapping

For a LiDAR point at angle \\(\theta\\):

$$
x_{pixel} = \frac{W}{2} + \frac{\theta}{\text{FOV}/2} \cdot \frac{W}{2}
$$

Where:
- \\(W\\): Image width in pixels
- \\(\theta\\): LiDAR angle (positive = left, negative = right)
- FOV: Camera field of view

## Implementation

### Core Fusion Logic

```python
def project_lidar_to_image(scan, image, camera_fov=30):
    """
    Project LiDAR points onto camera image
    """
    h, w = image.shape[:2]
    half_fov = camera_fov / 2

    # Middle row for 2D LiDAR projection
    y_row = h // 2

    angles = np.arange(scan.angle_min, scan.angle_max,
                       scan.angle_increment)
    angles_deg = np.degrees(angles)

    for i, (angle, distance) in enumerate(zip(angles_deg, scan.ranges)):
        # Filter to camera FOV
        if abs(angle) > half_fov:
            continue

        # Skip invalid readings
        if distance < scan.range_min or distance > scan.range_max:
            continue

        # Map angle to pixel x-coordinate
        x_pixel = int(w/2 - (angle / half_fov) * (w/2))

        # Bounds check
        if 0 <= x_pixel < w:
            # Color based on distance
            color = distance_to_color(distance)
            cv2.circle(image, (x_pixel, y_row), 3, color, -1)

    return image
```

### ROS Package Structure

```
fusion_package/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── fusion.launch
└── scripts/
    └── fusion.py
```

## Networking Configuration

### Challenge

Connecting Raspberry Pi to laptop hotspot required special configuration.

### Solution: NetworkManager with YAML

```yaml
# /etc/netplan/01-network-manager.yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "HotspotName":
          password: "password"
```

### ROS Network Setup

On TurtleBot3 (Raspberry Pi):
```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<RASPBERRY_PI_IP>
```

On PC:
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<PC_IP>
```

## Results

### Before Calibration

- LiDAR range: ±45°
- Camera FOV: 30°
- Result: LiDAR points extended beyond image boundaries

### After Calibration

- LiDAR range: ±15°
- Camera FOV: 30°
- Result: Proper alignment of LiDAR points within image

### Visual Output

```
┌─────────────────────────────────────┐
│                                     │
│                                     │
│   ●●●                   ●●●●●●     │
│      ●●●●             ●●●          │
│          ●●●●●●●●●●●●●             │
│                                     │
│                                     │
└─────────────────────────────────────┘
       (Distance-colored LiDAR points
        overlaid on camera image)
```

## Lessons Learned

1. **FOV Matching is Critical**: LiDAR and camera FOVs must be aligned
2. **2D LiDAR Limitation**: Only provides single scan plane
3. **Network Configuration**: ROS multi-machine setup requires careful IP management
4. **Time Synchronization**: Approximate sync works for most applications

## Future Improvements

| Improvement | Benefit |
|-------------|---------|
| 3D LiDAR | Full point cloud projection |
| Extrinsic calibration | More accurate alignment |
| Kalman filtering | Temporal smoothing |
| Object detection | Higher-level fusion |

## Summary

Key takeaways from this sensor fusion project:
1. 2D LiDAR provides horizontal scan data only
2. Camera FOV must be measured and matched
3. ROS simplifies multi-sensor integration
4. Network configuration is crucial for distributed systems

