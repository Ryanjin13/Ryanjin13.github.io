---
title: "SLAM Basic"
date: 2024-06-21
description: "Simultaneous Localization and Mapping fundamentals"
categories: ["2D Vision"]
tags: ["SLAM", "Robotics", "Computer Vision", "Localization"]
draft: false
---

{{< katex >}}

## Overview

SLAM (Simultaneous Localization and Mapping) solves the chicken-and-egg problem: to localize, you need a map; to build a map, you need localization.

## The SLAM Problem

Given:
- Control inputs: \(u_{1:t}\)
- Observations: \(z_{1:t}\)

Estimate:
- Robot pose: \(x_{1:t}\)
- Map: \(m\)

$$
P(x_{1:t}, m | z_{1:t}, u_{1:t})
$$

## SLAM Components

```
┌─────────────┐     ┌─────────────┐
│   Sensor    │────→│  Frontend   │
│  (Camera,   │     │ (Feature    │
│   LiDAR)    │     │  Extraction)│
└─────────────┘     └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │   Backend   │
                    │(Optimization)│
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
         [Pose]        [Map]      [Loop Closure]
```

## Types of SLAM

### By Sensor

| Type | Sensor | Characteristics |
|------|--------|-----------------|
| Visual SLAM | Camera | Rich features, scale ambiguity |
| LiDAR SLAM | LiDAR | Accurate depth, expensive |
| RGB-D SLAM | RGB-D | Direct depth, limited range |

### By Approach

| Approach | Description |
|----------|-------------|
| Filter-based | EKF-SLAM, Particle Filter |
| Graph-based | Pose graph optimization |
| Direct | Use pixel intensities |
| Feature-based | Extract and match features |

## Frontend: Feature Extraction

### Keypoint Detection

Common detectors:
- SIFT, SURF, ORB
- FAST corners
- Harris corners

### Descriptor Matching

Match features between frames:

$$
d_{match} = \min_j \| f_i - f_j \|
$$

### Motion Estimation

**Essential Matrix (calibrated):**
$$
x_2^T E x_1 = 0
$$

**Fundamental Matrix (uncalibrated):**
$$
x_2^T F x_1 = 0
$$

## Backend: Optimization

### Pose Graph Optimization

Minimize error between measurements and estimates:

$$
x^* = \arg\min_x \sum_{ij} e_{ij}^T \Omega_{ij} e_{ij}
$$

Where:
- \(e_{ij}\): Error between poses i and j
- \(\Omega_{ij}\): Information matrix

### Bundle Adjustment

Joint optimization of poses and map points:

$$
\min_{T, P} \sum_{i,j} \| p_{ij} - \pi(T_i, P_j) \|^2
$$

Where:
- \(T_i\): Camera pose i
- \(P_j\): 3D point j
- \(\pi\): Projection function

## Loop Closure

Detect when robot revisits a location:

1. **Detection:** Bag-of-words, place recognition
2. **Verification:** Geometric consistency check
3. **Correction:** Add constraint to pose graph

Corrects accumulated drift.

## Popular SLAM Systems

| System | Type | Features |
|--------|------|----------|
| ORB-SLAM2/3 | Visual | Feature-based, loop closure |
| LSD-SLAM | Visual | Direct, semi-dense |
| Cartographer | LiDAR | 2D/3D, submap-based |
| LOAM | LiDAR | Edge and planar features |
| RTAB-Map | RGB-D | Real-time, loop closure |

## Evaluation Metrics

| Metric | Description |
|--------|-------------|
| ATE | Absolute Trajectory Error |
| RPE | Relative Pose Error |
| Map accuracy | Compared to ground truth |

## Challenges

1. **Dynamic environments** - Moving objects
2. **Scale drift** - Monocular vision
3. **Computational cost** - Real-time requirements
4. **Initialization** - Bootstrap problem
5. **Feature-poor environments** - Blank walls
