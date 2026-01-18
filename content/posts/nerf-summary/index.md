---
title: "NeRF Summary"
date: 2024-06-20
description: "Neural Radiance Fields for novel view synthesis"
categories: ["3D Vision"]
tags: ["NeRF", "3D Reconstruction", "Neural Rendering"]
draft: false
---

{{< katex >}}

## Overview

NeRF (Neural Radiance Fields) represents 3D scenes as continuous functions learned by neural networks, enabling high-quality novel view synthesis from a set of input images.

## Core Concept

NeRF learns a function:

$$
F_\theta: (x, y, z, \theta, \phi) \rightarrow (r, g, b, \sigma)
$$

Input:
- Position: \((x, y, z)\)
- View direction: \((\theta, \phi)\)

Output:
- Color: \((r, g, b)\)
- Density: \(\sigma\)

## Pipeline

```
Input Images → Camera Poses → Ray Casting → MLP Network → Volume Rendering → Output Image
```

### 1. Positional Encoding

Convert coordinates to higher dimensions for better learning:

$$
\gamma(p) = (\sin(2^0\pi p), \cos(2^0\pi p), ..., \sin(2^{L-1}\pi p), \cos(2^{L-1}\pi p))
$$

3D coordinates → 60-dimensional representation (L=10)

### 2. Ray Sampling

For each pixel, cast ray from camera:

$$
\mathbf{r}(t) = \mathbf{o} + t\mathbf{d}
$$

Where:
- \(\mathbf{o}\): Camera origin
- \(\mathbf{d}\): Ray direction
- \(t\): Distance along ray

### 3. Volume Rendering

Accumulate color along ray:

$$
C(\mathbf{r}) = \int_{t_n}^{t_f} T(t) \cdot \sigma(\mathbf{r}(t)) \cdot \mathbf{c}(\mathbf{r}(t), \mathbf{d}) \, dt
$$

Where transmittance:

$$
T(t) = \exp\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s)) \, ds\right)
$$

## Training

### Loss Function

Photometric loss between rendered and ground truth:

$$
L = \sum_{\mathbf{r} \in R} \| \hat{C}(\mathbf{r}) - C(\mathbf{r}) \|_2^2
$$

### Process

1. Sample rays from training images
2. Sample points along each ray
3. Query MLP for color and density
4. Render pixel color via volume rendering
5. Backpropagate loss

**Note:** Each image requires backpropagation across all pixels.

## Implicit Representation

| Explicit (Voxels) | Implicit (NeRF) |
|-------------------|-----------------|
| Discrete coordinates | Continuous function |
| Fixed resolution | Arbitrary resolution |
| Memory intensive | Memory efficient |
| Fast inference | Slow inference |

NeRF samples real-valued coordinates continuously, enabling high-detail synthesis without explicit point storage.

## Inference

1. Define novel camera pose
2. Cast rays through each pixel
3. Sample points along rays
4. Query network for colors/densities
5. Accumulate via volume rendering

**Complexity:** Higher resolution = more computation. Accumulation stops when density reaches maximum threshold.

## Limitations

- Slow training and inference
- Requires accurate camera poses
- Static scenes only (original NeRF)
- Per-scene optimization

## Extensions

| Method | Improvement |
|--------|-------------|
| Instant-NGP | Fast training via hash encoding |
| Mip-NeRF | Anti-aliasing |
| NeRF-W | Handle varying lighting |
| D-NeRF | Dynamic scenes |
| 3D Gaussian Splatting | Real-time rendering |
