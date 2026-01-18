---
title: "3D Gaussian Covariance"
date: 2024-07-05
description: "How covariance matrices define Gaussian shape in 3D Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "3D Reconstruction", "Covariance"]
draft: false
---

{{< katex >}}

## Overview

In 3D Gaussian Splatting, the covariance matrix is a key factor that defines the shape, size, and orientation of each Gaussian primitive.

## Covariance Matrix

### 2D Gaussian

$$
\Sigma_{2D} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} \\ \sigma_{xy} & \sigma_y^2 \end{bmatrix}
$$

### 3D Gaussian

$$
\Sigma_{3D} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} & \sigma_{xz} \\ \sigma_{xy} & \sigma_y^2 & \sigma_{yz} \\ \sigma_{xz} & \sigma_{yz} & \sigma_z^2 \end{bmatrix}
$$

## Effect of Covariance Values

### Diagonal Elements (Variance)

Control scaling along each axis:

| Element | Effect |
|---------|--------|
| \(\sigma_x^2\) | Spread in X direction |
| \(\sigma_y^2\) | Spread in Y direction |
| \(\sigma_z^2\) | Spread in Z direction |

### Off-Diagonal Elements (Covariance)

Control correlation between axes:

| Covariance | Distribution Shape |
|------------|-------------------|
| \(\sigma_{xy} = 0\) | Axis-aligned ellipse |
| \(\sigma_{xy} > 0\) | Tilted toward quadrants 1 & 3 |
| \(\sigma_{xy} < 0\) | Tilted toward quadrants 2 & 4 |

## Decomposition for Learning

Covariance matrix is decomposed for stable optimization:

$$
\Sigma = RSS^TR^T
$$

Where:
- \(R\): Rotation matrix (quaternion-based)
- \(S\): Scaling matrix (diagonal)

### Learnable Parameters

```
Per Gaussian:
- Position: (x, y, z)
- Scale: (s_x, s_y, s_z)
- Rotation: quaternion (q_w, q_x, q_y, q_z)
- Opacity: α
- Color: SH coefficients
```

## 3D to 2D Projection

### Projection Process

1. Set off-diagonal covariance to zero (axis-aligned)
2. Apply rotation to 3D Gaussian
3. Project to 2D image plane
4. Alpha-blend overlapping Gaussians

### 2D Covariance from 3D

$$
\Sigma_{2D} = JW\Sigma W^T J^T
$$

Where:
- \(J\): Jacobian of projection
- \(W\): World-to-camera transformation

## Rendering

### Alpha Blending

$$
C = \sum_{i=1}^{N} c_i \alpha_i \prod_{j=1}^{i-1}(1 - \alpha_j)
$$

Where:
- \(c_i\): Color of Gaussian i
- \(\alpha_i\): Opacity at pixel

### Opacity Calculation

$$
\alpha = o \cdot \exp\left(-\frac{1}{2}(x-\mu)^T \Sigma^{-1} (x-\mu)\right)
$$

Full opacity requires:
1. High opacity value (\(o\))
2. Sufficient Gaussian density (small \(\Sigma\))
3. Pixel near Gaussian center

## Training

Network optimizes per Gaussian:
1. **Position displacement** - Move centers
2. **Scale expansion/contraction** - Resize Gaussians
3. **Opacity levels** - Adjust transparency

Fine pixel-level subdivision during training achieves sharp details.
