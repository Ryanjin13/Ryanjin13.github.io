---
title: "3D Gaussian Covariance and Shape"
date: 2024-07-03
description: "How covariance matrices determine Gaussian shapes in 3D Gaussian Splatting"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Covariance", "3D Reconstruction"]
draft: false
---

{{< katex >}}

## Overview

In 3D Gaussian Splatting, the covariance matrix determines the shape and orientation of each Gaussian. Understanding this relationship is crucial for the rendering algorithm.

## Covariance and Shape

### 2D Gaussian Shape

The covariance matrix controls the ellipse:

$$
\Sigma = \begin{pmatrix} \sigma_x^2 & \sigma_{xy} \\ \sigma_{xy} & \sigma_y^2 \end{pmatrix}
$$

### Effect of Off-Diagonal Elements

| Covariance | Shape |
|------------|-------|
| \\(\sigma_{xy} = 0\\) | Axis-aligned ellipse |
| \\(\sigma_{xy} > 0\\) | Tilted toward quadrants 1 & 3 |
| \\(\sigma_{xy} < 0\\) | Tilted toward quadrants 2 & 4 |

### Visual Representation

```
σxy = 0:     σxy > 0:      σxy < 0:
   ○           ╱              ╲
  ( )        ╱              ╲
   ○       ╱                  ╲
Axis-aligned  Tilted right   Tilted left
```

## 3D Covariance Matrix

### Full 3×3 Matrix

$$
\Sigma = \begin{pmatrix}
\sigma_x^2 & \sigma_{xy} & \sigma_{xz} \\
\sigma_{xy} & \sigma_y^2 & \sigma_{yz} \\
\sigma_{xz} & \sigma_{yz} & \sigma_z^2
\end{pmatrix}
$$

### Diagonal Elements

Control the scale along each axis:
- \\(\sigma_x^2\\): Spread in x direction
- \\(\sigma_y^2\\): Spread in y direction
- \\(\sigma_z^2\\): Spread in z direction

### Off-Diagonal Elements

Control the tilt/rotation:
- Correlation between axes
- Determines orientation of ellipsoid

## Parameterization in Gaussian Splatting

### Decomposition

To ensure positive semi-definiteness:

$$
\Sigma = RSS^TR^T
$$

Where:
- \\(R\\): Rotation matrix (quaternion)
- \\(S\\): Scale matrix (diagonal)

### Simplified Approach

The algorithm often treats off-diagonal elements as zero initially:

$$
S = \begin{pmatrix} s_x & 0 & 0 \\ 0 & s_y & 0 \\ 0 & 0 & s_z \end{pmatrix}
$$

Then rotates the axis-aligned Gaussian.

## Projection to 2D

When rendering, 3D Gaussians project to 2D:

$$
\Sigma' = JW\Sigma W^TJ^T
$$

The 3D ellipsoid becomes a 2D ellipse on screen.

## Learning Process

### Trainable Parameters

Each Gaussian learns:

| Parameter | Purpose |
|-----------|---------|
| Position (μ) | Center location |
| Scale (s) | Size in each direction |
| Rotation (q) | Orientation (quaternion) |
| Opacity (α) | Transparency |
| Color (SH) | View-dependent appearance |

### Opacity Learning

- Opacity represents maximum density value
- Increases toward Gaussian center
- Training adjusts to match target appearance

### Achieving Full Opacity

Complete opacity requires:
1. Sufficient pixel-level decomposition
2. Adequate training iterations
3. Proper density control (split/clone)

## Density Distribution

### Gaussian Probability Density

$$
G(x) = \frac{1}{(2\pi)^{3/2}|\Sigma|^{1/2}} e^{-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)}
$$

Density is highest at center, falls off exponentially.

### Visualization

```
Cross-section of Gaussian:

      ░░░░░
    ░░▒▒▒▒▒░░
   ░▒▒▓▓▓▓▓▒▒░
  ░▒▓▓████▓▓▒░   ← Highest density at center
   ░▒▒▓▓▓▓▓▒▒░
    ░░▒▒▒▒▒░░
      ░░░░░
```

## Covariance and Rendering

### Alpha Blending

Each pixel accumulates Gaussian contributions:

$$
C = \sum_i c_i \alpha_i T_i
$$

Where \\(\alpha_i\\) depends on Gaussian density at that pixel.

### Coverage

Larger covariance = more pixels covered:
- Wide Gaussians: Many pixels, lower per-pixel density
- Narrow Gaussians: Fewer pixels, higher per-pixel density

## Optimization Considerations

### Memory Efficiency

Storing full covariance: 6 unique values
Storing scale + rotation: 3 + 4 = 7 values

Similar memory, but rotation parameterization is more stable.

### Numerical Stability

Covariance must be positive semi-definite:
- Direct optimization can violate this
- RSS^TR^T always valid
