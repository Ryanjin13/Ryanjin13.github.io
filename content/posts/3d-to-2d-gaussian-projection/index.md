---
title: "3D to 2D Gaussian Projection"
date: 2024-06-22
description: "Mathematical method for converting 3D Gaussians to 2D splats for rendering"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "Projection", "Computer Graphics"]
draft: false
---

{{< katex >}}

## Overview

In 3D Gaussian Splatting, each 3D Gaussian must be projected onto the 2D image plane for rendering. This post explains the mathematical transformation.

## 3D Gaussian Definition

### Probability Density Function

$$
G(\mathbf{x}) = e^{-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x}-\boldsymbol{\mu})}
$$

Where:
- \\(\boldsymbol{\mu} \in \mathbb{R}^3\\): Center (mean)
- \\(\Sigma \in \mathbb{R}^{3 \times 3}\\): Covariance matrix

## Covariance Decomposition

### Parameterization

$$
\Sigma = RSS^TR^T
$$

Where:
- \\(R\\): Rotation matrix (3×3, orthogonal)
- \\(S\\): Scale matrix (diagonal)

### Scale Matrix

$$
S = \begin{pmatrix} s_x & 0 & 0 \\ 0 & s_y & 0 \\ 0 & 0 & s_z \end{pmatrix}
$$

### Why This Decomposition?

1. Guarantees positive semi-definiteness
2. Intuitive parameters (rotation + scale)
3. Easy to optimize

### Independence of Axes

Zero off-diagonal covariance means axes are independent:
- Each dimension's variance is separate
- No correlation between x, y, z directions

## 3D to 2D Projection

### Zwicker et al. Method

The 2D covariance after projection:

$$
\Sigma' = JW\Sigma W^T J^T
$$

Where:
- \\(W\\): View transformation matrix (world to camera)
- \\(J\\): Jacobian of the projective transformation
- \\(\Sigma'\\): 2D covariance (2×2 matrix)

### View Transformation

$$
W = \begin{pmatrix} R_{cam} & t \\ 0 & 1 \end{pmatrix}
$$

Transforms world coordinates to camera coordinates.

### Jacobian of Projection

For perspective projection:

$$
J = \begin{pmatrix}
\frac{f_x}{z} & 0 & -\frac{f_x x}{z^2} \\
0 & \frac{f_y}{z} & -\frac{f_y y}{z^2}
\end{pmatrix}
$$

Where:
- \\(f_x, f_y\\): Focal lengths
- \\(x, y, z\\): Point in camera coordinates

### Resulting 2D Gaussian

The 3×3 covariance reduces to 2×2:

$$
\Sigma'_{2D} \in \mathbb{R}^{2 \times 2}
$$

This 2D Gaussian is the "splat" rendered on screen.

## Properties of Orthogonal Matrices

### Key Property

For rotation matrix \\(R\\):

$$
R^{-1} = R^T
$$

This simplifies many calculations.

### Transformation of Covariance

When applying rotation \\(A\\) to data with covariance \\(\Sigma\\):

$$
\Sigma_{new} = A\Sigma A^T
$$

This is the \\(ABA^T\\) form common in statistics.

## Diagonal Approximation

### Simplification

Using only diagonal scaling (ignoring rotation):

$$
\Sigma \approx S^2 = \begin{pmatrix} s_x^2 & 0 & 0 \\ 0 & s_y^2 & 0 \\ 0 & 0 & s_z^2 \end{pmatrix}
$$

### Visual Distortion

This creates axis-aligned ellipsoids that may not match actual shape.

### Why It Works in Practice

As training progresses:
- Large Gaussians split into smaller ones
- Smaller Gaussians approximate any shape
- Visual artifacts diminish

## Implementation Details

### Mean Projection

Project center point:

$$
\mathbf{p} = \pi(W\boldsymbol{\mu})
$$

Standard pinhole camera projection.

### Covariance Projection

1. Transform to camera space: \\(W\Sigma W^T\\)
2. Apply Jacobian: \\(J(W\Sigma W^T)J^T\\)
3. Extract upper-left 2×2 block

### Rendering

With 2D mean and covariance:
- Evaluate Gaussian at each pixel
- Weight by opacity
- Blend colors

## Summary

| Step | Input | Output |
|------|-------|--------|
| 3D Definition | \\(\mu, \Sigma\\) | 3D Gaussian |
| View Transform | \\(W\\) | Camera-space Gaussian |
| Projection | \\(J\\) | 2D splat parameters |
| Rasterization | Pixel coords | Gaussian weight per pixel |
