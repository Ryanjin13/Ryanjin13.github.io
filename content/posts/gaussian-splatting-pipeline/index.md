---
title: "Gaussian Splatting Pipeline"
date: 2024-06-21
description: "3D Gaussian Splatting rendering architecture and workflow"
categories: ["3D Vision"]
tags: ["Gaussian Splatting", "3D Reconstruction", "Neural Rendering"]
draft: false
---

{{< katex >}}

## Overview

3D Gaussian Splatting is a novel approach for real-time radiance field rendering. This post explains the complete rendering pipeline from 3D Gaussians to final image.

## Pipeline Architecture

```
3D Gaussians → Projection → Tile-based Rasterization → Rendered Image
                                    ↓
                              Loss Function ← Target Image
                                    ↓
                            Gradient Update
```

## Stage 1: 3D Gaussian Representation

Each Gaussian is defined by:

$$
G(x) = e^{-\frac{1}{2}(x-\mu)^T \Sigma^{-1} (x-\mu)}
$$

**Parameters per Gaussian:**
- Position \\(\mu \in \mathbb{R}^3\\)
- Covariance \\(\Sigma \in \mathbb{R}^{3 \times 3}\\)
- Opacity \\(\alpha \in [0,1]\\)
- Spherical harmonics coefficients for view-dependent color

## Stage 2: Projection Mapping

Project 3D Gaussians to 2D screen space:

$$
\Sigma' = J W \Sigma W^T J^T
$$

Where:
- \\(W\\): World-to-camera transformation
- \\(J\\): Jacobian of projective transformation

The projected 2D covariance determines the Gaussian's footprint on screen.

## Stage 3: Differentiable Tile Rasterizer

### Tile-based Processing

Screen divided into tiles (typically 16×16 pixels):

1. **Culling:** Identify Gaussians intersecting each tile
2. **Sorting:** Order by depth (front-to-back)
3. **Blending:** Alpha compositing per pixel

### Per-Pixel Rendering

For each pixel, blend overlapping Gaussians:

$$
C = \sum_{i=1}^{N} c_i \alpha_i \prod_{j=1}^{i-1}(1 - \alpha_j)
$$

Where:
- \\(c_i\\): Color of Gaussian i
- \\(\alpha_i\\): Opacity contribution at pixel

### Why Tile-based?

| Advantage | Description |
|-----------|-------------|
| Parallelism | Each tile processed independently |
| Cache efficiency | Spatial locality |
| GPU optimization | Maps well to GPU architecture |

## Stage 4: Training Loop

### Forward Pass

1. Render image from current Gaussians
2. Compare with ground truth

### Loss Function

$$
L = (1-\lambda)L_1 + \lambda L_{D-SSIM}
$$

Where:
- \\(L_1\\): Pixel-wise L1 loss
- \\(L_{D-SSIM}\\): Structural similarity loss
- \\(\lambda\\): Typically 0.2

### Backward Pass

Gradients flow through:
- Color computation
- Alpha blending
- 2D projection
- 3D Gaussian parameters

### Adaptive Density Control

During training:
- **Split:** Large Gaussians with high gradient
- **Clone:** Small Gaussians with high gradient
- **Prune:** Low opacity or large Gaussians

## Optimization Details

### Differentiability

Key insight: Alpha blending is differentiable:

$$
\frac{\partial C}{\partial \alpha_i} = c_i \prod_{j<i}(1-\alpha_j) - \sum_{k>i} c_k \alpha_k \prod_{j<k, j\neq i}(1-\alpha_j)
$$

### Covariance Parameterization

To ensure positive semi-definiteness:

$$
\Sigma = RSS^TR^T
$$

Where:
- \\(R\\): Rotation (quaternion)
- \\(S\\): Scale (diagonal)

## Performance

| Metric | 3D Gaussian Splatting |
|--------|----------------------|
| Training time | ~30 minutes |
| Rendering FPS | 100+ |
| Quality (PSNR) | ~25-30 dB |
| Memory | ~500MB per scene |

## Comparison with NeRF

| Aspect | NeRF | 3DGS |
|--------|------|------|
| Representation | Implicit (MLP) | Explicit (Gaussians) |
| Rendering | Ray marching | Rasterization |
| Speed | Slow | Real-time |
| Editability | Difficult | Easy |
